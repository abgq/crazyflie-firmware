#define DEBUG_MODULE "dw1kDeck"

#include "stm32fxxx.h"
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "autoconf.h"
#include "cfassert.h"
#include "debug.h"
#include "deck.h"
#include "estimator.h"
#include "log.h"
#include "mem.h"
#include "nvicconf.h"
#include "param.h"
#include "statsCnt.h"
#include "system.h"

#include "deca_device_api.h"
#include "deca_regs.h"

#define CS_PIN DECK_GPIO_IO1

// LOCO deck alternative IRQ and RESET pins(IO_2, IO_3) instead of default (RX1, TX1), leaving UART1 free for use
#ifdef CONFIG_DECK_LOCODECK_USE_ALT_PINS
#define GPIO_PIN_IRQ DECK_GPIO_IO2

#ifndef CONFIG_LOCODECK_ALT_PIN_RESET
#define GPIO_PIN_RESET DECK_GPIO_IO3
#else
#define GPIO_PIN_RESET DECK_GPIO_IO4
#endif

#define EXTI_PortSource EXTI_PortSourceGPIOB
#define EXTI_PinSource  EXTI_PinSource5
#define EXTI_LineN      EXTI_Line5
#else
#define GPIO_PIN_IRQ    DECK_GPIO_RX1
#define GPIO_PIN_RESET  DECK_GPIO_TX1
#define EXTI_PortSource EXTI_PortSourceGPIOC
#define EXTI_PinSource  EXTI_PinSource11
#define EXTI_LineN      EXTI_Line11
#endif

#define UUS_TO_DWT_TIME 64000

#define DW1K_COUNTER_MASK_40_BITS UINT64_C(0xFFFFFFFFFF)

// @note - example for computing the PDTCP: PRF = 64, PACS = 8, PDTCV = X, PDTCP = (127 * 4) * (1/499200000) * 8 * (X + 1) * 10e6 ≈ Y microseconds.
#define RESP_RX_PREAMBLE_DETECTION_TIMEOUT_UUS 200

#define TASK_DELAY_MS 10    

static bool isInit = false;

static uint32_t lastRangingCounter = 0;

static dwt_config_t uwb_config = {3, DWT_PRF_64M, DWT_PLEN_64, DWT_PAC8, 9, 9, 0, DWT_BR_6M8, DWT_PHRMODE_STD, (64 + 1 + 8 - 8)};

static uint8_t uwb_poll_frame[] = {0x41, 0x88, UINT8_C(0x00), 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, UINT8_C(0x00), UINT8_C(0x00)};
static uint8_t uwb_resp_frame[] = {0x41, 0x88, UINT8_C(0x00), 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, UINT8_C(0x00), UINT8_C(0x00), UINT8_C(0x00), UINT8_C(0x00), UINT8_C(0x00), UINT8_C(0x00)};

static uint8_t uwb_frame_seqnum = UINT8_C(0x00);
static uint8_t uwb_measnum = UINT8_C(0x00);

static uint32_t uwb_poll_tx_resp_rx_delta = UINT32_C(0x00);
static uint32_t uwb_poll_rx_resp_tx_delta = UINT32_C(0x00);

static uint64_t uwb_poll_tx_ts = UINT64_C(0x00);
static uint64_t uwb_resp_rx_ts = UINT64_C(0x00);

static volatile uint32_t uwb_sys_status = UINT32_C(0x00);

static uint8_t spiTxBuffer[128] = {UINT8_C(0x00)};
static uint8_t spiRxBuffer[128] = {UINT8_C(0x00)};
static uint8_t uwb_rx_buffer[128] = {UINT8_C(0x00)};

static uint16_t spiSpeed = SPI_BAUDRATE_2MHZ;

static TaskHandle_t dw1kTaskHandle = 0;

int writetospi(uint16_t head_octets_number, const uint8_t* head_octets_buffer, uint32_t body_octets_number, const uint8_t* body_octets_buffer)
{
    spiBeginTransaction(spiSpeed);
    digitalWrite(CS_PIN, LOW);
    memcpy(spiTxBuffer, head_octets_buffer, head_octets_number);
    memcpy(spiTxBuffer + head_octets_number, body_octets_buffer, body_octets_number);
    spiExchange(head_octets_number + body_octets_number, spiTxBuffer, spiRxBuffer);
    digitalWrite(CS_PIN, HIGH);
    spiEndTransaction();
    return 0;
}

int readfromspi(uint16_t head_octets_number, const uint8_t* head_octets_buffer, uint32_t body_octets_number, uint8_t* body_octets_buffer)
{
    spiBeginTransaction(spiSpeed);
    digitalWrite(CS_PIN, LOW);
    memcpy(spiTxBuffer, head_octets_buffer, head_octets_number);
    memset(spiTxBuffer + head_octets_number, 0, body_octets_number);
    spiExchange(head_octets_number + body_octets_number, spiTxBuffer, spiRxBuffer);
    memcpy(body_octets_buffer, spiRxBuffer + head_octets_number, body_octets_number);
    digitalWrite(CS_PIN, HIGH);
    spiEndTransaction();
    return 0;
}

void deca_sleep(unsigned int time_ms)
{
    vTaskDelay(M2T(time_ms));
}

void dw1kHandleInterrupt(void* parameters)
{
    uwb_sys_status = dwt_read32bitreg(SYS_STATUS_ID);

    if (uwb_sys_status & SYS_STATUS_TXFRS)
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);
    }

    if (uwb_sys_status & (SYS_STATUS_RXPHE | SYS_STATUS_RXRFSL))
    {
        dwt_rxreset();
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    }
    else if (uwb_sys_status & (SYS_STATUS_RXPTO | SYS_STATUS_RXSFDTO))
    {
        dwt_write32bitreg(SYS_STATUS_ID, (SYS_STATUS_RXPTO | SYS_STATUS_RXSFDTO));
    }
    else if (uwb_sys_status & (SYS_STATUS_LDEDONE | SYS_STATUS_RXFCG))
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD);

        dwt_readfromdevice(RX_BUFFER_ID, UINT8_C(0x00), sizeof(uwb_resp_frame) - UINT8_C(0x02), uwb_rx_buffer);

        if (uwb_rx_buffer[UINT8_C(0x02)] == uwb_frame_seqnum)
        {
            dwt_readfromdevice(TX_TIME_ID, TX_TIME_TX_STAMP_OFFSET, TX_TIME_TX_STAMP_LEN, (uint8_t*)(&uwb_poll_tx_ts));

            dwt_readfromdevice(RX_TIME_ID, RX_TIME_RX_STAMP_OFFSET, RX_TIME_RX_STAMP_LEN, (uint8_t*)(&uwb_resp_rx_ts));

            uwb_poll_tx_resp_rx_delta = (uint32_t)((uwb_resp_rx_ts - uwb_poll_tx_ts) & DW1K_COUNTER_MASK_40_BITS);

            uwb_poll_rx_resp_tx_delta = *((uint32_t*)(&uwb_rx_buffer[sizeof(uwb_resp_frame) - UINT8_C(0x06)]));

            lastRangingCounter = (uwb_poll_tx_resp_rx_delta - uwb_poll_rx_resp_tx_delta);
            
            uwb_measnum++;
        }
        else
        {
        }
    }
}

static void dw1kDeckTask(void* parameters)
{
    systemWaitStart();
        
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = M2T(TASK_DELAY_MS);
    
    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        uwb_poll_frame[UINT8_C(2)] = uwb_frame_seqnum;

        dwt_writetodevice(TX_BUFFER_ID, UINT8_C(0x00), sizeof(uwb_poll_frame) - UINT8_C(0x02), uwb_poll_frame);

        dwt_writetxfctrl(sizeof(uwb_poll_frame), UINT8_C(0x00), UINT8_C(0x01));

        dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint8_t)(SYS_CTRL_TXSTRT | SYS_CTRL_WAIT4RESP));

        if (ulTaskNotifyTake(pdTRUE, M2T(5)) > 0)
        {
            dw1kHandleInterrupt(NULL);
        }
        else
        {
            uwb_measnum = 0;
        }

        uwb_frame_seqnum++;
    }
}

#if CONFIG_DECK_LOCODECK_USE_ALT_PINS
void __attribute__((used)) EXTI5_Callback(void)
#else
void __attribute__((used)) EXTI11_Callback(void)
#endif
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    // Unlock interrupt handling task
    vTaskNotifyGiveFromISR(dw1kTaskHandle, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD();
    }
}

void reset_DW1000(void)
{
    // Reset the DW1000 chip
    digitalWrite(GPIO_PIN_RESET, 0);
    vTaskDelay(M2T(10));
    digitalWrite(GPIO_PIN_RESET, 1);
    vTaskDelay(M2T(10));
}

static void dw1kDeckInit(DeckInfo* info)
{
    if (isInit)
    {
        return;
    }

    EXTI_InitTypeDef EXTI_InitStructure;

    spiBegin();

    // Set up interrupt
    SYSCFG_EXTILineConfig(EXTI_PortSource, EXTI_PinSource);

    EXTI_InitStructure.EXTI_Line = EXTI_LineN;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Init pins
    pinMode(CS_PIN, OUTPUT);
    pinMode(GPIO_PIN_RESET, OUTPUT);
    pinMode(GPIO_PIN_IRQ, INPUT);

    reset_DW1000();

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        while (0x01U)
        {
        };
    }

    dwt_configure(&uwb_config);
    dwt_setleds(DWT_LEDS_ENABLE);
    dwt_setinterrupt(DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFSL | /*DWT_INT_TFRS |*/ DWT_INT_RXDFR | DWT_INT_RXPTO);

    dwt_setpreambledetecttimeout(RESP_RX_PREAMBLE_DETECTION_TIMEOUT_UUS);

    DEBUG_PRINT("dw1kDeck initialization complete\n");

    xTaskCreate(dw1kDeckTask, "dw1kDeckTask", configMINIMAL_STACK_SIZE, NULL, LPS_DECK_TASK_PRI, &dw1kTaskHandle);
    // Normally hardware initialization would happen here. For the example we
    // simply flag the driver as initialized.
    isInit = true;
}

static bool dw1kDeckTest(void)
{
    DEBUG_PRINT("Hello dw1kDeck!\n");
    return isInit;
}

static const DeckDriver dw1kDeckDriver = {
    .name = "dw1kDeck",
    .init = dw1kDeckInit,
    .test = dw1kDeckTest,
};

DECK_DRIVER(dw1kDeckDriver);

// --------------------------- Parameters ----------------------------------
// Parameters are grouped under a namespace. Each PARAM_ADD_* call publishes a
// variable that can be read/written from cfclient or the Crazyflie Python API.

PARAM_GROUP_START(deck)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, dw1k, &isInit)
PARAM_GROUP_STOP(deck)

// ----------------------------- Logging ------------------------------------
// Log groups expose read-only telemetry. Each LOG_ADD_* macro publishes one
// value that can be streamed or fetched through the logging subsystem.

LOG_GROUP_START(dw1k)
LOG_ADD(LOG_UINT32, rangingCounter, &lastRangingCounter)
// LOG_ADD(LOG_UINT8, sequenceNumber, &uwb_frame_seqnum)
LOG_ADD(LOG_UINT8, measurementNumber, &uwb_measnum)
LOG_GROUP_STOP(dw1k)
