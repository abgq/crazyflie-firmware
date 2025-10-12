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

#define DEFAULT_RX_TIMEOUT 10000

#define UUS_TO_DWT_TIME 64000

// @note - example for computing the PDTCP: PRF = 64, PACS = 8, PDTCV = X, PDTCP = (127 * 4) * (1/499200000) * 8 * (X + 1) * 10e6 ≈ Y microseconds.
#define RESP_RX_PREAMBLE_DETECTION_TIMEOUT_UUS 70

static bool isInit = false;

static uint16_t rangingIntervalMs = 1000;
static float lastRangeMeters = 0.0f;

static uint32_t successfulMeasurements = 0;
static uint32_t failedMeasurements = 0;

// static uint8_t resetCountersRequest = 0;

// static void resetCountersCallback(void)
// {
//   if (!isInit) {
//     return;
//   }

//   if (resetCountersRequest) {
//     successfulMeasurements = 0;
//     failedMeasurements = 0;
//     resetCountersRequest = 0;
//   }
// }

static uint8_t spiTxBuffer[196];
static uint8_t spiRxBuffer[196];
static uint16_t spiSpeed = SPI_BAUDRATE_2MHZ;

int writetospi(uint16_t head_octets_number, const uint8_t* head_octets_buffer, uint32_t body_octets_number, const uint8_t* body_octets_buffer)
{
    spiBeginTransaction(spiSpeed);
    digitalWrite(CS_PIN, LOW);
    memcpy(spiTxBuffer, head_octets_buffer, head_octets_number);
    memcpy(spiTxBuffer + head_octets_number, body_octets_buffer, body_octets_number);
    spiExchange(head_octets_number + body_octets_number, spiTxBuffer, spiRxBuffer);
    digitalWrite(CS_PIN, HIGH);
    spiEndTransaction();
    // STATS_CNT_RATE_EVENT(&spiWriteCount);
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
    // STATS_CNT_RATE_EVENT(&spiReadCount);
    return 0;
}

void deca_sleep(unsigned int time_ms)
{
    vTaskDelay(M2T(time_ms));
}

static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
// static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0};

volatile uint32_t uwb_sys_status = 0x00U;

static void dw1kDeckTask(void* parameters)
{
    systemWaitStart();
    while (1)
    {
        // Write the poll message to the TX buffer and configure the TX frame length.
        dwt_writetodevice(TX_BUFFER_ID, 0x00U, sizeof(tx_poll_msg) - 0x02U, tx_poll_msg);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 0x01U);

        // Start the TX.
        dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint8_t)(SYS_CTRL_TXSTRT));

        while (0x01)
        {
            uwb_sys_status = dwt_read32bitreg(SYS_STATUS_ID);
            if (uwb_sys_status & SYS_STATUS_TXFRS)
            {
                dwt_write32bitreg(SYS_STATUS_ID, uwb_sys_status);
                break;
            }
            vTaskDelay(M2T(1));
        }

        vTaskDelay(M2T(1000));
    }
}

static dwt_config_t uwb_config = {
    3,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_64,     /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (64 + 1 + 8 - 8) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

void reset_DW1000(void)
{
    // Reset the DW1000 chip
    digitalWrite(GPIO_PIN_RESET, 0);
    vTaskDelay(M2T(10));
    digitalWrite(GPIO_PIN_RESET, 1);
    vTaskDelay(M2T(10));
}

// void port_set_dw1000_slowrate(void)
// {
// }

// port_set_dw1000_fastrate()
// {
// }

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
    // port_set_dw1000_slowrate();
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        while (0x01U)
        {
        };
    }
    // port_set_dw1000_fastrate();
    dwt_configure(&uwb_config);
    dwt_setleds(DWT_LEDS_ENABLE);
    // dwt_setinterrupt(DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFSL | DWT_INT_TFRS | DWT_INT_RXDFR | DWT_INT_RXPTO);
    // dwt_setpreambledetecttimeout(RESP_RX_PREAMBLE_DETECTION_TIMEOUT_UUS);

    DEBUG_PRINT("dw1kDeck initialization complete\n");

    xTaskCreate(dw1kDeckTask, "dw1kDeckTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
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
// PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, resetCounters, &resetCountersRequest, resetCountersCallback)
PARAM_GROUP_STOP(deck)

// ----------------------------- Logging ------------------------------------
// Log groups expose read-only telemetry. Each LOG_ADD_* macro publishes one
// value that can be streamed or fetched through the logging subsystem.

LOG_GROUP_START(dw1k)
LOG_ADD(LOG_UINT8, ready, &isInit)
LOG_ADD(LOG_UINT16, interval, &rangingIntervalMs)
LOG_ADD(LOG_FLOAT, range, &lastRangeMeters)
LOG_ADD(LOG_UINT32, successCount, &successfulMeasurements)
LOG_ADD(LOG_UINT32, failCount, &failedMeasurements)
LOG_GROUP_STOP(dw1k)
