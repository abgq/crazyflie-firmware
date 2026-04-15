#!/bin/bash

set -e  # Exit on error

UPSTREAM_URL="https://github.com/bitcraze/crazyflie-firmware.git"
WORK_BRANCH="wzl/master"

# Check if upstream remote exists, if not add it
if ! git remote | grep -q "upstream"; then
    echo "Adding upstream remote..."
    git remote add upstream $UPSTREAM_URL
fi

# Fetch upstream changes
echo "Fetching upstream..."
git fetch upstream

# Update fork's master
echo "Updating local master..."
git checkout master
git merge upstream/master
git push origin master

# Rebase work branch on top of updated master
echo "Rebasing $WORK_BRANCH on master..."
git checkout $WORK_BRANCH
git rebase master

# Force push rebased branch
echo "Force pushing $WORK_BRANCH..."
git push origin $WORK_BRANCH --force

echo "Done! All branches are up to date."