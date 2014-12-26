#!/bin/bash

# Build job count
BUILDJOBS=${BUILDJOBS:-2}
BUILDJOBID=${BUILDJOBID:-a}

# Config count
CONFCOUNT=`find nuttx/configs -name \*defconfig -print | wc -l`
# Number of non-commented (#) and non-empty lines in ignore file
IGNORECOUNT=`cat .travis_ignored_configs.txt | sed '/^\s*#/d;/^\s*$/d' | wc -l`
# Get rid of whitespace
CONFCOUNT=$(echo $CONFCOUNT )
IGNORECOUNT=$(echo $IGNORECOUNT )

echo -e "\n"
echo -e "\0033[34mNuttX Configs: $CONFCOUNT total, $IGNORECOUNT excluded from build\0033[0m"
echo -e "(Add or remove configs to file .travis_ignored_configs to exclude)"
echo -e "----------------------------------------------------------------------------\n"

CONFIGS=configs.txt

find nuttx/configs -name \*defconfig -print > $CONFIGS

# Lines per file
LINES_TOTAL=$(wc -l <${CONFIGS})
((LINES_PER_FILE = (LINES_TOTAL + BUILDJOBS - 1) / BUILDJOBS))

# Perform the split
split -a1 -l${LINES_PER_FILE} ${CONFIGS} configparts.

echo -e "Operating total $BUILDJOBS build jobs"
echo -e "Running job: configparts.$BUILDJOBID"

cat "configparts.$BUILDJOBID" | xargs -L1 nuttx/tools/build_config.sh
