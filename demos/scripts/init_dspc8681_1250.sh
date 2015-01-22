#!/bin/sh
#====================================
if [ $# -lt 1 ]
then
echo "Usage: ./init.sh <number_of_dsps>"
exit
fi

cd ../..

cp ./demos/scripts/initcfg_1250.txt ./demos/scripts/initcfg.txt
#load init.hex to initialize DDR and ETH Phy
# syntax
# ./utils/dsp_utils/bin/dsp_utils load <dsp_no> <core_no> <entry_point> <dsp hex image name> <boot config filename|0>
i=0
while [ $i -lt $1 ]
do
./demos/dsp_utils/bin/dsp_utils load $i 0 0x00860000 ./sdk/dsp_projects/dsp_init/build/bin/dspc8681/init.out ./demos/scripts/initcfg.txt
i=$((i+1))
done
