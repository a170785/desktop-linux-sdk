#!/bin/sh
if [ $# -lt 1 ]
then
echo "Usage: ./dnld_demo_loopback.sh <number_of_dsps>"
exit
fi
# syntax
# ../../..demos/dsp_utils/bin/dsp_utils load <dsp_no> <core_no> <entry_point | 0> <dsp hex image name>
i=0

while [ $i -lt $1 ]
do

../../../demos/dsp_utils/bin/dsp_utils load $i 0xffff 0 ../c66x/demo_loopback/build/bin/demo_loopback.out ./bootcfg/bootcfg_$i.txt
i=$((i+1))
done
