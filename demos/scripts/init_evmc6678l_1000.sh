#!/bin/sh
#====================================


cd ../..
cp ./demos/scripts/initcfg_1000.txt ./demos/scripts/initcfg.txt
#load init.hex to initialize DDR and ETH Phy
# syntax
# ./utils/dsp_utils/bin/dsp_utils load <dsp_no> <core_no> <entry_point> <dsp hex image name> <boot config filename|0>

./demos/dsp_utils/bin/dsp_utils load 0 0 0x00860000 ./sdk/dsp_projects/dsp_init/build/bin/evmc6678l/init.out ./demos/scripts/initcfg.txt
