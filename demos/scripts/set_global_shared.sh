#!/bin/sh
#====================================
if [ $# -lt 1 ]
then
echo Usage: "./set_global_shared.sh <number_of_dsps>"
exit
fi

cd ../
# Initalise global shared memory ; required if the global shared memory is used by dsp image just after load.
#syntax
#./sdk/utils/dsp_utils/bin/dsp_utils init_global_shared_mem <dsp_no> <reserved DSP region> <Number of Global shared buffers> < Buffer size ><populate sytem config :0|1>

i=0
while [ $i -lt $1 ]
do

./dsp_utils/bin/dsp_utils init_global_shared_mem $i 0 1 0x400000 1

i=$((i+1))
done
