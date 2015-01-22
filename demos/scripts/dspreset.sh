#!/bin/sh
if [ $# -lt 1 ]
then
echo "Usage: ./dspreset.sh <number_of_dsps>"
exit
fi
i=0
#syntax
# ../utils/dsp_utils/bin/dsp_utils <dsp_number> 
while [ $i -lt $1 ]
do

../dsp_utils/bin/dsp_utils reset_dsp $i 
i=$((i+1))
done
