#!/bin/sh

if [ $# -lt 2 ]
then
echo "Usage: $0 <num_of_dsps> <payload size in hex>"
exit
fi

# Syntax
# ../host/bin/demo_filetest <inputfile> <outputfile> <0|1|2|3>(0: Memcopy; 1: Direct Map; 2 : DMA 3 : Huge buf) <dspbits>  <num of dsps> <payload size > <dsp image>
# dspbits: Bit map of participating cores # 0 :  only core number 0 participating
# 0x3: 0 & 1 cores participating
# 0xffffffff indicates all 32 cores participating

../host/bin/demo_filetest ../../../../testfiles/test.file testout.file 3 0xffffffff $1 $2 ../c66x/demo_loopback/build/bin/demo_loopback.out
