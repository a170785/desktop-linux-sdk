#!/bin/sh

root_dir=../..

#Get pci devices info
../dsp_utils/bin/dsp_utils print_pci_info > pciinfo.txt

num_of_devices=$(awk -F: '$1 ~ "TI PCI devices" {print $2}' pciinfo.txt)
echo Num of devices $num_of_devices
#syntax
# ../utils/dsp_utils/bin/dsp_utils <dsp_number> 
i=0
while [ $i -lt $num_of_devices ]
do
$root_dir/demos/dsp_utils/bin/dsp_utils reset_dsp $i

i=$((i+1))
done
