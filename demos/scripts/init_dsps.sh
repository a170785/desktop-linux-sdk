#!/bin/sh
#====================================
if [ $# -lt 1 ]
then
echo "Usage: $0 <cpu frequency>"
exit
fi

root_dir=../..
#Get pci devices info
../dsp_utils/bin/dsp_utils print_pci_info > pciinfo.txt

#load init.hex to initialize DDR and ETH Phy
# syntax
# ./utils/dsp_utils/bin/dsp_utils load <dsp_no> <core_no> <entry_point> <dsp hex image name> <boot config filename|0>
num_of_devices=$(awk -F: '$1 ~ "TI PCI devices" {print $2}' pciinfo.txt)
echo Number of devices: $num_of_devices
i=0
while [ $i -lt $num_of_devices ]
do
switch_device=$(awk '$1=="dsp_id" && $2=='$i' { print $5}' pciinfo.txt)
dsp_device=evmc6678l
if [ "$switch_device" -eq "8624" ]
then
dsp_device=dspc8681
fi
if [ "$switch_device" -eq "8748" ]
then
dsp_device=dspc8682
fi

echo Device: $i : $dsp_device
$root_dir/demos/dsp_utils/bin/dsp_utils load $i 0 0x00860000 $root_dir/sdk/dsp_projects/dsp_init/build/bin/$dsp_device/init.out initcfg_$1.txt
i=$((i+1))
done
