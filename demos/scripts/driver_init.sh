#!/bin/sh
#Load driver if not already installed
sudo -v
if [ ! -e /dev/cmem ]
then
sudo sh ./load.sh
fi
#Manually set permissions immediately to enable execution of demos
sudo chmod ugo+rw /dev/cmem
# Do dummy run to make enable pcie window permissions
echo Executing dummy application to set permissions for pci memory windows
sudo ../dsp_utils/bin/dsp_utils init_global_shared_mem 0 0 0 0x400000 0 

