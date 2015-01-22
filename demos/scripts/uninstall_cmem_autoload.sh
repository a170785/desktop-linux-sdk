#!/bin/sh
# Copy cmem driver to kernel driver directory
echo Un-Installing cmem driver from auto load
kernel_name=$(uname -r)
if [ -e /lib/modules/$kernel_name/kernel/drivers/cmem ]
then
cd  /lib/modules/$kernel_name/kernel/drivers/cmem
sudo rmmod cmem_dev.ko
cd ..
sudo rm -rf cmem
fi
sudo sed -i '/cmem_dev/d' /etc/modules
echo Auto load for cmem removed
