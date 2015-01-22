#!/bin/sh
# Load driver if not already installed
if [ ! -e /dev/cmem ]
then
sudo sh ./load.sh
fi
# Set permissions
sudo chmod ugo+rw /dev/cmem
# Create shell script for automatic load through udev
original_path_value=$(pwd)
cd ../..
dlsdk_path_value=$(pwd)
cd $original_path_value
# Copy cmem driver to kernel driver directory
echo Installing cmem driver for auto load
kernel_name=$(uname -r)
if [ ! -e /lib/modules/$kernel_name/kernel/drivers/cmem ]
then
sudo mkdir /lib/modules/$kernel_name/kernel/drivers/cmem
fi
sudo cp ../../sdk/cmem/module/cmem_dev.ko /lib/modules/$kernel_name/kernel/drivers/cmem/.
cmem_hits=$(sudo grep -c "cmem_dev" /etc/modules)
if [ $cmem_hits -eq 0 ]
then
sudo sed -i '$ a cmem_dev' /etc/modules
fi
# Copy information file
echo "DESKTOP_LINUX_SDK_PATH="$dlsdk_path_value"" > cmeminfo.txt
sudo cp cmeminfo.txt /lib/modules/$kernel_name/kernel/drivers/cmem/.
echo updating auto loading of driver.  Please wait...
sudo depmod -a
echo Configuration for auto load of cmem complete.
