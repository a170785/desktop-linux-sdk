#!/bin/sh
sudo -v
#Manually set permissions immediately to enable execution of demos
sh ./driver_init.sh

# Create shell script for automatic load through udev
original_path_value=$(pwd)
cd ../..
dlsdk_path_value=$(pwd)
cd $original_path_value
echo "#!/bin/sh" > udev/c6678_udev.sh
echo "DESKTOP_LINUX_SDK_PATH="$dlsdk_path_value"" >> udev/c6678_udev.sh
cat udev/c6678_udev.txt >> udev/c6678_udev.sh

#copy file to udev area
sudo cp udev/c6678_udev.sh /etc/udev/rules.d/.
sudo cp udev/20-c6678.rules /etc/udev/rules.d/.
sudo touch /var/log/c6678_udev.log
sudo chmod ugo+rw /etc/udev/rules.d/20-c6678.rules
sudo chmod ugo+x /etc/udev/rules.d/c6678_udev.sh
sudo chmod ugo+rw /var/log/c6678_udev.log


