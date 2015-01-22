#!/bin/sh
sudo -v
if [ -e /dev/cmem ]
then

#remove driver 
sudo sh ./unload.sh
#remove files to udev area
fi
if [ -e /etc/udev/rules.d/20-c6678.rules ]
then
sudo rm /etc/udev/rules.d/20-c6678.rules
fi
if [ -e /etc/udev/rules.d/c6678_udev.sh ]
then
sudo rm /etc/udev/rules.d/c6678_udev.sh
fi

if [ -e /etc/udev/rules.d/20-c6678.rules ]
then
sudo rm /etc/udev/rules.d/20-c6678.rules
fi

if [ -e udev/c6678_udev.sh ]
then
rm udev/c6678_udev.sh
fi
