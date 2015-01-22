#!/bin/sh
# Find maximum total physical memory available
if [ ! -e grub.orig ]
then
echo ERROR: original grub not found. Please check.
exit
fi
# check sudo access right at the beginning
sudo -v

orig_grub_line=$(grep "GRUB_CMDLINE_LINUX=" grub.orig)
updated_grub_line=$(sudo grep "GRUB_CMDLINE_LINUX=" /etc/default/grub)
echo Grub configuration line getting replaced to remove reserved memory for PCIE access
echo Current line: $updated_grub_line
echo Original line to be restored: $orig_grub_line
echo CAUTION: This will free up the reserved memory for general usage
echo "Please confirm to replace line(Y/n)"
read userInput
if [ "$userInput" = "Y" ]
then
sudo sed -i -e '/GRUB_CMDLINE_LINUX=/ c \
'"$orig_grub_line"'' /etc/default/grub 
sudo update-grub
if [ "$?" = "0" ]
then
rm grub.orig
else
echo error updating grub. Uninstall incomplete..
fi
echo Please reboot for configuration to take effect
else
echo GRUB NOT configured. Check parameters.
exit
fi
if [ -e ../../sdk/cmem/module/cmemcfg.orig.h ]
then
# Bring back original file 
cp ../../sdk/cmem/module/cmemcfg.orig.h ../../sdk/cmem/module/cmemcfg.h
echo Changed ../../sdk/cmem/module/cmemcfg.h
echo Rebuilding cmem
cd ../../sdk/cmem/module
make clean
make
echo cmem build complete.
echo Please reboot for the settings to take effect
else
echo ERROR: Original file missing. Please check.  Uninstall incomplete...
fi

