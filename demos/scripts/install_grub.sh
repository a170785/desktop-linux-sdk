#!/bin/sh

if [  $# -lt 1 ]
then

echo Usage: sudo ./install_grub.sh \<Reserve_memory_in_MB\>\[consistent_memory_in_MB\]\>
exit
fi
# check sudo access right at the beginning
sudo -v
# Find maximum total physical memory available
max_physical_mb=$(sudo dmidecode -t 17 | awk '$1~/Size/ && $3~/MB/ {print $2}' | awk '{ total_mem += $1 } END { print total_mem }')
max_physical=$((max_physical_mb*1024*1024))
# Calculate size from max 
echo Maximum physical memory  $max_physical
# Default reserved consistent memory = 8
reserved_consistent_memory_mb=8
if [ $# -gt 1 ]
then
reserved_consistent_memory_mb=$2
fi
#round up to multiple of 8 MB
reserved_consistent_memory_mb=$((((reserved_consistent_memory_mb+7)/8)*8))
if [ $1 -lt $reserved_consistent_memory_mb ]
then
echo Reserved consistent memory greater than total reserved memory
exit
fi
echo Reserved consistent memory $reserved_consistent_memory_mb MB
reserved_consistent_memory=$((reserved_consistent_memory_mb*1024*1024))

num_hits=$(sudo grep -c "GRUB_CMDLINE_LINUX=" /etc/default/grub)
#echo Hits GRUB_CMDLINE_LINUX $num_hits
# make sure limited max is aligned to 8 MB
limited_max_mb=$((((max_physical_mb-$1)/8)*8))
if [ $limited_max_mb -lt 4096 ]
then
echo Reservation requested $1 MB. Max memory after reservation: $limited_max_mb MB 
echo "ERROR: Cannot use GRUB reservation scheme with < 4 GB memory. Add more memory or reduced reserved size "
exit
fi
limited_max=$((limited_max_mb*1024*1024))
reserved_size_mb=$((max_physical_mb-limited_max_mb))
echo Requested allocation $1 MB Reserved: $reserved_size_mb MB
reserved_size=$((reserved_size_mb*1024*1024))
if [ $limited_max_mb -lt 512 ]
then
echo ERROR: Not sufficient memory to reserve. Please check available memory
exit
fi
echo Limiting to max memory $limited_max_mb MB
#==============================================================================
if [ $num_hits -eq 0 ]
then
echo Adding line to /etc/default/grub
echo "GRUB_CMDLINE_LINUX=\"mem="$limited_max_mb"m\""
echo CAUTION: This will limit your maximum memory available for other applications
echo "Please confirm to Add line(Y/n)"
read userInput
if [ "$userInput" = "Y" ]
then
if [ ! -e grub.orig ]
then
cp /etc/default/grub grub.orig
fi 
sudo sed -i -e '/GRUB_CMDLINE_LINUX_DEFAULT=/ a \
GRUB_CMDLINE_LINUX="mem='$limited_max_mb'm"' /etc/default/grub
else
echo GRUB NOT configured. Check parameters.
exit
fi
fi
echo numhits : $num_hits
#==============================================================================
if [ $num_hits -eq 1 ]
then
cur_grub_line=$(sudo grep "GRUB_CMDLINE_LINUX=" /etc/default/grub)
new_grub_line=$(sed -e '/GRUB_CMDLINE_LINUX=/ s/mem=[0-9a-zA-Z]*//g' /etc/default/grub | sed -e 's/GRUB_CMDLINE_LINUX="/GRUB_CMDLINE_LINUX="mem='$limited_max_mb'M /' | grep "GRUB_CMDLINE_LINUX=")

if [ -e grub.orig ]
then
echo There is already a grub.orig preserved by earlier install 
orig_grub_line=$(sudo grep "GRUB_CMDLINE_LINUX=" grub.orig)
echo current grub line: $cur_grub_line
echo orig grub line: $orig_grub_line 
echo If the preserved original grub line need to be used as the starting point of installation. 
echo Enter P. \(else press Enter to use current grub file\)
read userInput
if [ "$userInput" = "P" ]
then
new_grub_line=$(sed -e '/GRUB_CMDLINE_LINUX=/ s/mem=[0-9a-zA-Z]*//g' grub.orig | sed -e 's/GRUB_CMDLINE_LINUX="/GRUB_CMDLINE_LINUX="mem='$limited_max_mb'M /' | grep "GRUB_CMDLINE_LINUX=")
else
cp /etc/default/grub grub.orig
fi
fi
echo Grub configuration line getting replaced to reserve memory for PCIE access
echo current line: $cur_grub_line
echo New line: $new_grub_line
echo CAUTION: This will limit your maximum memory available for other applications
echo "Please confirm to replace line(Y/n)"
read userInput
if [ "$userInput" = "Y" ]
then
if [ ! -e grub.orig ]
then
cp /etc/default/grub grub.orig
fi
sudo sed -i -e '/GRUB_CMDLINE_LINUX=/ c \
'"$new_grub_line"'' /etc/default/grub 
else
echo GRUB NOT configured. Check parameters.
exit
fi
fi
#==============================================================================
echo Updating grub to reserve memory.
echo This may take some time to complete. Please wait...
sudo update-grub
if [ "$?" = "0" ]
then
echo Grub update success.  Need reboot for Grub to take effect.
else
echo Grub update failed...
sudo cp grub.orig /etc/default/grub
exit
fi
echo Updated Cmd line: 
grep  "GRUB_CMDLINE_LINUX=" /etc/default/grub
echo GRUB configuration complete.
#==============================================================================
limited_max_string=$(printf "%x" $limited_max)
reserved_size_string=$(printf "%x" $reserved_size)
consistent_size_string=$(printf "%x" $reserved_consistent_memory)
if [ -e ../../sdk/cmem/module/cmemcfg.orig.h ]
then
# Bring back original file 
cp ../../sdk/cmem/module/cmemcfg.orig.h ../../sdk/cmem/module/cmemcfg.h
else
# If it is first time backup original file
echo backing up original cmemcfg file
cp ../../sdk/cmem/module/cmemcfg.h ../../sdk/cmem/module/cmemcfg.orig.h
fi
sed -i -e '/CMEM_CFG_USE_DMA_COHERENT_ALLOC/  c \
\/\/#define CMEM_CFG_USE_DMA_COHERENT_ALLOC'  -e '/#define CMEM_CFG_RESERVED_MEM_START_ADDRESS/  c \
#define CMEM_CFG_RESERVED_MEM_START_ADDRESS 0x'$limited_max_string'' -e '/#define CMEM_CFG_RESERVED_MEM_SIZE/  c \
#define CMEM_CFG_RESERVED_MEM_SIZE 0x'$reserved_size_string'' -e '/#define RESERVED_CONSISTENT_MEM_SIZE/  c \
#define RESERVED_CONSISTENT_MEM_SIZE 0x'$consistent_size_string'' ../../sdk/cmem/module/cmemcfg.h
echo Changed ../../sdk/cmem/module/cmemcfg.h
echo Rebuilding cmem
cd ../../sdk/cmem/module
make clean
make
echo cmem rebuild complete
echo If required install_cmem_autoload.sh for automatic loading of driver
echo Please reboot for settings to take effect
