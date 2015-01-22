#*******************************************************************************
#* FILE PURPOSE: Lower level makefile for Creating Platform Lite Libraries
#*******************************************************************************
#* FILE NAME: ./lib/ti.platform.dspc8681.lite.lib.mk
#*
#* DESCRIPTION: Defines Source Files, Compilers flags and build rules
#*
#*
#*******************************************************************************
#

#
# Macro definitions referenced below
#
empty =
space =$(empty) $(empty)
CC = $(C6X_GEN_INSTALL_PATH)/bin/cl6x -c -mo -g -q -k -eo.o -mv6600 --abi=elfabi
AC = $(C6X_GEN_INSTALL_PATH)/bin/cl6x -c -qq -mv6600 --abi=elfabi
ARIN = $(C6X_GEN_INSTALL_PATH)/bin/ar6x rq
LD = $(C6X_GEN_INSTALL_PATH)/bin/lnk6x --abi=elfabi
RTSLIB = -l $(C6X_GEN_INSTALL_PATH)/lib/rts6600_elf.lib
INCS = -I. -I$(strip $(subst ;, -I,$(subst $(space),\$(space),$(INCDIR))))
OBJEXT = lite.oe66
AOBJEXT = lite.se66
INTERNALDEFS = -Dti_targets_elf_C66 -D_PLATFORM_LITE_  -DMAKEFILE_BUILD -eo.$(OBJEXT) -ea.$(AOBJEXT) -fr=$(@D) -fs=$(@D) -ppa -ppd=$@.dep
INTERNALLINKDEFS = -o $@ -m $@.map
OBJDIR = $(LIBDIR)

#List the COMMONSRC Files
COMMONSRCC= \
    src/evm66x_i2c.c \
    src/evm66x_nand.c \
    src/dspc8681.c \
    src/dspc8681_phy.c \
    src/evmc66x_fpga.c \
    src/evmc66x_gpio.c \
    src/evmc66x_i2c_eeprom.c \
    src/evmc66x_nor.c \
    src/evmc66x_spi.c \
    src/evmc66x_uart.c \
    src/platform.c 

# FLAGS for the COMMONSRC Files
COMMONSRCCFLAGS =   -i./src -i. -i./test 

# Make Rule for the COMMONSRC Files
COMMONSRCCOBJS = $(patsubst %.c, $(OBJDIR)/%.$(OBJEXT), $(COMMONSRCC))

$(COMMONSRCCOBJS): $(OBJDIR)/%.$(OBJEXT): %.c
	-@echo cle66 $< ...
	if [ ! -d $(@D) ]; then $(MKDIR) $(@D) ; fi;
	$(RM) $@.dep
	$(CC) $(COMMONSRCCFLAGS) $(INTERNALDEFS) $(INCS) -fc $< 
	-@$(CP) $@.dep $@.pp; \
         $(SED) -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
             -e '/^$$/ d' -e 's/$$/ :/' < $@.pp >> $@.dep; \
         $(RM) $@.pp 

#Create Empty rule for dependency
$(COMMONSRCCOBJS):.\lib\ti.platform.dspc8681.lite.lib.mk
.\lib\ti.platform.dspc8681.lite.lib.mk:

#Include Depedency for COMMONSRC Files
ifneq (clean,$(MAKECMDGOALS))
 -include $(COMMONSRCCOBJS:%.$(OBJEXT)=%.$(OBJEXT).dep)
endif


$(LIBDIR)/$(ARLITEDIR)/ti.platform.dspc8681.lite.lib : $(COMMONSRCCOBJS)
	@echo archiving $? into $@ ...
	if [ ! -d $(LIBDIR)/$(ARLITEDIR) ]; then $(MKDIR) $(LIBDIR)/$(ARLITEDIR) ; fi;
	$(ARIN) $@ $?

