#*******************************************************************************
#* FILE PURPOSE: Lower level makefile for Creating Platform Libraries
#*******************************************************************************
#* FILE NAME: ./lib/ti.platform.dspc8682.ae66e.mk
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
CC = $(C6X_GEN_INSTALL_PATH)/bin/cl6x -c -mo -o3 -q -k -eo.o -DBIGENDIAN -me -mv6600 --abi=elfabi
AC = $(C6X_GEN_INSTALL_PATH)/bin/cl6x -c -qq -me -mv6600 --abi=elfabi
ARIN = $(C6X_GEN_INSTALL_PATH)/bin/ar6x rq
LD = $(C6X_GEN_INSTALL_PATH)/bin/lnk6x --abi=elfabi
RTSLIB = -l $(C6X_GEN_INSTALL_PATH)/lib/rts6600e_elf.lib
INCS = -I. -I$(strip $(subst ;, -I,$(subst $(space),\$(space),$(INCDIR))))
OBJEXT = oe66e
AOBJEXT = se66e
INTERNALDEFS = -Dti_targets_elf_C66_big_endian  -DMAKEFILE_BUILD -eo.$(OBJEXT) -ea.$(AOBJEXT) -fr=$(@D) -fs=$(@D) -ppa -ppd=$@.dep
INTERNALLINKDEFS = -o $@ -m $@.map
OBJDIR =  $(LIBDIR)

#List the COMMONSRC Files
COMMONSRCC= \
    src/evm66x_i2c.c \
    src/evm66x_nand.c \
    src/dspc8682.c \
    src/dspc8682_phy.c \
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
	-@echo cle66e $< ...
	if [ ! -d $(@D) ]; then $(MKDIR) $(@D) ; fi;
	$(RM) $@.dep
	$(CC) $(COMMONSRCCFLAGS) $(INTERNALDEFS) $(INCS) -fc $< 
	-@$(CP) $@.dep $@.pp; \
         $(SED) -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
             -e '/^$$/ d' -e 's/$$/ :/' < $@.pp >> $@.dep; \
         $(RM) $@.pp 

#Create Empty rule for dependency
$(COMMONSRCCOBJS):.\lib\ti.platform.dspc8682.ae66e.mk
.\lib\ti.platform.dspc8682.ae66e.mk:

#Include Depedency for COMMONSRC Files
ifneq (clean,$(MAKECMDGOALS))
 -include $(COMMONSRCCOBJS:%.$(OBJEXT)=%.$(OBJEXT).dep)
endif


$(LIBDIR)/$(ARRELDIR)/ti.platform.dspc8682.ae66e : $(COMMONSRCCOBJS)
	@echo archiving $? into $@ ...
	if [ ! -d $(LIBDIR)/$(ARRELDIR) ]; then $(MKDIR) $(LIBDIR)/$(ARRELDIR) ; fi;
	$(ARIN) $@ $?

