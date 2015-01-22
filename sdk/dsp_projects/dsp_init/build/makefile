#*
#*
#* Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/ 
#* 
#* 
#*  Redistribution and use in source and binary forms, with or without 
#*  modification, are permitted provided that the following conditions 
#*  are met:
#*
#*    Redistributions of source code must retain the above copyright 
#*    notice, this list of conditions and the following disclaimer.
#*
#*    Redistributions in binary form must reproduce the above copyright
#*    notice, this list of conditions and the following disclaimer in the 
#*    documentation and/or other materials provided with the   
#*    distribution.
#*
#*    Neither the name of Texas Instruments Incorporated nor the names of
#*    its contributors may be used to endorse or promote products derived
#*    from this software without specific prior written permission.
#*
#* 
#* 
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
#*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
#*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
#*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
#*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
#*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#*

OBJDIR = Debug
OBJEXT = obj
BINDIR = bin
EXEEXT = out

# PLatform libary Definitions
export DSPC681_LIB_DIR :=$(PDK_INSTALL_PATH)/packages/ti/platform/dspc8681/platform_lib/lib/debug
export DSPC682_LIB_DIR :=$(PDK_INSTALL_PATH)/packages/ti/platform/dspc8682/platform_lib/lib/debug
export EVMC6678L_LIB_DIR :=$(PDK_INSTALL_PATH)/packages/ti/platform/evmc6678l/platform_lib/lib/debug

export DSPC8681_LIB :="ti.platform.dspc8681.lite.lib" 
export DSPC8682_LIB :="ti.platform.dspc8682.lite.lib" 
export EVMC6678L_LIB :="ti.platform.evm6678l.lite.lib" 

# INCLUDE Directory
export INCDIR := ../../../..;$(C6X_GEN_INSTALL_PATH)/include;$(PDK_INSTALL_PATH)/packages;$(PDK_INSTALL_PATH)/packages/ti/csl/.;$(PDK_INSTALL_PATH)/packages/ti/platform/.;
export LIBINCDIR := $(C6X_GEN_INSTALL_PATH)/lib;$(C6X_GEN_INSTALL_PATH)/include;$(PDK_INSTALL_PATH)/packages/ti/csl/lib;

CC = $(C6X_GEN_INSTALL_PATH)/bin/cl6x -mv6600 -g  --define=_TMS320C6X --define=COMBINED_WITH_BOOT_LOAD -eo.o --abi=elfabi --diag_warning=225
CC_OPT = -mo -o3 -q -k 
AC = $(C6X_GEN_INSTALL_PATH)/bin/cl6x -c -qq -mv6600 --abi=elfabi
LD = $(C6X_GEN_INSTALL_PATH)/bin/cl6x -mv6600 -g --define=_TMS320C6X --define=COMBINED_WITH_BOOT_LOAD --diag_warning=225 --abi=eabi -z --warn_sections --reread_libs --rom_model
RTSLIB = -l$(C6X_GEN_INSTALL_PATH)/lib/rts6600_elf.lib
LIBS = -l"ti.csl.intc.ae66" -l"ti.csl.ae66" 
INCS = -I. -I$(strip $(subst ;, -I,$(subst $(space),\$(space),$(INCDIR))))
LIBINCS = -i$(strip $(subst ;, -i,$(subst $(space),\$(space),$(LIBINCDIR))))


INTERNALDEFS = -Dti_targets_elf_C66  -DMAKEFILE_BUILD -eo.$(OBJEXT) -ea.$(AOBJEXT) -fr=$(@D) -fs=$(@D) -ppa -ppd=$@.dep
INTERNALLINKDEFS = -o $@ -m $@.map

# Common Macros used in make

ifndef RM
export RM = rm -f
endif

ifndef CP
export CP = cp -p
endif

export MKDIR = mkdir -p

ifndef RMDIR
export RMDIR = rm -rf
endif

ifndef SED
export SED = sed
endif

ifndef MAKE
export MAKE = make
endif
SRC_FILES= ../src/main.c

CMD_FILE=init.cmd

# FLAGS for the COMMONSRC Files
SRC_FLAGS =   -i./src -i. -i./test 

# Make Rule for the COMMONSRC Files
OBJ_FILES = $(patsubst %.c, $(OBJDIR)/%.$(OBJEXT), $(SRC_FILES))


# PHONY Targets
.PHONY: all dspc8681_init dspc8682_init evm6678_init clean cleanall 

# all rule
all: .executables
.executables: $(BINDIR)/dspc8681/init.out $(BINDIR)/dspc8682/init.out $(BINDIR)/evmc6678l/init.out

dspc8681_init:  $(BINDIR)/dspc8681/init.out

dspc8682_init:  $(BINDIR)/dspc8682/init.out

evm6678_init:  $(BINDIR)/evmc6678l/init.out

$(OBJDIR)/%.$(OBJEXT) : ../src/%.c
	@mkdir -p $(dir $@)
	$(CC) $(INCLUDES) -c $< -o $@

$(OBJ_FILES): $(OBJDIR)/%.$(OBJEXT): %.c
	-@echo cle66 $< ...
	if [ ! -d $(@D) ]; then $(MKDIR) $(@D) ; fi;
	$(RM) $@.dep
	$(CC) $(SRC_FLAGS) $(INTERNALDEFS) $(INCS) -fc $< 
	-@$(CP) $@.dep $@.pp; \
         $(SED) -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
             -e '/^$$/ d' -e 's/$$/ :/' < $@.pp >> $@.dep; \
         $(RM) $@.pp 

# Clean Rule
clean:: clean_package
# Clean Top Level Object Directory 
clean_package :
#	rm -rf objects 
#       rm -rf executables


# Compile build
$(BINDIR)/dspc8681/init.out: $(OBJ_FILES) 
	@mkdir -p $(dir $@)
	$(LD) -i$(DSPC681_LIB_DIR) $(LIBINCS) $(LIBS) -l$(DSPC8681_LIB) $(RTSLIB) -o $@ -m $(patsubst %.out, %.map, $@) $< $(CMD_FILE) 
	-@echo 'Finished building $@'

$(BINDIR)/dspc8682/init.out: $(OBJ_FILES) 
	@mkdir -p $(dir $@)
	$(LD) -i$(DSPC682_LIB_DIR) $(LIBINCS) $(LIBS) -l$(DSPC8682_LIB) $(RTSLIB) -o $@ -m $(patsubst %.out, %.map, $@) $< $(CMD_FILE) 
	-@echo 'Finished building $@'

$(BINDIR)/evmc6678l/init.out: $(OBJ_FILES) 
	@mkdir -p $(dir $@)
	$(LD) -i$(EVMC6678L_LIB_DIR) $(LIBINCS) $(LIBS) -l$(EVMC6678L_LIB) $(RTSLIB) -o $@ -m $(patsubst %.out, %.map, $@) $< $(CMD_FILE) 
	-@echo 'Finished building $@'

# Rule to clean 
clean  ::
	-$(RM) $(BINDIR)/dspc8681/init.out
	-$(RM) $(BINDIR)/dspc8681/init.map
	-$(RM) $(BINDIR)/dspc8682/init.out
	-$(RM) $(BINDIR)/dspc8682/init.map
	-$(RM) $(BINDIR)/evmc6678l/init.out
	-$(RM) $(BINDIR)/evmc6678l/init.map
	-$(RM) $(OBJ_FILES)
	-$(RM) $(OBJ_FILES).dep
	-@echo 'Finished clean'
	-@echo ' '

