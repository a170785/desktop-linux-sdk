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

OBJDIR = Release
OBJEXT = obj
AOBJEXT = aobj
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
export INCDIR := ../../../inc;$(C6X_GEN_INSTALL_PATH)/include;$(PDK_INSTALL_PATH)/packages;$(PDK_INSTALL_PATH)/packages/ti/csl/.;$(PDK_INSTALL_PATH)/packages/ti/platform/.;$(DESKTOP_LINUX_SDK_DIR)/sdk/mailBox;$(DESKTOP_LINUX_SDK_DIR)/sdk/mailBox/inc;
export LIBINCDIR := $(C6X_GEN_INSTALL_PATH)/lib;$(C6X_GEN_INSTALL_PATH)/include;$(PDK_INSTALL_PATH)/packages/ti/csl/lib;

CC = $(C6X_GEN_INSTALL_PATH)/bin/cl6x -mv6600 -g  --display_error_number --diag_warning=225 --abi=eabi 
SRC_FLAGS = 
CC_OPT = -O3
AC = $(C6X_GEN_INSTALL_PATH)/bin/cl6x -c -qq -mv6600 --abi=elfabi
LD = $(C6X_GEN_INSTALL_PATH)/bin/cl6x -mv6600 -g $(SRC_FLAGS) --display_error_number --diag_warning=225 --abi=eabi $(CC_OPT) -z --warn_sections --reread_libs --rom_model
RTSLIB = -l$(C6X_GEN_INSTALL_PATH)/lib/rts6600_elf.lib
LIBS = -l"ti.csl.intc.ae66" -l"ti.csl.ae66" 
INCS = -I. -I$(strip $(subst ;, -I,$(subst $(space),\$(space),$(INCDIR))))
LIBINCS = -i$(strip $(subst ;, -i,$(subst $(space),\$(space),$(LIBINCDIR))))


INTERNALDEFS = -DMAKEFILE_BUILD -eo.$(OBJEXT) -ea.$(AOBJEXT) -fr=$(@D) -fs=$(@D) -ppa -ppd=$@.dep
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
SRC_FILES= ../src/main.c  \
	   ../src/demo_loopback.c \
	   $(DESKTOP_LINUX_SDK_DIR)/sdk/mailBox/src/mailBox.c \
	   $(DESKTOP_LINUX_SDK_DIR)/sdk/mailBox/c66x/src/mailBox_c66x.c 

VPATH := $(sort $(dir $(SRC_FILES)))
CMD_FILE=demo_loopback.cmd

# Make Rule for the COMMONSRC Files
PP_FILES = $(patsubst %.c, $(OBJDIR)/%.pp, $(notdir $(SRC_FILES)))
OBJ_FILES = $(patsubst %.c, $(OBJDIR)/%.$(OBJEXT), $(notdir $(SRC_FILES)))
#DEP_FILES = $(patsubst %.$(OBJEXT), %.dep

# PHONY Targets
.PHONY: all demo_loopback clean

# all rule
all: .executables

.executables: $(BINDIR)/demo_loopback.out

demo_loopback:  $(BINDIR)/demo_loopback.out


$(OBJDIR)/%.$(OBJEXT) : %.c
	@mkdir -p $(OBJDIR)/$(dir $@)
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

# Compile build
$(BINDIR)/demo_loopback.out: $(OBJ_FILES) 
	@mkdir -p $(dir $@)
	$(LD) -i$(EVMC6678L_LIB_DIR) $(LIBINCS) $(LIBS) -l$(EVMC6678L_LIB) $(RTSLIB) -o $@ -m $(patsubst %.out, %.map, $@) $(OBJ_FILES) $(CMD_FILE) 
	-@echo 'Finished building $@'

# Rule to clean 
clean  ::
	-$(RM) $(BINDIR)/demo_loopback.out
	-$(RM) $(BINDIR)/demo_loopback.map
	-$(RM) $(OBJ_FILES)
	-$(RM) $(PP_FILES)
	-@echo 'Finished clean'
	-@echo ' '

