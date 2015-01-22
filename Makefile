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

export DESKTOP_LINUX_SDK_DIR := $(CURDIR)
export TOOLCHAIN_PREFIX=

#Export cross compiler location in PATH, to be used with MCSDK 2.1 video
#PATH := $(PATH):$(CSTOOL_PATH)

.PHONY: all demos sdk demos_clean sdk_clean sdk_dsp sdk_dsp_clean sdk_reset sdk_reset_clean help clean

all: sdk sdk_demos

# clean: sdk_clean sdk_demos_clean
driver = pciedrv cmem cmem/module

mailbox = mailBox/host

SDK_BUILD = $(driver) $(mailbox) bufmgr dnldmgr sync 

SDK_DSP_BUILD = dsp_projects/dsp_init/build

SDK_RESET_BUILD = dsp_projects/dsp_reset/build

SDK_DEMOS_BUILD = filetestdemo/host dsp_utils

SDK_DEMOS_DSP_BUILD =  filetestdemo/c66x/demo_loopback/build 

help:
	@echo 
	@echo "Available build targets for host build are :"
	@echo "demos		: Build Linux SDK demos"
	@echo "sdk		: Build SDK, including driver"
	@echo "demos_clean	: Clean demos"
	@echo "sdk_clean	: Clean SDK"
	@echo "all		: Rebuild all SDK"
	@echo "clean 		: Clean all SDK"

	@echo "Available build targets for dsp images are :"
	@echo "Note: These need execution of source setup_dsp_build_env.sh"
	@echo "      with the appropriate paths set in the file"
	@echo "sdk_dsp   	: Clean SDK dsp images"
	@echo "sdk_dsp_clean	: Clean SDK"

################################################
export SDK_DIR:=$(DESKTOP_LINUX_SDK_DIR)/sdk

export DEMOS_DIR:=$(DESKTOP_LINUX_SDK_DIR)/demos
################################################

SDK_BUILD_CLEAN = $(patsubst %,%.clean,$(SDK_BUILD))
SDK_DEMOS_BUILD_CLEAN = $(patsubst %,%.clean,$(SDK_DEMOS_BUILD))
SDK_DSP_BUILD_CLEAN = $(patsubst %,%.clean,$(SDK_DSP_BUILD))
SDK_RESET_CLEAN = $(patsubst %,%.clean,$(SDK_RESET_BUILD))
SDK_DEMOS_DSP_BUILD_CLEAN = $(patsubst %,%.clean,$(SDK_DEMOS_DSP_BUILD))

clean: $(SDK_BUILD_CLEAN) $(SDK_DEMOS_BUILD_CLEAN)
sdk : $(SDK_BUILD)
sdk_demos : $(SDK_DEMOS_BUILD)
sdk_clean: $(SDK_BUILD_CLEAN)
demos_clean: $(SDK_DEMOS_BUILD_CLEAN)

sdk_dsp	 : $(SDK_DSP_BUILD) $(SDK_DEMOS_DSP_BUILD)
sdk_dsp_clean	 : $(SDK_DSP_BUILD_CLEAN) $(SDK_DEMOS_DSP_BUILD_CLEAN)

sdk_reset	 : $(SDK_RESET_BUILD) 
sdk_reset_clean	 : $(SDK_RESET_CLEAN)

$(SDK_BUILD): 
	make -C $(SDK_DIR)/$@

$(SDK_DEMOS_BUILD): 
	make -C $(DEMOS_DIR)/$@

$(SDK_DSP_BUILD):
	make -C $(SDK_DIR)/$@

$(SDK_RESET_BUILD):
	make -C $(SDK_DIR)/$@

$(SDK_DEMOS_DSP_BUILD): 
	make -C $(DEMOS_DIR)/$@

$(SDK_BUILD_CLEAN):
	make -C  $(SDK_DIR)/$(@:.clean=) clean

$(SDK_RESET_CLEAN):
	make -C  $(SDK_DIR)/$(@:.clean=) clean

$(SDK_DEMOS_BUILD_CLEAN):
	make -C  $(DEMOS_DIR)/$(@:.clean=) clean

$(SDK_DSP_BUILD_CLEAN):
	make -C  $(SDK_DIR)/$(@:.clean=) clean

$(SDK_DEMOS_DSP_BUILD_CLEAN):
	make -C  $(DEMOS_DIR)/$(@:.clean=) clean

%.clean: %
	make -C $< clean

