#*******************************************************************************
#* FILE PURPOSE: Top level makefile for Creating Platform Libraries
#*******************************************************************************
#* FILE NAME: makefile
#*
#* DESCRIPTION: Defines Compiler tools paths, libraries , Build Options 
#*
#*
#*******************************************************************************
#*
# (Mandatory) Specify where various tools are installed.

# Output for prebuilt generated libraries
export LIBDIR ?= ./lib

# Library directory macros
export ARRELDIR = release
export ARLITEDIR = debug
export ARDBGDIR = debug

# ROOT Directory
export ROOTDIR := ../../../..

# INCLUDE Directory
export INCDIR := ../../../..;$(C6X_GEN_INSTALL_PATH)/include;$(PDK_INSTALL_PATH)/ti/platform/dspc8682/platform_lib/include/;$(PDK_INSTALL_PATH)/.;$(PDK_INSTALL_PATH)/ti/csl/.;$(PDK_INSTALL_PATH)/ti/platform/.;$(ROOTDIR)

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

# PHONY Targets
.PHONY: all clean cleanall 

# FORCE Targets
FORCE: 

# all rule
all: .executables
.executables: .libraries
.libraries:

# Clean Rule
clean:: clean_package
# Clean Top Level Object Directory 
clean_package :
	$(RMDIR) $(LIBDIR)/*/
	$(RMDIR) package/cfg


# Make rule to create $(LIBDIR)/$(ARRELDIR)/ti.platform.dspc8682.ae66 library
.libraries: $(LIBDIR)/$(ARRELDIR)/ti.platform.dspc8682.ae66
$(LIBDIR)/$(ARRELDIR)/ti.platform.dspc8682.ae66: FORCE
	$(MAKE) -f ./lib/ti.platform.dspc8682.ae66.mk $@
# Rule to clean $(LIBDIR)/$(ARRELDIR)/ti.platform.dspc8682.ae66 library
clean ::
	$(RM) $(LIBDIR)/$(ARRELDIR)/ti.platform.dspc8682.ae66


# Make rule to create $(LIBDIR)/$(ARRELDIR)/ti.platform.dspc8682.ae66e library
.libraries: $(LIBDIR)/$(ARRELDIR)/ti.platform.dspc8682.ae66e
$(LIBDIR)/$(ARRELDIR)/ti.platform.dspc8682.ae66e: FORCE
	$(MAKE) -f ./lib/ti.platform.dspc8682.ae66e.mk $@
# Rule to clean $(LIBDIR)/$(ARRELDIR)/ti.platform.dspc8682.ae66e library
clean ::
	$(RM) $(LIBDIR)/$(ARRELDIR)/ti.platform.dspc8682.ae66e

# Make rule to create $(LIBDIR)/$(ARLITEDIR)/ti.platform.dspc8682.lite.lib library
.libraries: $(LIBDIR)/$(ARLITEDIR)/ti.platform.dspc8682.lite.lib
$(LIBDIR)/$(ARLITEDIR)/ti.platform.dspc8682.lite.lib: FORCE
	$(MAKE) -f ./lib/ti.platform.dspc8682.lite.lib.mk $@
# Rule to clean $(LIBDIR)/$(ARLITEDIR)/ti.platform.dspc8682.lite.lib library
clean ::
	$(RM) $(LIBDIR)/$(ARLITEDIR)/ti.platform.dspc8682.lite.lib


# Make rule to create $(LIBDIR)/$(ARLITEDIR)/ti.platform.dspc8682.lite.libe library
.libraries: $(LIBDIR)/$(ARLITEDIR)/ti.platform.dspc8682.lite.libe
$(LIBDIR)/$(ARLITEDIR)/ti.platform.dspc8682.lite.libe: FORCE
	$(MAKE) -f ./lib/ti.platform.dspc8682.lite.libe.mk $@
# Rule to clean $(LIBDIR)/ti.platform.dspc8682.lite.libe library
clean ::
	$(RM) $(LIBDIR)/ti.platform.dspc8682.lite.libe	

# Make rule to create $(LIBDIR)/$(ARDBGDIR)/ti.platform.dspc8682.ae66 library
.libraries: $(LIBDIR)/$(ARDBGDIR)/ti.platform.dspc8682.ae66
$(LIBDIR)/$(ARDBGDIR)/ti.platform.dspc8682.ae66: FORCE
	$(MAKE) -f ./lib/ti.platform.dspc8682.dbg.ae66.mk $@
# Rule to clean $(LIBDIR)/$(ARDBGDIR)/ti.platform.dspc8682.ae66 library
clean ::
	$(RM) $(LIBDIR)/$(ARDBGDIR)/ti.platform.dspc8682.ae66


# Make rule to create $(LIBDIR)/$(ARDBGDIR)/ti.platform.dspc8682.ae66e library
.libraries: $(LIBDIR)/$(ARDBGDIR)/ti.platform.dspc8682.ae66e
$(LIBDIR)/$(ARDBGDIR)/ti.platform.dspc8682.ae66e: FORCE
	$(MAKE) -f ./lib/ti.platform.dspc8682.dbg.ae66e.mk $@
# Rule to clean $(LIBDIR)/$(ARDBGDIR)/ti.platform.dspc8682.ae66e library
clean ::
	$(RM) $(LIBDIR)/$(ARDBGDIR)/ti.platform.dspc8682.ae66e		

