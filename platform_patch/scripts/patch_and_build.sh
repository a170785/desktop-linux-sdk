#!/bin/sh
export PDK_INSTALL_PATH_ROOT=$PDK_INSTALL_PATH
cp -r ../csl_patch/packages/ti  $PDK_INSTALL_PATH_ROOT/packages/
cp -r ../dspc8681 $PDK_INSTALL_PATH_ROOT/packages/ti/platform/.
cp -r ../dspc8682 $PDK_INSTALL_PATH_ROOT/packages/ti/platform/.
export PDK_INSTALL_PATH=$PDK_INSTALL_PATH_ROOT/packages

cd $PDK_INSTALL_PATH_ROOT/packages/ti/platform/dspc8681/platform_lib/
make clean
make all

cd $PDK_INSTALL_PATH_ROOT/packages/ti/platform/dspc8682/platform_lib/
make clean
make all

