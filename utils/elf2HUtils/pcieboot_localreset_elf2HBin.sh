export TARGET=6678
export ENDIAN=little

echo CGT_INSTALL_DIR set as: ${CGT_INSTALL_DIR}
echo TARGET set as: ${TARGET}

echo Converting .out to HEX ...
if [ ${ENDIAN} == little ]
then
${C6X_GEN_INSTALL_PATH}/bin/hex6x -order L pcieboot_localreset.rmd $DESKTOP_LINUX_SDK_DIR/sdk/dsp_projects/dsp_reset/build/bin/pcieboot_localreset.out
else
${C6X_GEN_INSTALL_PATH}/bin/hex6x -order M pcieboot_localreset.rmd  $DESKTOP_LINUX_SDK_DIR/sdk/dsp_projects/dsp_reset/build/bin/pcieboot_localreset.out
fi
if [ ! -e bttbl2hfile/Bttbl2Hfile ]
then
cd bttbl2hfile 
make
cd ..
fi
./bttbl2hfile/Bttbl2Hfile pcieboot_localreset.btbl pcieboot_localreset.h pcieboot_localreset.bin
if [ ! -e hfile2array/hfile2array ]
then
cd hfile2array
make
cd ..
fi
./hfile2array/hfile2array pcieboot_localreset.h pcieLocalReset.h localResetCode

cp pcieLocalReset.h $DESKTOP_LINUX_SDK_DIR/sdk/dnldmgr/inc/.


