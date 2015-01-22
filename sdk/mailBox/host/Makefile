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


# Compiler name (with path, if not in path)
CC=$(TOOLCHAIN_PREFIX)gcc -ggdb
AR=$(TOOLCHAIN_PREFIX)ar rcs

OUT=lib/libmailbox.a

# Mainly used for include paths
INCLUDES=-I .. -I ../inc -I ../.. -I ../../pciedrv  -I ../../config -D MAILBOX_ON_HOST

SOURCES_COMMON=../src/mailBox.c

SOURCES=src/mailBox_host.c

OBJECTS_COMMON=$(patsubst ../src/%.c, ../obj/%.o, $(SOURCES_COMMON))
OBJECTS=$(patsubst src/%.c, obj/%.o, $(SOURCES))

all: $(SOURCES) ${SOURCES_COMMON} $(OBJECTS) $(OBJECTS_COMMON) $(OUT)

obj/%.o : src/%.c
	@mkdir -p $(dir $@)
	$(CC) $(INCLUDES) -c $< -o $@

../obj/%.o : ../src/%.c
	@mkdir -p $(dir $@)
	$(CC) $(INCLUDES) -c $< -o $@

$(OUT): $(OBJECTS) $(OBJECTS_COMMON)
	@mkdir -p $(dir $@)
	$(AR) $(OUT) $(OBJECTS) $(OBJECTS_COMMON)

clean:
	@rm -fr $(OBJECTS_COMMON)
	@rm -fr $(OBJECTS)
	@rm -fr $(OUT)
