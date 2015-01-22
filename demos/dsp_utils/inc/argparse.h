/*
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/




/* Parsing arguments */
#pragma once

typedef struct _arg_template_t
{
	char keyword[32];
	int minimal_argc;
	int (*handler) (int _argc, char** argv);
} arg_template_t;

extern int parse_cmd_line(int argc, char** argv,
						  const arg_template_t* cmd, unsigned num_cmds, int* do_ret);

/**
 * Convert command line parameter to number.
 * @param argc Parameter from main().
 * @param argv Parameter from main().
 * @param start_item First argument in argv[] array.
 * @param num_to_parse Number of arguments to be parsed (after argv[start]).
 * @param values The array holds the arguments.
 **/
extern int get_param_value_array(int argc, char** argv, unsigned start_item, unsigned num_to_parse, unsigned* values);

/** Convert a numeric string to unsigned integer.
 * @param str [in] The numeric string to be converted.
 * @param value [out] The converted value.
 * @return 0 on success. Non-zero means illegal numeric string.
 */
extern int str2ul(const char * str, unsigned int* value);
