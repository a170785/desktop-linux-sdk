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



#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "argparse.h"

#ifndef stricmp
int stricmp(const char* s1, const char* s2)
{
    int c1, c2;
    for (;; s1++, s2++)
    {
        c1 = toupper ( *s1 );
        c2 = toupper ( *s2 );
        if (c1 != c2)
            return c1 - c2;
        if (c1 == 0)
            break;
    }
    return 0;
}
#endif

int parse_cmd_line(int argc, char** argv, const arg_template_t* cmd,
    unsigned num_cmds, int* do_ret)
{
    unsigned i;

    if (argc < 2)
        return -1; // supplied parameters is too few.
    for (i = 0; i < num_cmds; i++)
    {
        if (stricmp(cmd[i].keyword, argv[1]) == 0)
        {
            if (argc < cmd[i].minimal_argc)
            {
                break;
            }
            else
            {
                *do_ret = cmd[i].handler(argc, argv);
                return 0;
            }
        }
    }
    return -1;
}

/** Convert a numeric string to unsigned integer.
* @param str [in] The numeric string to be converted.
* @param value [out] The converted value.
* @return 0 on success. Non-zero means illegal numeric string.
*/
int str2ul(const char * str, unsigned int* value)
{
    unsigned int sum = 0;
    int i;
    char *ptr = (char *) str;
    int state = 0;
    char tmp;

    for (i = 0;; i++)
    {

        tmp = *ptr;

        switch (state)
        {

        case 0:
            if (tmp == '0')
            {
                state = 1;
            }
            else if (tmp >= '1' && tmp <= '9')
            {
                sum = tmp - '0';
                state = 4;
            }
            else
            {
                state = 6;
            }

            break;

        case 1:
            if (tmp == 'x')
            {
                state = 2;
            }
            else if (tmp == 'b')
            {
                state = 3;
            }
            else if (tmp >= '0' && tmp <= '9')
            {
                state = 4;
            }
            else if (tmp == '\0')
            {
                state = 5;
            }
            else
            {
                state = 6;
            }

            break;

        case 2:
            if (tmp >= '0' && tmp <= '9')
            {
                sum = (sum * 16) + (tmp - '0');
            }
            else if (tmp >= 'A' && tmp <= 'F')
            {
                sum = (sum * 16) + (tmp - 'A' + 10);
            }
            else if (tmp >= 'a' && tmp <= 'f')
            {
                sum = (sum * 16) + (tmp - 'a' + 10);
            }
            else if (tmp == '\0')
            {
                state = 5;
            }
            else
            {
                state = 6;
            }
            if (i > 10)
                state = 6;
            break;

        case 3:
            if (tmp == '0' || tmp == '1')
            {
                sum = (sum * 2) + (tmp - '0');
            }
            else if (tmp == '\0')
            {
                state = 5;
            }
            else
            {
                state = 6;
            }
            if (i > 34)
                state = 6;

            break;

        case 4:
            if (tmp >= '0' && tmp <= '9')
            {
                sum = (sum * 10) + (tmp - '0');
            }
            else if (tmp == '\0')
            {
                state = 5;
            }
            else
            {
                state = 6;
            }
            if (i > 9)
                state = 6;
            break;

        case 5:
            *value = sum;
            return 0;
            break;

        case 6:
            return -1;
            break;

        }

        ptr++;
    }
}

int get_param_value_array(int argc, char** argv, unsigned start_item,
    unsigned num_to_parse, unsigned* values)
{
    unsigned i;
    int ret;

    if ((start_item + num_to_parse - 1) > argc)
        return -1;
    for (i = 0; i < num_to_parse; start_item++, i++)
    {
        ret = str2ul(argv[start_item], &values[i]);

        if (ret)
        {
            return -1;
        }
    }

    return 0;
}

