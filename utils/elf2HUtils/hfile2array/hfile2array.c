/******************************************************************************
 * Copyright (c) 2011 Texas Instruments Incorporated - http://www.ti.com
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
 *****************************************************************************/

/***************************************************************************************
 * FILE NAME: hex2h.c
 *
 * DESCRIPTION: A simple utility to convert an ASCII hex file to a header file .
 *
 ***************************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

int main(int argc, char **argv)
{
    FILE *out, *in, *in_hdr;
    char line[200] = "uint8_t ";
    char line_hdr[200];

    if (argc != 4)
    {
        printf ("Usage: hfile2array.exe <in_file> <out_file> <array_name>\n");
        return (-1);
    }
    
    argv++;

    in = fopen((const char *)(*argv++), "rt");
    
    if (in == NULL)
    {
        argv--;
        printf ("Can not open %s\n", *argv);
        return (-1);
    }
 
    out = fopen((const char *)(*argv++), "wt");
    if (out == NULL)
    {
        argv--;
        printf ("Can not open %s\n", *argv);
        return (-1);
    }

    in_hdr = fopen("header.txt", "rt");
    
    if (in_hdr == NULL)
    {
        printf ("Can not open header.tx\n");
        return (-1);
    }

    while (1)
    {
        if (fgets(line_hdr, 200, in_hdr) != NULL)
        {
            if (fputs(line_hdr, out) < 0)
            {
                printf ("fputs %s error\n", line_hdr);
                goto error;
            }
        }
        else
        {
            /* EOF reached */
            break;
        }
    }

    strcat (line, (const char *)(*argv++));
    strcat (line, "[] = {\n");
    
    if (fputs(line, out) < 0)
    {
        printf ("fputs %s error\n", line);
        goto error;
    }

    while (1)
    {
        if (fgets(line, 200, in) != NULL)
        {
            if (fputs(line, out) < 0)
            {
                printf ("fputs %s error\n", line);
                goto error;
            }
        }
        else
        {
            /* EOF reached */
            break;
        }
    }

    if (fputs("};\n", out) < 0)
    {
        printf ("fputs };\\n error\n");
        goto error;
    }

    fclose(in);
    fclose(in_hdr);
    fclose(out);

    return 0;

error:
    fclose(in);
    fclose(in_hdr);
    fclose(out);
    
    return  (-1);
}
