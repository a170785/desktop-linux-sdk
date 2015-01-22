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
/* This short program is used to convert a boot table to a
   header file so DSP code can be easily included by the host software. The host 
   will boot DSP ultimately through HPI. SRIO, EMAC, etc.
   E.g.: 

      hex6x DspCode.rmd ==> DspCode.btbl
      Bttbl2Hfile DspCode.btbl DspCode.h

   Meanwhile, it can also generate a binary image if desired.
      Bttbl2Hfile DspCode.btbl DspCode.h DspCode.bin
 */
#include <stdio.h>
#include <string.h>

int asciiByte (unsigned char c)
{
  if ((c >= '0') && (c <= '9'))
    return (1);

  if ((c >= 'A') && (c <= 'F'))
    return (1);

  if ((c >= 'a') && (c <= 'f'))
    return (1);

  return (0);
}


int toNum (unsigned char c)
{
  if ((c >= '0') && (c <= '9'))
    return (c - '0');

  else if ((c >= 'A') && (c <= 'F'))
    return (c - 'A'+10);

 else  return (c - 'a' + 10);
}


int main (int argc, char *argv[])
{
  FILE *strin;
  FILE *strout;
  FILE *strbin;
 
  int i=0;
  char linein[132];
  char lineout[200];
  char *pin, *pout;
  unsigned int value;

  /* Verify the number of args */
  if ((argc != 3)  && (argc != 4))  {
    printf ("usage: %s inputfile output_header_file\n", argv[0]);
	printf ("Or: \n");
    printf ("usage: %s inputfile output_header_file output_binary_file\n", argv[0]);
    return (-1);
  }

  /* Open all the files */
  strin = fopen (argv[1], "rb");
  if (strin == NULL)  {
    printf ("could not open input file %s\n", argv[1]);
    return (-1);
  }

  strout = fopen (argv[2], "wb+");
  if (strout == NULL)  {
    printf ("could not open file %s to write\n", argv[2]);
    return (-1);
  }

  if(argc == 4){
	strbin = fopen (argv[3], "wb+");
	if (strbin == NULL)  {
		printf ("could not open file %s to write\n", argv[3]);
		return (-1);
	}
  }

  /* Ignore the first two lines */
  fgets (linein, 132, strin);
  fgets (linein, 132, strin);
  
  while(feof(strin)==0) {
	  fgets (linein, 132, strin);
	  if(!asciiByte(linein[0])) break;  /* if not ASCII, end */

	  pin = linein;
	  pout = lineout;
	  strcpy(lineout," ");
	
	  while(1) {
		  *pout++ ='0';
		  *pout++ ='x';

		  if(argc == 4) {
			  value = (toNum(*pin))<<4 | toNum(*(pin+1));
			  fputc(value,strbin);
		  }

		  *pout++ = *pin++;
		  *pout++ = *pin++;
		  if(*pin==' ') pin++;
		  *pout++ = ',';
		  *pout++ = ' ';
		  if(*pin == 0x0D) break;
		  if(*pin == 0x0A) break;
		  if(*pin == 0x00) break;
		  if(*pin == EOF) break;
	  }

	  /* next line */
  	  *pout++=0x0d;
	  *pout++=0x0a;
	  *pout++=0x00;

	  /* save it */
	  fprintf(strout,"%s",lineout);
  }
  
  fclose(strin);
  fclose(strout);

  if(argc == 4) { 
	  fclose(strbin);
  }

  return (0);
}

