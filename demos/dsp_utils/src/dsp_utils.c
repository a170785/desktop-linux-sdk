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

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "argparse.h"
#include "bspversion.h"
//#include "ihexparser.h"

#include "stdint.h"
#include "pciedrv.h"
#include "cmem_drv.h"
#include "dnldmgr.h"
#define TOTAL_NUM_CORES_PER_CHIP    8

#define LINE_BUF_SZ (1024)
static int get_line_from_file( FILE* fp, char* buf, unsigned maxlen )
{
  int ch;
  unsigned idx;
  int state = 0;

  buf[0] = 0;
  for ( idx = 0; idx < maxlen - 1; )
  {
    ch = fgetc ( fp );
    if ( ch < 0 ) break;
    if ( ch == 0x0d )
    {
      state = 1;
      continue;
    }
    else if ( ch == 0x0a )
    {
      state = 1;
      break;
    }
    else
    {
      if ( state )
      {
        ungetc ( ch, fp );
        break;
      }
      buf[idx] = ch;
      idx++;
      buf[idx] = 0;
    }
  }
  if ( (ch < 0) && (idx == 0) ) return -1; // indicate EOF
    return 0;
} 

#define MAX_BOOT_CFG_BYTES_PER_LINE  8
static int parse_boot_config_file( const char* filename, boot_cfg_t *bootcfg )
{
  char* line_buffer;
  unsigned int state;
  uint32_t val[MAX_BOOT_CFG_BYTES_PER_LINE];
  int ret_value, num_values=0;
  uint32_t *boot_config_p;
  FILE* fp;
  int i;

  /* Open boot config file */
  fp = fopen ( filename, "rb" );
  if ( !fp ) return -1;

  line_buffer = (char*) malloc ( LINE_BUF_SZ );
  if(line_buffer == NULL)
   return (-1);

  state= 0;

  while ( !feof ( fp ) )
  {
    ret_value = get_line_from_file ( fp, line_buffer, LINE_BUF_SZ );
    if ( ret_value < 0 )
    {
      ret_value = 0;
      break;
    }
    if (line_buffer[0] == '#')
    {
      continue;
    }
    switch(state)
    {
      case 0: 
        /* Read DSP Boot config address */
        sscanf(line_buffer,"%x",&bootcfg->config_dsp_addr);
        printf("\n DSP boot config addr 0x%x", bootcfg->config_dsp_addr); fflush(stdout);

        state= 1;
        break;
 
      case 1:
        /* Read DSP Boot config size */
        sscanf(line_buffer,"%x",&bootcfg->config_size_b);
        printf("\n DSP boot config size in bytes %d", bootcfg->config_size_b); fflush(stdout);

        boot_config_p = bootcfg->config_p;
        state= 2;
        break;

      case 2:
        /* Read DSP Boot config size */
        ret_value = sscanf(line_buffer,"%x %x %x %x %x %x %x %x",&val[0],&val[1],&val[2],&val[3],&val[4],&val[5],&val[6],&val[7] );
        printf("\nBoot config words:");
        for(i=0; i< ret_value; i++)
        {
          *boot_config_p = val[i];       
          printf(" 0x%x,",*boot_config_p);
          boot_config_p++;
          num_values++;
        }
        break;
          
    }
  }
  if(state != 2)
  {
    printf("\n"); 
    free(line_buffer);
    fclose ( fp );
    return(-1);
  }
#if 1 /* Debug */
  if(bootcfg->config_size_b != num_values*sizeof(uint32_t))
  {
    printf("\n Error: Num values (%d) Not matching config size(%d)", num_values, bootcfg->config_size_b);
  }   
   
#endif
  free(line_buffer);
  fclose ( fp );
  return(0);
}

char tst_pci_dsp_buf[1024*1024];

static int do_dsp_load(int argc, char** argv)
{
#if 1
  unsigned values[3];
  int ret = 0;
  boot_cfg_t bootcfg;
  boot_cfg_t *boot_cfg_p = NULL;
  uint32_t boot_cfg_words[256];
  void *dsp_image_handle;
  uint32_t dsp_entry_point;

  enum
  {
      chip = 0, core = 1, entry = 2
  };

  /* Get parameters */
  ret = get_param_value_array(argc, argv, 2, 3, values);
  if (ret)
  {
    printf("ERROR: failed to get_param_value_array\n");
    return -1;
  }

  /* Open pcie driver */
  ret =  pciedrv_open(NULL);
  if (0 != ret)
  {
    printf("ERROR: pciedrv could not opened\n");
    return -1;
  }
  /* Check if boot config is needed */
  if(strcmp(argv[6],"0")!=0)
  {
     printf("\nboot config file %s", argv[6]);fflush(stdout);
     bootcfg.config_p = boot_cfg_words;
     boot_cfg_p = &bootcfg;
     ret = parse_boot_config_file( argv[6], boot_cfg_p );
     if(ret != 0)
     {
        printf("ERROR: Boot config failed \n");
        return(-1);
     }
  }

  /* Get DSP Image */
  if(dnldmgr_get_image(argv[5], &dsp_image_handle, &dsp_entry_point) != 0)
  {
    printf("\n ERROR: Get DSP image failed ");
    return(-1);
  }

  /* Check entry point */
  if(values[entry] !=0 )
  {
    dsp_entry_point = values[entry];
    printf("\n Overriding image entry point with input %x", dsp_entry_point);
  }
  else if(dsp_entry_point == 0xffffffff)
  {
    printf("ERROR: Entry point not found in file, please provide..");
    return(-1);
  }

  /* Load image */
  ret = dnldmgr_load_image(values[chip], values[core], dsp_image_handle, dsp_entry_point, boot_cfg_p);
  if(0 != ret)
  {
      printf("ERROR: Download image failed \n");
      return -1;
  }
  /* free DSP Image handle */
  dnldmgr_free_image(dsp_image_handle);

  /* Close pcie driver */
  ret = pciedrv_close();

  if (0 != ret)
  {
      printf("ERROR: pciedrv could not closed\n");
      return -1;
  }
  printf(" Download image success ! \n");
  return ret;
#else
  int ret = 0;
  ret =  pciedrv_open(NULL);
  if (0 != ret)
  {
    printf("ERROR: pciedrv could not opened\n");
    return -1;
  }
  
  memset(tst_pci_dsp_buf, 0xaa, sizeof(tst_pci_dsp_buf));
  ret = pciedrv_dsp_write(0, 0x81000000, tst_pci_dsp_buf, sizeof(tst_pci_dsp_buf));
  if(ret != 0)
  {
	fprintf(stderr, "\nERROR:pciedrv_dsp_write: ret =%d", ret);
	return -1;
  }	
  
#endif
 }







#define BOOT_ENTRY_LOCATION_ADDR 0x87FFFC
static int do_dsp_local_reset(int argc, char** argv)
{
  unsigned values[4];
  int ret = 0;
  uint32_t execution_wait_count=0;
  uint32_t read_boot_entry_location_value[TOTAL_NUM_CORES_PER_CHIP];
  int i, check_flag =0;

  enum
  {
    chip = 0
  };

  /* Get parameter value */
  ret = get_param_value_array(argc, argv, 2, 1, values);
  if (ret)
  {
    printf("ERROR: failed to get_param_value_array\n");
    return -1;
  }
 
  /* Open pcie driver */
  ret =  pciedrv_open(NULL);
  if (0 != ret)
  {
    printf("ERROR: pciedrv could not opened\n");
    return -1;
  }
  /* Put the DSP in reset */
  ret = dnldmgr_reset_dsp(values[chip], 0, NULL, 0 , NULL);
  if(0 != ret)
  {
    printf("ERROR: DSP putting into failed \n");
    goto err_reset;
  }

  /* Take the DSP out of reset */
  ret = dnldmgr_reset_dsp(values[chip], 1, NULL, 0 , NULL);
  if(0 != ret)
  {
    printf("ERROR: DSP out of reset failed \n");
    goto err_reset;
  }
  /* Check to see if reset is complete */
  while(1){
    for(i=0; i< TOTAL_NUM_CORES_PER_CHIP; i++)
    {
      ret = pciedrv_dsp_read( values[chip], ((0x10 + i) << 24) + BOOT_ENTRY_LOCATION_ADDR, (unsigned char *)&read_boot_entry_location_value[i], 4);
      if(ret != 0)
      {
        printf("\nERROR: pciedrv_dsp_read failed\n");
      }
    } 
    for(i=0; i< TOTAL_NUM_CORES_PER_CHIP; i++)
    {
      if(read_boot_entry_location_value[i] != 0)
        break;
    }
    execution_wait_count++;
    if(execution_wait_count > 1000) 
    {
      printf("\" ERROR: Reset code is not working : Timedout\n ");
      goto err_reset;
    }
    if(i == TOTAL_NUM_CORES_PER_CHIP)
      break;
    usleep(1000);

  };
   
  printf("\n Iterations waited for entry point to clear %d\n", execution_wait_count);
  /* Close pciedriver */
  ret = pciedrv_close();
   
  if (0 != ret)
  {
    printf("ERROR: pciedrv could not be closed\n");
    return -1;
  }
  printf("Dsp %d:  DSP Reset success ! \n", values[chip]);

  return ret;

err_reset:
  printf("Dsp %d:  DSP Reset Fail ! \n", values[chip]);
  pciedrv_close();
  return -1;   


}

static int do_init_global_shared_mem(int argc, char** argv)
{
  unsigned values[4];
  int ret = 0;
  cmem_host_buf_desc_t buf_desc[32];
  uint32_t dsp_start_addr;
  pciedrv_open_config_t pciedrv_open_config;
  cmem_host_frame_desc_t shared_frame_desc;

  int i;

  enum
  {
    chip = 0, reserved_mem_range = 1, number_of_buffers = 2, size_of_buffer = 3, populate_sys_info = 4
  };

  /* Get parameter value */
  ret = get_param_value_array(argc, argv, 2, 5, values);
  if (ret)
  {
    printf("ERROR: failed to get_param_value_array\n");
    return -1;
  }
  if(values[number_of_buffers] > 32) {
    printf("ERROR: number of buffers > 32 not allowed \n");
    return -1;
  }
  /* Open contiguous memory driver */
  ret = cmem_drv_open();
  if(ret !=0) {
    printf("\nERROR: dma mem driver open failed \n");
    return(-1);
  }
  printf("\n drv api open complete \n");  fflush(stdout);

  /* Allocate buffer from cmem driver */
  ret = cmem_drv_alloc(values[number_of_buffers], values[size_of_buffer],
          HOST_BUF_TYPE_PERSISTENT ,  buf_desc);
  if(0 != ret)
  {
    printf("ERROR: contiguous memory allocation failed\n");
    return -1;
  }
  printf("\n Contiguous memory alloc complete \n");fflush(stdout);
  /* Initialise memory to 0 */
  for ( i = 0; i < values[number_of_buffers]; i++)
  {
    memset(buf_desc[i].userAddr, 0, values[size_of_buffer]);
  }

  /* configuration for pcie driver */
  memset(&pciedrv_open_config, 0 , sizeof(pciedrv_open_config_t));
  pciedrv_open_config.dsp_outbound_reserved_mem_size = values[reserved_mem_range];
  pciedrv_open_config.start_dma_chan_num = 0;
  pciedrv_open_config.num_dma_channels = 0;
  pciedrv_open_config.start_param_set_num = 0;
  pciedrv_open_config.num_param_sets = 0;
  pciedrv_open_config.dsp_outbound_block_size = values[size_of_buffer];
  if(values[populate_sys_info])
  {
    /* Place holder for later use */
  }

  /* Open pcie driver */
  ret =  pciedrv_open(&pciedrv_open_config);
  if (0 != ret)
  {
    printf("ERROR: pciedrv could not opened\n");
    return -1;
  }
  /* Allocate dsp memory range */
  ret = pciedrv_dsp_memrange_alloc(values[chip], values[reserved_mem_range],  &dsp_start_addr);
  if(0 != ret)
  {
    printf("ERROR: memrange alloc failed \n");
    return -1;
  }
  printf("\n mem range alloc complete\n");
  /* Map host buffer to dsp memory range */
  ret = pciedrv_map_hostbufs_to_dsp_memrange(values[chip],  values[number_of_buffers], buf_desc, dsp_start_addr);
  if(0 != ret)
  {
    printf("ERROR: map dsp mem range failed \n");
    return -1;
  }
  printf("\n  Map dsp mem range complete \n");

  /* Close contiguous memory driver */
  ret = cmem_drv_close();
  if (0 != ret)
  {
    printf("ERROR: dma mem driver could not closed\n");
    return -1;
  }

  /* Close pcie driver */
  ret = pciedrv_close();

  if (0 != ret)
  {
    printf("ERROR: pciedrv could not closed\n");
    return -1;
  }
  printf(" DSP Init global shared complete ! \n");
  return ret;
}

#define MAX_NUM_PCI_DEVICES 32
static int do_print_pci_info(int argc, char** argv)
{
  int ret = 0;
  pciedrv_device_info_t *devices_info;
  int i;
  uint32_t num_devices;

  /* Open pcie driver */
  ret =  pciedrv_open(NULL);
  if (0 != ret)
  {
    printf("ERROR: pciedrv could not opened\n");
    return -1;
  }
  /* Get number of devices */
  num_devices = pciedrv_get_num_devices();

  devices_info = malloc(num_devices*sizeof(pciedrv_device_info_t));
  if(devices_info == NULL)
  {
    printf("ERROR: malloc failed pciedrv_devices_info_t\n");
    return(-1);
  }
  /* Get Pcie info from driver */
  ret = pciedrv_get_pci_info(devices_info);
  if ( 0 != ret )
  {
    printf("ERROR: get pci info failed \n");
    return(-1);
  }
  /* Print information on devices*/
  printf(" Number of TI PCI devices: %d\n", num_devices);
  for(i=0; i < num_devices; i++)
  {
    printf("dsp_id %d Switch device: %x\n",i, devices_info[i].switch_device);
  }
  for(i=0; i < num_devices; i++)
  {
    printf("%d:path  %s\n",i, devices_info[i].sysfspath);
  }
  /* close driver */
  ret = pciedrv_close();

  if (0 != ret)
  {
      printf("ERROR: pciedrv could not closed\n");
      return -1;
  }
  return ret;

}

static int do_print_mac_address(int argc, char** argv)
{
  int ret = 0;
  int i;
  uint32_t num_devices;
  uint32_t read_value1, read_value2;

  /* Open pcie driver */
  ret =  pciedrv_open(NULL);
  if (0 != ret)
  {
      printf("ERROR: pciedrv could not opened\n");
      return -1;
  }
  /* Get number of devices */
  num_devices = pciedrv_get_num_devices();

  printf(" Number of TI PCI devices: %d\n", num_devices);
  /* Read and print MAC address */
  for(i=0; i < num_devices; i++)
  {
    if(pciedrv_dsp_read(i, 0x2620110, (uint8_t *)&read_value1, 4) != 0)
    {
      printf("ERROR: Reading MAC address failed\n");
      return -1;
    }
   if(pciedrv_dsp_read(i, 0x2620114, (uint8_t *)&read_value2, 4) != 0)
    {
      printf("ERROR: Reading MAC address failed\n");
      return -1;
    }
    printf("dsp_id %d MAC ADDR %x %x\n",i, (read_value2 & 0xffff), read_value1);
  }

  /* close driver */
  ret = pciedrv_close();
 
  if (0 != ret)
  {
      printf("ERROR: pciedrv could not closed\n");
      return -1;
  }
  return ret;

}

/** Print help information to console */
static void do_help(const char *prog)
{
  printf("%s load <chip#0~3> <core#0~7> <image entry point> <image file name (hex)><boot config file name|0>\n\tDownload program.\n", prog);
  printf("%s reset_dsp <chip#0~3>\n\tReset DSP chip.\n", prog);
  printf("%s init_global_shared_mem <chip#0-3> <reserved_mem_range> <number_of_buffers> <buffer_size><populate_system_config:0|1>\n",prog);
  printf("%s print_pci_info", prog);
  printf("%s print_mac_address", prog);
}


#define NUM_CMDS (5)
static const arg_template_t defcmd[NUM_CMDS] = {
  { "load", 7, do_dsp_load},
  {"reset_dsp",2, do_dsp_local_reset},
  {"init_global_shared_mem",7, do_init_global_shared_mem},
  {"print_pci_info",1,do_print_pci_info},
  {"print_mac_address",1,do_print_mac_address}

};


/*-------------------------------------------------------------------------
* main entry point
*-----------------------------------------------------------------------*/
int main(int argc, char** argv)
{
  int ret;

  if (parse_cmd_line(argc, argv, defcmd, NUM_CMDS, &ret) != 0)
  {
      do_help(argv[0]);
  }

  return ret;
}

