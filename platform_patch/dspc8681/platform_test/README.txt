This is the read me file for platform utils test project. 

1) In order to build the project in CCSv5, perform the following steps:
   (1) Open CCSv5 IDE.
   (2) Click on Project
   (3) Select 'Import Existing CCS/CCE Eclipse project' option and click on Next button.
   (4) Mark 'Select root directory' option and browse to the project directory on 
       harddisk (pdk_#_##_##_##\packages\ti\platform\dspc8681\platform_test).
   (5) Uncheck the checkbox 'Copy projects in to workspace'. If you want to preserve the original 
	   copy,then tick the option 'Copy projects in to workspace'.
       Note that you will need to configure the include path appropriately since the 
       default paths no longer applies.
   (6) Mark on the project 'platform_test_dspc8681' and click on Finish option.
   (7) Click on Project->Build Project option to build the marked project in CCSv5 IDE.
    
2) Launch the C6678L emulator configuration. 
    
3) Now select the 'platform_test_dspc8681' project in 'C/C++ Projects' window of CCSv5 IDE. Right click 
	 on the 'platform_utils_test' project and select 'Set as Active Project' option. 
		
4) Connect the XDS510USB emulator or on board USB emulator ( XDS 100 emulator) between HOST machine and EVM board.
   - Power on the EVM board.	
   - Click on Run->Load->'Load the program' option to load output file into the target.
   - You can use your SD510USB emulator for this or use the on board emulator.

4a) If you are not running UART test then skip this step 
    - Set test_uart = 1 in testconfig/platform_test_input.txt (described in step 7)
    - Connect the UART straight cable between PC serial port and EVM UART connector(COM SEL1)
    - Open the PC "Device Manager" and look for "USB Serial Port". The proporty of
      the device would say Location as "on TI XDS100 Channel B". Note the com port
      number (eg: COM20)
    - Open Hyperterminal or any serial port connection utility. Set 'Bits per sec' 
      as 115200(default baud rate in application), 'Data bits' as 8, 'Parity' as none, 
      'Stop bits' as none, Hardware flow as 'None' and enable the echo option in 
      setting of serial port connection utility
    - Select serial port as COM port as discovered in previous sub-step (eg: COM20)

5) Press F8 to run the project.

6) On successfull execution you will see the test report in the console.

7) There is a test configuration file pdk_#_##_##_##\\packages\ti\platform\dspc8681\platform_test\testconfig 
   with name platform_test_input.txt. Edit this file to change the configuration options. 

The valid file syntax is

<token_string> = <value>
<token_string> = <value>
<token_string> = <value>
....

The valid token_strings are given as follows

init_pll  = < 1 to init >
init_ddr  = < 1 to init >
init_uart = < 1 to init >
init_tcsl = < 1 to init >
init_phy  = < 1 to init >

print_info               = < 1 to print >
print_current_core_id    = < 1 to print >
print_switch_state       = < 1 to print >
test_eeprom              = < 1 to run test >
test_nand                = < 1 to run test >
test_nor                 = < 1 to run test >
test_led                 = < 1 to run test >
test_uart                = < 1 to run test >
run_external_memory_test = < 1 to run test >
run_internal_memory_test = < 1 to run test >

init_config_pll1_pllm     = < PLL1 pllm value, 0 to run with default under platform library >
init_config_uart_baudrate = < UART baud rate >

nand_test_block_number = < value >
led_test_loop_count    = < value >
led_test_loop_delay    = < value >
ext_mem_test_base_addr = < value >
ext_mem_test_length    = < value >
int_mem_test_core_id   = < value >

