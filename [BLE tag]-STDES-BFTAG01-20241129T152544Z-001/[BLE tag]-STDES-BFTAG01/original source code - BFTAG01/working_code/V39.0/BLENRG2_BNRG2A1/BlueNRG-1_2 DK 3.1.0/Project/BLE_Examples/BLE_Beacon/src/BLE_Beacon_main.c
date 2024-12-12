
/******************** (C) COPYRIGHT 2018 STMicroelectronics ********************
* File Name          : BLE_Beacon_main.c
* Author             : RF Application Team
* Version            : 1.1.0
* Date               : 15-January-2016
* Description        : Code demostrating the BLE Beacon application
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
* @file BLE_Beacon_main.c
* @brief This is a BLE beacon demo that shows how to configure a BlueNRG-1,2 device 
* in order to advertise specific manufacturing data and allow another BLE device to
* know if it is in the range of the BlueNRG-1 beacon device. 
* It also provides a reference example about how using the 
* BLE Over-The-Air (OTA) Service Manager firmware upgrade capability.
* 

* \section Serial_IO Serial I/O
@table
| Parameter name  | Value            | Unit      |
----------------------------------------------------
| Baudrate        | 115200 [default] | bit/sec   |
| Data bits       | 8                | bit       |
| Parity          | None             | bit       |
| Stop bits       | 1                | bit       |
@endtable

* \section Usage Usage

The Beacon demo configures a BlueNRG-1,2 device in advertising mode (non-connectable mode) with specific manufacturing data.
It transmits advertisement packets at regular intervals which contain the following manufacturing data:
@table   
------------------------------------------------------------------------------------------------------------------------
| Data field              | Description                       | Notes                                                  |
------------------------------------------------------------------------------------------------------------------------
| Company identifier code | SIG company identifier (1)        | Default is 0x0030 (STMicroelectronics)                 |
| ID                      | Beacon ID                         | Fixed value                                            |
| Length                  | Length of the remaining payload   | NA                                                     |
| Location UUID           | Beacons UUID                      | It is used to distinguish specific beacons from others |
| Major number            | Identifier for a group of beacons | It is used to group a related set of beacons           |                                              
| Minor number            | Identifier for a single beacon    | It is used to identify a single beacon                 |                                       
| Tx Power                | 2's complement of the Tx power    | It is used to establish how far you are from device    |                                       
@endtable

- (1): SIG company identifiers are available on https://www.bluetooth.org/en-us/specification/assigned-numbers/company-identifiers
- NA : Not Applicable;
NOTEs:
- OTA Service Manager support requires to build application by enabling only ST_USE_OTA_SERVICE_MANAGER_APPLICATION=1 (preprocessor, linker) options and through files: OTA_btl.[ch] (refer to Release_with_OTA_ServiceManager IAR workspace).
- OTA FW upgrade feature is supported only on BlueNRG-2, BLE stack v2.x.

* \section Compiler Instruction
This Firmware is developed to work specifically with the board: X-NUCLEO BNRG2A1 standalone (No STM32 required)
To compile the code make sure that the "C/C++ Compiler preprocessor options are set as following:
1) LS_SOURCE=LS_SOURCE_INTERNAL_RO
2) SMPS_INDUCTOR=SMPS_INDUCTOR_NONE
3) BLUENRG2_DEVICE
4) HS_SPEED_XTAL=HS_SPEED_XTAL_32MHZ
5) USER_BUTTON=BUTTON_1
6) BOR_CONFIG = BOR_OFF
7) BLE_STACK_CONFIGURATION = BLE_STACK_FULL_CONFIGURATION

**/

/** @addtogroup BlueNRG1_demonstrations_applications
*  BlueNRG-1 Beacon demo \see BLE_Beacon_main.c for documentation.
*
*@{
*/

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
*/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stdlib.h"
#include <string.h>
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "sleep.h"
#include "SDK_EVAL_Config.h"
#include "Beacon_config.h"
#include "OTA_btl.h"
#include "clock.h"
#include "BlueNRG1_i2c.h"
#include "BLE_Beacon_mydefine.h"
#include "TAG.h"
#include "Base_Station.h"

/* Private variables ---------------------------------------------------------*/
extern uint8_t uart_rx_count;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void My_Error_Report(char *str, char ret) {
  if(ret != 0)
    printf ("Error in %s 0x%04xr\n", str, ret);
}

void My_Create_MAC(void) {
  /* My_Create_MAC Documentation
  The BlueNRG-1 and BlueNRG-2 devices do not have a valid preassigned MAC 
  address, but a unique serial number (read only for the user).
  The unique serial number is a six byte value stored at address 0x100007F4: 
  it is stored as two words (8 bytes) at address 0x100007F4 and 0x100007F8 
  with unique serial number padded with 0xAA55. */
  
  bdaddr[0] = FLASH_ReadByte(0x100007F8);
  bdaddr[1] = FLASH_ReadByte(0x100007F4);
}

int main(void) {  
  //  wb_rx = malloc(ntb*sizeof(tClockTime));  
  uint8_t ret;
  SystemInit();                 // System Init
  SdkEvalIdentification();      // Identify BlueNRG-1 platform
  
#if USE_UART
  My_SdkEvalComUartInit(UART_BAUDRATE); /* Init the UART peripheral */
#endif  
  
  ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params); // BlueNRG-1 stack init
  My_Error_Report("BlueNRG_Stack_Initialization", ret);  
  My_Create_MAC();              // Create Mac Address
  My_GPIO_Configuration();      // Configure GPIO 
  My_Main();                    // Main
  
  while(1) 
  {     
    BTLE_StackTick();
    My_While(); 
  }
}
/****************** BlueNRG-1 Sleep Management Callback ********************************/

SleepModes App_SleepMode_Check(SleepModes sleepMode) {
  if(SdkEvalComIOTxFifoNotEmpty() || SdkEvalComUARTBusy())
    return SLEEPMODE_RUNNING;
  
  return SLEEPMODE_NOTIMER;
}

/***************************************************************************************/

#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
*/
