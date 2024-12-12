#include "BLE_Beacon_mydefine.h"

#if TAG 

/* Includes ------------------------------------------------------------------*/
#include "TAG.h"

/* Private variables ---------------------------------------------------------*/
#if VAR
uint8_t my_adv_data[] = {NPK_TAG, AD_TYPE_MANUFACTURER_SPECIFIC_DATA, PREAMBLE, RTPS, NTB, CRC, TAG_POWER_LEVEL, TEMPERATURE, HUMIDITY, TADV_MSB, TADV_LSB};
uint8_t device_name[] = { 'S', 'T', 'P', 'V', 'T', 'A', 'G' };
uint8_t local_name[]  = {AD_TYPE_COMPLETE_LOCAL_NAME, 'S', 'T', 'P', 'V', 'T', 'A', 'G' };
uint8_t bdaddr[] = { 0x02, 0x30, 0x71, 0x04, 0x26, FW_Version }; // MAC Address Initialization
uint8_t *transmission_params;   // Transmission Parameters (RTPS and NTB)
uint8_t rtps = RTPS;            // RTPS = Radio Transmission Period in Seconds
uint8_t ntb = NTB;              // NTB  = Number of Transmitted Beacons
uint8_t BLE_TXE_status = 0;
tClockTime tag_rx_time = 0;     // Timer for the TAG in RX Mode (Measures How long the TAG stays in RX Mode)
tag_states tag_state = IDLE;    // Status of the TAG FSM
#endif

/* Private functions ---------------------------------------------------------*/

void My_GPIO_Configuration(void) {  
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_UART | CLOCK_PERIPH_GPIO, ENABLE);
  GPIO_InitType GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin = BLE_WFE | BTH;
  GPIO_WriteBit(BLE_WFE, Bit_SET);    
  GPIO_InitStructure.GPIO_Mode = GPIO_Output;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = DISABLE;
  GPIO_Init( &GPIO_InitStructure); 
  
  /* Configure GPIO pin: BLE_TXE */ 
  GPIO_InitStructure.GPIO_Pin = BLE_TXE;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Input;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = ENABLE;
  GPIO_Init( &GPIO_InitStructure);  
}

void My_I2C_Configuration(uint8_t I2C_OperatingMode){ // I2C_OperatingMode [I2C_OperatingMode_Master I2C_OperatingMode_Slave]
  GPIO_InitType GPIO_InitStructure;
  I2C_InitType I2C_InitStruct;
  
  /* Enable I2C and GPIO clocks */
  if((I2C_Type*)SDK_EVAL_I2C == I2C2) {
    SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO | CLOCK_PERIPH_I2C2, ENABLE);
  }
  else {
    SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO | CLOCK_PERIPH_I2C1, ENABLE);
  }
  
  /* Configure I2C pins */
  GPIO_InitStructure.GPIO_Pin = I2C_CLK_PIN ;
  GPIO_InitStructure.GPIO_Mode = SDK_EVAL_I2C_CLK_MODE;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = DISABLE;
  GPIO_Init(&GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = I2C_DATA_PIN;
  GPIO_InitStructure.GPIO_Mode = SDK_EVAL_I2C_DATA_MODE;  
  GPIO_Init(&GPIO_InitStructure);
  
  /* Configure I2C in master mode */
  I2C_StructInit(&I2C_InitStruct);
  I2C_InitStruct.I2C_OperatingMode = I2C_OperatingMode;
  I2C_InitStruct.I2C_ClockSpeed = SDK_EVAL_I2C_CLK_SPEED;
  I2C_Init((I2C_Type*)SDK_EVAL_I2C, &I2C_InitStruct);
  
  I2C_SetRxThreshold((I2C_Type*)SDK_EVAL_I2C, 1);
  I2C_SetTxThreshold((I2C_Type*)SDK_EVAL_I2C, 1);
  
  /* Clear all I2C pending interrupts */
  I2C_ClearITPendingBit((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MSK);
}

void My_I2C_Write(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t NumByteToWrite){
  I2C_TransactionType t;
  
  /* Write the slave address */
  t.Operation = I2C_Operation_Write;
  t.Address = DeviceAddr;
  t.StartByte = I2C_StartByte_Disable;
  t.AddressType = I2C_AddressType_7Bit;
  t.StopCondition = I2C_StopCondition_Enable;
  t.Length = NumByteToWrite;
  
  /* Flush the slave address */
  I2C_FlushTx((I2C_Type*)SDK_EVAL_I2C);
  while (I2C_WaitFlushTx((I2C_Type*)SDK_EVAL_I2C) == I2C_OP_ONGOING);
  
  /* Begin transaction */
  I2C_BeginTransaction((I2C_Type*)SDK_EVAL_I2C, &t);  // Send DeviceAddr
  
  for(uint8_t i=0; i<NumByteToWrite;i++) {
    I2C_FillTxFIFO((I2C_Type*)SDK_EVAL_I2C, pBuffer[i]);
  }
  
  /* Wait loop */
  do {
    if(I2C_GetStatus((I2C_Type*)SDK_EVAL_I2C) == I2C_OP_ABORTED)
      return;
    
  } while (I2C_GetITStatus((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTD) == RESET);
  
  /* Clear pending bits */
  I2C_ClearITPendingBit((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTD | I2C_IT_MTDWS);
}

void My_I2C_Read(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t NumByteToRead){
  /* Reads Sensors Data from the MCU */
  I2C_TransactionType t;
  
  /* read data */
  t.Operation = I2C_Operation_Read;
  t.Address = DeviceAddr;
  t.StartByte = I2C_StartByte_Disable;
  t.AddressType = I2C_AddressType_7Bit;
  t.StopCondition = I2C_StopCondition_Enable;
  t.Length = NumByteToRead;  
  I2C_BeginTransaction((I2C_Type*)SDK_EVAL_I2C, &t);
  
  /* Wait loop */
  do {
    if(I2C_OP_ABORTED == I2C_GetStatus((I2C_Type*)SDK_EVAL_I2C))
      return;
    
  } while (RESET == I2C_GetITStatus((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTD));
  
  /* Clear pending bits */
  I2C_ClearITPendingBit((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTD | I2C_IT_MTDWS);
  
  /* Get data from RX FIFO */
  while(NumByteToRead--) {
    *pBuffer = I2C_ReceiveData((I2C_Type*)SDK_EVAL_I2C);
    pBuffer ++;
  }
}

void TAG_Device_Init(void) {
  uint8_t ret;
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  
  /* Set the device public address */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
  
#if DEBUG  
  My_Error_Report("aci_hal_write_config_data", ret);
#endif
  
  /* Init the GATT */
  ret = aci_gatt_init();
#if DEBUG 
  My_Error_Report("aci_gatt_init", ret);
#endif
  
  /* Init the GAP */
  ret = aci_gap_init(0xF, 0x00, sizeof(device_name), &service_handle, 
                     &dev_name_char_handle, &appearance_char_handle);  
#if DEBUG 
  My_Error_Report("aci_gap_init", ret);      
#endif
  
  /* Update device name */
  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, 
                                   sizeof(device_name), device_name);  
#if DEBUG  
  My_Error_Report("aci_gatt_update_char_value", ret);
#endif
}

void aci_hal_end_of_radio_activity_event(uint8_t Last_State, uint8_t Next_State, uint32_t Next_State_SysTime) {  
  if (Last_State == 0x01 && tag_state == TX) {
    GPIO_WriteBit(BTH, Bit_SET); /* Back to Harvest Energy */
  }                                              
}

void hci_le_advertising_report_event(uint8_t Num_Reports, Advertising_Report_t Advertising_Report[]) {
  for(int i = 0; i <= Num_Reports; i++) {
    int j = 0;
    for(j = 4; j >= 2; j--) {
      if (Advertising_Report[i].Address[j] != bdaddr[j]) {        
        break;
      }     
    }    
    if (j < 2 && Advertising_Report[i].Data[1] == 0xFF) { 
#if OTAP
      if (Advertising_Report[i].Data[5] == Advertising_Report[i].Data[2]^
          Advertising_Report[i].Data[3]^Advertising_Report[i].Data[4]) 
      {                                                                                         // Check CRC                           
        aci_gap_terminate_gap_proc(0x80);                                                       // Turn Off Radio ASAP !!! 
        if (FLASH_ReadWord(DATA_STORAGE_ADDR + 4) == 0XFFFFFFFF &&                              // Check If Word 2 of the Flash is Erased
            (Advertising_Report[i].Data[3] != rtps || Advertising_Report[i].Data[4] != ntb))
        {                                                                                       // Check If the received packet matches with the stored values
          FLASH_Unlock();                                                                       // Unlock the Flash Memory         
          FLASH_ProgramWord(DATA_STORAGE_ADDR + 4, Advertising_Report[i].Data[3] 
                            << 8 | Advertising_Report[i].Data[4]);                              // Write the 2nd Word of the Flash
          FLASH_Lock();                                                                         // Lock the Memory   
        }
        GPIO_WriteBit(BTH, Bit_SET);                                                            // Back to Harvest Energy
      }
#endif
    }
  }
}

static void TAG_Beaconing(void) {  
  uint8_t ret = BLE_STATUS_SUCCESS;
  
  /* disable scan response */
  ret = hci_le_set_scan_response_data(0,NULL);
#if DEBUG  
  My_Error_Report("hci_le_set_scan_response_data", ret);
#endif
  
  /* put device in non connectable mode */
  ret = aci_gap_set_discoverable(ADV_NONCONN_IND, 160, 160, PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                 sizeof(local_name), local_name, 0, NULL, 0, 0); 
#if DEBUG  
  My_Error_Report("aci_gap_set_discoverable", ret);
#endif
  
#if ENABLE_FLAGS_AD_TYPE_AT_BEGINNING
  /* Set the  ADV data with the Flags AD Type at beginning of the 
  advertsing packet,  followed by the beacon manufacturer specific data */
  ret = hci_le_set_advertising_data (sizeof(adv_data), adv_data);
#if DEBUG  
  My_Error_Report("hci_le_set_advertising_data", ret);
#endif
  
#else
  /* Delete the TX power level information */
  ret = aci_gap_delete_ad_type(AD_TYPE_TX_POWER_LEVEL); 
#if DEBUG  
  My_Error_Report("aci_gap_delete_ad_type", ret);
#endif
  
  /* Update the ADV data with the BEACON manufacturing data */
  ret = aci_gap_update_adv_data(sizeof(my_adv_data), my_adv_data);  
#if DEBUG  
  My_Error_Report("aci_gap_update_adv_data", ret);
#endif
  
#endif
  
}

void TAG_FSM(void) {  
  /* The TAG FSM comprises 3 states: 
  IDLE  --> TAG is setting up 
  TX    --> TAG is Transmitting at Max Power as the signal BLE_TXE is '1'
  RX    --> TAG is Scanning Mode to Receive Packets from the Base Station as the signal BLE_TXE is '0'
  WMF   --> (Write MCU FLASH) The TAG is Sending Data throught the UART to write the MCU EEPROM. Regardless of the BLE_TXE status
  EBF   --> (Erase BLE Flash) The TAG is Erasing internal Flash and updating the 1st Word of the Flash. Regardless of the BLE_TXE status
  */  
  
  if (tag_state == IDLE) {
    
#if OTAP
    if (FLASH_ReadWord(DATA_STORAGE_ADDR + 8) != 0XFFFFFFFF) {         // Check if the 3rd Word is Empty
      tag_state = EBF;                                                 // EBF = Erase BLE Flash
      uint8_t ntb_temp  = FLASH_ReadByte(DATA_STORAGE_ADDR + 4);       // Read the 1st byte of the 2nd Word and Save the content in RAM
      uint8_t rtps_temp = FLASH_ReadByte(DATA_STORAGE_ADDR + 5);       // Read the 2nd byte of the 2nd Word and Save the content in RAM   
      FLASH_Unlock();                                                  // Unlock the Flash
      FLASH_ErasePage(DATA_STORAGE_PAGE);                              // Erase the Flash
      FLASH_ProgramWord(DATA_STORAGE_ADDR, rtps_temp << 8 | ntb_temp); // Program the 1st Word of the Flash
      FLASH_Lock();                                                    // Unlock the Flash
      GPIO_WriteBit(BTH, Bit_SET);                                     // Back to Harvest Energy
    } 
    else if (FLASH_ReadWord(DATA_STORAGE_ADDR + 4) != 0XFFFFFFFF) {    // Check if the 2nd Word is Empty
      tag_state = WMF;                                                 // WMF STATE (WMF = WRITE MCU FLASH)         
      BLE_TXE_status = GPIO_ReadBit(BLE_TXE);                          // Read the status of BLE_TXE (This is needed for the ACK process)            
      GPIO_WriteBit(BLE_WFE, Bit_RESET);                               // RESET the BLE_WFE Signal to start the Communication with the MCU     
      while(GPIO_ReadBit(BLE_TXE) == BLE_TXE_status);                  // Wait Acknowledge from MCU through the I2C is ready    
      My_I2C_Configuration(I2C_OperatingMode_Master);                  // Configure I2C as Master   
      My_I2C_Write((transmission_params + 4), MCU_ADDRESS, 2);         // Send NTB (1st) and RTPS (2nd) to MCU through the I2C      
      I2C_DeInit(I2C2);                                                // I2C Deinit 
      FLASH_Unlock();                                                  // Unlock the Flash
      FLASH_ProgramWord(DATA_STORAGE_ADDR + 8, 0XFFFFFFFE);            // Write the 3rd Word
      FLASH_Lock();                                                    // Lock the Flash  
      GPIO_WriteBit(BTH, Bit_SET);                                     // Back to Harvest Energy
    }
    else
#endif
      if (GPIO_ReadBit(BLE_TXE) == 1) {
        tag_state = TX;                                        // SET TAG STATE to TX
#if SENSORS
        float sensor_data[4];
        My_I2C_Configuration(I2C_OperatingMode_Master);        // Initiailize I2C #2 as Master
        My_I2C_Read((uint8_t*)sensor_data, MCU_ADDRESS, 16);   // Read Sensor Data
        my_adv_data[7]  = (uint8_t)sensor_data[0];             // Humidity
        my_adv_data[8]  = (uint8_t)sensor_data[1];             // Temperature
        my_adv_data[9]  = (uint8_t)sensor_data[2];             // tw_msb (Proportional to Harvested Energy)
        my_adv_data[10] = (uint8_t)sensor_data[3];             // tw_lsb (Proportional to Harvested Energy)        
        I2C_DeInit(I2C2);                                      // DeInit I2C2   
#endif
        TAG_Device_Init();                                     // Device Configuration      
        aci_hal_set_radio_activity_mask(0x002);                // Enable Callback "aci_hal_end_of_radio_activity_event"
        my_adv_data[3] = rtps;                                 // Update Data Packet
        my_adv_data[4] = ntb;                                  // Update Data Packet
        my_adv_data[5] = PREAMBLE^rtps^ntb;                    // Update Data Packet        
        aci_hal_set_tx_power_level(1,TAG_POWER_LEVEL);         // Set TX at Max Power
        TAG_Beaconing();                                       // Start Beaconing and WFI
      }   
#if !OTAP
      else {
        tag_state = TX;                                        // SET TAG STATE to TX
        TAG_Device_Init();                                     // Device Configuration
        aci_hal_set_radio_activity_mask(0x002);                // Enable Callback "aci_hal_end_of_radio_activity_event"
        aci_hal_set_tx_power_level(0,0);                       // Set TX at Min Power
        TAG_Beaconing();                                       // Start Beaconing and WFI        
      }
#else
else if (FLASH_ReadWord(DATA_STORAGE_ADDR + 4) == 0XFFFFFFFF){ // Check If 2nd Word of the Flash Memory is Empty
  tag_state = RX;                                              // TAG RX STATE 
  TAG_Device_Init();                                           // Device Configuration
  tag_rx_time = 0;                                             // Initialize the timer to End RX Phase
  Clock_Init();                                                // Start the timer  
  aci_gap_start_observation_proc(LE_Scan_Interval,(uint16_t)((TAG_RX_TIMEOUT_TH-2)/0.625),0,0,0,0);  // 4) Listening and WFI (Callback "hci_le_advertising_report_event" or Timer "tag_rx_time"))
}        
#endif
  }
}

void My_IT(void) {
#if OTAP
  if (tag_state == RX && tag_rx_time++ >= TAG_RX_TIMEOUT_TH) {  // Check for timeout during RX state    
    tag_rx_time = 0;
#if !DEBUG
    GPIO_WriteBit(BTH, Bit_SET);   // Back to Harvest Energy
    BlueNRG_Sleep(SLEEPMODE_NOTIMER, 0, 0);
#endif
  }
#endif 
}

void My_Main(void) {
  
#if (OTAP && FROM_FLASH)
  if (FLASH_ReadWord(DATA_STORAGE_ADDR) == 0XFFFFFFFF) {   // Check if Flash Word 1 of Flash is Empty    
    FLASH_Unlock();                                        // UnLock the Flash  
    FLASH_ProgramWord(DATA_STORAGE_ADDR, RTPS << 8 | NTB); // Program Flash with Default Values
    FLASH_Lock();                                          // Lock the Flash
  }
  else {
    transmission_params = (uint8_t*)DATA_STORAGE_ADDR;     // Reading Transmission Parameters from Memory  
    rtps  = *(transmission_params + 1);                    // Extracting rtps
    ntb = *transmission_params;                            // Extracting ntb
  }
#else  
  rtps = RTPS;
  ntb  = NTB;
#endif
  
  TAG_FSM();  
}

void My_While(void) {
  if (GPIO_ReadBit(BTH) == 1) {
    BlueNRG_Sleep(SLEEPMODE_CPU_HALT, 0, 0);
  } 
#if DEBUG  
  GPIO_WriteBit(BTH, Bit_RESET);
  GPIO_WriteBit(BLE_WFE, Bit_RESET);
#endif 
  
}
#endif

