#include "BLE_Beacon_mydefine.h"

#if TAG 

/* Includes ------------------------------------------------------------------*/
#include "TAG.h"

/* Private variables ---------------------------------------------------------*/
uint8_t my_adv_data[] = { NPK_TAG, AD_TYPE_MANUFACTURER_SPECIFIC_DATA, PREAMBLE, RTPS, NTB, CRC, TAG_POWER_LEVEL, HUMIDITY, TEMPERATURE};
uint8_t device_name[] = { 'S', 'T', 'P', 'V', 'T', 'A', 'G' };
uint8_t local_name[]  = {AD_TYPE_COMPLETE_LOCAL_NAME, 'S', 'T', 'P', 'V', 'T', 'A', 'G' };
uint8_t bdaddr[] = { 0x02, 0x30, 0x71, 0x19, 0x04, 0x26 }; // MAC Address Initialization
uint8_t *transmission_params;   // Transmission Parameters (RTPS and NTB)
uint8_t rtps = RTPS;            // RTPS = Radio Transmission Period in Seconds
uint8_t ntb = NTB;              // NTB  = Number of Transmitted Beacons
uint8_t uart_rx_count = 0;
uint8_t BLE_TXE_status = 0;
tClockTime tag_rx_time = 0;     // Timer for the TAG in RX Mode (Measures How long the TAG stays in RX Mode)
tag_states tag_state = IDLE;    // Status of the TAG FSM
uint16_t uart_data = 0;
/* Private functions ---------------------------------------------------------*/

#if USE_UART || OTAP || SENSORS
void My_SdkEvalComUartInit(uint32_t baudrate) {
  /* 
  ------------ USART configuration -------------------
  - BaudRate = 115200 baud  
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
  
  UART_InitType UART_InitStructure;
  
  /* GPIO Periph clock enable */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_UART | CLOCK_PERIPH_GPIO, ENABLE);
  
  GPIO_InitType GPIO_InitStructure;
    
#if SENSORS
  if (tag_state == TX) {
    GPIO_InitStructure.GPIO_Pin = UART_RX;
    GPIO_InitStructure.GPIO_Mode = SDK_EVAL_UART_RX_MODE;
    GPIO_InitStructure.GPIO_Pull = DISABLE;
    GPIO_InitStructure.GPIO_HighPwr = DISABLE;
    GPIO_Init(&GPIO_InitStructure);
    UART_InitStructure.UART_WordLengthReceive  = UART_WordLength_8b;
    UART_InitStructure.UART_Mode = UART_Mode_Rx;
  }
  else if (tag_state == RX) {
    GPIO_InitStructure.GPIO_Pin = UART_TX;
    GPIO_InitStructure.GPIO_Mode = SDK_EVAL_UART_TX_MODE;
    GPIO_InitStructure.GPIO_Pull = DISABLE;
    GPIO_InitStructure.GPIO_HighPwr = DISABLE;
    GPIO_Init(&GPIO_InitStructure);
    UART_InitStructure.UART_WordLengthReceive  = UART_WordLength_8b;
    UART_InitStructure.UART_Mode = UART_Mode_Tx;
  }
#else  
  GPIO_InitStructure.GPIO_Pin = UART_TX;
  GPIO_InitStructure.GPIO_Mode = SDK_EVAL_UART_TX_MODE;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = DISABLE;
  GPIO_Init(&GPIO_InitStructure);
  UART_InitStructure.UART_WordLengthReceive  = UART_WordLength_8b;
  UART_InitStructure.UART_Mode = UART_Mode_Tx;  /* Only TX enabled, single wire connection */
#endif  
  
  UART_InitStructure.UART_BaudRate = baudrate;
  UART_InitStructure.UART_WordLengthTransmit = UART_WordLength_8b;
  
  UART_InitStructure.UART_StopBits = UART_StopBits_1;
  UART_InitStructure.UART_Parity = UART_Parity_No;
  UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
  UART_InitStructure.UART_FifoEnable = ENABLE;
  UART_Init(&UART_InitStructure);
  
#if SENSORS  
  /* Interrupt as soon as data is received. */
  UART_RxFifoIrqLevelConfig(FIFO_LEV_1_64);
#endif
  
  /* Enable UART */
  UART_Cmd(ENABLE);
  
#if SENSORS    
  /* Enable the UART Interrupt */
  NVIC_InitType NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = UART_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  UART_ITConfig(UART_IT_RX, ENABLE);
#endif
  
}
#endif

void My_GPIO_Configuration(void) {  
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_UART | CLOCK_PERIPH_GPIO, ENABLE);
  GPIO_InitType GPIO_InitStructure;
  
#if OTAP  
  GPIO_InitStructure.GPIO_Pin = BLE_WFE | BTH;
#else
  GPIO_InitStructure.GPIO_Pin = BTH;
#endif 
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Output;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = ENABLE;
  GPIO_Init( &GPIO_InitStructure);  
  
  /* Configure GPIO pin: BLE_TXE */ 
  GPIO_InitStructure.GPIO_Pin = BLE_TXE;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Input;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = ENABLE;
  GPIO_Init( &GPIO_InitStructure); 
}

void My_Main(void) {
  
#if (OTAP && FROM_FLASH)
  if (FLASH_ReadWord(DATA_STORAGE_ADDR) == 0XFFFFFFFF) {        // 1) Check if Flash Word 1 of Flash is Empty    
    FLASH_Unlock();                                             // 2) UnLock the Flash  
    FLASH_ProgramWord(DATA_STORAGE_ADDR, RTPS << 8 | NTB);      // 3) Program Flash with Default Values
    FLASH_Lock();                                               // 4) Lock the Flash
  }
  else {
    transmission_params = (uint8_t*)DATA_STORAGE_ADDR;          // 1) Reading Transmission Parameters from Memory  
    rtps  = *(transmission_params + 1);                         // 2) Extracting rtps
    ntb = *transmission_params;                                 // 3) Extracting ntb
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

void My_IT(void) {
#if OTAP
  if (tag_state == RX && tag_rx_time++ >= TAG_RX_TIMEOUT_TH) {  // Check for timeout during RX state
    tag_rx_time = 0;
#if !DEBUG
    GPIO_WriteBit(BTH, Bit_SET);   // Back to Harvest Energy
#endif
  }
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
    for(j = 5; j >= 2; j--) {
      if (Advertising_Report[i].Address[j] != bdaddr[j]) {        
        break;
      }     
    }    
    if (j < 2 && Advertising_Report[i].Data[1] == 0xFF) { 
#if OTAP
      if (Advertising_Report[i].Data[5] == Advertising_Report[i].Data[2]^
          Advertising_Report[i].Data[3]^Advertising_Report[i].Data[4]) 
      {                                                                                         // 1) Check CRC                           
        aci_gap_terminate_gap_proc(0x80);                                                       // 2) Turn Off Radio ASAP !!! 
        if (FLASH_ReadWord(DATA_STORAGE_ADDR + 4) == 0XFFFFFFFF &&                              // 3) Check If Word 2 of the Flash is Erased
            (Advertising_Report[i].Data[3] != rtps || Advertising_Report[i].Data[4] != ntb))
        {                                                                                       // 4) Check If the received packet matches with the stored values
          FLASH_Unlock();                                                                       // 5) Unlock the Flash Memory         
          FLASH_ProgramWord(DATA_STORAGE_ADDR + 4, Advertising_Report[i].Data[3] 
                            << 8 | Advertising_Report[i].Data[4]);                              // 6) Write the 2nd Word of the Flash
          FLASH_Lock();                                                                         // 7) Lock the Memory   
        }
        GPIO_WriteBit(BTH, Bit_SET);                                                            // 8) Back to Harvest Energy
      }
#endif
    }
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
    if (FLASH_ReadWord(DATA_STORAGE_ADDR + 8) != 0XFFFFFFFF) {                  // 1) Check if the 3rd Word is Empty
      tag_state = EBF;                                                          // 2) EBF = Erase BLE Flash
      uint8_t ntb_temp  = FLASH_ReadByte(DATA_STORAGE_ADDR + 4);                // 3) Read the 1st byte of the 2nd Word and Save the content in RAM
      uint8_t rtps_temp = FLASH_ReadByte(DATA_STORAGE_ADDR + 5);                // 4) Read the 2nd byte of the 2nd Word and Save the content in RAM   
      FLASH_Unlock();                                                           // 5) Unlock the Flash
      FLASH_ErasePage(DATA_STORAGE_PAGE);                                       // 6) Erase the Flash
      FLASH_ProgramWord(DATA_STORAGE_ADDR, rtps_temp << 8 | ntb_temp);          // 7) Program the 1st Word of the Flash
      FLASH_Lock();                                                             // 8) Unlock the Flash
      GPIO_WriteBit(BTH, Bit_SET);                                              // 9) Back to Harvest Energy
    } 
    else if (FLASH_ReadWord(DATA_STORAGE_ADDR + 4) != 0XFFFFFFFF) {             // 1) Check if the 2nd Word is Empty
      My_SdkEvalComUartInit(UART_BAUDRATE);                                     // 2) Initialize UART to Transmit to MCU
      BLE_TXE_status = GPIO_ReadBit(BLE_TXE);                                   // 3) Read the status of BLE_TXE (This is needed for the ACK process)      
      GPIO_WriteBit(BLE_WFE, Bit_SET);                                          // 4) SET the BLE_WFE Signal to start the Communication with the MCU
      tag_state = WMF;                                                          // 5) WMF STATE (WMF = WRITE MCU FLASH)
      while(GPIO_ReadBit(BLE_TXE) == BLE_TXE_status);                           // 6) Wait Acknowledge from MCU through the BLE_TXE signal (That changes its status)
      printf("%c%c", *(transmission_params + 4), *(transmission_params + 5));   // 7) Send NTB (1st) and RTPS (2nd) to MCU through UART 
      while(UART_GetFlagStatus(UART_FLAG_BUSY) == SET);                         // 8) Wait until the UART is Not Busy 
      FLASH_Unlock();                                                           // 9) Unlock the Flash
#if !UART_DEBUG
      FLASH_ProgramWord(DATA_STORAGE_ADDR + 8, 0XFFFFFFFE);                     // 10) Write the 3rd Word
#endif
      FLASH_Lock();                                                             // 11) Lock the Flash  
      GPIO_WriteBit(BTH, Bit_SET);                                              // 12) Back to Harvest Energy
    }
    else
#endif
      if (GPIO_ReadBit(BLE_TXE) == 1) {
        tag_state = TX;                                   // 1) SET TAG STATE to TX
#if SENSORS
        My_SdkEvalComUartInit(UART_BAUDRATE);             // 2) Initialize UART to Receive from MCU  
        //while (uart_rx_count < 2);                      // 3) Wait that Data are received through Uart
#endif
        TAG_Device_Init();                                // 2) Device Configuration      
        aci_hal_set_radio_activity_mask(0x002);           // 3) Enable Callback "aci_hal_end_of_radio_activity_event"
        my_adv_data[3] = rtps;                            // 4) Update Data Packet
        my_adv_data[4] = ntb;                             // 5) Update Data Packet
        my_adv_data[5] = PREAMBLE^rtps^ntb;               // 6) Update Data Packet        
        aci_hal_set_tx_power_level(1,TAG_POWER_LEVEL);    // 7) Set TX at Max Power
        TAG_Beaconing();                                  // 8) Start Beaconing and WFI
      }   
#if !OTAP
      else {
        tag_state = TX;                           // 1) SET TAG STATE to TX
        TAG_Device_Init();                        // 2) Device Configuration
        aci_hal_set_radio_activity_mask(0x002);   // 3) Enable Callback "aci_hal_end_of_radio_activity_event"
        aci_hal_set_tx_power_level(0,0);          // 4) Set TX at Min Power
        TAG_Beaconing();                          // 5) Start Beaconing and WFI
      }
#else
else if (FLASH_ReadWord(DATA_STORAGE_ADDR + 4) == 0XFFFFFFFF){  // Check If 2nd Word of the Flash Memory is Empty
  tag_state = RX;                                               // TAG RX STATE
  TAG_Device_Init();                                            // 1) Device Configuration
  tag_rx_time = 0;                                              // 2) Initialize the timer to End RX Phase
  Clock_Init();                                                 // 3) Start the timer
  aci_gap_start_observation_proc(LE_Scan_Interval,(uint16_t)((TAG_RX_TIMEOUT_TH-2)/0.625),0,0,0,0);  // 4) Listening and WFI (Callback "hci_le_advertising_report_event" or Timer "tag_rx_time"))
}        
#endif
  }
}

#endif

