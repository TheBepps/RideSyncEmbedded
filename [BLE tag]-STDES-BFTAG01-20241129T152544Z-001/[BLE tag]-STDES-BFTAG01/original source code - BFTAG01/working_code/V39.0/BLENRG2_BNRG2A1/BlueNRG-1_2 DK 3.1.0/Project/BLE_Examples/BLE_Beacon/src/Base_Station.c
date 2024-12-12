#include "BLE_Beacon_mydefine.h"

#if !TAG 

/* Includes ------------------------------------------------------------------*/
#include "Base_Station.h"

/* Private variables ---------------------------------------------------------*/
//tClockTime *wb_rx;                    // Array of delay between Window Blind Received Packets

tClockTime wb_rx[NTB-1];                // Array of delay between Window Blind Received Packets
tClockTime wb_rx_max = 0;               // Maximum Number of Window Blind Received Packets
tClockTime wb_rx_min = 0xFFFFFFFF;      // Minimum Number of Window Blind Received packets
tClockTime sv_th = 0;                   // Service Threshold, Manages the Base Station to Send Beacons and Print
tClockTime pdel  = 0;                   // Enlapsed time after the 1st received Packet
tClockTime wb_time = 0;                 // Window Blinds Timer
tClockTime time_stamp_old = 0; 
tClockTime time_stamp_diff = 0;
tClockTime time_stamp_init=0;
bs_states bs_state = DARK;              // Status of the BS FSM (DARK, LIGHT, WB)
bs_com_states bs_com_state = BS_RX;
double wb_avg = 0;                      // Average Delay enlapsed between the Received Packets
float Success_Rate = 0;                 // Success Rate of the Radio Communication
uint8_t bdaddr[] = { 0x02, 0x30, 0x71, 0x19, 0x04, 0x26 };  // MAC Address Initialization
//uint8_t my_adv_data[] = { NPK_BS, AD_TYPE_MANUFACTURER_SPECIFIC_DATA, PREAMBLE, RTPS, NTB, CRC, BS_POWER_LEVEL }; // Transmitted Packet
uint8_t my_adv_data[] = { NPK_BS, AD_TYPE_MANUFACTURER_SPECIFIC_DATA, PREAMBLE, RTPS, NTB, CRC, LIGHT_THRESHOLD, WB_THRESHOLD, WB_CLOSE_TH, DARK_THRESHOLD, CRC1}; // Transmitted Packet
uint8_t device_name[] = { 'S', 'T', 'B', 'S' };
uint8_t local_name[]  = {AD_TYPE_COMPLETE_LOCAL_NAME, 'S', 'T', 'B', 'S' };
uint8_t cnt_nrp = 0;                    // Counts the Number of Received Packets
uint8_t *transmission_params;           // Transmission Parameters (rtps and ntb)
uint8_t *bs_params;                     // Base Station Parameters (light_th, wb_th, wbclose_th and dark_th)
uint8_t rtps = RTPS;                    // RTPS = Radio Transmission Period in Seconds
uint8_t ntb  = NTB;                     // NTB  = Number of Transmitted Beacons
uint8_t light_th = LIGHT_THRESHOLD;     // LIGHT THRESHOLD in Minutes
uint8_t wb_th = WB_THRESHOLD;           // WINDOW BLINDS time hysteresis THRESHOLD in Minutes
uint8_t wbclose_th = WB_CLOSE_TH;       // WINDOW BLINDS CLOSE THRESHOLD
uint8_t dark_th = DARK_THRESHOLD;       // DARK THRESHOLD in Minutes
uint8_t rtps_tag = 0;                   
uint8_t ntb_tag  = 0;             
uint8_t pkloss = 0;                     // pkloss = Number of Packet Lost
#if OTAP
uint32_t prv; 
uint8_t bs_tx_en = BS_TX_EN;            // Enables the Base Station to Transmit
uint32_t bs_beacon_count = 0;           // Counts the Beacons Emitted by the Base Station during otap
uint32_t bs_ack_beacon_count = 0;       // Counts the Beacons Emitted by the Base Station for acknowledge
uint8_t sv_th_flag = 0;
uint8_t rtps_otap = 0;                  // rtps_otap = Radio Transmission Period in Seconds tranmsitted by the App in OTAP
uint8_t ntb_otap = 0;                   // ntb_otap  = Number of Transmitted Beacons transmitted by the App in OTAP
#endif 

/* Private functions ---------------------------------------------------------*/
#if USE_UART
void My_SdkEvalComUartInit(uint32_t baudrate) { 
  UART_InitType UART_InitStructure;
  
  /* GPIO Periph clock enable */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_UART | CLOCK_PERIPH_GPIO, ENABLE);
  
  GPIO_InitType GPIO_InitStructure;
  
  /* Configure GPIO_Pin_4 as UART_RXD */ 
  /* Configure GPIO_Pin_5 as UART_TXD */
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = SDK_EVAL_UART_TX_MODE;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = DISABLE;
  GPIO_Init(&GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = SDK_EVAL_UART_RX_MODE;
  GPIO_Init(&GPIO_InitStructure);
  
  /* 
  ------------ USART configuration -------------------
  - BaudRate = 115200 baud  
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
  
  UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;  
  UART_InitStructure.UART_BaudRate = baudrate;
  UART_InitStructure.UART_WordLengthTransmit = UART_WordLength_8b;
  UART_InitStructure.UART_WordLengthReceive = UART_WordLength_8b;
  UART_InitStructure.UART_StopBits = UART_StopBits_1;
  UART_InitStructure.UART_Parity = UART_Parity_No;
  UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
  UART_InitStructure.UART_FifoEnable = ENABLE;
  UART_Init(&UART_InitStructure);
  
  /* Interrupt as soon as data is received. */
  UART_RxFifoIrqLevelConfig(FIFO_LEV_1_64);
  
  /* Enable UART */
  UART_Cmd(ENABLE);  
}
#endif

void My_GPIO_Configuration(void) {  
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_UART | CLOCK_PERIPH_GPIO, ENABLE);
  GPIO_InitType GPIO_InitStructure;
  
  /* Configure GPIO pin: BLE_TXD */
  GPIO_InitStructure.GPIO_Pin = BLE_RXD | GPIO_Pin_14; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Output;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = ENABLE;
  GPIO_Init( &GPIO_InitStructure);  
}

void My_Main(void) {
  
#if (OTAP && FROM_FLASH)
  if (FLASH_ReadWord(DATA_STORAGE_ADDR) == 0XFFFFFFFF) {        // 1) Check if Flash is Enabled for Writing    
    FLASH_Unlock();                                             // 2) UnLock the Flash 
    FLASH_ProgramWord(DATA_STORAGE_ADDR, RTPS << 8 | NTB);      // 3) Write Flash (DATA_STORAGE_ADDR = 0x1007C000) 
    FLASH_Lock();                                               // 4) Lock the Flash
  }
  else {
    transmission_params = (uint8_t*)DATA_STORAGE_ADDR;          // 1) Reading Transmission Parameters from Memory  
    rtps  = *(transmission_params + 1);                         // 2) Extracting rtps
    ntb = *transmission_params;                                 // 3) Extracting ntb
  }
  
  if (FLASH_ReadWord(DATA_STORAGE_ADDR + 4) == 0XFFFFFFFF) {                                           // 1) Check if the 2nd work of the Flash is Empty    
    FLASH_Unlock();                                                                                    // 2) UnLock the Flash 
    FLASH_ProgramWord(DATA_STORAGE_ADDR + 4, LIGHT_THRESHOLD << 24 | WB_THRESHOLD << 16 | WB_CLOSE_TH << 8 | DARK_THRESHOLD); // 3) Write Flash (DATA_STORAGE_ADDR = 0x1007C000) 
    FLASH_Lock();                                                                                      // 4) Lock the Flash
  }
  else {
    bs_params = (uint8_t*)(DATA_STORAGE_ADDR + 4); // 1) Reading Base Station Parameters from the 2nd word of the flash Memory
    dark_th = *(bs_params);        // 2) Extracting dark_th
    wbclose_th = *(bs_params + 1); // 3) Extracting wbclose_th
    wb_th = *(bs_params + 2);      // 4) Extracting wb_th
    light_th  = *(bs_params + 3);  // 5) Extracting light_th
  }
  
#else  
  rtps = RTPS;
  ntb  = NTB;
  dark_th = DARK_THRESHOLD;
  wb_th = WB_THRESHOLD;
  wbclose_th = WB_CLOSE_TH;
  light_th = LIGHT_THRESHOLD;
#endif
  my_adv_data[3] = rtps;                                                      // 1) Update Data Packet for RTPS
  my_adv_data[4] = ntb;                                                       // 2) Update Data Packet for NTB
  my_adv_data[5] = PREAMBLE^rtps^ntb;                                         // 3) Updating Data Packet for CRC
  my_adv_data[6] = light_th;                                                  // 4) Updating Data Packet for LIGHT_THRESHOLD
  my_adv_data[7] = wb_th;                                                     // 5) Updating Data Packet for WINDOW_BLIND_THRESHOLD
  my_adv_data[8] = wbclose_th;                                                // 6) Updating Data Packet for WINDOW_BLIND_CLOSE_THRESHOLD
  my_adv_data[9] = dark_th;                                                   // 7) Updating Data Packet for DARK_THRESHOLD
  my_adv_data[10] = PREAMBLE^light_th^wb_th^dark_th;                          // 8) Updating Data Packet for CRC1
  
  BS_Device_Init();                                                           // Device Configuration
  Clock_Init();                                                               // System Clock Configuration
  aci_gap_start_observation_proc(LE_Scan_Interval,LE_Scan_Window,0,0,0,0);    // Receiving Data
}

void My_While(void) {  
  GPIO_WriteBit(GPIO_Pin_14, Bit_RESET);
  GPIO_WriteBit(BLE_RXD, Bit_RESET); 
}

void My_IT(void) {
  
  time_stamp_diff++;
  
  if (bs_state != DARK && time_stamp_diff > (dark_th*60000)) {
    bs_state =  DARK;
    cnt_nrp = 0;
    printf(" \r\n");
    printf("GOOD EVENING, PLEASE TURN LIGHTS ON \r\n");
    printf(" \r\n");
    time_stamp_diff = 0;
  }
  
  if (bs_state == WB && wb_time++ > (wb_th*60000)) {
    printf("\r\n OPEN, WINDOW BLINDS \r\n\n");
    bs_state = LIGHT;
    wb_time = 0;
  }
  
  /* Calculating The Servive Threshold sv_th
  The Service Threshold, Manages the Base Station to Send Beacons and Print
  It is defined as follows:
  If The sv_th is 4 * Number of expected packest (NTB)* The minimum estimates delay time enlapsed 
  between received packets, than it is assumed that all the packets have been received.
  Else all the packets are assumed to be received within the 90% of Radio Transmission Period. 
  */
  
  
  if (cnt_nrp == ntb){
    sv_th = (tClockTime)(ntb*wb_rx_min);
  }
  else if (cnt_nrp >= (ntb/2)) {  // Check if Received at least half of the expected packets
    // if ((tClockTime)(4*ntb*wb_rx_min) < (tClockTime)(1000*rtps)) {
    if (bs_tx_en == 0){
      sv_th = (tClockTime)(4*ntb*wb_rx_min);
    }
    else{
      sv_th = (tClockTime)(ntb*wb_rx_min);
    }
  }
  //}
  else {
    if (bs_tx_en == 0){
      sv_th = (tClockTime)(0.9*1000*rtps);
    }
    else{
      sv_th = (tClockTime)(ntb*wb_rx_min);
    }
  }
  
  pdel = Clock_Time() - time_stamp_init;
  if (pdel >= (tClockTime)(sv_th) && sv_th_flag == 1) {  /* Check that all the packets has been received */
    
    sv_th_flag = 0;
    
    if (cnt_nrp > (ntb/2)) { // Check if Received at least half of the expected packets     
      pkloss = 0;
      for(int i = 0; i <= (cnt_nrp-1); i++) {
        if (wb_rx[i] > 1.8*wb_rx_min) {
          pkloss++;
        }
        if (wb_rx[i] > 2.8*wb_rx_min) {
          pkloss++;
        }
      }
      wb_avg = (wb_avg*(cnt_nrp-1))/((cnt_nrp-1) + pkloss);
    }  
    
    if (bs_state == WB) {
      cnt_nrp = 0;
      printf("WB %5.1f %5.1f\r\n", wb_avg, Success_Rate);
    }  
    else 
      if (bs_state == LIGHT) {
        cnt_nrp = 0;
        printf("LIGHT %5.1f %5.1f\r\n", wb_avg, Success_Rate);
      }
    
    if (bs_state == LIGHT && Success_Rate >= SR_TH && wb_avg <= WB_CLOSE_TH) { 
      cnt_nrp = 0;
      bs_state = WB;
      wb_time = 0;
      printf("\r\nCLOSE WINDOW BLINDS WB %4.2f %4.2f \r\n\n", wb_avg, Success_Rate);
    }
    
    if (bs_state == WB && Success_Rate >= SR_TH && wb_avg <= WB_CLOSE_TH) {
      wb_time = 0;
    }  
#if OTAP
    if (bs_tx_en == 1) {                              // 1) Base Station is Enabled to Transmit
      my_adv_data[3] = rtps_otap;                     // 2) Update Data Packet for RTPS
      my_adv_data[4] = ntb_otap;                      // 3) Update Data Packet for NTB
      my_adv_data[5] = PREAMBLE^rtps_otap^ntb_otap;   // 4) Updating Data Packet for CRC
      bs_com_state = BS_TX;                           // 5) Update State Variable
      aci_gap_terminate_gap_proc(0x80);               // 6) Disable the Radio in RX Mode
      aci_hal_set_radio_activity_mask(0x002);         // 7) Enable the Base Station to Transmit
      BS_Beaconing();                                 // 8) Start Beaconing      
    }    
    
    if (bs_com_state == BS_ACK){                         // 1) Base Station is Acknowledge State
      my_adv_data[6] = light_th;                         // 2) Update Data Packet for LIGHT_THRESHOLD
      my_adv_data[7] = wb_th;                            // 3) Update Data Packet for WINDOW BLIND THRESHOLD
      my_adv_data[8] = wbclose_th;                       // 4) Update Data Packet for WINDOW BLIND CLOSE THRESHOLD
      my_adv_data[9] = dark_th;                          // 5) Update Data Packet for DARK THRESHOLD
      my_adv_data[10] = PREAMBLE^light_th^wb_th^dark_th; // 6) Update Data Packet for CRC1
      aci_gap_terminate_gap_proc(0x80);                  // 7) Disable the Radio in RX Mode
      aci_hal_set_radio_activity_mask(0x002);            // 8) Enable the Base Station to Transmit
      BS_Beaconing();                                    // 9) Start Beaconing
    }
#endif 
  }
}
void aci_hal_end_of_radio_activity_event(uint8_t Last_State, uint8_t Next_State, uint32_t Next_State_SysTime) {
  GPIO_WriteBit(GPIO_Pin_14, Bit_SET); 
  
#if OTAP
  /* The Base Station is intended to principally be a receiver. 
  Occasionally can send beacons to program the battery-free TAG. 
  When this happens, after a predefined number of beacons (5*rtps) 
  the Base Station stops tranmistting and returns to work as Receiver. */
  
  if (bs_com_state == BS_ACK && bs_ack_beacon_count++ > 100 && Last_State == 0X01) {
    bs_com_state = BS_RX;                                                    // 1) Set to BS_RX state
    bs_ack_beacon_count = 0;                                                 // 2) Reset ACK Beacon Counter
    aci_gap_set_non_discoverable();                                          // 3) Stop beaconing
    aci_gap_start_observation_proc(LE_Scan_Interval,LE_Scan_Window,0,0,0,0); // 4) Back to Listen    
  }

  if (bs_com_state == BS_TX && bs_beacon_count++ > 8*rtps && Last_State == 0X01) { // 1) Check the Number of emitted Beacons (defined depending on RTPS)
    bs_com_state = BS_RX;                                                    // 1) Set to BS_RX state
    bs_beacon_count = 0;                                                     // 2) Reset Counter
    aci_gap_set_non_discoverable();                                          // 3) Stop beaconing
    aci_gap_start_observation_proc(LE_Scan_Interval,LE_Scan_Window,0,0,0,0); // 4) Back to Listen
  }
#endif                                           
}

void hci_le_advertising_report_event(uint8_t Num_Reports, Advertising_Report_t Advertising_Report[]) {
  //  for(int i = 0; i <= Num_Reports; i++) {
  int j = 0;
  int i = 0;
  uint8_t light_th_tmp = 0;
  uint8_t wb_th_tmp = 0;
  uint8_t wbclose_th_tmp = 0;
  uint8_t dark_th_tmp = 0;  
#if OTAP
  if (Advertising_Report[i].Length_Data > NPK_APP && (Advertising_Report[i].Data[4] == PREAMBLE_APP_TAG)){           
    if (Advertising_Report[i].Data[4]^Advertising_Report[i].Data[5]^
        Advertising_Report[i].Data[6] == Advertising_Report[i].Data[7]){ // Check CRC
          if (Advertising_Report[i].Data[5] != 0 && Advertising_Report[i].Data[6] != 0){
            rtps_otap = Advertising_Report[i].Data[5];
            ntb_otap  = Advertising_Report[i].Data[6];
          }
          if (rtps_otap != rtps || ntb_otap != ntb){
            bs_tx_en = 1;
          }
        }
  }
  else if (Advertising_Report[i].Length_Data > NPK_APP && (Advertising_Report[i].Data[4] == PREAMBLE_APP_BS)){ 
    if (Advertising_Report[i].Data[4]^Advertising_Report[i].Data[5]^
        Advertising_Report[i].Data[6]^Advertising_Report[i].Data[7]^
          Advertising_Report[i].Data[8] == Advertising_Report[i].Data[9]){ // Check CRC
            if (Advertising_Report[i].Data[5] != 0 && Advertising_Report[i].Data[6] != 0 && 
                Advertising_Report[i].Data[7] != 0 && Advertising_Report[i].Data[8] != 0){
                  light_th_tmp = Advertising_Report[i].Data[5];
                  wb_th_tmp    = Advertising_Report[i].Data[6];
                  wbclose_th_tmp  = Advertising_Report[i].Data[7];
                  dark_th_tmp  = Advertising_Report[i].Data[8];
                  bs_com_state = BS_ACK;
                  if (light_th_tmp != light_th || wb_th_tmp != wb_th || wbclose_th_tmp != wbclose_th || dark_th_tmp != dark_th){
                    light_th = light_th_tmp;
                    wb_th = wb_th_tmp;
                    wbclose_th = wbclose_th_tmp;
                    dark_th = dark_th_tmp;
                    FLASH_Unlock();                                             // 1) UnLock the Flash 
                    FLASH_ErasePage(DATA_STORAGE_PAGE);                         // 2) Erase the Flash
                    FLASH_ProgramWord(DATA_STORAGE_ADDR, rtps << 8 | ntb);      // 3) Write the 1st word of the Flash (DATA_STORAGE_ADDR = 0x1007C000) 
                    FLASH_ProgramWord(DATA_STORAGE_ADDR + 4, light_th << 24 | wb_th << 16 | wbclose_th << 8 | dark_th); // 4) Write the 2nd word of the Flash (DATA_STORAGE_ADDR = 0x1007C004) 
                    FLASH_Lock();                                               // 5) Lock the Flash    
                  }
                }
          }
  }
  else    
#endif
    if  ((Advertising_Report[i].Length_Data > NPK_TAG) && (Advertising_Report[i].Data[2] == PREAMBLE)){
      for(j = 5; j >= 2; j--) {
        if (Advertising_Report[i].Address[j] != bdaddr[j]) {  
          break;
        }     
      }            
      if (j < 2) { 
        uint8_t temp[6];
        memcpy(temp,Advertising_Report[i].Data,sizeof(temp));
        if (Advertising_Report[i].Data[2]^Advertising_Report[i].Data[3]^
            Advertising_Report[i].Data[4] == Advertising_Report[i].Data[5]){ // Check CRC
              rtps_tag = Advertising_Report[i].Data[3];
              ntb_tag = Advertising_Report[i].Data[4];
              rtps = rtps_tag; // Synchronize base station rtps parameter with tag rtps parameter 
              ntb  = ntb_tag;  // Synchronize base station ntb  parameter with tag ntb  parameter
              sv_th_flag = 1;
              if (rtps_otap == rtps_tag && ntb_otap == ntb_tag && bs_tx_en == 1) {
                bs_tx_en = 0;
              }
            }
        BS_FSM();
        return;
      }
    }
}

void BS_Device_Init(void) {
  uint8_t ret;
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  
  /* Set the device public address */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
  
#if DEBUG  
  My_Error_Report("aci_hal_write_config_data", ret);
#endif
  
  /* Setting the TX Power to Maximum */
  ret = aci_hal_set_tx_power_level(1, BS_POWER_LEVEL); 
#if DEBUG 
  My_Error_Report("aci_hal_set_tx_power_level", ret);  
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

#if OTAP
static void BS_Beaconing(void) {   
  uint8_t ret = BLE_STATUS_SUCCESS;
  
  /* disable scan response */
  ret = hci_le_set_scan_response_data(0,NULL);
#if DEBUG  
  My_Error_Report("hci_le_set_scan_response_data", ret);
#endif
  
  /* put device in non connectable mode */
  ret = aci_gap_set_discoverable(ADV_NONCONN_IND, 160, 180, PUBLIC_ADDR, NO_WHITE_LIST_USE,
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
#endif

void BS_FSM(void) {
  GPIO_WriteBit(BLE_RXD, Bit_SET);   
  Success_Rate = 0;   
  wb_rx_max = 0;    
  wb_rx_min = (tClockTime)(1000*rtps);
  
  if (bs_state == DARK && time_stamp_diff > (light_th*60000)) {
    cnt_nrp = 0;
    printf(" \r\n");
    printf("GOOD MORNING :-) \r\n");
    printf(" \r\n");
    bs_state = LIGHT;
    memset(wb_rx, 0, sizeof(wb_rx));
    time_stamp_diff = 0;
    time_stamp_old = Clock_Time();
  }
  if (bs_state != DARK) {
    time_stamp_diff = Clock_Time() - time_stamp_old;
    time_stamp_old = Clock_Time();
    if (cnt_nrp == 0) {                       // Recognizing first Packet
      memset(wb_rx, 0, sizeof(wb_rx)); 
      time_stamp_init = Clock_Time();         // Get the time of the 1st Received Packet
      cnt_nrp++;                              // First Packet Received
    }
    else
    {       
      if (cnt_nrp < ntb) {                   // If Received Packets is < of Expected Packets 
        wb_rx[cnt_nrp-1] = time_stamp_diff;  // Write the time difference in the wb_rx vector        
        uint32_t wb_avg_temp = 0;
        for(int c = 0; c < cnt_nrp; c++){  
          wb_avg_temp += wb_rx[c];           // Calculates Average
          if(wb_rx[c] < wb_rx_min) {
            wb_rx_min = wb_rx[c];            // Calculates Min
          }  
          if(wb_rx[c] > wb_rx_max) {
            wb_rx_max = wb_rx[c];            // Calculates Max
          }    
        }      
        wb_avg = wb_avg_temp/cnt_nrp; 
        cnt_nrp ++;
        Success_Rate = 100*cnt_nrp/ntb;   // Calculate  Success Rate
      }     
      time_stamp_diff = 0; 
    }
  }
}

#endif
