#include "BLE_Beacon_mydefine.h"

#if TAG

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "bluenrg1_events.h"
#include "ble_const.h"
#include "sleep.h"
#include "SDK_EVAL_Config.h"
#include "OTA_btl.h"
#include "clock.h"

/* Defines ------------------------------------------------------------------*/
/* Defines for GPIO Configuration */
#define BLE_TXE GPIO_Pin_8      // BLE_TXE --> BLE TX ENABLED
#define BTH GPIO_Pin_6          // BTH --> Back To Harvesting
#define BLE_WFE GPIO_Pin_11     // BLE_WFE --> BLE Write Flash Enable
#define UART_TX GPIO_Pin_5      // Configure GPIO_Pin_5 as UART_TX
#define UART_RX GPIO_Pin_4      // Configure GPIO_Pin_5 as UART_RX

/* Defines for BLE in TAG in Scan Mode */
#define LE_Scan_Interval 0x4000         // LE_Scan_Interval(time) = LE_Scan_Interval * 0.625ms 
#define LE_Scan_Window   0x4000         // LE_Scan_Window(time)   = LE_Scan_Window * 0.625ms 

/* Defines for FLASH Programming */
#define N_PAGES            (_MEMORY_FLASH_SIZE_/N_BYTES_PAGE)
#define DATA_STORAGE_PAGE  (N_PAGES-8)   /* Reserved Flash Page */
#define DATA_STORAGE_ADDR  (((N_PAGES-8)*N_BYTES_PAGE) + FLASH_START) // 0X1007C000

#define TAG_RX_TIMEOUT_TH 10       // Threshold to stop TAG RX Mode (in msec)
#define TAG_POWER_LEVEL 7

/* Private variables ---------------------------------------------------------*/
typedef enum {IDLE, TX, RX, WMF, EBF} tag_states;
extern uint8_t bdaddr[];
extern uint8_t my_adv_data[];
extern uint8_t device_name[]; 
extern uint8_t local_name[]; 
extern uint8_t *tparams;        // Transmission Parameters (RTPS and NTB)
extern uint8_t rtps;            // RTPS = Radio Transmission Period in Seconds
extern uint8_t ntb;             // NTB  = Number of Transmitted Beacons
extern uint8_t BLE_TXE_status;
extern tClockTime tag_rx_time;
extern tag_states tag_state;

/* Private function prototypes -----------------------------------------------*/
void My_SdkEvalComUartInit(uint32_t baudrate);
void My_GPIO_Configuration(void);
void My_Main(void);
void My_While(void);
void My_IT(void);
void hci_le_advertising_report_event(uint8_t Num_Reports, Advertising_Report_t Advertising_Report[]);
void aci_hal_end_of_radio_activity_event(uint8_t Last_State, uint8_t Next_State, uint32_t Next_State_SysTime);
void TAG_Device_Init(void);
static void TAG_Beaconing(void);
void TAG_FSM(void);

#endif