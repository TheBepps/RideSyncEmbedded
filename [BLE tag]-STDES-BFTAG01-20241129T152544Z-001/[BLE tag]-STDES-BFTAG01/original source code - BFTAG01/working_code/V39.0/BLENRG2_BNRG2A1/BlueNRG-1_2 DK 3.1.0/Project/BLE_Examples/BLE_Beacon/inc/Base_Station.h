#include "BLE_Beacon_mydefine.h"

#if !TAG

/* Includes ------------------------------------------------------------------*/
#include <stdio.h> 
#include "string.h"
#include "bluenrg1_events.h"
#include "ble_const.h"
#include "SDK_EVAL_Config.h"
#include "OTA_btl.h"
#include "clock.h"

/* Defines ------------------------------------------------------------------*/
/* Defines for Base Station in Transmission Mode */
#define BS_TX_EN 0
//#define N_BS_TX_BEACONS 5  // Number of Beacons emitted by the Base Station

/* Defines for GPIO Configuration */
#define BLE_RXD    GPIO_Pin_6    // BLE_RXD --> BLE RX Done

/* Defines for BLE in BS in Scan Mode */
#define LE_Scan_Interval 0x4000 // LE_Scan_Interval(time) = LE_Scan_Interval * 0.625ms 
#define LE_Scan_Window   0x4000 // LE_Scan_Window(time)   = LE_Scan_Window * 0.625ms 

/* Defines for BS FSM */
#define LIGHT_THRESHOLD 6  // Minutes
#define DARK_THRESHOLD  5  // Minutes
#define WB_THRESHOLD    10 // Minutes

#define SR_TH 60        // Success Rate Threshold
#define WB_CLOSE_TH 90  // Window Blind CLOSE Threshold

/* Define for FLASH Programming */
#define N_PAGES            (_MEMORY_FLASH_SIZE_/N_BYTES_PAGE)
#define DATA_STORAGE_PAGE  (N_PAGES-8)   /* Reserved Flash Page */
#define DATA_STORAGE_ADDR  (((N_PAGES-8)*N_BYTES_PAGE) + FLASH_START)  // 0X1007C000

#define BS_POWER_LEVEL 7

/* Private variables ---------------------------------------------------------*/
typedef enum {DARK, LIGHT, WB} bs_states;
typedef enum {BS_TX, BS_RX, BS_ACK} bs_com_states;

extern tClockTime  time_stamp_old; 
extern tClockTime time_stamp_diff;
extern tClockTime time_stamp_init;
extern tClockTime wb_time;              // Timer for Window Blinds
//extern tClockTime *wb_rx;             // Array of delay between Window Blind Received packets
extern tClockTime wb_rx[NTB-1];         // Array of delay between Window Blind Received packets
extern tClockTime wb_rx_max;            // Maximum Number of Window Blind Received packets
extern tClockTime wb_rx_min;            // Minimum Number of Window Blind Received packets
extern bs_states bs_state;              // Status of the Light (DARK, LIGHT, WB)
extern bs_com_states bs_com_state;
extern double wb_avg;                   // Average Number of Window Blind Received packets
extern float Success_Rate;              // Success Rate of the communication radio
extern uint8_t bdaddr[];                // MAC Address Initialization
extern uint8_t my_adv_data[];           // BLE ADVERTISING Data Packet
extern uint8_t device_name[];           // Device Name as dislayed in the App
extern uint8_t local_name[];            // Device Name as dislayed in the App
extern uint8_t cnt_nrp;                 // Counts the Number of the Received Packets
extern uint8_t *transmission_params;    // Transmission Parameters (RTPS and NTB)
extern uint8_t rtps;                    // RTPS = Radio Transmission Period in Seconds
extern uint8_t ntb;                     // NTB  = Number of Transmitted Beacons
extern uint8_t pkloss;                  // pkloss = Numner of Packet Lost
#if OTAP
extern uint8_t bs_tx_en;                // Enables the Base Station to Transmit
extern uint32_t bs_beacon_count;         // Count Beacons Emitted by the Base Station
#endif 

/* Private function prototypes -----------------------------------------------*/
void My_SdkEvalComUartInit(uint32_t baudrate);
void My_GPIO_Configuration(void);
void My_Main(void);
void My_While(void);
void My_IT(void);
void hci_le_advertising_report_event(uint8_t Num_Reports, Advertising_Report_t Advertising_Report[]);
void aci_hal_end_of_radio_activity_event(uint8_t Last_State, uint8_t Next_State, uint32_t Next_State_SysTime);
void BS_Device_Init(void);
static void BS_Beaconing(void);
void BS_FSM(void);

#endif
