#include "stdio.h"
#include "stdlib.h"
#include "main.h"

#define UART_DEBUG 0
#define OTAP 1
#define SENSORS 0
#define FROM_FLASH 1
#define RF_CONTROL 0				// If '1' Enables Radio Activity Control
#define NTB_CONTROL 1				// If '1' Enables Window Blinds Mode
#define TADV_CONTROL 1
#define TRISE_CONTROL 0
#define COMPARATOR 0

#define NOD_BLE 2					// Number of Data Exchanged between BLE and MCU (2 --> RTPS and NTB)
#define NOD_SENSORS 2				// Number of Data Exchanged between HTS and MCU (2 --> Temperature and Humidity)

#define LPTIM_TIME_UNIT 291   		// 291 = 1 sec  @ 37 Khz LSI Clock and Prescaler = 128
#define RTPS 20						// 291 = 1 sec  @ 37 Khz LSI Clock and Prescaler = 128
#define NTB 6						// NTB = Number of Transmitted Beacons
#define TADV_MIN 30         		// TADV_MIN*SF ~ 100 msec -- Minimum Advertising Interval
#define TFALL_MIN 15         		// TFALL_MIN ~ 50 msec

#define TRISE_TH 2					// TRISE*SF

#define Pulse_WB 0					// 29 = 0.1 sec @ 37 Khz LSI Clock and Prescaler = 128
#define SF 1000/LPTIM_TIME_UNIT		// 1000/291 = 3.45 msec  @ 37 Khz LSI Clock and Prescaler = 128

#define ARR 0xFFFF					// Auto Reload Register Value --> The system goes in timeout after ARR*128/37000 seconds.
#define NOVF 18				        // Number of Overflows

#define PROTO
#define VAR

typedef enum {WAIT, SEND, DARK} mode_state;

extern LPTIM_HandleTypeDef hlptim1;
extern UART_HandleTypeDef hlpuart1;
extern DMA_HandleTypeDef hdma_lpuart1_rx;

#if COMPARATOR
extern COMP_HandleTypeDef hcomp1;
#endif

#if PVD
extern PWR_PVDTypeDef sConfigPVD;
#endif

extern mode_state ms;
extern uint8_t Tx_En_Count;
extern uint32_t Pulse;

extern uint8_t *transmission_params;	// Pointer as required by the function to receive Data from BLE through UART

void My_While(void);
void My_Main(void);
void HAL_PWR_PVDCallback(void);

