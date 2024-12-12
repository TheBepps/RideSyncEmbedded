#include "stdio.h"
#include "stdlib.h"
#include "hts221.h"
#include "i2c.h"

#define OTAP 1
#define SENSORS 1

#define FROM_FLASH 1
#define NTB_CONTROL 1
#define TADV_CONTROL 1

#define HTS221_ODR 21       // HTS221 Output data rate in ms.
							// (Mimimum time to wait to get reliable data from the HTS221 Sensor)

#define NOD_BLE 2			// Number of Data Exchanged between BLE and MCU (2 --> RTPS and NTB)
#define NOD_SENSORS 8		// Number of bytes Exchanged between HTS and MCU ( 4 Temperature and 4 Humidity)

#define LPTIM_TIME_UNIT 289 // 289 = 1 sec  @ 37 Khz LSI Clock and Prescaler = 128
#define RTPS 20				// RTPS = Radio Transmission Period in Seconds
#define NTB 6				// NTB = Number of Transmitted Beacons

#define TADV_MIN 31         // TADV_MIN*SF  ~ 90 msec -- Minimum Advertising Interval
#define TFALL_MIN 3         // TFALL_MIN*SF ~ 10 msec -- Minimum Fall Time

#define SF 1000/LPTIM_TIME_UNIT // 1000/291 = 3.45 msec  @ 37 Khz LSI Clock and Prescaler = 128
#define ARR 0xFFFF				// Auto Reload Register Value --> The system goes in timeout after ARR*128/37000 seconds.

#define PROTO
#define VAR

typedef enum {WAIT, SEND} mode_state;

extern LPTIM_HandleTypeDef hlptim1;

extern uint8_t *transmission_params; // Pointer as required by the function to receive Data from BLE through I2C

void My_Write_Mcu_Flash(void);

void My_While(void);
void My_Main(void);

//void My_Write_Mcu_Flash(void)__attribute__((optimize("-O0")));
//static void My_Delay(uint16_t Delay)__attribute__((optimize("-O0")));

