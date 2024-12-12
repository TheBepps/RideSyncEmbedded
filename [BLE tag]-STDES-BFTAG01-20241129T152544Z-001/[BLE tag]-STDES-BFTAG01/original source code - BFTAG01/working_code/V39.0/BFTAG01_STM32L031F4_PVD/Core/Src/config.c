
#include "config.h"

#ifdef VAR
mode_state ms = DARK;
uint8_t  ntb = 0;				// ntb = Number of Transmitted Beacons
uint16_t rtp = 0;				// rtp = Radio Transmission Period in Counter Timer Units (uint16_t to be compatible with the LPTIM_ARR size)
uint8_t  rtps = 0;				// rtps = Radio Transmission Period in seconds (e.g. 20 means 20 seconds)
uint8_t  cnt_ntb = 0;   		// cnt_ntb = Counts the Number of Transmitted Beacons
uint8_t  *transmission_params;	// Pointer as required by the function to receive Data from BLE through UART
uint8_t  pvdi = 0;				// Flag for the interrupt from pvd
PWR_PVDTypeDef sConfigPVD;
uint8_t  cnt_ovf = 0;
uint16_t t_ble_on_dark = 0;
uint32_t vh = PWR_PVDLEVEL_5;  // Vh = 2.8V
uint32_t vhi = PWR_PVDLEVEL_5; // Vh = 2.8V
uint8_t cdark = 0;
#endif

#ifdef PROTO
static void My_Ble_On_Send(void);
static void My_Ble_On_Wait(void);
static void My_Ble_Off(void);
static void My_Enter_Stop_Mode_WFI(void);
static void My_Enter_Stop_Mode_WFI_PVD(void);
static void My_Enter_Stop_Mode_WFI_BTH(void);
static void My_Tadv_Ctrl(void);
static void My_Set_Vh(void);
static void My_Set_Vh_IT(void);
static void My_Set_Vhi_IT(void);
static void My_Set_Vh(void);
static void My_Set_Vhi_IT(void);
static void My_Set_Vh_Dn(void);
static void My_Set_Vh_Up_IT(void);
#endif

#if SENSORS
uint8_t *sensor_data;			// Pointer to get sensor data
#endif

#if TRISE_CONTROL
uint16_t trise = 0;
#endif

#if TADV_CONTROL
uint16_t tadv = 0;
#endif

#if OTAP	// PROGRAMMING THE EEPROM
static void My_Write_Mcu_Flash(void){
	HAL_Delay(6);															 // Delay to allow the settling of the BLE_WFE signal
	if (HAL_GPIO_ReadPin(BLE_WFE_GPIO_Port, BLE_WFE_Pin) == 1) {		     // 1) Check if BLE is ready to send Data
		__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
		HAL_UART_DeInit(&hlpuart1);
		HAL_UART_Init(&hlpuart1);
		HAL_UART_MspInit(&hlpuart1);									     // 2) Activate Uart as it is needed only in this case
		HAL_FLASHEx_DATAEEPROM_Unlock();								     // 3) Unlock The Flash
		HAL_UART_Receive_DMA(&hlpuart1, transmission_params, NOD_BLE+2);     // 4) Receiving Data through LPUART & DMA
		HAL_GPIO_TogglePin(BLE_TXE_GPIO_Port, BLE_TXE_Pin);				     // 5) ACK to BLE end of received data
		while (HAL_GPIO_ReadPin(BTH_GPIO_Port, BTH_Pin) == 0);               // 6) Waiting for END of Transmission from BLE
		HAL_FLASHEx_DATAEEPROM_Lock();									     // 7) Lock the Flash
		HAL_NVIC_SystemReset();											     // 8) Reset the MCU
	}
}
#endif

static void My_Ble_On_Send(void){
	HAL_GPIO_WritePin(BLE_TXE_GPIO_Port, BLE_TXE_Pin, GPIO_PIN_SET);    // Enable the BLE to transmit
	HAL_GPIO_WritePin(VBLUE_GPIO_Port,   VBLUE_Pin,   GPIO_PIN_SET);   	// Turn on the   BLE IC
	HAL_GPIO_WritePin(VBLUE1_GPIO_Port,  VBLUE1_Pin,  GPIO_PIN_SET);   	// Turn on the   BLE IC
	HAL_GPIO_WritePin(VBLUE2_GPIO_Port,  VBLUE2_Pin,  GPIO_PIN_SET);   	// Turn on the   BLE IC
	HAL_Delay(1); 														// Reset must be released at least 30 usec after the VBLUE is up and stable
	HAL_GPIO_WritePin(BLE_RST_GPIO_Port, BLE_RST_Pin, GPIO_PIN_SET);  	// Un-Reset the  BLE IC
}

static void My_Ble_On_Wait(void){
	HAL_GPIO_WritePin(BLE_TXE_GPIO_Port, BLE_TXE_Pin, GPIO_PIN_RESET);  // Enable the BLE to transmit
	HAL_GPIO_WritePin(VBLUE_GPIO_Port,   VBLUE_Pin,   GPIO_PIN_SET);    // Turn on the   BLE IC
	HAL_GPIO_WritePin(VBLUE1_GPIO_Port,  VBLUE1_Pin,  GPIO_PIN_SET);    // Turn on the   BLE IC
	HAL_GPIO_WritePin(VBLUE2_GPIO_Port,  VBLUE2_Pin,  GPIO_PIN_SET);    // Turn on the   BLE IC
	HAL_Delay(1); 														// Reset must be released at least 30 usec after the VBLUE is up and stable
	HAL_GPIO_WritePin(BLE_RST_GPIO_Port, BLE_RST_Pin, GPIO_PIN_SET);  	// Un-Reset the  BLE IC
}

static void My_Ble_Off(void){
	HAL_GPIO_WritePin(BLE_TXE_GPIO_Port,   BLE_TXE_Pin,   GPIO_PIN_RESET);	// Reset the    BLE_TXE (Back to Harvest)
	HAL_GPIO_WritePin(VBLUE_GPIO_Port,     VBLUE_Pin,     GPIO_PIN_RESET); 	// Turn OFF the BLE (Back to Harvest)
	HAL_GPIO_WritePin(VBLUE1_GPIO_Port,    VBLUE1_Pin,    GPIO_PIN_RESET); 	// Turn OFF the BLE (Back to Harvest)
	HAL_GPIO_WritePin(VBLUE2_GPIO_Port,    VBLUE2_Pin,    GPIO_PIN_RESET); 	// Turn OFF the BLE (Back to Harvest)
	HAL_GPIO_WritePin(BLE_RST_GPIO_Port,   BLE_RST_Pin,   GPIO_PIN_RESET); 	// Reset the    BLE (Back to Harvest)
}

static void My_Enter_Stop_Mode_WFI(void){
	HAL_NVIC_EnableIRQ(PVD_IRQn);
	HAL_NVIC_EnableIRQ(LPTIM1_IRQn);
	HAL_NVIC_ClearPendingIRQ(PVD_IRQn);
	HAL_NVIC_ClearPendingIRQ(EXTI4_15_IRQn);
	HAL_NVIC_ClearPendingIRQ(LPTIM1_IRQn);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_PVDO);									// CLear Flag
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);										// CLear Flag
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI); 	// Device in Stop Mode and WFI from PVD and LPTIM1
}

static void My_Enter_Stop_Mode_WFI_PVD(void){
	HAL_NVIC_EnableIRQ(PVD_IRQn);
	HAL_NVIC_ClearPendingIRQ(PVD_IRQn);
	HAL_NVIC_ClearPendingIRQ(EXTI4_15_IRQn);
	HAL_NVIC_ClearPendingIRQ(LPTIM1_IRQn);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_PVDO);									// CLear Flag
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);										// CLear Flag
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI); 	// Device in Stop Mode and WFI from PVD and LPTIM1
}

static void My_Enter_Stop_Mode_WFI_BTH(void){
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI); // Device in Stop Mode and WFI from PVD and LPTIM1
}

static void My_Tadv_Ctrl(void){
	uint16_t tadv_del = 0;
	tadv = HAL_LPTIM_ReadCounter(&hlptim1) - tadv;	// Measure Advertising Time After Transmission
	if (tadv <= TADV_MIN) {							// Check compliancy with the Minimum Adertising Time
		tadv_del = HAL_LPTIM_ReadCounter(&hlptim1);
		while(HAL_LPTIM_ReadCounter(&hlptim1) < tadv_del + TADV_MIN); // Set tadv Delay
	}
	else {
		tadv_del = HAL_LPTIM_ReadCounter(&hlptim1);
		while(HAL_LPTIM_ReadCounter(&hlptim1) < tadv_del + TFALL_MIN); // Set tadv Delay
	}
}

static void My_Set_Vh(void){
	if (cdark == 0){
		My_Set_Vh_Up_IT();								// Increase Vh Threshold and Interrupt for Rising vdd
	}
	else {
		My_Set_Vh_Dn(); 								// Decrease the Threshold and set Normal;
		if ((__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) == 1)) {	// Check if Vstor is below the Decreased Threshold
			My_Set_Vh_IT();								// Set PVD to decreased Threshold and set Interrupt for Rising vdd
		}
		else{
			My_Set_Vhi_IT();							// Keep the Initial Threshold and set Interrupt for Rising vdd
		}
	}
}

static void My_Set_Vh_IT(void){
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_PVDO);	    // CLear Flag
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);			// CLear Flag
	sConfigPVD.PVDLevel = vh;          	        // Set PVD Threshold vh
	sConfigPVD.Mode = PWR_PVD_MODE_IT_FALLING;	// Set PVD to detect Rising Vdd and set Interrupt
	HAL_PWR_ConfigPVD(&sConfigPVD);			   	// Configure PVD
}

static void My_Set_Vhi_IT(void){
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_PVDO);	    // CLear Flag
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);			// CLear Flag
	sConfigPVD.PVDLevel = vhi;          	    // Set PVD Threshold  to vhi
	sConfigPVD.Mode = PWR_PVD_MODE_IT_FALLING;	// Set PVD to detect Rising Vdd and set Interrupt
	HAL_PWR_ConfigPVD(&sConfigPVD);			   	// Configure PVD
}

static void My_Set_Vh_Dn(void){
	if (sConfigPVD.PVDLevel == PWR_PVDLEVEL_5){
		vhi = PWR_PVDLEVEL_5;
		vh = PWR_PVDLEVEL_4; // Vh = 2.6V
	}
	else if (sConfigPVD.PVDLevel == PWR_PVDLEVEL_4){
		vhi = PWR_PVDLEVEL_4;
		vh = PWR_PVDLEVEL_3; // Vh = 2.4V
	}
	else if (sConfigPVD.PVDLevel == PWR_PVDLEVEL_3){
		vhi = PWR_PVDLEVEL_3;
		vh = PWR_PVDLEVEL_2; // Vh = 2.2V
	}
	else if (sConfigPVD.PVDLevel == PWR_PVDLEVEL_2){
		vhi = PWR_PVDLEVEL_2;
		vh = PWR_PVDLEVEL_2; // Vh = 2.2V
	}
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_PVDO);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	sConfigPVD.PVDLevel = vh;
	sConfigPVD.Mode = PWR_PVD_MODE_NORMAL;
	HAL_PWR_ConfigPVD(&sConfigPVD);
}

static void My_Set_Vh_Up_IT(void){
	if (vh == PWR_PVDLEVEL_2){
		sConfigPVD.PVDLevel = PWR_PVDLEVEL_3;
		vh = PWR_PVDLEVEL_3; // Vh = 2.4V
	}
	else if (vh == PWR_PVDLEVEL_3){
		sConfigPVD.PVDLevel = PWR_PVDLEVEL_4;
		vh = PWR_PVDLEVEL_4; // Vh = 2.6V
	}
	else if (vh == PWR_PVDLEVEL_4){
		sConfigPVD.PVDLevel = PWR_PVDLEVEL_5;
		vh = PWR_PVDLEVEL_5; // Vh = 2.8V
	}
	else if (vh == PWR_PVDLEVEL_5){
		sConfigPVD.PVDLevel = PWR_PVDLEVEL_5;
		vh = PWR_PVDLEVEL_5; // Vh = 2.8V
	}
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_PVDO);		// CLear Flag
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);			// CLear Flag
	sConfigPVD.Mode = PWR_PVD_MODE_IT_FALLING;	// Set PVD to detect Rising Vdd
	HAL_PWR_ConfigPVD(&sConfigPVD);			   	// Configure PVD
}

void My_Main(void){

	HAL_GPIO_WritePin(VHTS_GPIO_Port, VHTS_Pin, GPIO_PIN_RESET);

	sConfigPVD.PVDLevel = PWR_PVDLEVEL_5;      	// Set PVD Threshold to 3.0 Volt
	sConfigPVD.Mode = PWR_PVD_MODE_IT_FALLING;	// Set PVD tp detect Rising Vdd
	HAL_PWR_ConfigPVD(&sConfigPVD);			   	// Configure PVD
	HAL_PWR_EnablePVD();	          			// Enable PVD

#if SENSORS
	sensor_data = (uint8_t*)malloc(NOD_SENSORS*sizeof(uint8_t));
	*sensor_data = 27;		 // Temperature
	*(sensor_data + 1) = 50; // Humidity
#endif

#if FROM_FLASH
	transmission_params = (uint8_t*)DATA_EEPROM_BASE; 											   // DATA_EEPROM_BASE = 0x08080000 Address of the 1st Byte of the EEPPROM MEMORY

	if (*((uint32_t*)transmission_params) == 0) {   										       // 1) Check if the 1st Word of the EEPROM Memory is Empty
		HAL_FLASHEx_DATAEEPROM_Unlock();                                						   // 2) UnLock the EEPROM
		HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, DATA_EEPROM_BASE, NTB);         // 3) Program the 1st BYTE of the EEPROM with the NTB value
		HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, DATA_EEPROM_BASE + 1, RTPS);    // 4) Program the 2nd BYTE of the EEPROM with the RTPS value
		HAL_FLASHEx_DATAEEPROM_Lock();                                							   // 5) Lock the EEPROM
	}
	else {
		ntb  = *(transmission_params);      // Extracting ntb From Flash
		rtps = *(transmission_params + 1);	// Extracting rtps From Flash
	}

#else
	rtps = RTPS;
	ntb  = NTB;
#endif

	HAL_UART_MspDeInit(&hlpuart1);           // Deactivate LPUART to save energy
	HAL_LPTIM_Counter_Start(&hlptim1, ARR);  // Start the LPTIM1 Counter
}

void My_While(void){

	HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);

	sConfigPVD.PVDLevel = PWR_PVDLEVEL_6;      	  // Set PVD Threshold to 3.0 Volt
	sConfigPVD.Mode = PWR_PVD_MODE_NORMAL;	      // Set PVD Normal
	HAL_PWR_ConfigPVD(&sConfigPVD);			   	  // Configure PVD

	if(__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) == 0){   // if Vstor is > 3.0 Volt
		My_Ble_On_Wait();			   			  // Turn BLE ON and do not Transmit
		sConfigPVD.PVDLevel = PWR_PVDLEVEL_4;     // Set PVD Threshold to 2.6 Volt
		sConfigPVD.Mode = PWR_PVD_MODE_IT_RISING; // Set PVD to detect Falling Vdd
		HAL_PWR_ConfigPVD(&sConfigPVD);			  // Configure PVD
		My_Enter_Stop_Mode_WFI_PVD();  			  // Enter Stop Mode and WFI from PVD
		My_Ble_Off();				   			  // Turn BLE OFF
	}

	if ((__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) == 1) && (__HAL_PWR_GET_FLAG(PWR_CSR_VREFINTRDYF) == 1)) { // if Vstor < 3.0 Volt and Internal Reference is Ready
		My_Set_Vh();
		pvdi = 0;		            			  // Reset pvdi
		My_Enter_Stop_Mode_WFI();   			  // Enter Stop Mode and WFI from PVD or LPTIM
	}

	if (pvdi == 0){ 											 // If pvdi = 0
		ms = DARK;												 // Set Mode State to DARK
		if (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) == 1) {            // Check if Vstor < 3.0V
			if (++cnt_ovf >= NOVF){ 					 		 // If Overflow
				cnt_ovf = 0;							 		 // Reset Overflow Counter
				cdark++;								 		 // Increase cdark
				My_Ble_On_Send(); 						 		 // Turn On BLE to Transmit
				t_ble_on_dark = HAL_LPTIM_ReadCounter(&hlptim1); // Time Stamp Transmission Window
				while(HAL_LPTIM_ReadCounter(&hlptim1) < (t_ble_on_dark + ntb*TADV_MIN)); // Time Window for BLE transmission
				My_Ble_Off();	    					         // Turn Off BLE
			}
		}
	}
	else { // Vstor > Vh
		cdark = 0;
		if (ms == DARK){
			ms = SEND;
		}
		if (ms == SEND){
			if (cnt_ntb++ < ntb){						   // Check Number of Transmitted Beacons
				cnt_ovf = 0;							   // Reset cnt_ovf
				My_Ble_On_Send();					       // Turn On BLE to Transmit
				My_Enter_Stop_Mode_WFI_BTH();		       // Enter Stop Mode and WFI from BTH
				My_Tadv_Ctrl();					           // Controls Minimum Advertising Time REquirement
				My_Ble_Off();						       // Turn Off BLE
				tadv = HAL_LPTIM_ReadCounter(&hlptim1);    // Time Stamp tadv
			}
			else{
				ms  = WAIT;
				cnt_ovf = 0;							   // Reset cnt_ovf
				HAL_NVIC_EnableIRQ(LPTIM1_IRQn);		   // Enable LPTIM1 Interrupts
				HAL_LPTIM_Counter_Stop(&hlptim1);          // Stop  LPTIM1 Counter
				HAL_LPTIM_Counter_Start_IT(&hlptim1, ARR); // Start LPTIM1 Counter
				cnt_ntb = 0;							   // Reset cnt_ntb
				rtp = (uint16_t)rtps*LPTIM_TIME_UNIT;	   // Set rtp
				My_Ble_On_Wait();				       	   // Turn BLE ON and does not Transmit
				My_Enter_Stop_Mode_WFI_BTH();	           // Enter Stop Mode and WFI from BTH
				My_Ble_Off();					           // Turn BLE OFF
				tadv = HAL_LPTIM_ReadCounter(&hlptim1);    // Time Stamp tadv
			}
		}
		else if (ms == WAIT){
			if (HAL_LPTIM_ReadCounter(&hlptim1) < rtp){
				tadv = HAL_LPTIM_ReadCounter(&hlptim1);    // Time Stamp of Advertising Time
				cnt_ovf = 0;							   // Reset cnt_ovf
				My_Ble_On_Wait();				       	   // Turn BLE ON and does not Transmit
				My_Write_Mcu_Flash();			       	   // Write MCU Flash
				My_Enter_Stop_Mode_WFI_BTH();	       	   // Enter Stop Mode and WFI from BTH
				My_Ble_Off();					       	   // Turn BLE OFF
				tadv = HAL_LPTIM_ReadCounter(&hlptim1);    // Time Stamp tadv
			}
			else{
				ms = SEND;
				cnt_ovf = 0;							   // Reset cnt_ovf
				HAL_NVIC_EnableIRQ(LPTIM1_IRQn);		   // Enable LPTIM1 Interrupts
				HAL_LPTIM_Counter_Stop(&hlptim1);          // Stop  LPTIM1 Counter
				HAL_LPTIM_Counter_Start_IT(&hlptim1, ARR); // Start LPTIM1 Counter
				My_Ble_On_Wait();					       // Turn On BLE
				My_Enter_Stop_Mode_WFI_BTH();		       // Enter Stop Mode and WFI from BTH
				My_Ble_Off();						       // Turn Off BLE
				tadv = HAL_LPTIM_ReadCounter(&hlptim1);    // Time Stamp tadv
			}
		}
	}

#if SENSORS
	if (ms == SEND) {                 // If the BLE is in Send Mode
		HAL_UART_MspInit(&hlpuart1);  // Activate LPHUART
		HAL_UART_Transmit(&hlpuart1, sensor_data, NOD_SENSORS, (uint32_t)(SF*TAFV_MIN)); // Transmit Data
		HAL_UART_MspDeInit(&hlpuart1);
	}
#endif

}

