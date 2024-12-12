
#include "config.h"

#ifdef VAR
PWR_PVDTypeDef sConfigPVD;
mode_state ms = SEND;

uint16_t rtp = 0;				// rtp = Radio Transmission Period in Counter Timer Units (uint16_t to be compatible with the LPTIM_ARR size)
uint8_t  ntb = 0;				// ntb = Number of Transmitted Beacons
uint8_t  rtps = 0;				// rtps = Radio Transmission Period in seconds (e.g. 20 means 20 seconds)
uint8_t  cnt_ntb = 0;   		// cnt_ntb = Counts the Number of Transmitted Beacons
uint8_t  *transmission_params;	// Pointer as required by the function to receive Data from BLE through I2C
uint16_t tadv = 0;				// Advertising Time
uint16_t tw = 0;			    // Timing Window (BLE Transmission Timing Window)
uint8_t cnt_wfe = 0; 			// Count BLE_WFE event (Falling Edge)

#if SENSORS
uint16_t *hts221_temp_adc_cal_value;     // T0_OUT
uint16_t *hts221_temp_deg_cal_value;     // T0_Degc
float *hts221_temp_slope;                // Sensor Temperature Calibration Slope

uint16_t *hts221_humidity_adc_cal_value; // H0_T0_OUT
uint16_t *hts221_humidity_cal_value;     // H0_RH
float *hts221_humidity_slope;            // Sensor Humidity Calibration Slope

uint16_t hts221_temp_data;               // ADC Measured Temperature Value
uint16_t hts221_humidity_data;	         // ADC Measured Humidity Value
float sensor_data[4];                    // Calculated and Measured Temperature and Humidity Value
#endif

#endif

#if OTAP	// PROGRAMMING THE EEPROM
uint32_t hal_error = 0;
#if My_HAL
HAL_StatusTypeDef HAL_I2C_Slave_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef dmaxferstatus;

	if (hi2c->State == HAL_I2C_STATE_READY)
	{
		if ((pData == NULL) || (Size == 0U))
		{
			hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
			return  HAL_ERROR;
		}
		/* Process Locked */
		__HAL_LOCK(hi2c);

		hi2c->State       = HAL_I2C_STATE_BUSY_RX;
		hi2c->Mode        = HAL_I2C_MODE_SLAVE;
		hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

		/* Prepare transfer parameters */
		hi2c->pBuffPtr    = pData;
		hi2c->XferCount   = Size;
		hi2c->XferSize    = hi2c->XferCount;
		hi2c->XferOptions = I2C_NO_OPTION_FRAME;
		hi2c->XferISR     = I2C_Slave_ISR_DMA;

		if (hi2c->hdmarx != NULL)
		{
			/* Set the I2C DMA transfer complete callback */
			hi2c->hdmarx->XferCpltCallback = I2C_DMASlaveReceiveCplt;

			/* Set the DMA error callback */
			hi2c->hdmarx->XferErrorCallback = I2C_DMAError;

			/* Set the unused DMA callbacks to NULL */
			hi2c->hdmarx->XferHalfCpltCallback = NULL;
			hi2c->hdmarx->XferAbortCallback = NULL;

			/* Enable the DMA channel */
			dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmarx, (uint32_t)&hi2c->Instance->RXDR, (uint32_t)pData, hi2c->XferSize);
		}
		else
		{
			/* Update I2C state */
			hi2c->State     = HAL_I2C_STATE_LISTEN;
			hi2c->Mode      = HAL_I2C_MODE_NONE;

			/* Update I2C error code */
			hi2c->ErrorCode |= HAL_I2C_ERROR_DMA_PARAM;

			/* Process Unlocked */
			__HAL_UNLOCK(hi2c);

			return HAL_ERROR;
		}

		if (dmaxferstatus == HAL_OK)
		{
			/* Enable Address Acknowledge */
			hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

			/* Process Unlocked */
			__HAL_UNLOCK(hi2c);

			/* Note : The I2C interrupts must be enabled after unlocking current process
                to avoid the risk of I2C interrupt handle execution before current
                process unlock */
			/* Enable ERR, STOP, NACK, ADDR interrupts */
			I2C_Enable_IRQ(hi2c, I2C_XFER_LISTEN_IT);

			/* Enable DMA Request */
			hi2c->Instance->CR1 |= I2C_CR1_RXDMAEN;
		}
		else
		{
			/* Update I2C state */
			hi2c->State     = HAL_I2C_STATE_LISTEN;
			hi2c->Mode      = HAL_I2C_MODE_NONE;

			/* Update I2C error code */
			hi2c->ErrorCode |= HAL_I2C_ERROR_DMA;

			/* Process Unlocked */
			__HAL_UNLOCK(hi2c);

			return HAL_ERROR;
		}

		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}
#endif

void My_Write_Mcu_Flash(void){
	if (HAL_GPIO_ReadPin(BLE_WFE_GPIO_Port, BLE_WFE_Pin) == 0){            // Check if BLE is ready to send Data
		__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);     // Raise Regulator Voltage to bias Flash Memory
		HAL_I2C_DeInit(&hi2c1);											   // De Initialize I2C
		hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;				   // Disable Clock Stretching
		HAL_GPIO_WritePin(VHTS_GPIO_Port,    VHTS_Pin,    GPIO_PIN_SET);   // Turn on HTS221
		HAL_I2C_Init(&hi2c1);											   // Initialize I2C
		HAL_FLASHEx_DATAEEPROM_Unlock();								   // Unlock The Flash
		hal_error = My_I2C_Slave_Receive(&hi2c1, transmission_params, NOD_BLE, 20, BLE_TXE_GPIO_Port, BLE_TXE_Pin);
		while (HAL_GPIO_ReadPin(BTH_GPIO_Port, BTH_Pin) == 0);             // Waiting for END of Transmission from BLE
		HAL_FLASHEx_DATAEEPROM_Lock();									   // Lock the Flash
		HAL_GPIO_WritePin(VHTS_GPIO_Port,    VHTS_Pin,    GPIO_PIN_RESET); // Turn off HTS221
		HAL_NVIC_SystemReset();											   // Reset the MCU
	}
}
#endif

#if SENSORS // If SENSORS ARE USED
static void My_Sensors_Send_Data(void){      // Transfers the acquired data from the MCU to the BLE
	HAL_I2C_MspInit(&hi2c1);				 // Initialize I2C
	HAL_I2C_Slave_Transmit(&hi2c1, (uint8_t*)sensor_data, sizeof(sensor_data), 20); // 3) I2C Transmit
	HAL_I2C_MspDeInit(&hi2c1);               // De-init I2C
}

static void My_hts221_config(void){ // Reads the calibration value and Memorize in the MCU Flash Memory.

	if (*((uint32_t*)hts221_temp_adc_cal_value) == 0 && *((uint32_t*)hts221_temp_slope) == 0) {                          // 1) Check if the 1st Word of the EEPROM Memory is Empty
		HAL_FLASHEx_DATAEEPROM_Unlock();                                						                         // 2) UnLock the EEPROM
		hts221_read_temperature_cal_values(hts221_temp_adc_cal_value, hts221_temp_deg_cal_value, hts221_temp_slope);     // 3) Read Temperature Calibration Values
		HAL_FLASHEx_DATAEEPROM_Lock();                                							                         // 4) Lock the EEPROM
	}

	if (*((uint32_t*)hts221_humidity_adc_cal_value) == 0 && *((uint32_t*)hts221_humidity_slope) == 0) {   	              // 1) Check if the 1st Word of the EEPROM Memory is Empty
		HAL_FLASHEx_DATAEEPROM_Unlock();                                						                          // 2) UnLock the EEPROM
		hts221_read_humidity_cal_values(hts221_humidity_adc_cal_value, hts221_humidity_cal_value, hts221_humidity_slope); // Read the Humidity Calibration Values
		HAL_FLASHEx_DATAEEPROM_Lock();                                							                          // 5) Lock the EEPROM
	}
}

static void My_hts221(void){                                     // Reads the hts221 data
	hts221_init(); 											     // Initialize HTS221
	HAL_Delay(HTS221_ODR);									     // Very Critical Delay (Wait that both the sensors are ready --> See note on DRDY at page 24 of the datasheet)
	hts221_read_temperature(&hts221_temp_data);                  // Read temperature Measurement
	hts221_read_humidity(&hts221_humidity_data);                 // Read Humidity Measurement
	sensor_data[0] = (*hts221_temp_deg_cal_value + *hts221_temp_slope*(hts221_temp_data - *hts221_temp_adc_cal_value))/8;	          // Temperature in Celsius
	sensor_data[1] = (*hts221_humidity_cal_value + *hts221_humidity_slope*(hts221_humidity_data - *hts221_humidity_adc_cal_value))/2; // Relative Humidity in %
}

#endif

static void My_BLE_WFE_conf(uint32_t mode){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = BLE_WFE_Pin;
	GPIO_InitStruct.Mode = mode;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BLE_WFE_GPIO_Port, &GPIO_InitStruct);
}

static void My_VBLUE_conf(uint32_t mode){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = mode;
	GPIO_InitStruct.Pull = GPIO_NOPULL;

	GPIO_InitStruct.Pin = VBLUE_Pin;
	HAL_GPIO_Init(VBLUE_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = VBLUE1_Pin;
	HAL_GPIO_Init(VBLUE1_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = VBLUE2_Pin;
	HAL_GPIO_Init(VBLUE2_GPIO_Port, &GPIO_InitStruct);
}

static void My_Power_Management(uint8_t ON, uint8_t TX, uint8_t HTS){ // Turns On Sensors and BLE with transmission
	if (ON == 1){
		My_BLE_WFE_conf(GPIO_MODE_IT_FALLING); // Configure BLE_WFE GPIO as External Interrupt Falling
		My_VBLUE_conf(GPIO_MODE_OUTPUT_PP);    // Configure VBLUE, VBLUE1 and VBLUE2 in Output PP
		if (TX == 1){
			HAL_GPIO_WritePin(BLE_TXE_GPIO_Port, BLE_TXE_Pin, GPIO_PIN_SET);   // Enable the BLE to transmit
		}
		else {
			HAL_GPIO_WritePin(BLE_TXE_GPIO_Port, BLE_TXE_Pin, GPIO_PIN_RESET); //Disable The BLE to transmit
		}
		if (HTS == 1){
			HAL_GPIO_WritePin(VHTS_GPIO_Port,    VHTS_Pin,    GPIO_PIN_SET);   // Turn on HTS221
		}
		else{
			HAL_GPIO_WritePin(VHTS_GPIO_Port,    VHTS_Pin,    GPIO_PIN_RESET); // Turn off HTS221
		}
		HAL_NVIC_ClearPendingIRQ(EXTI2_3_IRQn);
		HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
		HAL_NVIC_ClearPendingIRQ(EXTI2_3_IRQn);
		HAL_GPIO_WritePin(VBLUE_GPIO_Port,   VBLUE_Pin,   GPIO_PIN_SET);   	   // Turn on the BLE IC
		HAL_GPIO_WritePin(VBLUE1_GPIO_Port,  VBLUE1_Pin,  GPIO_PIN_SET);   	   // Turn on the BLE IC
		HAL_GPIO_WritePin(VBLUE2_GPIO_Port,  VBLUE2_Pin,  GPIO_PIN_SET);   	   // Turn on the BLE IC
		HAL_Delay(1); 														   // Reset must be released at least 30 usec after the VBLUE is up and stable
		HAL_GPIO_WritePin(BLE_RST_GPIO_Port, BLE_RST_Pin, GPIO_PIN_SET);  	   // Un-Reset the  BLE IC
	}
	else{
		cnt_wfe = 0;
		My_BLE_WFE_conf(GPIO_MODE_ANALOG);  // Configure BLE_WFE GPIO as Input
		My_VBLUE_conf(GPIO_MODE_ANALOG);   // Configure VBLUE, VBLUE1 and VBLUE2 in Output PP
		HAL_GPIO_WritePin(BLE_TXE_GPIO_Port,   BLE_TXE_Pin,   GPIO_PIN_RESET); // Turn OFF the BLE_TXE signal (Back to Harvest)
		HAL_GPIO_WritePin(VBLUE_GPIO_Port,     VBLUE_Pin,     GPIO_PIN_RESET); // Turn OFF the BLE (Back to Harvest)
		HAL_GPIO_WritePin(VBLUE1_GPIO_Port,    VBLUE1_Pin,    GPIO_PIN_RESET); // Turn OFF the BLE (Back to Harvest)
		HAL_GPIO_WritePin(VBLUE2_GPIO_Port,    VBLUE2_Pin,    GPIO_PIN_RESET); // Turn OFF the BLE (Back to Harvest)
		HAL_GPIO_WritePin(BLE_RST_GPIO_Port,   BLE_RST_Pin,   GPIO_PIN_RESET); // Reset the    BLE (Back to Harvest)
		HAL_GPIO_WritePin(VHTS_GPIO_Port,      VHTS_Pin,      GPIO_PIN_RESET); // Turn OFF VHTS
		HAL_GPIO_DeInit(BLE_WFE_GPIO_Port,     BLE_WFE_Pin);
	}
}

static void My_Enter_Stop_Mode_WFI(void){
	HAL_NVIC_EnableIRQ(PVD_IRQn);
	HAL_NVIC_ClearPendingIRQ(PVD_IRQn);
	HAL_NVIC_ClearPendingIRQ(EXTI4_15_IRQn);
	HAL_NVIC_ClearPendingIRQ(EXTI2_3_IRQn);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_PVDO);								// CLear Flag
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);									// CLear Flag
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI); // Device in Stop Mode and WFI from PVD and LPTIM1
}

static void My_Tadv_Ctrl(void){
	if (cnt_ntb > 0){
		uint16_t tadv_del = 0;
		if (HAL_LPTIM_ReadCounter(&hlptim1) > tadv){
			tadv = HAL_LPTIM_ReadCounter(&hlptim1) - tadv;	// Measure Advertising Time After Transmission
		}
		else
		{
			tadv = (HAL_LPTIM_ReadCounter(&hlptim1) - (ARR - tadv));
		}
		if (tadv <= TADV_MIN) {							// Check compliancy with the Minimum Adertising Time
			tadv_del = HAL_LPTIM_ReadCounter(&hlptim1);
			while(HAL_LPTIM_ReadCounter(&hlptim1) < (tadv_del + TADV_MIN - tadv)); // Set tadv Delay
		}
		else {
			tadv_del = HAL_LPTIM_ReadCounter(&hlptim1);
			while(HAL_LPTIM_ReadCounter(&hlptim1) < tadv_del + TFALL_MIN); // Set tadv Delay
		}
	}
}

static void My_Tadv_Meas(void){
	if (cnt_ntb == 1){
		tw = HAL_LPTIM_ReadCounter(&hlptim1);
	}
	if (cnt_ntb == ntb){
		tw = HAL_LPTIM_ReadCounter(&hlptim1) - tw;
		sensor_data[2] = *((uint8_t*)&(tw)+1); // High byte
		sensor_data[3] = *((uint8_t*)&(tw)+0); // Low byte
	}
}

static void My_Set_PVD(uint32_t PWR_PVDLEVEL, uint32_t PWR_PVD_MODE_IT){
	sConfigPVD.PVDLevel = PWR_PVDLEVEL; // Set PVD Threshold
	sConfigPVD.Mode = PWR_PVD_MODE_IT;	// Set PVD to detect Rising Vdd and set Interrupt
	HAL_PWR_ConfigPVD(&sConfigPVD);		// Configure PVD
}

void My_Main(void){

	HAL_I2C_MspDeInit(&hi2c1); // Deinitialize I2C

	My_Set_PVD(PWR_PVDLEVEL_5, PWR_PVD_MODE_IT_FALLING); // Set PVD at level 5 and rising vdd
	HAL_PWR_EnablePVD();	          			         // Enable PVD

#if SENSORS
	hts221_temp_adc_cal_value = (uint16_t*)(DATA_EEPROM_BASE + 4); // (0x08080004)
	hts221_temp_deg_cal_value = (uint16_t*)(DATA_EEPROM_BASE + 8); // (0x0808000C)
	hts221_temp_slope         = (float*)(DATA_EEPROM_BASE + 12);   // (0x08080010)

	hts221_humidity_adc_cal_value = (uint16_t*)(DATA_EEPROM_BASE + 16); // (0x08080014)
	hts221_humidity_cal_value     = (uint16_t*)(DATA_EEPROM_BASE + 20); // (0x08080018)
	hts221_humidity_slope         = (float*)(DATA_EEPROM_BASE + 24);    // (0x0808001C)
#endif

#if FROM_FLASH
	transmission_params = (uint8_t*)DATA_EEPROM_BASE; 											// DATA_EEPROM_BASE = 0x08080000 Address of the 1st Byte of the EEPPROM MEMORY

	if (*((uint32_t*)transmission_params) == 0) {   										    // Check if the 1st Word of the EEPROM Memory is Empty
		HAL_FLASHEx_DATAEEPROM_Unlock();                                						// UnLock the EEPROM
		HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, DATA_EEPROM_BASE, NTB);      // Program the 1st BYTE of the EEPROM with the NTB value
		HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, DATA_EEPROM_BASE + 1, RTPS); // Program the 2nd BYTE of the EEPROM with the RTPS value
		HAL_FLASHEx_DATAEEPROM_Lock();                                							// Lock the EEPROM
	}
	else {
		ntb  = *(transmission_params);      // Extracting ntb From Flash
		rtps = *(transmission_params + 1);	// Extracting rtps From Flash
	}

#else
	rtps = RTPS;
	ntb  = NTB;
#endif

	HAL_LPTIM_Counter_Start(&hlptim1, ARR);  // Start the LPTIM1 Counter

}

void My_While(void){

	My_Set_PVD(PWR_PVDLEVEL_6, PWR_PVD_MODE_NORMAL);        // Set PVD at the Highest Voltage and Normal Mode

	if(__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) == 0){             // If Vstor is > 3.0 Volt
		My_Power_Management(1,0,1);			   			    // Turn Power On, BLE, No Tx, Sensors
		HAL_I2C_MspInit(&hi2c1);				 			// Initialize I2C
		My_hts221_config();						 			// Fetch data from hts221 sensor (one time)
		HAL_I2C_MspDeInit(&hi2c1);               			// Deinitialize I2C
		My_Set_PVD(PWR_PVDLEVEL_4, PWR_PVD_MODE_IT_RISING); // Set PVD to 4 with Interrupt for Falling vdd
		My_Enter_Stop_Mode_WFI();                           // Enter Stop Mode and WFI from PVD
		My_Power_Management(0,0,0);				   		    // Turn Power OFF
	}
	else
	{
		if (__HAL_PWR_GET_FLAG(PWR_CSR_VREFINTRDYF) == 1){       // if Vstor < 3.0 Volt and Internal Reference is Ready
			My_Set_PVD(PWR_PVDLEVEL_6, PWR_PVD_MODE_IT_FALLING); // Set PVD to 6 with Interrupt for Rising vdd
			My_Enter_Stop_Mode_WFI();   			             // Enter Stop Mode and WFI from PVD or BTH or WFE
		}
	}

	if (ms == SEND){
		if (cnt_ntb++ < ntb){						 // Check Number of Transmitted Beacons
			My_Power_Management(1,1,1);  		     // Turn Power On, Yes BLE, Yes Tx, Yes Sensors
#if SENSORS
			My_Sensors_Send_Data();                  // Send HTS221 sensor data to BLE
#endif
			My_Enter_Stop_Mode_WFI();                // Enter Stop Mode and WFI from BTH or PVD or WFE
			My_Tadv_Ctrl();					         // Controls Minimum Advertising Time REquirement
			My_Power_Management(0,0,0);				 // Turn Power Off
			My_Tadv_Meas();							 // Measure tadv
			tadv = HAL_LPTIM_ReadCounter(&hlptim1);  // Time Stamp tadv
		}
		else{
			ms  = WAIT;
			cnt_ntb = 0;							 // Reset cnt_ntb
			HAL_LPTIM_Counter_Stop(&hlptim1);        // Stop  LPTIM1 Counter
			HAL_LPTIM_Counter_Start(&hlptim1, ARR);  // Start LPTIM1 Counter
			rtp = (uint16_t)rtps*LPTIM_TIME_UNIT;	 // Set rtp
			My_Power_Management(1,0,0);				 // Turn Power On, Yes BLE, No TX, No Sensors
			My_Enter_Stop_Mode_WFI();				 // Enter Stop Mode and WFI from BTH
			My_Power_Management(0,0,0);				 // Turn Power Off
			tadv = HAL_LPTIM_ReadCounter(&hlptim1);  // Time Stamp tadv
		}
	}

	else if (ms == WAIT){
		if (HAL_LPTIM_ReadCounter(&hlptim1) < rtp){
			My_Power_Management(1,0,0);				 // Turn Power On, Yes BLE, No TX, No Sensors
			My_Enter_Stop_Mode_WFI();				 // Enter Stop Mode and WFI from BTH
			My_Power_Management(0,0,0);				 // Turn Power Off
			tadv = HAL_LPTIM_ReadCounter(&hlptim1);  // Time Stamp tadv
		}
		else{
			ms = SEND;
			HAL_LPTIM_Counter_Stop(&hlptim1);        // Stop  LPTIM1 Counter
			HAL_LPTIM_Counter_Start(&hlptim1, ARR);  // Start LPTIM1 Counter
			HAL_I2C_MspDeInit(&hi2c1);               // Deinitialize I2C
			My_Power_Management(1,0,1);				 // Turn Power On, Yes BLE,  No TX, Yes Sensors
#if SENSORS
			HAL_I2C_MspInit(&hi2c1);				 // Initialize I2C
			My_hts221_config();						 // Fetch data from hts221 sensor (one time)
			My_hts221();							 // Read data from hts221 sensor
			My_Power_Management(0,0,0);				 // Turn Power Off
			HAL_I2C_MspDeInit(&hi2c1);               // Deinitialize I2C
			tadv = HAL_LPTIM_ReadCounter(&hlptim1);  // Time Stamp tadv
#else
			My_Enter_Stop_Mode_WFI();				 // Enter Stop Mode and WFI from BTH
			My_Power_Management(0,0,0);				 // Turn Power Off
			tadv = HAL_LPTIM_ReadCounter(&hlptim1);  // Time Stamp tadv
#endif
		}
	}
}
