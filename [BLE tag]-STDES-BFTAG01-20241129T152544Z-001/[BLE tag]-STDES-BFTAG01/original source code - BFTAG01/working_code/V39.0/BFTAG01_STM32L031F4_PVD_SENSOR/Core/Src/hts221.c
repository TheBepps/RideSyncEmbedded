/*
 * hts221.c
 *
 *  Created on: Jun 16, 2020
 *      Author: roberto larosa
 */

#include "hts221.h"

/* I2C Initialization. Writes hts221 Configuration Registers (See pag. 21, 22, 23 and 24 of hts221 data sheet) */
void hts221_init(void){
	uint8_t hts221_init_data = AV_CONF_VALUE;
	HAL_I2C_Mem_Write(&hi2c1, HTS21_I2C_WRITE_ADDR, AV_CONF_ADDR,   I2C_MEMADD_SIZE_8BIT, &hts221_init_data, sizeof(uint8_t), I2C_TIMEOUT);

	hts221_init_data = CTRL_REG1_VALUE;
	HAL_I2C_Mem_Write(&hi2c1, HTS21_I2C_WRITE_ADDR, CTRL_REG1_ADDR, I2C_MEMADD_SIZE_8BIT, &hts221_init_data, sizeof(uint8_t), I2C_TIMEOUT);

	hts221_init_data = CTRL_REG2_VALUE;
	HAL_I2C_Mem_Write(&hi2c1, HTS21_I2C_WRITE_ADDR, CTRL_REG2_ADDR, I2C_MEMADD_SIZE_8BIT, &hts221_init_data, sizeof(uint8_t), I2C_TIMEOUT);

	hts221_init_data = CTRL_REG3_VALUE;
	HAL_I2C_Mem_Write(&hi2c1, HTS21_I2C_WRITE_ADDR, CTRL_REG3_ADDR, I2C_MEMADD_SIZE_8BIT, &hts221_init_data, sizeof(uint8_t), I2C_TIMEOUT);
}

/* Read Output Registers for Temperature (See pag. 26 of hts221 data sheet) */
void hts221_read_temperature(uint16_t *hts221_temperature_data){
	uint8_t hts221_temperature_data_lsb = 0;
	uint8_t hts221_temperature_data_msb = 0;

	HAL_I2C_Mem_Read(&hi2c1, HTS21_I2C_READ_ADDR, TEMP_OUT_L_ADDR, I2C_MEMADD_SIZE_8BIT, &hts221_temperature_data_lsb, sizeof(uint8_t), I2C_TIMEOUT); // Read register 2A
	HAL_I2C_Mem_Read(&hi2c1, HTS21_I2C_READ_ADDR, TEMP_OUT_H_ADDR, I2C_MEMADD_SIZE_8BIT, &hts221_temperature_data_msb, sizeof(uint8_t), I2C_TIMEOUT); // Read register 2B
    *hts221_temperature_data = hts221_temperature_data_msb << 8 | hts221_temperature_data_lsb;
}

/* Read Calibration Register for Temperature (See pag. 26 of hts221 data sheet) */
void hts221_read_temperature_cal_values(uint16_t *hts221_temp_adc_cal_value, uint16_t *hts221_temp_deg_cal_value, float *hts221_temp_slope){
	uint8_t  T0_T1_msb = 0, T0_out_lsb = 0, T0_out_msb = 0, T1_out_lsb = 0, T1_out_msb = 0;
	uint16_t T0_out = 0, T1_out = 0;
	uint16_t T0_DegC = 0, T1_DegC = 0;

	while(T0_DegC == 0){ // Check that the Sensor is Ready after Turn On
		HAL_I2C_Mem_Read(&hi2c1, HTS21_I2C_READ_ADDR, T0_degC_x8, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&T0_DegC, sizeof(uint8_t), I2C_TIMEOUT); // Read register 32
	}
	HAL_I2C_Mem_Read(&hi2c1, HTS21_I2C_READ_ADDR, T1_degC_x8, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&T1_DegC,    sizeof(uint8_t), I2C_TIMEOUT);  // Read register 33
	HAL_I2C_Mem_Read(&hi2c1, HTS21_I2C_READ_ADDR, T0_T1_MSB,  I2C_MEMADD_SIZE_8BIT, &T0_T1_msb,  sizeof(uint8_t), I2C_TIMEOUT);  // Read register 35
	HAL_I2C_Mem_Read(&hi2c1, HTS21_I2C_READ_ADDR, T0_OUT_LSB, I2C_MEMADD_SIZE_8BIT, &T0_out_lsb, sizeof(uint8_t), I2C_TIMEOUT);  // Read register 3C
	HAL_I2C_Mem_Read(&hi2c1, HTS21_I2C_READ_ADDR, T0_OUT_MSB, I2C_MEMADD_SIZE_8BIT, &T0_out_msb, sizeof(uint8_t), I2C_TIMEOUT);  // Read register 3D
	HAL_I2C_Mem_Read(&hi2c1, HTS21_I2C_READ_ADDR, T1_OUT_LSB, I2C_MEMADD_SIZE_8BIT, &T1_out_lsb, sizeof(uint8_t), I2C_TIMEOUT);  // Read register 3E
	HAL_I2C_Mem_Read(&hi2c1, HTS21_I2C_READ_ADDR, T1_OUT_MSB, I2C_MEMADD_SIZE_8BIT, &T1_out_msb, sizeof(uint8_t), I2C_TIMEOUT);  // Read register 3F

	// See Figure #9 of the hts221 datasheet
	T1_DegC = (T0_T1_msb & 0b00001100) << 6 | T1_DegC;
	T0_DegC = (T0_T1_msb & 0b00000011) << 8 | T0_DegC;
	T1_out = T1_out_msb << 8 | T1_out_lsb;
	T0_out = T0_out_msb << 8 | T0_out_lsb;

	*hts221_temp_adc_cal_value = T0_out;  // adc calibration value
	*hts221_temp_deg_cal_value = T0_DegC; // temp in Celsius calibration value
	*hts221_temp_slope = (float)((T1_DegC - T0_DegC))/(float)(T1_out - T0_out);	// Temperature Slope (Calibration)
}

/* Read Calibration Register for Humidity (See pag. 26 of hts221 data sheet) */
void hts221_read_humidity_cal_values(uint16_t *hts221_humidity_adc_cal_value, uint16_t *hts221_humidity_cal_value, float *hts221_humidity_slope){
	uint8_t  h0_out_lsb = 0,  h0_out_msb = 0, h1_out_lsb = 0, h1_out_msb; // RH Calibration Values (See HTS221 Data Sheet Pag. 28)
	uint8_t  h0_rh = 0,   h1_rh = 0; // RH Calibration Values in %   (See HTS221 Data Sheet Pag. 28)
    uint16_t h0_out = 0, h1_out = 0;

	while(h0_rh == 0){ // Check that the Sensor is Ready after Turn On
		HAL_I2C_Mem_Read(&hi2c1, HTS21_I2C_READ_ADDR, H0_rH_x2, I2C_MEMADD_SIZE_8BIT, &h0_rh, sizeof(uint8_t), I2C_TIMEOUT);
	}
	HAL_I2C_Mem_Read(&hi2c1, HTS21_I2C_READ_ADDR, H1_rH_x2,      I2C_MEMADD_SIZE_8BIT, &h1_rh,      sizeof(uint8_t), I2C_TIMEOUT); // Read register 31
	HAL_I2C_Mem_Read(&hi2c1, HTS21_I2C_READ_ADDR, H0_T0_OUT_LSB, I2C_MEMADD_SIZE_8BIT, &h0_out_lsb, sizeof(uint8_t), I2C_TIMEOUT); // Read register 36
	HAL_I2C_Mem_Read(&hi2c1, HTS21_I2C_READ_ADDR, H0_T0_OUT_MSB, I2C_MEMADD_SIZE_8BIT, &h0_out_msb, sizeof(uint8_t), I2C_TIMEOUT); // Read register 37
	HAL_I2C_Mem_Read(&hi2c1, HTS21_I2C_READ_ADDR, H1_T0_OUT_LSB, I2C_MEMADD_SIZE_8BIT, &h1_out_lsb, sizeof(uint8_t), I2C_TIMEOUT); // Read register 3A
	HAL_I2C_Mem_Read(&hi2c1, HTS21_I2C_READ_ADDR, H1_T0_OUT_LSB, I2C_MEMADD_SIZE_8BIT, &h1_out_msb, sizeof(uint8_t), I2C_TIMEOUT); // Read register 3B

	// See Figure #10 (Pag. 28) of the hts221 datasheet
	h0_out = h0_out_msb << 8 | h0_out_lsb;
	h1_out = h1_out_msb << 8 | h1_out_lsb;

	*hts221_humidity_adc_cal_value = h0_out;					                // adc calibration value
	*hts221_humidity_cal_value     = h0_rh;		       		                    // Humidity calibration value
	*hts221_humidity_slope = (float)(h1_rh - h0_rh)/(float)(h1_out - h0_out);	// Humidity Slope (Calibration)
}

/* Read Output Registers for Humidity (See pag. 26 of hts221 data sheet) */
void hts221_read_humidity(uint16_t *hts221_humidity_data){
	uint8_t hts221_humidity_data_lsb = 0;
	uint8_t hts221_humidity_data_msb = 0;

	HAL_I2C_Mem_Read(&hi2c1, HTS21_I2C_READ_ADDR, HUMIDITY_OUT_L_ADDR, I2C_MEMADD_SIZE_8BIT, &hts221_humidity_data_lsb, sizeof(uint8_t), I2C_TIMEOUT); // Read register 28
	HAL_I2C_Mem_Read(&hi2c1, HTS21_I2C_READ_ADDR, HUMIDITY_OUT_H_ADDR, I2C_MEMADD_SIZE_8BIT, &hts221_humidity_data_msb, sizeof(uint8_t), I2C_TIMEOUT); // Read register 29
	*hts221_humidity_data = hts221_humidity_data_msb << 8 | hts221_humidity_data_lsb;
}



