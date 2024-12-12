/*
 * hts221.h
 *
 *  Created on: Jun 16, 2020
 *      Author: roberto larosa
 */

#ifndef INC_HTS221_H_
#define INC_HTS221_H_

#include "i2c.h"


#define HTS21_I2C_WRITE_ADDR 0XBE	// I2C WRITE ADDRESS (See I2C Operation in the Data Sheet)
#define HTS21_I2C_READ_ADDR  0XBF	// I2C READ  ADDRESS (See I2C Operation in the Data Sheet)
#define I2C_TIMEOUT 100				// 100 msec

// Register Address (See Register Mapping in the hts221 Data Sheet)
#define WHO_AM_I_ADDR       0x0F
#define AV_CONF_ADDR        0x10
#define CTRL_REG1_ADDR      0x20
#define CTRL_REG2_ADDR      0x21
#define CTRL_REG3_ADDR      0x22
#define STATUS_REG_ADDR     0x27
#define HUMIDITY_OUT_L_ADDR 0x28
#define HUMIDITY_OUT_H_ADDR 0x29
#define TEMP_OUT_L_ADDR     0x2A
#define TEMP_OUT_H_ADDR     0x2B

// Temperature Calibration Values (See section #8 of the hts221 data sheet)
#define T0_OUT_LSB			0x3C
#define T0_OUT_MSB			0x3D
#define T1_OUT_LSB			0x3E
#define T1_OUT_MSB			0x3F
#define T0_degC_x8			0x32
#define T1_degC_x8			0x33
#define T0_T1_MSB			0x35

// Humidity Calibration Values (See section #8 of the hts221 data sheet)
#define H0_T0_OUT_LSB		0x36
#define H0_T0_OUT_MSB		0x37
#define H1_T0_OUT_LSB		0x3A
#define H1_T0_OUT_MSB		0x3B
#define H0_rH_x2			0x30
#define H1_rH_x2			0x31

// Setting Up Register Values (See Register Description in the hts221 Data Sheet)
#define AV_CONF_VALUE   0b00000000 // Value is 0 to reduce power to minimum
#define CTRL_REG1_VALUE 0b10000100 // Configured Active and for One Shot measurement mode to save Energy
#define CTRL_REG2_VALUE	0b00000001 // Configured for One Shot measurement mode to save Energy
#define CTRL_REG3_VALUE	0b00000100 // DRDY Enabled, Active High, Push Pull

void hts221_init(void);
void hts221_read_reg(uint8_t hts221_reg, uint8_t *hts221_reg_data);
void hts221_write_reg(uint8_t hts221_reg, uint8_t *hts221_reg_data);
void hts221_read_temperature(uint16_t *hts221_temperature_data);
void hts221_read_humidity(uint16_t *hts221_humidity_data);
void hts221_read_humidity_cal_values(uint16_t *hts221_adc_hum_lsb_cal_value, uint16_t *hts221_humidity_cal_value, float *hts221_humidity_slope);
void hts221_read_temperature_cal_values(uint16_t *hts221_adc_temp_lsb_cal_value, uint16_t *hts221_temperature_cal_value, float *hts221_temperature_slope);

#endif /* INC_HTS221_H_ */
