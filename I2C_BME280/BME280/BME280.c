/*
 * BME280.c
 *
 *  Created on: Aug 24, 2026
 *      Author: ADMIN
 */

#include "main.h"

#include "BME280.h"

void BME280_init_default_Params(BME280_Params_t *params) {
	params->mode = BME280_MODE_FORCED;
	params->filter = BME280_IIR_TSB;
	params->os_press = BME280_STANDARD;
	params->os_temp = BME280_STANDARD;
	params->os_hum = BME280_STANDARD;
}

//read calibration/trimming values
int TrimRead(I2C_HandleTypeDef *hi2c, int16_t *T, int16_t *P, int16_t *H) {
	uint8_t trimdata[32];

	//Read NVM from 0x88 to 0xA1
	if (HAL_I2C_Mem_Read(hi2c, DEVICE_ADDRESS << 1, BME280_BASE_REG_TP, 1, (uint8_t*)&trimdata, BME280_TP_SIZE, HAL_MAX_DELAY) != HAL_OK) {
		return 1; //error NUM for debugging
	}

	//Read NVM from 0xE1 to 0xE7
	if (HAL_I2C_Mem_Read(hi2c, DEVICE_ADDRESS << 1, BME280_BASE_REG_HUM, 1, (uint8_t*)(trimdata + 25), BME280_HUM_SIZE, HAL_MAX_DELAY) != HAL_OK) {
			return 2; //error NUM for debugging
	}

	//Temp coefficients
	*T = (int16_t)(trimdata[1] << 8 | trimdata[0]);
	*(T + 2) = (int16_t)(trimdata[3] << 8 | trimdata[2]);
	*(T + 4) = (int16_t)(trimdata[5] << 8 | trimdata[4]);

	//Pressure coefficients
	*P = (int16_t)(trimdata[7] << 8 | trimdata[6]);
	*(P + 2) = (int16_t)(trimdata[9] << 8 | trimdata[8]);
	*(P + 4) = (int16_t)(trimdata[11] << 8 | trimdata[10]);
	*(P + 6) = (int16_t)(trimdata[13] << 8 | trimdata[12]);
	*(P + 8) = (int16_t)(trimdata[15] << 8 | trimdata[14]);
	*(P + 10) = (int16_t)(trimdata[16] << 8 | trimdata[16]);
	*(P + 12) = (int16_t)(trimdata[17] << 8 | trimdata[18]);
	*(P + 14) = (int16_t)(trimdata[19] << 8 | trimdata[20]);
	*(P + 16) = (int16_t)(trimdata[21] << 8 | trimdata[22]);

	//Humidity coefficients
	*H = (int16_t)trimdata[24];
	*(H + 2) = (int16_t)(trimdata[26] << 8 | trimdata[25]);
	*(H + 4) = (int16_t)trimdata[27];
	*(H + 6) = (int16_t)(trimdata[28] << 4 | (trimdata[29] & 0x0F));
	*(H + 8) = (int16_t)((trimdata[30] << 4) | (trimdata[29] >> 4));
	*(H + 10) = (int16_t)trimdata[24];

	return 0; //successful
}

HAL_StatusTypeDef write_register8(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t value) {
	HAL_StatusTypeDef returnstatus;
	uint16_t tx_buff;
	tx_buff = (DEVICE_ADDRESS << 1);

	returnstatus = HAL_I2C_Mem_Write(hi2c, tx_buff, addr, 1, (uint8_t*)&value, 1, HAL_MAX_DELAY);

	return returnstatus;

}

int8_t BME280_Init(I2C_HandleTypeDef *hi2c, int16_t *TD, int16_t *PD, int16_t *HD) {
	BME280_Params_t par;
	BME280_init_default_Params(&par);

	uint8_t write_data = 0;
	uint8_t check_data = 0;
	uint16_t add_buff;
	add_buff = (DEVICE_ADDRESS << 1);

	if(TrimRead(hi2c, TD, PD, HD) != 0) {
		return 1;
	}

	//reset the component
	write_data = BME280_RESET_VALUE;
	if (HAL_I2C_Mem_Write(hi2c, add_buff, BME280_REG_RESET, 1, (uint8_t*)&write_data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return 2;
	}

	HAL_Delay(100);

	//set Humidity Oversampling
	write_data = par.os_hum;
	if (HAL_I2C_Mem_Write(hi2c, add_buff, BME280_REG_CTRL_HUM, 1, (uint8_t*)&write_data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return 3;
	}
	HAL_Delay(100);
	HAL_I2C_Mem_Read(hi2c, add_buff, BME280_REG_CTRL_HUM, 1, (uint8_t*)&check_data, 1, HAL_MAX_DELAY);
	if(check_data != write_data) {
		return 4;
	}

	//set IIR filter coefficient and standby filter
	write_data = BME280_IIR_TSB;
	if (HAL_I2C_Mem_Write(hi2c, add_buff, BME280_REG_CONFIG, 1, (uint8_t*)&write_data, 1, HAL_MAX_DELAY) != HAL_OK){
		return 5;
	}
	HAL_Delay(100);
	HAL_I2C_Mem_Read(hi2c, add_buff, BME280_REG_CONFIG, 1, (uint8_t*)&check_data, 1, HAL_MAX_DELAY);
	if(check_data != write_data) {
		return 6;
	}

	//set Pressure-Temp Oversampling, and Operating mode
	write_data = ((par.os_temp)<<5)|((par.os_press)<<2)|(par.mode);
	if (HAL_I2C_Mem_Write(hi2c, add_buff, BME280_REG_CTRL_MEAS, 1, (uint8_t*)&write_data, 1, HAL_MAX_DELAY) != HAL_OK){
		return 7;
	}
	HAL_Delay(100);

	return 0; //successful

}

