/*
 * BME280.c
 *
 *  Created on: Aug 24, 2026
 *      Author: Marvin Kamande
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

//Read and store raw data
HAL_StatusTypeDef Read_Raw_BME280 (I2C_HandleTypeDef *hi2c ,BME280_TypeDef* Raw) {
	HAL_StatusTypeDef returnstatus;
	uint8_t buf[8] = {0};

	returnstatus = HAL_I2C_Mem_Read(hi2c, DEVICE_ADDRESS << 1, PRESS_REG_ADDRESS, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&buf, 8, HAL_MAX_DELAY);
	if (returnstatus != HAL_OK) {
		return returnstatus;
	}

	Raw->press = (buf[0] << 12) + (buf[1] << 4) + (buf[2] >> 4);
	Raw->temp = (buf[3] << 12) + (buf[4] << 4) + (buf[5] >> 4);
	Raw->hum = (buf[6] << 8) + buf[7];

	return returnstatus;

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
	*T = (uint16_t)(trimdata[1] << 8 | trimdata[0]);
	*(T + 1) = (int16_t)(trimdata[3] << 8 | trimdata[2]);
	*(T + 2) = (int16_t)(trimdata[5] << 8 | trimdata[4]);

	//Pressure coefficients
	*P = (uint16_t)(trimdata[7] << 8 | trimdata[6]);
	*(P + 1) = (int16_t)(trimdata[9] << 8 | trimdata[8]);
	*(P + 2) = (int16_t)(trimdata[11] << 8 | trimdata[10]);
	*(P + 3) = (int16_t)(trimdata[13] << 8 | trimdata[12]);
	*(P + 4) = (int16_t)(trimdata[15] << 8 | trimdata[14]);
	*(P + 5) = (int16_t)(trimdata[17] << 8 | trimdata[16]);
	*(P + 6) = (int16_t)(trimdata[19] << 8 | trimdata[18]);
	*(P + 7) = (int16_t)(trimdata[21] << 8 | trimdata[20]);
	*(P + 8) = (int16_t)(trimdata[23] << 8 | trimdata[22]);

	//Humidity coefficients
	*H = (int16_t)trimdata[24];
	*(H + 1) = (int16_t)(trimdata[26] << 8 | trimdata[25]);
	*(H + 2) = (int16_t)trimdata[27];
	*(H + 3) = (int16_t)(trimdata[28] << 4 | (trimdata[29] & 0x0F));
	*(H + 4) = (int16_t)((trimdata[30] << 4) | (trimdata[29] >> 4));
	*(H + 5) = (int16_t)trimdata[31];

	return 0; //successful
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

int32_t BME280_Compensate_T(int32_t adc_T, int32_t* t_fine, int16_t* DT) {
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3)-((int32_t)(*DT)<<1))) * ((int32_t)(*(DT+1)))) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)(*DT))) * ((adc_T>>4) - ((int32_t)(*DT))))>> 12) *((int32_t)(*(DT+2)))) >> 14;
	*t_fine = var1 + var2;
	T = ((*t_fine) * 8 + 128)>>8;
	return T;
}

uint32_t BME280_Compensate_P(int32_t adc_P, int32_t* t_fine, int16_t* DP) {
	int64_t var1, var2, P;
	var1 = ((int64_t)(*t_fine)) - 128000;
	var2 = var1 * var1 * ((int64_t)(*(DP+5)));
	var2 = var2 + ((var1 * (int64_t)(*(DP+4)))<<17);
	var2 = var2 + (((int64_t)(*(DP+3)))<<35);
	var1 = ((var1 * var1 * ((int64_t)(*(DP+2))))>>8) + ((var1 * ((int64_t)(*(DP+1))))<<12);
	var1 = ((((int64_t)1)<<47)+var1) * (((int64_t)(*DP))>>33);
	if (var1 == 0){
		return 0; //avoid exception caused by division by 0
	}
	P = 1048576 - adc_P;
	P = (((P<<31)-var2) * 3125)/var1;
	var1 = (((int64_t)(*(DP+8))) * (P>>13) * (P>>13)) >> 25;
	var2 = (((int64_t)(*(DP+7))) * P) >> 19;
	P = ((P + var1 + var2) >> 8) + (((int64_t)(*(DP+6))) << 4);

	return (uint32_t)P;
}

uint32_t BME280_Compensate_H(int32_t adc_H, int32_t* t_fine, int16_t* DH) {
	int32_t v_x1_u32r;
	v_x1_u32r = ((*t_fine) - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)(*(DH+3))) << 20) - (((int32_t)(*(DH+4))) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)(*(DH+5)))) >> 10) * (((v_x1_u32r * ((int32_t)(*(DH+2)))) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)(*(DH+1))) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)(*DH))) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (uint32_t)(v_x1_u32r >> 12);
}

void BME280_Measure(I2C_HandleTypeDef* hi2c, BME280_TypeDef* Raw, int16_t* DigT, int16_t* DigP, int16_t* DigH, BME280_Actual* Final) {
	int32_t t_fine = 0;
	HAL_StatusTypeDef rstatus;

	rstatus = Read_Raw_BME280(hi2c, Raw);
	if (rstatus != HAL_OK) {
		Error_Handler();
	} else {
		Final->Temp = BME280_Compensate_T(Raw->temp, &t_fine, DigT) / 100.0; //Temp in Celsius
		Final->Press = BME280_Compensate_P(Raw->press, &t_fine, DigP) / 100.0; //Pressure in hPa
		Final->Hum = BME280_Compensate_H(Raw->hum, &t_fine, DigH) / 1024.0; // Humidity in %RH
	}
}
