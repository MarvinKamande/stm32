/*
 * BME280.h
 *
 *  Created on: Aug 24, 2026
 *      Author: ADMIN
 */

#ifndef BME280_H_
#define BME280_H_

#ifdef DEBUG
#include <stdio.h>
#define BME280_DBG(...) printf(__VA_ARGS__);
#endif

//BME280 registers
#define PRESS_REG_ADDRESS 0xF7
#define BME280_REG_CONFIG 0xF5
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_STATUS 0xF3
#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_RESET 0xE0
#define BME280_BASE_REG_HUM 0xE1
#define BME280_BASE_REG_TP 0x88
#define DEVICE_ADDRESS 0x76


#define BME280_TP_SIZE 25
#define BME280_HUM_SIZE 7
#define BME280_RESET_VALUE 0xB6
#define BME280_STANDARD 0x01
#define BME280_MODE_FORCED 0x02
#define BME280_IIR_TSB 0x24



typedef struct {
	int8_t mode;
	int8_t filter;
	int8_t os_press;
	int8_t os_temp;
	int8_t os_hum;
} BME280_Params_t;

#endif /* BME280_H_ */
