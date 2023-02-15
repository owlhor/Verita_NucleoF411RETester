/*
 * INA219.h
 *
 *  Created on: Feb 15, 2023
 *      Author: owl_hor
 */

#ifndef INC_INA219_H_
#define INC_INA219_H_

#include "stm32f4xx_hal.h"

#define INA219_ADDR_1 0b10000000 // 1000000

#define INA219_Config_Reset 0x399F

#define INA219_RG_Config	0x00
#define INA219_RG_ShuntV    0x01
#define INA219_RG_BusV 		0x02
#define INA219_RG_PoWer		0x03
#define INA219_RG_Current   0x04
#define INA219_RG_Calibra 	0x05

//static enum {PG_40mv, PG_80mv, PG_160mv, PG_320mv}INA219_PG_Set;

typedef union _INA219_RD_Strc{
	struct INA219RD{
		// LSB  I2C Frame#2
		// MSB  I2C Frame#2

		// LSB  I2C Frame#1

		uint32_t CURR__: 16;
		uint32_t __ENT: 16;
		uint32_t POWR: 16;
		uint32_t Bus_V: 16;
		uint32_t SHUNT_V: 16;
		// MSB  I2C Frame#1
	}INA219RD;

	uint8_t D8[8];
}INA219_RD_Strc;


typedef union _INA219_Conf_Strc{
	struct INA219CF{
		uint16_t mode: 3;
		uint16_t sadc: 4;
		uint16_t badc: 4;
		uint16_t pg  : 2;
		uint16_t brng: 1;
		uint16_t rsrv: 1;
		uint16_t rest: 1;

		}INA219CF;
	uint8_t D8[2];
}INA219_Conf_Strc;


uint16_t INA219Read_cx(I2C_HandleTypeDef *hi2c,uint8_t dv_addr, uint8_t ina_rg);

#endif /* INC_INA219_H_ */
