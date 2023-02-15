/*
 * INA219.c
 *
 *  Created on: Feb 15, 2023
 *      Author: owl_hor
 */

#include "INA219.h"

union{
	uint16_t D16[2];
	uint8_t D8[4];
}INACBffr;

uint16_t INA219Read_cx(I2C_HandleTypeDef *hi2c,uint8_t dv_addr, uint8_t ina_rg){

	HAL_I2C_Mem_Read(hi2c, dv_addr, ina_rg, I2C_MEMADD_SIZE_8BIT, &INACBffr.D8[1], 2, 10);

	return INACBffr.D16[1] | INACBffr.D16[0];

}
