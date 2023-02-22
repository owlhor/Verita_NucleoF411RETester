/*
 * Verita_PTC.h
 *
 *  Created on: Feb 22, 2023
 *      Author: owl_hor
 */

#ifndef INC_VERITA_PTC_H_
#define INC_VERITA_PTC_H_

#include "stm32f4xx_hal.h"

typedef enum{
	VRT_ERROR  = 0x90U,
	VRT_OK     = 0x91U,
	VRT_Busy   = 0x92U,
	VRT_Regain = 0x93U,
	VRT_Next   = 0x94U,
} VRTPTC_StatusTypedef;


VRTPTC_StatusTypedef Rx_Verita_engine(uint8_t *Rxbffr, uint32_t *regisk);

#endif /* INC_VERITA_PTC_H_ */
