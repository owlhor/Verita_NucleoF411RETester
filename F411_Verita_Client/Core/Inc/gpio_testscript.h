/*
 * gio_testscript.h
 *
 *  Created on: Feb 21, 2023
 *      Author: owl_hor
 */

#ifndef INC_GPIO_TESTSCRIPT_H_
#define INC_GPIO_TESTSCRIPT_H_

#include "stm32f4xx_hal.h"

/// Have a problem to insert array of pins
//typedef struct{
//	GPIO_TypeDef Port;
//	uint8_t List_PIN[];
//}GPIOx_PortPINTypedef;

uint32_t gpio_selftest_input_pupdr_1(GPIO_TypeDef* GPIOx,uint16_t *Lista_GPIOx);
uint32_t gpio_selftest_output_pp_1(GPIO_TypeDef* GPIOx,uint16_t *Lista_GPIOx);
uint32_t gpio_selftest_output_od_1(GPIO_TypeDef* GPIOx,uint16_t *Lista_GPIOx);

#endif /* INC_GPIO_TESTSCRIPT_H_ */
