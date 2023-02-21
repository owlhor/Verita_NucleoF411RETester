/*
 * gio_testscript.h
 *
 *  Created on: Feb 21, 2023
 *      Author: owl_hor
 */

#ifndef INC_GPIO_TESTSCRIPT_H_
#define INC_GPIO_TESTSCRIPT_H_

#include "stm32f4xx_hal.h"



uint32_t gpio_selftest_input_pupdr_1(GPIO_TypeDef* GPIOx,uint16_t *Lista_GPIOx);

#endif /* INC_GPIO_TESTSCRIPT_H_ */
