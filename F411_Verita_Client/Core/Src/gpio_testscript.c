/*
 * gpio_testscript.c
 *
 *  Created on: Feb 21, 2023
 *      Author: owl_hor
 */


#include "gpio_testscript.h"


//// lists All port - pin to inspect first // avoid special pin like osilators / UART
//// GPIO_PIN_x is in bit position format (0 2 4 8 16 ...) which loss if stored in that form and log2() to calculate back
//uint16_t List_GPIOC[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13};


uint32_t gpio_selftest_input_pupdr_1(GPIO_TypeDef* GPIOx,uint16_t *Lista_GPIOx){

	uint32_t result = 0;

	uint32_t temp_mode = GPIOx->MODER;
	uint32_t temp_pupdr = GPIOx->PUPDR;

	//uint16_t sizearr = sizeof(Lista_GPIOx); // / sizeof(List_GPIOC[0])
	//// use instead of sizeof which return array length input into function as 4
	uint16_t sizearr = 0;
	for(register int i = 0; i <= 17 ;i++){
		if(Lista_GPIOx[i] == 20){
			break;
		}else{
			sizearr++;
		}
	}


	//// ------------------ Input PULLUP ------------------------------
	for(register int i = 0;i < sizearr; i++){
		temp_mode &= ~( 0b11 << (Lista_GPIOx[i] * 2U)); // clear only 2 register want to reconfig by shift 11 to prefer position then & it's invert to the previous read
		temp_mode |= ( GPIO_MODE_INPUT << (Lista_GPIOx[i] * 2U));
	}
	GPIOx->MODER = temp_mode;


	for(register int i = 0;i < sizearr; i++){
		temp_pupdr &= ~( 0b11 << (Lista_GPIOx[i] * 2U)); // clear only 2 register want to reconfig by shift 11 to prefer position then & it's invert to the previous read
		temp_pupdr |= ( GPIO_PULLUP << (Lista_GPIOx[i] * 2U));
	}
	GPIOx->PUPDR = temp_pupdr;
	HAL_Delay(5);
	result |= (GPIOx->IDR) << 16;

	//// ------------------ Input PULLDOWN ------------------------------
	for(register int i = 0;i < sizearr; i++){
		temp_pupdr &= ~( 0b11 << (Lista_GPIOx[i] * 2U)); // clear only 2 register want to reconfig by shift 11 to prefer position then & it's invert to the previous read
		temp_pupdr |= ( GPIO_PULLDOWN << (Lista_GPIOx[i] * 2U));
	}
	GPIOx->PUPDR = temp_pupdr;
	HAL_Delay(5);
	result |= GPIOx->IDR;

	return result;

	}

