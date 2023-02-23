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
	HAL_Delay(1);
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

//// output pushpull
uint32_t gpio_selftest_output_pp_1(GPIO_TypeDef* GPIOx,uint16_t *Lista_GPIOx){
	uint32_t temp_mode = GPIOx->MODER;
	uint32_t temp_bsrr = 0;
	uint32_t result = 0;

	uint16_t sizearr = 0;
	//// use instead of sizeof
	for(register int i = 0; i <= 17 ;i++){
		if(Lista_GPIOx[i] == 20){
			break;
		}else{
			sizearr++;
		}
	}

	///// -------------- Set Output Pushpull ---------------------
	for(register int i = 0;i < sizearr; i++){
			temp_mode &= ~( 0b11 << (Lista_GPIOx[i] * 2U)); // clear only 2 register want to reconfig by shift 11 to prefer position then & it's invert to the previous read
			temp_mode |= ( GPIO_MODE_OUTPUT_PP << (Lista_GPIOx[i] * 2U));
		}
	GPIOx->MODER = temp_mode;

	//// write SET to BSRR
	for(register int i = 0;i < sizearr; i++){
			temp_bsrr &= ~( 0b1 << Lista_GPIOx[i]); // clear only register want to reconfig by shift 11 to prefer position then & it's invert to the previous read
			temp_bsrr |= ( GPIO_PIN_SET << Lista_GPIOx[i]);
		}
	GPIOx->BSRR = temp_bsrr; // insert 1 into SET position

	//// Read Back #1 get set
	HAL_Delay(1);
	result |= (GPIOx->IDR) << 16;
	HAL_Delay(1);

	//// write RESET to BSRR
	GPIOx->BSRR = temp_bsrr << 16; // insert 1 into RESET position << 16
	//// Read Back #2 get Reset
	HAL_Delay(1);
	result |= GPIOx->IDR;

	return result;
}

//// output opendrain
/* Open drain mode: A “0” in the Output register activates the N-MOS whereas a “1”
 * in the Output register leaves the port in Hi-Z (the P-MOS is never activated)
 * */
uint32_t gpio_selftest_output_od_1(GPIO_TypeDef* GPIOx,uint16_t *Lista_GPIOx){
	uint32_t temp_mode = GPIOx->MODER;
	uint32_t temp_bsrr = 0;
	uint32_t temp_pupdr = 0;
	uint32_t result = 0;

	uint16_t sizearr = 0;
	//// use instead of sizeof
	for(register int i = 0; i <= 17 ;i++){
		if(Lista_GPIOx[i] == 20){
			break;
		}else{
			sizearr++;
		}
	}

	///// -------------- Set Output  opendrain ---------------------
	for(register int i = 0;i < sizearr; i++){
			temp_mode &= ~( 0b11 << (Lista_GPIOx[i] * 2U)); // clear only 2 register want to reconfig by shift 11 to prefer position then & it's invert to the previous read
			temp_mode |= ( GPIO_MODE_OUTPUT_OD << (Lista_GPIOx[i] * 2U));
		}
	GPIOx->MODER = temp_mode;

	//// Set Pullup for Hi-Z State read
	for(register int i = 0;i < sizearr; i++){
		temp_pupdr &= ~( 0b11 << (Lista_GPIOx[i] * 2U)); // clear only 2 register want to reconfig by shift 11 to prefer position then & it's invert to the previous read
		temp_pupdr |= ( GPIO_PULLUP << (Lista_GPIOx[i] * 2U));
	}
	GPIOx->PUPDR = temp_pupdr;

	//// write SET to BSRR
	for(register int i = 0;i < sizearr; i++){
			temp_bsrr &= ~( 0b1 << Lista_GPIOx[i]); // clear only register want to reconfig by shift 11 to prefer position then & it's invert to the previous read
			temp_bsrr |= ( GPIO_PIN_SET << Lista_GPIOx[i]);
		}
	GPIOx->BSRR = temp_bsrr; // insert 1 into SET position

	//// Read Back #1 get set
	HAL_Delay(1);
	result |= (GPIOx->IDR) << 16;
	HAL_Delay(1);

	//// write RESET to BSRR
	GPIOx->BSRR = temp_bsrr << 16; // insert 1 into RESET position << 16
	//// Read Back #2 get Reset
	HAL_Delay(1);
	result |= GPIOx->IDR;

	return result;
}
