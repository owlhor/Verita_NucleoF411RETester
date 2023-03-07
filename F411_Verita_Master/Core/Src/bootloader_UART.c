/*
 * bootloader_UART.c
 *
 *  Created on: Mar 7, 2023
 *      Author: owl_hor
 */

#include "bootloader_UART.h"

/*
 *
 * step 1 - push BOOT0 High
 * step 2 - Reset: pull NRST down
 * step 3 - wait bootloader startup for 74.5ms ++
 *
 * step x - UART send 0x7F
 * */


void bootloader_UART_UP(UART_HandleTypeDef *huart, uint8_t *databoot){


	////


	static uint32_t index = 0; // index of flash in .bin array

	// pseudo concept
//	for(register int i = 0;i < 16384;i++){
//		send = databoot[index];
//		index++;
//	}



}
