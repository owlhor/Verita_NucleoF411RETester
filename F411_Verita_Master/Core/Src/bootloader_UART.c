/*
 * bootloader_UART.c
 *
 *  Created on: Mar 7, 2023
 *      Author: owl_hor
 */

#include "bootloader_UART.h"

/* In booting step, THe other operation (especially display writing) must be terminated
 *
 * step 1 - push BOOT0 High
 * step 2 - Reset: pull NRST down
 * step 3 - wait bootloader startup for 74.5ms ++
 *
 * step x - UART send 0x7F
 *
 * step x - Erase (old?) memory (?)
 * step x - Write Memory CMD with code
 * */

void BL_UART_Start(UART_HandleTypeDef *huart){

	uint8_t UARTBL_Start = 0x7F;
	//// step 1: Push BOOT0 -> HIGH
	HAL_GPIO_WritePin(BOOT0_Trigger_GPIO_PORT, BOOT0_Trigger_GPIO_PIN, GPIO_PIN_SET);

	//// step 2: Reset Client, push SET NMOS
	HAL_GPIO_WritePin(Client_NRST_Trg_GPIO_PORT, Client_NRST_Trg_GPIO_PIN, GPIO_PIN_SET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(Client_NRST_Trg_GPIO_PORT, Client_NRST_Trg_GPIO_PIN, GPIO_PIN_RESET);
	//// step 3: wait bootloader startup for 74.5ms ++
	HAL_Delay(85); // 80 is not enough

	//// send 0x7F
	HAL_UART_Transmit(huart, &UARTBL_Start, 1, 10);

	BL_UART_wait_ACK(huart, 20);



}

void BL_UART_Finish(){
	//// pull BOOT0 back
	HAL_GPIO_WritePin(BOOT0_Trigger_GPIO_PORT, BOOT0_Trigger_GPIO_PIN, GPIO_PIN_RESET);

	//// step 2: Reset Client, push SET NMOS
//	HAL_GPIO_WritePin(Client_NRST_Trg_GPIO_PORT, Client_NRST_Trg_GPIO_PIN, GPIO_PIN_SET);
//	HAL_Delay(20);
//	HAL_GPIO_WritePin(Client_NRST_Trg_GPIO_PORT, Client_NRST_Trg_GPIO_PIN, GPIO_PIN_RESET);
}

void BL_UART_GET_CMD(UART_HandleTypeDef *huart, uint8_t *bufferd){
	/* code 0x00 Gets the version and the allowed commands supported by the current
		version of the protocol.

		STM32F411RE Bootloader read result:
		UART Protocol ver: 3.1
		Available CMD: Get, GetVersion, Get ID, ReadMem, Go, WriteMem, ExtendErase,
						Write pro-unprotect, Readout pro-unprotect
	 * */
	uint8_t Start_GET[2] = {0x00, 0xFF};
	uint8_t response = 0;


	HAL_UART_Transmit(huart, &Start_GET[0], 2, 10);
	response = BL_UART_wait_ACK(huart, 10);

	if(response == UB_ACK){
		HAL_UART_Receive(huart, bufferd, 16, 15);
	}else if(response == UB_NACK){

		for(int i = 0; i < 10; i++){
			bufferd[i] = 0xAA;
		}
	}else{}

}

UARTBootloader_state BL_UART_GETVersion(UART_HandleTypeDef *huart, uint8_t *bufferd){
	/* code 0x01 Gets the protocol version.
	 * */
	uint8_t Start_GV[2] = {0x01, 0xFE};
	uint8_t response = 0;


	HAL_UART_Transmit(huart, &Start_GV[0], 2, 10);

	response = BL_UART_wait_ACK(huart, 10);

	if(response == UB_ACK){
		HAL_UART_Receive(huart, bufferd, 4, 10);

		return UB_ACK;

	}else if(response == UB_NACK){

		for(int i = 0; i < 10; i++){
			bufferd[i] = 0xAA;

			return UB_NACK;
		}
	}else{}

	return UB_NACK;
}

UARTBootloader_state BL_UART_GETID(UART_HandleTypeDef *huart, uint8_t *bufferd){
	/* code 0x02 Get the version of the chip ID (identification)
	 * */
	uint8_t Start_GV[2] = {0x02, 0xFD};
	uint8_t response = 0;


	HAL_UART_Transmit(huart, &Start_GV[0], 2, 10);

	response = BL_UART_wait_ACK(huart, 10);

	if(response == UB_ACK){
		HAL_UART_Receive(huart, bufferd, 3, 10);
		response = BL_UART_wait_ACK(huart, 10);
		return UB_ACK;
	}else{
		return UB_NACK;
	}

	return UB_NACK;
}


UARTBootloader_state BL_UART_Write_Protect(UART_HandleTypeDef *huart){
	/*
	 * */
	uint8_t Start_WP[2] = {0x63, 0x9C};
//	Start_WPUN[0] = (uint8_t)UB_WriteUnProtect;
//	Start_WPUN[1] = ~(uint8_t)UB_WriteUnProtect;

	HAL_UART_Transmit(huart, &Start_WP[0], 2, 10);

	//// 2x ACK
	BL_UART_wait_ACK(huart, 500);
	return BL_UART_wait_ACK(huart, 500);
}

UARTBootloader_state BL_UART_Write_UnProtect(UART_HandleTypeDef *huart){
	/*
	 * */
	uint8_t Start_WPUN[2] = {0x73, 0x8C};
//	Start_WPUN[0] = (uint8_t)UB_WriteUnProtect;
//	Start_WPUN[1] = ~(uint8_t)UB_WriteUnProtect;

	HAL_UART_Transmit(huart, &Start_WPUN[0], 2, 10);

	//// 2x ACK
	BL_UART_wait_ACK(huart, 500);
	return BL_UART_wait_ACK(huart, 500);
}

UARTBootloader_state BL_UART_Readout_Protect(UART_HandleTypeDef *huart){
	/*
	 * */
	uint8_t Start_RDP_PRM[2] = {0x82, 0x7D};
	//uint8_t response = 0;

	HAL_UART_Transmit(huart, &Start_RDP_PRM[0], 2, 10);

	//// 2x ACK
	BL_UART_wait_ACK(huart, 500);
	return BL_UART_wait_ACK(huart, 500);
}

UARTBootloader_state BL_UART_Readout_UnProtect(UART_HandleTypeDef *huart){
	/*
	 * */
	uint8_t Start_RDU_PRM[2] = {0x92, 0x6D};
	//uint8_t response = 0;

	HAL_UART_Transmit(huart, &Start_RDU_PRM[0], 2, 10);

	//// 2x ACK
	BL_UART_wait_ACK(huart, 500);
	return BL_UART_wait_ACK(huart, 500);

}


UARTBootloader_state BL_UART_wait_ACK(UART_HandleTypeDef *huart, uint16_t timeout){
	uint8_t tick = 1;
	uint8_t response = 0;
	uint32_t timestick;

	timestick = HAL_GetTick() + timeout;

	while(tick){
		HAL_UART_Receive(huart, &response, 1, 2);

		if(response == 0x79){
			return UB_ACK;
		}else if(response == 0x1F){
			return UB_NACK;
		}else{}

		if(HAL_GetTick()>= timestick){
			return UB_Timeout;
		}
	}
	return UB_NACK;
}
