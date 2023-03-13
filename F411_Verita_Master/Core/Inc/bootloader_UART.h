/*
 * bootloader_UART.h
 *
 *  Created on: Mar 7, 2023
 *      Author: owl_hor
 */

#ifndef INC_BOOTLOADER_UART_H_
#define INC_BOOTLOADER_UART_H_

#include "stm32f4xx_hal.h"

#define BOOT0_Trigger_GPIO_PORT GPIOA
#define BOOT0_Trigger_GPIO_PIN GPIO_PIN_8

#define Client_NRST_Trg_GPIO_PORT GPIOC
#define Client_NRST_Trg_GPIO_PIN GPIO_PIN_6

//// 16kb = 16384 bytes per sector
#define sector_0_RG 0x08000000 // - 0x0800 3FFF 16KB
#define sector_1_RG 0x08004000 // - 0x0800 7FFF 16KB
#define sector_2_RG 0x08008000 // - 0x0800 BFFF 16KB
#define sector_3_RG 0x0800C000 // - 0x0800 FFFF 16KB

//// 64kb = 65536 bytes
#define sector_4_RG 0x08010000 // - 0x0801 FFFF 64KB
#define sector_5_RG 0x08020000 // - 0x0803 FFFF 128KB
#define sector_6_RG 0x08040000 // - 0x0805 FFFF 128KB
#define sector_7_RG 0x08060000 // - 0x0807 FFFF 128KB

typedef enum{
	UB_GET 		      	= 0x00U,
	UB_GetVersion    	= 0x01U,
	UB_GetID		  	= 0x02U,
	UB_ReadMemory	  	= 0x11U,
	UB_GO			  	= 0x21U,
	UB_WriteMemory	  	= 0x31U,
	UB_Erase		  	= 0x43U,
	UB_ExtendErase	  	= 0x44U,
	UB_Special 		  	= 0x50U,
	UB_ExtendSpecial  	= 0x51U,
	UB_WriteProtect   	= 0x63U,
	UB_WriteUnProtect 	= 0x73U,
	UB_ReadOutProtect 	= 0x82U,
	UB_ReadOutUnProtect = 0x92U,
	UB_GetChksum		= 0xA1U
}UARTBootloader_CMD;

typedef enum{
	UB_ACK  = 0x79U,
	UB_NACK = 0x1FU,
	UB_Timeout = 0xABU //// defined by myself, non official
}UARTBootloader_state;

void BL_UART_Start(UART_HandleTypeDef *huart);
void BL_UART_Finish();
void BL_UART_UP(UART_HandleTypeDef *huart, uint8_t *databoot);
void BL_UART_GET_CMD(UART_HandleTypeDef *huart, uint8_t *bufferd);
UARTBootloader_state BL_UART_GETID(UART_HandleTypeDef *huart, uint8_t *bufferd);
UARTBootloader_state BL_UART_GETVersion(UART_HandleTypeDef *huart, uint8_t *bufferd);

UARTBootloader_state BL_UART_wait_ACK(UART_HandleTypeDef *huart, uint16_t timeout);
UARTBootloader_state BL_UART_Readout_Protect(UART_HandleTypeDef *huart);
UARTBootloader_state BL_UART_Readout_UnProtect(UART_HandleTypeDef *huart);
UARTBootloader_state BL_UART_Write_Protect(UART_HandleTypeDef *huart);
UARTBootloader_state BL_UART_Write_UnProtect(UART_HandleTypeDef *huart);

#endif /* INC_BOOTLOADER_UART_H_ */
