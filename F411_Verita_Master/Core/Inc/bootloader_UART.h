/*
 * bootloader_UART.h
 *
 *  Created on: Mar 7, 2023
 *      Author: owl_hor
 */

#ifndef INC_BOOTLOADER_UART_H_
#define INC_BOOTLOADER_UART_H_

#include "stm32f4xx_hal.h"

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


#endif /* INC_BOOTLOADER_UART_H_ */
