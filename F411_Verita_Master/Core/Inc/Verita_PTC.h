/*
 * Verita_PTC.h
 *
 *  Created on: Feb 22, 2023
 *  Lastupdate: Mar 7 , 2023
 *      Author: owl_hor
 */

#ifndef INC_VERITA_PTC_H_
#define INC_VERITA_PTC_H_

#include "stm32f4xx_hal.h"

//#define verita_big_ENDIAN

//// BufferSize must be evenly divided by Framesize
#define RxbufferSize_VRT 36
#define Framesize_VRT 9 //// don't modify to any which not 9

//// VRT Register map
#define VR_PA_PUPDR   0x00
#define VR_PA_OUT_PP  0x01
#define VR_PA_OUT_OD  0x02
#define VR_PB_PUPDR   0x03
#define VR_PB_OUT_PP  0x04
#define VR_PB_OUT_OD  0x05
#define VR_PC_PUPDR   0x06
#define VR_PC_OUT_PP  0x07
#define VR_PC_OUT_OD  0x08
#define VR_CPU_Temp   0x11
#define VR_FWID       0x12

typedef enum{
	VRT_ERROR    = 0x90U,
	VRT_OK       = 0x91U,
	VRT_Busy     = 0x92U,
	VRT_Regain   = 0x93U,
	VRT_Next     = 0x94U,
	VRT_DataLoss = 0x95U,
	VRT_UnEnc    = 0x96U
} VRTPTC_StatusTypedef;

typedef union _Verita_Register_Bank
{
	struct VRT_Mk{

		// 0x00 - 0x02
		uint32_t PA_PUPDR;
		uint32_t PA_OUT_PP;
		uint32_t PA_OUT_OD;

		// 0x03 - 0x05
		uint32_t PB_PUPDR;
		uint32_t PB_OUT_PP;
		uint32_t PB_OUT_OD;

		// 0x06 - 0x08
		uint32_t PC_PUPDR;
		uint32_t PC_OUT_PP;
		uint32_t PC_OUT_OD;

		// 0x09 - 0x0F
		uint32_t rsv1[8];

		// 0x11
		uint16_t cputemp;
		uint16_t rsv2;

		// 0x12
		uint32_t FirmwareVer;

		uint32_t rsv3[7];

	} Mark;

	uint32_t U32[20];
} Verita_Register_Bank;


VRTPTC_StatusTypedef Rx_Verita_engine(uint8_t *Rxbffr, uint32_t *regisk);
void Tx_UART_Verita_Packet_u8(UART_HandleTypeDef *huart, uint8_t regis,uint8_t *pdata, uint8_t size);
void Tx_UART_Verita_Packet_u32(UART_HandleTypeDef *huart, uint8_t regis,uint32_t pdata);

#endif /* INC_VERITA_PTC_H_ */
