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

//// BufferSize in byte must be evenly divided by Framesize
/* RxbufferSize_VRT = 12n byte = n slot, n > 10++ - for classic engine
 * RxbufferSize_VRT = 9 byte ("must be") for callBak-version engine, read 1 slot after callback
 * */
#define RxbufferSize_VRT 36
#define Framesize_VRT 9 //// don't modify to any which not 9

//// VRT Register map
#define VR_PA_PUPDR   0x01
#define VR_PA_OUT_PP  0x02
#define VR_PA_OUT_OD  0x03
#define VR_PB_PUPDR   0x04
#define VR_PB_OUT_PP  0x05
#define VR_PB_OUT_OD  0x06
#define VR_PC_PUPDR   0x07
#define VR_PC_OUT_PP  0x08
#define VR_PC_OUT_OD  0x09

#define VR_DataReq	  0x00
#define VR_CPU_Temp   0x11
#define VR_FWID       0x12

//// VRT Flag
#define VRF_GPIO_Runalltest 0x02

typedef enum{
	VRT_ERROR    = 0x90U,
	VRT_OK       = 0x91U,
	VRT_Busy     = 0x92U,
	VRT_DataLoss = 0x93U,
	VRT_UnEnc    = 0x94U
} VRTPTC_StatusTypedef;

typedef enum{
	VRC_Request  = 0xA0U,
	VRC_Next 	 = 0xA1U,
	VRC_Flag_1	 = 0xA2U
} VRTPTC_CMDef;

typedef union _Verita_Register_Bank
{
	struct VRT_Mk{

		// 0x00
		uint16_t DataReq;
		uint16_t flag_dataREQ;

		// 0x01 - 0x03
		uint32_t PA_PUPDR;
		uint32_t PA_OUT_PP;
		uint32_t PA_OUT_OD;

		// 0x04 - 0x06
		uint32_t PB_PUPDR;
		uint32_t PB_OUT_PP;
		uint32_t PB_OUT_OD;

		// 0x07- 0x09
		uint32_t PC_PUPDR;
		uint32_t PC_OUT_PP;
		uint32_t PC_OUT_OD;

		// 0xA B C D E F- 0x10
		uint32_t rsv1[7];

		// 0x11
		uint16_t cputemp;
		uint16_t rsv2;

		// 0x12
		uint32_t FirmwareVer;

		// 0x13 general flag
		uint8_t Flag_ger;
		uint8_t Flag_next; // next flag
		uint8_t Flag_aa;  // reserve
		uint8_t Flag_bb;

		//uint32_t rsv3[6];

	} Mark;

	uint32_t U32[25];
} Verita_Register_Bank;


VRTPTC_StatusTypedef Rx_Verita_engine(uint8_t *Rxbffr, Verita_Register_Bank *regisk);
VRTPTC_StatusTypedef Tx_Rq_Verita_engine(UART_HandleTypeDef *huart, Verita_Register_Bank *vrg_intn);
void Tx_UART_Verita_Packet_u8(UART_HandleTypeDef *huart, uint8_t regis,uint8_t *pdata, uint8_t size);
void Tx_UART_Verita_Packet_u32(UART_HandleTypeDef *huart, uint8_t regis,uint32_t pdata);
void Tx_UART_Verita_Command(UART_HandleTypeDef *huart, VRTPTC_CMDef cmd, uint8_t regis);

#endif /* INC_VERITA_PTC_H_ */
