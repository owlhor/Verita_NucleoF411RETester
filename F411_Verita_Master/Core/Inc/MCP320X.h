/*
 * MCP320X.h
 *
 *  Created on: 31 Jan 2023
 *      Author: owl_hor
 * Last update: 28 Feb 2023
 */

#ifndef INC_MCP320X_H_
#define INC_MCP320X_H_

#include "stm32f4xx_hal.h"

/* DFxy = Differential, x channel is IN+, y channel is IN-
 * CHx  = single-ended at x channel
 * */
//// write enum with long clear text -> call it typedef like gpio
typedef enum _MCPCHSlct{
	M8_Diff_01 = 0b10000,
	M8_Diff_10,
	M8_Diff_23,
	M8_Diff_32,
	M8_Diff_45,
	M8_Diff_54,
	M8_Diff_67,
	M8_Diff_76,
	M8_CH0,
	M8_CH1,
	M8_CH2,
	M8_CH3,
	M8_CH4,
	M8_CH5,
	M8_CH6,
	M8_CH7} MCP3208CHSelect;

typedef enum _REQFig{
	M2_Diff_01 = 0b100,
	M2_Diff_10,
	M2_SE_CH0,
	M2_SE_CH1} MCP3202CHSelec;


typedef union _MCP3202_SET{
	struct MC2
	{
		uint16_t bitread :12;
		uint16_t reserv :2; // LSB
		uint16_t REQFig :2; // MSB
	}MCP3202_U;
	uint16_t U16;
}MCP3202_SET;

//// use 16 bit send and filling,/ 32t in bitfield
//// call state as uint 32 -> config equals to external enum(outside struct)
typedef union _MCP3208_SET{
	struct M8DI_16
	{
		/* from the highest order byte to the lowest order byte and within each byte, from the LSB to the MSB.
		 * Bit 0 LSB
		 * Bit 0 MSB
		 * Bit 1 LSB
		 * Bit 1 MSB
		 * */

		uint32_t resr_null :6;
		uint32_t CHSlct :5;
		uint32_t reserv_start :5;

		uint32_t dontcare :16;


	}MCP3208_DI_16;
	struct M8DO_16
		{
			uint8_t resr_1:4;
			uint8_t MSBreadADC :4;
			uint8_t resr_2 :8;

			uint8_t resr_3 :8;
			uint8_t LSBreadADC :8;

		}MCP3208_DO_16;
	uint16_t U16[2];
	uint8_t U8[4];
}MCP3208_SET;




void MCP3202_READ(SPI_HandleTypeDef *hspi, uint16_t pTrX, uint16_t pRcX);

//uint16_t MCP3208_READ_16_DataSPI(SPI_HandleTypeDef *hspi, MCP3208ChannelSelect M8_channel);
uint16_t MCP3208_READ_8_DataSPI(SPI_HandleTypeDef *hspi, MCP3208CHSelect M8_channel);
uint16_t MCP3202_READ_8_DataSPI(SPI_HandleTypeDef *hspi, MCP3202CHSelec M2_channel);
float MCP320x_ADCbit_to_Volt(uint16_t adcbit);

#endif /* INC_MCP320X_H_ */
