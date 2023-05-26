/*
 * MCP320X.c
 *
 *  Created on: 31 Jan 2023
 *      Author: owl_hor
 * Last update: 28 Feb 2023
 */

#include "MCP320X.h"

#define MCP3202_SPI_CS_Port GPIOA
#define MCP3202_SPI_CS_Pin  GPIO_PIN_8

#define MCP3208_SPI_CS_Port  GPIOD
#define MCP3208_SPI_CS_Pin   GPIO_PIN_2



uint16_t MCP3202_READ_8_DataSPI(SPI_HandleTypeDef *hspi, MCP3202CHSelec M2_channel){

	//// Shitty bitshift to the correct position Fig 6-1, MCP3208, MICROCHIP
	uint8_t D8_MOSI[3];
	uint8_t D8_MISO[3];
	D8_MOSI[0] = M2_channel >> 2;
	D8_MOSI[1] = (M2_channel << 6) | 0x20;


	HAL_GPIO_WritePin(MCP3202_SPI_CS_Port,MCP3202_SPI_CS_Pin , GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(hspi, &D8_MOSI[0], &D8_MISO[0], 3, 100);

	//HAL_SPI_Abort(hspi);
	HAL_GPIO_WritePin(MCP3202_SPI_CS_Port,MCP3202_SPI_CS_Pin , GPIO_PIN_SET);

	return ((D8_MISO[1] << 8) + D8_MISO[2]) & 0x0FFF;
}

/* Read using SPI 16 Bit Data size MSB first
 * Set CS back with CpltCallback
 * */

//uint16_t MCP3208_READ_16_DataSPI(SPI_HandleTypeDef *hspi, MCP3208ChannelSelect M8_channel){
//
//	uint16_t D_MOSI[2] = {M8_channel << 5, 0x0000};
//	uint16_t D_MISO[2] = {0};
//
//	HAL_GPIO_WritePin(MCP3208_SPI_CS_Port,MCP3208_SPI_CS_Pin , GPIO_PIN_RESET);
//
//	HAL_SPI_TransmitReceive_IT(hspi, D_MOSI[0], D_MISO[0], 2);
//
//	return (D_MISO[0] + (D_MISO[1] >> 8) )& 0x0FFF;
//
//}

/* Read using SPI 8 Bit Data size MSB first
 * Ex.
 * AA_bitread = MCP3208_READ_8_DataSPI(&hspi3, M8_CH0);
	VADC_cv =  MCP3208_ADCbit_to_Volt(AA_bitread); // 5 / 4096 * 0.00122
 * */
uint16_t MCP3208_READ_8_DataSPI(SPI_HandleTypeDef *hspi, MCP3208CHSelect M8_channel){

	//// Shitty bitshift to the correct position Fig 6-1, MCP3208, MICROCHIP
	uint8_t D8_MOSI[3];
	uint8_t D8_MISO[3];
	D8_MOSI[0] = M8_channel >> 2;
	D8_MOSI[1] = M8_channel << 6;


	HAL_GPIO_WritePin(MCP3208_SPI_CS_Port,MCP3208_SPI_CS_Pin , GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(hspi, &D8_MOSI[0], &D8_MISO[0], 3, 100);

	//HAL_SPI_Abort(hspi);
	HAL_GPIO_WritePin(MCP3208_SPI_CS_Port,MCP3208_SPI_CS_Pin , GPIO_PIN_SET);

	return ((D8_MISO[1] << 8) + D8_MISO[2]) & 0x0FFF;
}


float MCP320x_ADCbit_to_Volt(uint16_t adcbit){
	return adcbit * 0.00122; // 5/4096
	//return adcbit * 0.001215; // 4.98/4096
	//return adcbit * 0.001225; // 5.02/4096
}

