/*
 * bootloader_UART.c
 *
 *  Created on: Mar 7, 2023
 *      Author: owl_hor
 */

#include "bootloader_UART.h"

/* STM32F411xx UART Bootloader
 *
 * notion record: https://celestial-pawpaw-92b.notion.site/ST-Link-USART-Bootloader-3128670163404f21b8770e02421ad07f#bded18ef2cf74c3fbfe68c81710f87cb
 *
 * Reference:
 * AN2606: General bootloader description, STMicroelectronics
 * AN3515: USART protocol used in the STM32 bootloade, STMicroelectronics
 *
 * ------======= External NMOS clamp target's NRST pin is required (recommend)-----------
 * 		or else, Push Low to Target's NRST directly (the Start, finish must be adapted)
 *
 * In booting step, The host's other operation (especially display writing) must be terminated
 *
 * 1. Push BOOT0 pin high
 * 2. Hardware Reset client toggle (NRST client must be separated to master)
 * 3. Wait for bootloader to start, following Bootloader startup timing diagram for 74.5ms ++
 * 4. Master send 0x7F to client’s UART bootloader port1
 * 		(PA9_TX, PA10_RX), wait for ACK
 * 5. CMD Applies
 * 6. When finish, Pull boot0 low & generate hardware reset
 *
 *  ---- sample of usages ----
 *  BL_UART_GET_CMD(&huart1, BL_UARTBuffer);
 *  BL_UART_GETVersion(&huart1, BL_UARTBuffer);
 *  BL_UART_GETID(&huart1, BL_UARTBuffer);
 *  BL_UART_ReadMem(&huart1, 0x08000000U, 255, BL_MemBuffer);
 *  BL_UART_Go(&huart1, 0x08007910U);
 *
 *  ---- These function must be used carefully, it can access option bytes ----
 *  ---  danger operation: Bootloader Lock (Temporary or forever) Risk
 *  BL_UART_Readout_Protect(UART_HandleTypeDef *huart);
 *  BL_UART_Readout_UnProtect(UART_HandleTypeDef *huart);
 *  BL_UART_Write_Protect(UART_HandleTypeDef *huart);
 *  BL_UART_Write_UnProtect(UART_HandleTypeDef *huart);
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

	//// reserve, pending for any operation that might be unfinished
	HAL_Delay(100);
	//// pull BOOT0 back
	HAL_GPIO_WritePin(BOOT0_Trigger_GPIO_PORT, BOOT0_Trigger_GPIO_PIN, GPIO_PIN_RESET);

	//// step 2: Reset Client, push SET NMOS
	HAL_GPIO_WritePin(Client_NRST_Trg_GPIO_PORT, Client_NRST_Trg_GPIO_PIN, GPIO_PIN_SET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(Client_NRST_Trg_GPIO_PORT, Client_NRST_Trg_GPIO_PIN, GPIO_PIN_RESET);
}

UARTBootloader_state BL_UART_GET_CMD(UART_HandleTypeDef *huart, uint8_t *bufferd){
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
		return UB_NACK;
//// for debug
//		for(int i = 0; i < 10; i++){
//			bufferd[i] = 0xAA;
//		}
	}else{}

	return UB_ACK;
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

UARTBootloader_state BL_UART_ReadMem(UART_HandleTypeDef *huart,uint32_t addr32, uint8_t numbyte, uint8_t *bufferd){
	/* @brief code 0x11 Reads up to 256 bytes of memory starting from an
	 * 			address specified by the application
	 * @param huart   - Pointer to a UART_HandleTypeDef structure that contains
     *                  the configuration information for the specified UART module.
	 * @param addr32  - start destination address to read (0x xxxx xxxx)
	 * @param numbyte - num of bytes need to be read (0 < n <= 255)
	 * @param bufferd - buffer to store read result
	 * */
	uint8_t Start_RM[2] = {0x11, 0xEE};
	uint8_t response = 0;
	uint8_t numbytx[2] = {0};
	uint8_t addr8[5] = {0};

	union{
		uint8_t  U8[4];
		uint32_t U32;
	}loga;

	//// block from do sth in danger zone
	if(addr32 >= 0x1FFF0000 && addr32 <= 0x1FFFFFFF){
		return UB_ParamERR;
	}


	loga.U32 = addr32;
	//// prepare address byte 3: MSB, byte 6: LSB
	addr8[0] = loga.U8[3];
	addr8[1] = loga.U8[2];
	addr8[2] = loga.U8[1];
	addr8[3] = loga.U8[0];

	//// XOR Chksum addr
	addr8[4] = addr8[0]^addr8[1]^addr8[2]^addr8[3];

	//// XOR Chksum numbyte
	numbytx[0] = numbyte;
	numbytx[1] = (uint8_t)(0 - numbyte)- 1; //// complements

	//// Bytes 1-2
	HAL_UART_Transmit(huart, &Start_RM[0], 2, 10);

	response = BL_UART_wait_ACK(huart, 10);
	if(response == UB_ACK){

		//// Bytes 3-6 Send ADDR Bytes +
		//// Byte  7 chksum
		HAL_UART_Transmit(huart, &addr8[0], 5, 15);

		response = BL_UART_wait_ACK(huart, 10);
		if(response == UB_ACK){

			//// byte 8-9 numbyte+chksum
			HAL_UART_Transmit(huart, &numbytx[0], 2, 15);

			response = BL_UART_wait_ACK(huart, 10);
			if(response == UB_ACK){

				//// Recieve data
				HAL_UART_Receive(huart, bufferd, numbyte + 1, 60);

				return UB_ACK;

			}else{return UB_NACK;}

		}else{return UB_NACK;}

	}else{return UB_NACK;}

	return UB_NACK;
}

UARTBootloader_state BL_UART_Go(UART_HandleTypeDef *huart,uint32_t addr32){
	/* code 0x21 Jumps to user application code located in the
	 * 			 internal flash memory or in the SRAM.
	 * @param huart   - Pointer to a UART_HandleTypeDef structure that contains
     *                  the configuration information for the specified UART module.
	 * @param addr32  - start destination address to read (0x xxxx xxxx)
	 * After the command is completed ->
	 * - initializes the registers of the peripherals used
	 *    by the bootloader to their default reset values
	 * - initializes the main stack
	 * */
	uint8_t Start_Go[2] = {0x21, 0xDE};
	uint8_t response = 0;
	uint8_t addr8[5] = {0};

	union{
		uint8_t  U8[4];
		uint32_t U32;
	}loga;

	//// block from do sth in danger zone
	if(addr32 >= 0x1FFF0000 && addr32 <= 0x1FFFFFFF){
		return UB_ParamERR;
	}


	loga.U32 = addr32;
	//// prepare address byte 3: MSB, byte 6: LSB
	addr8[0] = loga.U8[3];
	addr8[1] = loga.U8[2];
	addr8[2] = loga.U8[1];
	addr8[3] = loga.U8[0];

	//// XOR Chksum addr
	addr8[4] = addr8[0]^addr8[1]^addr8[2]^addr8[3];


	//// Bytes 1-2
	HAL_UART_Transmit(huart, &Start_Go[0], 2, 10);

	response = BL_UART_wait_ACK(huart, 10);
	if(response == UB_ACK){

		//// Bytes 3-6 Send ADDR Bytes +
		//// Byte  7 chksum
		HAL_UART_Transmit(huart, &addr8[0], 5, 15);

		response = BL_UART_wait_ACK(huart, 10);
		if(response == UB_ACK){
				return UB_ACK;

		}else{return UB_NACK;}

	}else{return UB_NACK;}

	return UB_NACK;
}

UARTBootloader_state BL_UART_WriteMem(UART_HandleTypeDef *huart,uint32_t addr32, uint8_t numbyte,const uint8_t *wdatum){
	/* @brief code 0x31 Writes up to 256 bytes to the RAM or flash memory starting from an
						address specified by the application
	 * @param huart   - Pointer to a UART_HandleTypeDef structure that contains
     *                  the configuration information for the specified UART module.
	 * @param addr32  - start destination address to write (0x xxxx xxxx)
	 * @param numbyte - num of bytes need to write (0 < n <= 255)
	 * @param wdatum  - write data bytes
	 *
	 * - 16 Mar 23 - *** To write all new datas to the target, Mass erased is required ***
	 *
	 *
	 * - gets a byte, N, which contains the number of data bytes to be received
	 * - receives the user data ((N + 1) bytes) and the checksum (XOR of N and of all data bytes)
	 * - programs the user data to memory starting from the received address
	 * - at the end of the command, if the write operation was successful, the bootloader
	 * 		transmits the ACK byte; otherwise it transmits an NACK byte to the application and aborts the command.
	 * */
	uint8_t Start_WM[2] = {0x31, 0xCE};
	uint8_t response = 0;
	uint8_t numbytx[2] = {0};
	uint8_t addr8[5] = {0};

	uint8_t bffr[260] = {0};

	union{
		uint8_t  U8[4];
		uint32_t U32;
	}loga;

	//// block from do sth in danger zone (option bytes, system memory)
	if(addr32 >= 0x1FFF0000 && addr32 <= 0x1FFFFFFF){
		return UB_ParamERR;
	}

	loga.U32 = addr32;
	//// prepare address byte 3: MSB, byte 6: LSB
	addr8[0] = loga.U8[3];
	addr8[1] = loga.U8[2];
	addr8[2] = loga.U8[1];
	addr8[3] = loga.U8[0];

	//// XOR Chksum addr
	addr8[4] = addr8[0]^addr8[1]^addr8[2]^addr8[3];

	//// XOR Chksum  (XOR of N and of all data bytes)
	numbytx[0] = numbyte;

	bffr[0] = numbyte;

	numbytx[1] = numbyte;
	for(register int i = 0;i <= numbyte;i++){
		numbytx[1] = numbytx[1]^wdatum[i];
		//// try
		bffr[i+1] = wdatum[i]; // bffr[i] = wdatum[i];
	}

	// add chksum to the last buffer,
	bffr[numbyte+2] = numbytx[1];

	//// Bytes 1-2
	HAL_UART_Transmit(huart, &Start_WM[0], 2, 10);

	response = BL_UART_wait_ACK(huart, 10);
	if(response == UB_ACK){

		//// Bytes 3-6 Send ADDR Bytes +
		//// Byte  7 chksum
		HAL_UART_Transmit(huart, &addr8[0], 5, 15);

		response = BL_UART_wait_ACK(huart, 10);
		if(response == UB_ACK){

//			//// byte 8-n numbyte+ Writedata +chksum -->send chksum separate cause leadtime -> Very much NAK occur
//			HAL_UART_Transmit(huart, &numbytx[0], 1, 5);
//			HAL_UART_Transmit(huart, wdatum, numbyte+1, 100);
//			HAL_UART_Transmit(huart, &numbytx[1], 1, 5); //// chksum

			//// byte 8-n numbyte+ Writedata +chksum
			//HAL_UART_Transmit(huart, &numbytx[0], 1, 5);
			HAL_UART_Transmit(huart, bffr, numbyte+3, 100); //

			response = BL_UART_wait_ACK(huart, 10);
			if(response == UB_ACK){
				return UB_ACK;
			}else{return UB_NACK;}


		}else{return UB_NACK;}

	}else{return UB_NACK;}

	return UB_NACK;
}

UARTBootloader_state BL_UART_ExtendEraseMem_SP(UART_HandleTypeDef *huart,UARTBootloader_Erase_CMD erasa){
	/* @brief code 0x44  Erases from one to all the flash memory pages using two-byte addressing mode
	 * 				(available only for USART bootloader v3.0 and higher)
	 * 				*** This function supports special erase only ***
	 * @param huart   - Pointer to a UART_HandleTypeDef structure that contains
     *                  the configuration information for the specified UART module.
     * @param erasa N number of pages to be erased, Use UARTBootloader_Erase_CMD as special erase
	 *
	 * */
	uint8_t Start_EER[2] = {0x44, 0xBB};
	uint8_t response = 0;
	uint8_t cmd[3] = {0};

	union{
		uint8_t  U8[2];
		uint16_t U16;
	}loga;

	loga.U16 = erasa;

	//// prepare Special erase CMD & Chksum
	cmd[0] = loga.U8[1];
	cmd[1] = loga.U8[0];
	cmd[2] = loga.U8[0]^loga.U8[1];

	//// Bytes 1-2
	HAL_UART_Transmit(huart, &Start_EER[0], 2, 10);

	response = BL_UART_wait_ACK(huart, 10);
	if(response == UB_ACK){

		//// Bytes 3-4 Send Special erase +
		//// Byte  5 chksum
		HAL_UART_Transmit(huart, &cmd[0], 3, 15);

		response = BL_UART_wait_ACK(huart, 10);
		if(response == UB_ACK){
				return UB_ACK;

		}else{return UB_NACK;}

	}else{return UB_NACK;}

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
	/* *** Danger -> Gives the result as Bootloader Lock ***
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


//UARTBootloader_state BL_UART_WriteMem_d(UART_HandleTypeDef *huart,uint32_t addr32, uint8_t numbyte,const uint8_t *wdatum){
//	/* 	0x31 dummy
//	 * */
//	uint8_t Start_WM[2] = {0x31, 0xCE};
//	uint8_t numbytx[2] = {0};
//	uint8_t addr8[5] = {0};
//
//	union{
//		uint8_t  U8[4];
//		uint32_t U32;
//	}loga;
//
//	//// block from do sth in danger zone (option bytes, system memory)
//	if(addr32 >= 0x1FFF0000 && addr32 <= 0x1FFFFFFF){
//		return UB_ParamERR;
//	}
//
//	loga.U32 = addr32;
//	//// prepare address byte 3: MSB, byte 6: LSB
//	addr8[0] = loga.U8[3];
//	addr8[1] = loga.U8[2];
//	addr8[2] = loga.U8[1];
//	addr8[3] = loga.U8[0];
//
//	//// XOR Chksum addr
//	addr8[4] = addr8[0]^addr8[1]^addr8[2]^addr8[3];
//
//	//// XOR Chksum  (XOR of N and of all data bytes)
//	numbytx[0] = numbyte;
//	//numbytx[1] = (uint8_t)(0 - numbyte)- 1; //// complements
//	numbytx[1] = numbyte;
//	for(register int i = 0;i < numbyte;i++){
//		numbytx[1] = numbytx[1]^wdatum[i];
//	}
//
//	//// Bytes 1-2
//	HAL_UART_Transmit(huart, &Start_WM[0], 2, 10);
//	HAL_Delay(1);
//
//		//// Bytes 3-6 Send ADDR Bytes +
//		//// Byte  7 chksum
//		HAL_UART_Transmit(huart, &addr8[0], 5, 15);
//		HAL_Delay(1);
//			//// byte 8-n numbyte+ Writedata +chksum
//			HAL_UART_Transmit(huart, &numbytx[0], 1, 5);
//			HAL_UART_Transmit(huart, wdatum, numbyte+1, 50);
//			HAL_UART_Transmit(huart, &numbytx[1], 1, 5); //// chksum
//
//
//	return UB_ACK;
//}

