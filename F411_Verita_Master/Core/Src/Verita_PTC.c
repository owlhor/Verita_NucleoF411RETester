/*
 * Verita_PTC.c
 *
 *  Created on: Feb 22, 2023
 *  Lastupdate: Mar 7 , 2023
 *      Author: owl_hor
 */

#include "Verita_PTC.h"

 /* ----------------------- UART Verita Frame -------------------------
  *  V R T + Addr of regis sent data + uint8_t Data[4] + chksum
  *  0x56 0x52 0x54 0xregis 0xda 0xda 0xda 0xda chksum
  *
  *
  *  ====== Sample Usage ======
  *  uint8_t RxBufferMtCl[RxbufferSize_VRT] = {0}; // Recieved packet buffer UART
  *  VRTPTC_StatusTypedef engst; // return engine status
  *  Verita_Register_Bank VRB;   // bank of registers define
  *
  *  HAL_UART_Receive_DMA(&huart6, &RxBufferMtCl[0], RxbufferSize_VRT);
  *
  *  main while(true){
  *  Rx_Verita_engine(RxBufferMtCl, VRB.U32);
  *  }
  */


static enum {init, unpack, decode}verita_engine;

VRTPTC_StatusTypedef Rx_Verita_engine(uint8_t *Rxbffr, uint32_t *regisk){
	/*
	 * @param Rxbffr - input uart buffer
	 * @param regisk - register need the result be stored
	 */
	static uint8_t logger[12] = {0}; /// log Rxbffr without head packet
	static uint8_t index = 0; // use in case the start of verita is not at Rxbffr[0]
	uint8_t chksum[2]  = {0};

	union{
		uint8_t  U8[4];
		uint32_t U32;
	}logu;


	switch (verita_engine){
	default:
	case init:

		if(Rxbffr[0 + index] == 0x56){ //flag_vrt_en ||
			verita_engine = unpack;
		}else{
		//// go to next index if head not found
		index+=9;
		index %= RxbufferSize_VRT; // overflow
		}
		break; //// init


	case unpack:

		//// [] Header Check ------------------------------------------------------------------
		if(Rxbffr[index + 0] == 0x56 && Rxbffr[index + 1] == 0x52 && Rxbffr[index + 2] == 0x54){

			//// log data first / prevent overwrite
			for(register int k = 0; k < 7; k++){
				logger[k] = Rxbffr[index + k + 3];
			}

			//// checksum here
			for(register int i = 0;i < 5; i++){
				chksum[0] += logger[i];
			}
			chksum[1] = ~chksum[0];
			if( chksum[1] == logger[5]){
				// pass
				//// mark that this data is already read
				Rxbffr[0 + index] = 0xFF;
				verita_engine = decode;
			}
			else{  //// checksum wrong
				return VRT_DataLoss;}
			//// mark that this data is already read

		}
		//// else wrong header
		else{
			verita_engine = init;

			index+=9;
			index %= RxbufferSize_VRT; // overflow

//			//// destroy data
//			for(register int i = 0;i < sizeof(Rxbffr); i++){
//				Rxbffr[i] = 0x00;
//			}
			return VRT_UnEnc;

		}
		break; //// unpack

	case decode:
		verita_engine = init;
		index += Framesize_VRT;
		index %= RxbufferSize_VRT; // overflow

		//// DATA phase, insert 32bit data into register box =================================
		if(logger[0] <= 0x20){


#ifdef verita_big_ENDIAN
			//// Big Endian
			logu.U8[3] = logger[1];
			logu.U8[2] = logger[2];
			logu.U8[1] = logger[3];
			logu.U8[0] = logger[4];
#else
			//// Little endian
			logu.U8[0] = logger[1];
			logu.U8[1] = logger[2];
			logu.U8[2] = logger[3];
			logu.U8[3] = logger[4];
#endif
			// place data into the request register
			regisk[logger[0]] = logu.U32;


			return VRT_OK;
		} //// -------------------------------------------------------------------------------

		//// CMD phase, return recieved Command =========================================
		if(logger[0] >= 0x90){
			switch(logger[0]){
				default:
				case 0x90:
					return VRT_ERROR;
				case 0x91:
					return VRT_OK;
				case 0x92:
					return VRT_Busy;
				case 0x93:
					return VRT_Regain;
				case 0x94:
					return VRT_Next;
			}
		}

		//// -------------------------------------------------------------------------------

		break; // decode
	} // end switch
	return VRT_ERROR;
}

//// Normal DMA mode
//VRTPTC_StatusTypedef Rx_Verita_engine(uint8_t *Rxbffr, uint32_t *regisk){
//	/*
//	 * @param Rxbffr - input uart buffer
//	 * @param regisk - register need the result be stored
//	 */
//	static uint8_t logger[12] = {0}; /// log Rxbffr without head packet
//	union{
//		uint8_t  U8[4];
//		uint32_t U32;
//	}logu;
//
//	uint8_t index = 0; // use in case the start of verita is not at Rxbffr[0]
//	uint8_t chksum[2]  = {0};
//
//
//
//	switch (verita_engine){
//	default:
//	case init:
//
//		if(Rxbffr[0] == 0x56){ //flag_vrt_en ||
//			verita_engine = unpack;
//		}
//		break; //// init
//
//
//	case unpack:
//
//		//// [] Header Check ------------------------------------------------------------------
//		if(Rxbffr[index + 0] == 0x56 && Rxbffr[index + 1] == 0x52 && Rxbffr[index + 2] == 0x54){
//
//			//// log data first / prevent overwrite
//			for(register int k = 0; k < 7; k++){
//				logger[k] = Rxbffr[k+3];
//			}
//
//			//// checksum here
//			for(register int i = 0;i < 5; i++){
//				chksum[0] += logger[i];
//			}
//			chksum[1] = ~chksum[0];
//			if( chksum[1] == logger[5]){
//				// pass
//				//// mark that this data is already read
//				Rxbffr[0] = 0xFF;
//				verita_engine = decode;
//			}
//			else{  //// checksum wrong
//				return VRT_DataLoss;}
//
////			//// mark that this data is already read
////			Rxbffr[0] = 0xFF;
////			verita_engine = decode;
//		}
//		//// else wrong header
//		else{
//			verita_engine = init;
//
////			//// destroy data
////			for(register int i = 0;i < sizeof(Rxbffr); i++){
////				Rxbffr[i] = 0x00;
////			}
//			return VRT_UnEnc;
//
//		}
//		break; //// unpack
//
//	case decode:
//		verita_engine = init;
//
//		//// DATA phase, insert 32bit data into register box =================================
//		if(logger[0] <= 0x20){
//
//
//#ifdef verita_big_ENDIAN
//			//// Big Endian
//			logu.U8[3] = logger[1];
//			logu.U8[2] = logger[2];
//			logu.U8[1] = logger[3];
//			logu.U8[0] = logger[4];
//#else
//			//// Little endian
//			logu.U8[0] = logger[1];
//			logu.U8[1] = logger[2];
//			logu.U8[2] = logger[3];
//			logu.U8[3] = logger[4];
//#endif
//			// place data into the request register
//			regisk[logger[0]] = logu.U32;
//
//
//			return VRT_OK;
//		} //// -------------------------------------------------------------------------------
//
//		//// CMD phase, return recieved Command =========================================
//		if(logger[0] >= 0x90){
//			switch(logger[0]){
//				default:
//				case 0x90:
//					return VRT_ERROR;
//				case 0x91:
//					return VRT_OK;
//				case 0x92:
//					return VRT_Busy;
//				case 0x93:
//					return VRT_Regain;
//				case 0x94:
//					return VRT_Next;
//			}
//		}
//
//		//// -------------------------------------------------------------------------------
//
//		break; // decode
//	} // end switch
//	return VRT_ERROR;
//}


void Tx_UART_Verita_Packet_u8(UART_HandleTypeDef *huart, uint8_t regis,uint8_t *pdata, uint8_t size){
	/* @brief Send uint8_t data[4] using UART - Verita packet
	 * @param huart - Pointer to a UART_HandleTypeDef structure that contains
     *                the configuration information for the specified UART module.
	 * @param regis - destination register need packet be inserted
	 * @param pdata - Pointer to data buffer (u8 or u16 data elements).
	 * @param size  - Amount of data elements (u8 or u16) to be received.
	 *
	 * */

	uint8_t posit = 4; // start new position
	uint8_t pack[16] = {0x56, 0x52, 0x54, regis};
	uint8_t chksum = 0;

	//// add data to packet
	for(register int j = 4; j < 4 + size ;j++){
			pack[j] = pdata[j-4];
			posit++;
		}
	//// Checksum generate , +4 means +3 start pack & +1 regis
	for(register int j = 3; j < size + 4;j++){
		chksum += pack[j];
	}
	pack[posit] = ~chksum;


	HAL_UART_Transmit(huart, (uint8_t*)pack, posit+1, 40);
}

void Tx_UART_Verita_Packet_u32(UART_HandleTypeDef *huart, uint8_t regis,uint32_t pdata){
	/* @brief Send uint32_t data using UART - Verita packet
	 * @param huart - Pointer to a UART_HandleTypeDef structure that contains
     *                the configuration information for the specified UART module.
	 * @param regis - destination register need packet be inserted
	 * @param pdata - uint32 data to send
	 * @param size  - Amount of data elements (u8 or u16) to be received.
	 *
	 * */

	//// Verita Header ////
	uint8_t pack[16] = {0x56, 0x52, 0x54, regis};

	uint8_t posit = 4; // start new position
	uint8_t chksum = 0;

	union{
		uint8_t  U8[4];
		uint32_t U32;
	}logu;

	//// add data to packet
	logu.U32 = pdata;
	for(register int j = 4; j < 8; j++){
			pack[j] = logu.U8[j-4];
			posit++;
		}
	//// Checksum generate , +4 means +3 start pack & +1 regis
	for(register int j = 3; j < 8; j++){
		chksum += pack[j];
	}
	pack[posit] = ~chksum;


	HAL_UART_Transmit(huart, (uint8_t*)pack, posit+1, 50);
}
