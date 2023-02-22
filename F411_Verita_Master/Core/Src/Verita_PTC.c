/*
 * Verita_PTC.c
 *
 *  Created on: Feb 22, 2023
 *      Author: owl_hor
 */

#include "Verita_PTC.h"

static enum {init, unpack, decode}verita_engine;

VRTPTC_StatusTypedef Rx_Verita_engine(uint8_t *Rxbffr, uint32_t *regisk){
	/*
	 * @param Rxbffr - input uart buffer
	 * @param regisk - register need the result be stored
	 */
	static uint8_t logger[12] = {0}; /// log Rxbffr without head packet
	union{
		uint8_t  U8[4];
		uint32_t U32;
	}logu;

	//uint8_t chksum = 0;

	switch (verita_engine){
	default:
	case init:

		if(Rxbffr[0] == 0x56){ //flag_vrt_en ||
			verita_engine = unpack;
		}
		break;

	case unpack:

		if(Rxbffr[0] == 0x56 && Rxbffr[1] == 0x52 && Rxbffr[2] == 0x54){

			//// log data first / prevent overwrite
			for(register int k = 0; k < 7; k++){
				logger[k] = Rxbffr[k+3];
			}

			//// checksum here
//			for(register int i = 0;i < 5; i++){
//				chksum += logger[i];
//			}
//			if(~chksum == logger[6]){
//				// pass
//			}

			//// mark that this data is already read
			Rxbffr[0] = 0xFF;
			verita_engine = decode;
		}
		else{
			verita_engine = init;

			//// destroy data
			for(register int i = 0;i < sizeof(Rxbffr); i++){
				Rxbffr[i] = 0x00;
			}
		}
		break;

	case decode:
		verita_engine = init;

		//// DATA phase, insert 32bit data into register box
		if(logger[0] <= 0x20){

			logu.U8[3] = logger[1];
			logu.U8[2] = logger[2];
			logu.U8[1] = logger[3];
			logu.U8[0] = logger[4];

			regisk[logger[0]] = logu.U32;
			return VRT_OK;
		}

		//// CMD phase, return recieved Command
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

		break;
	}
}
