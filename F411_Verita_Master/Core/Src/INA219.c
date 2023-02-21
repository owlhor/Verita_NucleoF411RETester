/*
 * INA219.c
 *
 *  Created on: Feb 15, 2023
 *      Author: owl_hor
 */

#include "INA219.h"


union{
	uint32_t D32;
	uint16_t D16[2];
	uint8_t D8[4];
}INACBffr;

float current_LSB = INA219_MAX_Expect_Current / 32768.0; // 2^15

/*  how data is stored & merge using INACBffr
 *  DataIN :  ABCD => D8.1, D8.2
 *  D16[0] :  AB 00  // D8[ 1 0 ]
 *  D16[1] :  00 CD  // D8[ 3 2 ]
 *  D16[1] | D16[0] = ABCD
 * */


uint16_t INA219Read_cx(I2C_HandleTypeDef *hi2c,uint8_t dv_addr, uint8_t ina_rg){
	/* @brief : General register read
	 * @param : hi2c - HAL_I2C used to read
	 * @param : dv_addr - address of INA219 device in the busline
	 * @param : ina_rg - register address need to access
	 * @Retval: raw bit value
	 * @ex. answer = INA219Read_cx(&hi2c1, INA219_ADDR_1, INA219_RG_Current);
	 * */

	INACBffr.D32 = 0; //// buffer clear
	HAL_I2C_Mem_Read(hi2c, dv_addr, ina_rg, I2C_MEMADD_SIZE_8BIT, &INACBffr.D8[1], 2, 10);
	return INACBffr.D16[1] | INACBffr.D16[0];
}

void INA219_BitReset(I2C_HandleTypeDef *hi2c,uint8_t dv_addr){
	/* @brief : generates a system reset that is the same as power-on reset.
	 * 			Don't forget to  Re-calibrate or zero Current & power will be returned
	 * @param : hi2c - HAL_I2C used to read
	 * @param : dv_addr - address of INA219 device in the busline
	 * */
	uint8_t resetx = 0x80;
	HAL_I2C_Mem_Write(hi2c, dv_addr, INA219_RG_Config, I2C_MEMADD_SIZE_8BIT, &resetx, 2, 10);

}

INA219_Conf_Strc configura;
void INA219_INIT_Calibrate(I2C_HandleTypeDef *hi2c,uint8_t dv_addr){
	/* @brief : Set initial config and insert calibration parameter
	 * @param : hi2c - HAL_I2C used to read
	 * @param : dv_addr - address of INA219 device in the busline
	 * */

	////// -------------------- Configuration -------------------------------------
	configura.INA219CF.reset = 0;
	configura.INA219CF.BRNG = BRNG_FSR_32V;
	configura.INA219CF.PGA = PGA_GainD4_160mv;
	configura.INA219CF.BADC = ADCI_12bit_532uS;
	configura.INA219CF.SADC = ADCI_12bit_532uS;
	configura.INA219CF.Mode = INAM_ShuntBusV_Continuous;

	uint8_t confictor_si2c[2] = {configura.D8[1], configura.D8[0]};
	HAL_I2C_Mem_Write(hi2c, dv_addr, INA219_RG_Config, I2C_MEMADD_SIZE_8BIT, &confictor_si2c[0], 2, 10);

	////// -------------------- Calibration -------------------------------------
	union{
		uint8_t  U8[2];
		uint16_t U16;
	}calibrator;

	// float current_LSB = INA219_MAX_Expect_Current / 32768.0; // 2^15
	//calibrator.U16 = (int16_t)(trunc( 0.04096 / (current_LSB * INA219_R_SHUNT_Val))) << 1;
	calibrator.U16 = trunc( 0.04096 / (current_LSB * INA219_R_SHUNT_Val));
#ifdef calibrate_EQ6
	calibrator.U16 = trunc((calibrator.U16 * MeaShuntCurrent_ExtMeter) / INA219_Current_Raw);
#endif
	uint8_t calibrator_si2c[2] = {calibrator.U8[1], calibrator.U8[0]}; //// switch byte to send high order first in I2C
	////  ex calibrator(I = 3A, 0.1Rshunt) = 4473 = 0x1179

	HAL_I2C_Mem_Write(hi2c, dv_addr, INA219_RG_Calibra, I2C_MEMADD_SIZE_8BIT, &calibrator_si2c[0], 2, 10);

}

void INA219_INIT(I2C_HandleTypeDef *hi2c,uint8_t dv_addr, INA219_Conf_Strc cfgra){
	/* @brief : Set initial config
	 * @param : hi2c - HAL_I2C used to read
	 * @param : dv_addr - address of INA219 device in the busline
	 * ex.
	    cofgra.INA219CF.reset = 0;
		cofgra.INA219CF.BRNG = BRNG_FSR_16V;
		cofgra.INA219CF.PGA = PGA_GainD8_320mv;
		cofgra.INA219CF.BADC = ADCI_12bit_532uS;
		cofgra.INA219CF.SADC = ADCI_10bit_148uS;
		cofgra.INA219CF.Mode = INAM_ShuntBusV_Continuous;

  	  	INA219_INIT(&hi2c1, INA219_ADDR_1, cofgra);
	 * */

	uint8_t confictor_si2c[2] = {cfgra.D8[1], cfgra.D8[0]};
	HAL_I2C_Mem_Write(hi2c, dv_addr, INA219_RG_Config, I2C_MEMADD_SIZE_8BIT, &confictor_si2c[0], 2, 10);

}

void INA219_Calibrate(I2C_HandleTypeDef *hi2c,uint8_t dv_addr){
	/* @brief : Insert calibration parameter
	 * @param : hi2c - HAL_I2C used to read
	 * @param : dv_addr - address of INA219 device in the busline
	 * */

	////// -------------------- Calibration -------------------------------------
	union{
		uint8_t  U8[2];
		uint16_t U16;
	}calibrator;


	calibrator.U16 = trunc( 0.04096 / (current_LSB * INA219_R_SHUNT_Val));
#ifdef calibrate_EQ6
	calibrator.U16 = trunc((calibrator.U16 * MeaShuntCurrent_ExtMeter) / INA219_Current_Raw);
#endif
	uint8_t calibrator_si2c[2] = {calibrator.U8[1], calibrator.U8[0]}; //// switch byte to send high order first in I2C
	HAL_I2C_Mem_Write(hi2c, dv_addr, INA219_RG_Calibra, I2C_MEMADD_SIZE_8BIT, &calibrator_si2c[0], 2, 10);

}

uint16_t INA219Read_BusV(I2C_HandleTypeDef *hi2c,uint8_t dv_addr){
	/* @brief : Bus Voltage read & calculate
	 * @param : hi2c - HAL_I2C used to read
	 * @param : dv_addr - address of INA219 device in the busline
	 * @Retval: bus voltage in mV
	 * @ex.
	 * */
	//// Bus Voltage Register contents must be shifted right >> by three 3 bits.
	//// At full-scale range = 32 V (decimal = 8000, hex = 1F40), and LSB = 4 mV
	//	if(INACBffr[2] & 0x01){ // still unsure how to deal with CNVR / OVF
//
//	}


	INACBffr.D32 = 0; //// buffer clear
	HAL_I2C_Mem_Read(hi2c, dv_addr, INA219_RG_BusV, I2C_MEMADD_SIZE_8BIT, &INACBffr.D8[1], 2, 10);

	return ((INACBffr.D16[1] | INACBffr.D16[0]) >> 3) * 4;
}

uint16_t INA219Read_Current(I2C_HandleTypeDef *hi2c,uint8_t dv_addr){
	/* @brief : Current read
	 * @param : hi2c - HAL_I2C used to read
	 * @param : dv_addr - address of INA219 device in the busline
	 * @Retval: current in mA
	 * @ex.
	 * */
	INACBffr.D32 = 0; //// buffer clear
	HAL_I2C_Mem_Read(hi2c, dv_addr, INA219_RG_Current, I2C_MEMADD_SIZE_8BIT, &INACBffr.D8[1], 2, 10);

	return INACBffr.D16[1] | INACBffr.D16[0];
}

float INA219Read_ShuntV(I2C_HandleTypeDef *hi2c,uint8_t dv_addr){
	/* @brief : Shunt Voltage read & calculate
	 * @param : hi2c - HAL_I2C used to read
	 * @param : dv_addr - address of INA219 device in the busline
	 * @Retval: shunt voltage in mV (.2f)
	 * @ex.
	 * */
	INACBffr.D32 = 0; //// buffer clear
	HAL_I2C_Mem_Read(hi2c, dv_addr, INA219_RG_ShuntV, I2C_MEMADD_SIZE_8BIT, &INACBffr.D8[1], 2, 10);

	//// Convert rawdata To V shunt from Table 7.Shunt Voltage Register Format

    int16_t rawshunt = INACBffr.D16[1] | INACBffr.D16[0];
    return rawshunt / 100.0;

//    if(rawshunt >= 0){
//    	return rawshunt / 100.0;
//    }else{
//    	// if 2's complements
//    	return (~rawshunt + 1) / 100.0;
//    }
}

float INA219Read_Power(I2C_HandleTypeDef *hi2c,uint8_t dv_addr){
	/* @brief : Power read
	 * @param : hi2c - HAL_I2C used to read
	 * @param : dv_addr - address of INA219 device in the busline
	 * @Retval: Power in mW
	 * @ex.
	 * */
	INACBffr.D32 = 0; //// buffer clear
	HAL_I2C_Mem_Read(hi2c, dv_addr, INA219_RG_PoWer, I2C_MEMADD_SIZE_8BIT, &INACBffr.D8[1], 2, 10);
	////  * 20, power_LSB = 20 x current_LSB & x 1000 make unit in mW
	return ((INACBffr.D16[1] | INACBffr.D16[0]) * (20000.0 * current_LSB));
}
