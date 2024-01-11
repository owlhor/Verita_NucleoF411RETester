/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

  /* ================ VERITA: Nucleo G474RE Tester ====  --->>>  Client  >>>----
   *
   * Author: owl_hor, FRAB#7 @FIBO KMUTT
   * Advisor: Pittiwut Teerakittikul, Ph.D & Puttinart Archewawanich (AlphaP2712)
   * --------------------------------------------------------------------------
   *  ===================== peripheral Usage  =====================
   *  - [ADC1]   -> Read MCU's temperature
   *  - [LPUART1] -> Send Results to UART Console
   *
   *  	(28 July 2023)
   *	BETA - BETA - BETA - BETA - BETA - BETA - BETA - BETA - BETA - BETA
   *	This script is still beta, Some hole can be occurred.
   *	Verita PTC is not completely implemented in this script.
   *
   */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "gpio_testscript.h"
#include "Verita_PTC.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIRMWARE_VER 0xBB261223 // 01 00 03 23  -- ver day month year 32-bit

#define fakeoffset 6 // MCU Temp Celcius offset fake
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */

//// ------------ ADC -----------
typedef struct{
	ADC_ChannelConfTypeDef Confix;
	uint16_t datt;
}ADCStructure;

ADCStructure ADCChannell[1];

uint16_t cputmpraw = 0;
float cputempCC = 0;

///// ----  ADC DMA --------
union _ADCRawread{
	uint16_t u16[14];
	uint32_t u32[7];
}ADCRawread;
//uint32_t ADCRawreadc[10] = {0};

uint16_t TSCALT1 = TEMPSENSOR_CAL1_TEMP;
uint16_t TSCALT2 = TEMPSENSOR_CAL2_TEMP;
uint16_t TCAL1; // read the ST factory calibrate value from the address = *TEMPSENSOR_CAL1_ADDR
uint16_t TCAL2;


//////// -- General Var ---------
uint8_t bluecounter = 0;
uint8_t counter_flagger = 0; // countif testscript is runned
uint8_t cnt_allpass = 0;
uint32_t timestamp_one = 0;
uint32_t timestamp_selftestdelay = 0;

//// --------- GPIO Read Buffer --------
uint8_t flag_gpioselftest = 0;

//// record GPIO setting before run testscript, set back after finish test
uint32_t gpio_rec_mode[3] = {0};
uint32_t gpio_rec_pupdr[3] = {0};

//// lists All port - pin to inspect first // avoid special pin like osilators / UART
//// GPIO_PIN_x is in bit position format (0 2 4 8 16 ...) which loss if stored in that form and log2() to calculate back
uint16_t List_GPIOA[] = {0,1,    4,5,6,7,8,9,10,11,12,      15,  20}; // 2,3 STLK RXTX / / 13 14 TMS TCK /
uint16_t List_GPIOB[] = {0,1,2,  4,5,6,7,8,9,10,11,12,13,14,15,  20}; // 3 SWO,
uint16_t List_GPIOC[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,        20}; // 14 15 OSC

char WR_A_PUPDR[40] = "\r\nA_PUR: "; ////
char WR_B_PUPDR[40] = "\r\nB_PUR: ";
char WR_C_PUPDR[40] = "\r\nC_PUR: ";
char WR_A_OPP[40]   = "\r\nA_OPP: "; ////
char WR_B_OPP[40]   = "\r\nB_OPP: ";
char WR_C_OPP[40]   = "\r\nC_OPP: ";
char WR_A_OOD[40]   = "\r\nA_OOD: ";
char WR_B_OOD[40]   = "\r\nB_OOD: ";
char WR_C_OOD[40]   = "\r\nC_OOD: ";

//// --------- ============== Verita PTC Register ============== ------------
Verita_Register_Bank VR_Cli;
uint8_t RxBufferMtCl[RxbufferSize_VRT] = {0}; // Recieved packet buffer
VRTPTC_StatusTypedef rslt;

//// ----------------------  UART Buffer  -------------------
char uartTXBf[100] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void CPUTemprdINIT();
uint16_t CPUTempread();
float ADCTVoltar(uint16_t btt, float vref);
float TempEquat(float Vs);
float Tempequat_G4(int16_t data, float vref);

void Compare_pin_32(uint32_t raw32, uint16_t *Lista_GPIOx, uint8_t gpst,char *outchar);
void resetgpio_char();
void CheckAllPass();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  char temp[]="\r\n----------------- G474_Verita_Client --------------------\r\n";
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)temp, strlen(temp),10);

  sprintf(uartTXBf, "Firmware ver: %08X \r\n ", FIRMWARE_VER);
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)uartTXBf, strlen(uartTXBf),10);


  //// ADC Start --------------------------
  //CPUTemprdINIT();

  HAL_ADC_Start_DMA(&hadc1, ADCRawread.u32, 6);
  //HAL_ADC_Start_DMA(&hadc1, ADCRawreadc, 6);

  //// ------------- UART Recieve : Circular DMA here--------------------------
  //HAL_UART_Receive_DMA(&huart6, &RxBufferMtCl[0], RxbufferSize_VRT);
  VR_Cli.Mark.FirmwareVer = FIRMWARE_VER;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  if(HAL_GetTick() >= timestamp_one){
	  		  timestamp_one += 1000;
	  		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	  		  //cputmpraw = CPUTempread();

	  		  sprintf(uartTXBf, "\r\n - - - - - - - - VERITA - - - G474RE - - - - - BETA - - - - - -\r\n");
	  		  HAL_UART_Transmit(&hlpuart1, (uint8_t*)uartTXBf, strlen(uartTXBf),10);

	  		  sprintf(uartTXBf, "Vrefint : %4d = %.2f V \r\n", (uint16_t)ADCRawread.u16[1], ADCTVoltar(ADCRawread.u16[1], 3.32));
	  		  HAL_UART_Transmit(&hlpuart1, (uint8_t*)uartTXBf, strlen(uartTXBf),10);
	  		  //// CPU temp reoprt // incomplete, odd value, formula
	  		  sprintf(uartTXBf, "TempMCU : %4d = %.2f C , (+ %d offset)  \r\n ", (uint16_t)ADCRawread.u16[0],
	  				Tempequat_G4(ADCRawread.u16[0], 3.3), fakeoffset);
	  		  HAL_UART_Transmit(&hlpuart1, (uint8_t*)uartTXBf, strlen(uartTXBf),10);
	  		  ////

	  		  //// ALL PASS report ////
			  //// Print GPIO Test Result
			  if(counter_flagger){
				  if(cnt_allpass >= 9){//// there're 9 pass

					  sprintf(uartTXBf, "\r\n+++ GPIO ALL PASS +++\r\n"); HAL_UART_Transmit(&hlpuart1, (uint8_t*)uartTXBf, strlen(uartTXBf),10);
				  }else{
					  sprintf(uartTXBf, "\r\n--- GPIO unhealthy ---\r\n"); HAL_UART_Transmit(&hlpuart1, (uint8_t*)uartTXBf, strlen(uartTXBf),10);
				  }

				  sprintf(uartTXBf, WR_A_PUPDR); HAL_UART_Transmit(&hlpuart1, (uint8_t*)uartTXBf, strlen(uartTXBf),10);

				  sprintf(uartTXBf, WR_A_OPP); HAL_UART_Transmit(&hlpuart1, (uint8_t*)uartTXBf, strlen(uartTXBf),10);

				  sprintf(uartTXBf, WR_A_OOD); HAL_UART_Transmit(&hlpuart1, (uint8_t*)uartTXBf, strlen(uartTXBf),10);

				  sprintf(uartTXBf, WR_B_PUPDR); HAL_UART_Transmit(&hlpuart1, (uint8_t*)uartTXBf, strlen(uartTXBf),10);

				  sprintf(uartTXBf, WR_B_OPP); HAL_UART_Transmit(&hlpuart1, (uint8_t*)uartTXBf, strlen(uartTXBf),10);

				  sprintf(uartTXBf, WR_B_OOD); HAL_UART_Transmit(&hlpuart1, (uint8_t*)uartTXBf, strlen(uartTXBf),10);

				  sprintf(uartTXBf, WR_C_PUPDR); HAL_UART_Transmit(&hlpuart1, (uint8_t*)uartTXBf, strlen(uartTXBf),10);

				  sprintf(uartTXBf, WR_C_OPP); HAL_UART_Transmit(&hlpuart1, (uint8_t*)uartTXBf, strlen(uartTXBf),10);

				  sprintf(uartTXBf, WR_C_OOD); HAL_UART_Transmit(&hlpuart1, (uint8_t*)uartTXBf, strlen(uartTXBf),10);
			  }
	  		  ////

	  		 }// timestamp_one

	  	  if(flag_gpioselftest){
	  		  //// delay wait for button release
	  		  if (HAL_GetTick() >= timestamp_selftestdelay){

	  			  VR_Cli.Mark.Flag_ger = VRF_GPIO_Runalltest;
	  			  flag_gpioselftest = 0;
	  		  }
	  	  } //// flag_gpioselftest

	  	//// Flag run all test
	  if(VR_Cli.Mark.Flag_ger == VRF_GPIO_Runalltest){
		  counter_flagger++;

		  ////record default GPIO setup before modified in testscript
		  gpio_rec_mode[0] = GPIOA->MODER;
		  gpio_rec_pupdr[0] = GPIOA->PUPDR;

		  //// Run GPIO Testscript all here or run before While
		  VR_Cli.Mark.PA_PUPDR = gpio_selftest_input_pupdr_1(GPIOA, List_GPIOA);
		  VR_Cli.Mark.PB_PUPDR = gpio_selftest_input_pupdr_1(GPIOB, List_GPIOB);
		  VR_Cli.Mark.PC_PUPDR = gpio_selftest_input_pupdr_1(GPIOC, List_GPIOC);

		  HAL_Delay(5);

		  VR_Cli.Mark.PA_OUT_PP = gpio_selftest_output_pp_1(GPIOA, List_GPIOA);
		  VR_Cli.Mark.PB_OUT_PP = gpio_selftest_output_pp_1(GPIOB, List_GPIOB);
		  VR_Cli.Mark.PC_OUT_PP = gpio_selftest_output_pp_1(GPIOC, List_GPIOC);

		  HAL_Delay(5);

		  VR_Cli.Mark.PA_OUT_OD = gpio_selftest_output_od_1(GPIOA, List_GPIOA);
		  VR_Cli.Mark.PB_OUT_OD = gpio_selftest_output_od_1(GPIOB, List_GPIOB);
		  VR_Cli.Mark.PC_OUT_OD = gpio_selftest_output_od_1(GPIOC, List_GPIOC);

		  //// revert back, enable to send UART again after crashed in testscript
		  GPIOA->MODER = gpio_rec_mode[0] ;
		  GPIOA->PUPDR = gpio_rec_pupdr[0] ;

		  //// clear previous buffer
		  resetgpio_char();
		  ////Compare_pin()
		  Compare_pin_32(VR_Cli.Mark.PA_PUPDR, List_GPIOA, 0, WR_A_PUPDR);
		  Compare_pin_32(VR_Cli.Mark.PA_OUT_PP, List_GPIOA, 0, WR_A_OPP);
		  Compare_pin_32(VR_Cli.Mark.PA_OUT_OD, List_GPIOA, 0, WR_A_OOD);

		  Compare_pin_32(VR_Cli.Mark.PB_PUPDR, List_GPIOB, 1,  WR_B_PUPDR);
		  Compare_pin_32(VR_Cli.Mark.PB_OUT_PP, List_GPIOB, 1, WR_B_OPP);
		  Compare_pin_32(VR_Cli.Mark.PB_OUT_OD, List_GPIOB, 1, WR_B_OOD);

		  Compare_pin_32(VR_Cli.Mark.PC_PUPDR, List_GPIOC, 2, WR_C_PUPDR);
		  Compare_pin_32(VR_Cli.Mark.PC_OUT_PP, List_GPIOC, 2, WR_C_OPP);
		  Compare_pin_32(VR_Cli.Mark.PC_OUT_OD, List_GPIOC, 2, WR_C_OOD);

		  HAL_Delay(10);

		  CheckAllPass();


		  //Tx_UART_Verita_Command(&huart6, VRC_Next, 0x00);
		  VR_Cli.Mark.Flag_ger = VRF_SendALLTestData;
		  VR_Cli.Mark.Flag_ger = 0;

	  	 }//// Flag run all test

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV32;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR_ADC1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//void CPUTemprdINIT(){
//	ADCChannell[0].Confix.Channel = ADC_CHANNEL_TEMPSENSOR_ADC1;
//	ADCChannell[0].Confix.Rank = 1;
//	ADCChannell[0].Confix.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
//}
//
//uint16_t CPUTempread(){
//	uint16_t tmpbf;
//
//	HAL_ADC_ConfigChannel(&hadc1, &ADCChannell[0].Confix); //
//
//	HAL_ADC_Start(&hadc1);
//
//	if(HAL_ADC_PollForConversion(&hadc1, 10)==HAL_OK) //10mSec timeout
//		{
//			//ReadData to confix channel
//			tmpbf = HAL_ADC_GetValue(&hadc1);
//		}
//
//	HAL_ADC_Stop(&hadc1);
//
//	return tmpbf;
//}


float ADCTVoltar(uint16_t btt, float vref){
	// convert 0-4096 ADC bit -> 0-Vref from device(3.3V, 3.25V)
	return (btt /4096.0) * vref;
}

//float TempEquat(float Vs){
//	//Vs = V tmp read , V25= 0.76V, Avg_slope = 2.5 mV
//	return ((Vs - 0.76)/(0.0025)) + 25.0; //2.5*0.001
//}

float Tempequat_G4(int16_t data, float vref){
	static int16_t TSCALT1 = TEMPSENSOR_CAL1_TEMP;
	static int16_t TSCALT2 = TEMPSENSOR_CAL2_TEMP;
	int16_t TCAL1 = (int32_t) *TEMPSENSOR_CAL1_ADDR; // 1037 read the ST factory calibrate value from the address
	int16_t TCAL2 = (int32_t) *TEMPSENSOR_CAL2_ADDR; // 1378

	float set1 = ((TSCALT2 - TSCALT1)*1.0) / ((TCAL2 - TCAL1)*1.0) ;
	float set2 = ( data * (vref/3.0) )-TCAL1;

	return set1 * set2 + 30 + fakeoffset; //// + 12 is fake offset
}
//// GPIO Testscript
void Compare_pin_32(uint32_t raw32, uint16_t *Lista_GPIOx, uint8_t gpst,char *outchar){
	/*  @brief compare uint32_t data given from gpio_testscript then compared to find the same pair
	 * 			(According to the gpio_testscript functions, if the read value of GPIO_input during
	 * 			force output(Push pull, open drain-pullup, Pullup-down)are the same,
	 * 			That pin is suspected to got some problem and need to be inspected)
	 *
	 * 			 and record the suspected pin into char for report in Terminal, display
	 * 	@param raw32       rawuint32_t data given from gpio_testscript functions
	 * 	@param Lista_GPIOx List of GPIOs bank need to be checked
	 * 	@param gpst        select report type [0 - PA_] [1 - PB_] [2 - PC_]
	 * 	@param outchar     char for record the compare result report
	 * */
	uint16_t raw32_N = raw32 & 0xFFFF;
	uint16_t raw32_P = (raw32 >> 16) & 0xFFFF;
	uint8_t iaa, iab, cntr_w = 0;
	char aadd[6];

	for(register int i = 0;i < 16;i++){
		if(Lista_GPIOx[i] >= 20){break;}

		iaa = (raw32_N >> Lista_GPIOx[i]) & 0x01;
		iab = (raw32_P >> Lista_GPIOx[i]) & 0x01;
		 if(iaa == iab){

			 cntr_w++; // count if match

			 //// add problem pin
			 switch(gpst){
			 default:
			 case 0: // A
				 sprintf(aadd, "PA%d", (uint8_t)Lista_GPIOx[i]); //
				 break;
			 case 1: // B
			 	 sprintf(aadd, "PB%d", (uint8_t)Lista_GPIOx[i]); //
			 	 break;
			 case 2: // C
			 	 sprintf(aadd, "PC%d", (uint8_t)Lista_GPIOx[i]); //
			 	 break;

			 }
			 strncat(outchar, aadd, 4);

			 //// add High, Low
			 if(iaa == 1){
				 sprintf(aadd, "_H");
			 }else if(iaa == 0){
				 sprintf(aadd, "_L");
			 }
			 strncat(outchar, aadd, 2);

			 //// add blank
			 sprintf(aadd, " ");
			 strncat(outchar, aadd, 1);
		 }
	}

	if(!cntr_w){
		sprintf(aadd, "_PASS");
		strncat(outchar, aadd, 7);
	}
}

//// GPIO Testscript
void resetgpio_char(){

	sprintf(WR_A_PUPDR, "\r\nA_PUR: ");
	sprintf(WR_B_PUPDR, "\r\nB_PUR: ");
	sprintf(WR_C_PUPDR, "\r\nC_PUR: ");

	sprintf(WR_A_OPP, "\r\nA_OPP: ");
	sprintf(WR_B_OPP, "\r\nB_OPP: ");
	sprintf(WR_C_OPP, "\r\nC_OPP: ");

	sprintf(WR_A_OOD, "\r\nA_OOD: ");
	sprintf(WR_B_OOD, "\r\nB_OOD: ");
	sprintf(WR_C_OOD, "\r\nC_OOD: ");
}

//// GPIO Testscript
void CheckAllPass(){
	cnt_allpass = 0; // init reset

	//// lazy Cat cat chekallpass
	////  \r,\n count as 1
	if(WR_A_PUPDR[9] == 95){cnt_allpass++;} //// 95 = "_"
	if(WR_B_PUPDR[9] == 95){cnt_allpass++;}
	if(strlen(WR_C_PUPDR) <= 16 && WR_C_PUPDR[10] == 67){cnt_allpass++;}// PC_13

	if(WR_A_OPP[9] == 95){cnt_allpass++;}
	if(WR_B_OPP[9] == 95){cnt_allpass++;}
	if(WR_C_OPP[9] == 95){cnt_allpass++;}

	if(WR_A_OOD[9] == 95){cnt_allpass++;}
	if(WR_B_OOD[9] == 95){cnt_allpass++;}
	if(WR_C_OOD[9] == 95){cnt_allpass++;}

}

//// ----------------GPIO_EXTI_Callback-----------------------------------------

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_13){
		bluecounter++;
		bluecounter%=4;

		flag_gpioselftest = 1;
		timestamp_selftestdelay = HAL_GetTick() + 600;
		}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
