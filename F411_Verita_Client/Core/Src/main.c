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

  /* ================ FRA506_Independent Study =================================
   * ================ VERITA: Nucleo F411RE Tester ====  --->>>  Client  >>>----
   *
   * Author: owl_hor, FRAB#7 @FIBO KMUTT
   * Advisor: Pittiwut Teerakittikul, Ph.D & Puttinart Archewawanich (AlphaP2712)
   * --------------------------------------------------------------------------
   *  ===== peripheral Usage (Last update: 7 Mar 2023) ===========
   *  - [ADC1]   -> Read MCU's temperature
   *  - [USART6] -> Verita protocol [Master <-> Client]
   *
   * ================ pin GPIO Usage ===========
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
#define FIRMWARE_VER 0x12050623 // 01 00 03 23  -- ver day month year 32-bit
//#define TIMx_PWM_En
//#define GPIO_SELFTEST_SC
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
typedef struct{
	ADC_ChannelConfTypeDef Confix;
	uint16_t datt;
}ADCStructure;

ADCStructure ADCChannell[1];

uint16_t cputmpraw = 0;
float cputempCC = 0;

uint8_t bluecounter = 0;
uint8_t counter_flagger = 0; // countif testscript is runned
uint8_t cnt_allpass = 0;
uint32_t timestamp_one = 0;
uint32_t timestamp_selftestdelay = 0;


//// --------- GPIO Read Buffer --------
//uint32_t temp_mode, temp_pupdr;
//uint16_t gpio_C_rd[4] = {0};
//uint32_t gpio_xpupd_rd[4] = {0};
//uint32_t gpio_xopp[3] = {0};
//uint32_t gpio_xood[3] = {0};

uint32_t gpio_rec_mode[3] = {0};
uint32_t gpio_rec_pupdr[3] = {0};

uint8_t flag_gpioselftest = 0;

//// lists All port - pin to inspect first // avoid special pin like osilators / UART
//// GPIO_PIN_x is in bit position format (0 2 4 8 16 ...) which loss if stored in that form and log2() to calculate back
uint16_t List_GPIOA[] = {0,1,    4,5,6,7,8,9,10,            15,  20}; // 2,3 STLK RXTX / 11 12 VRT MTxCL / 13 14 TMS TCK / 9, 10 Master UART_BL High Affect
uint16_t List_GPIOB[] = {0,1,2,  4,5,6,7,8,9,10,   12,13,14,15,  20}; // 11 is Vcap
uint16_t List_GPIOC[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,        20};

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

//char TxBufferMtCl[50] = {0}; // Sent some results to master
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void CPUTemprdINIT();
uint16_t CPUTempread();
float ADCTVolta(uint16_t btt);
float TempEquat(float Vs);

void GPIO_Selftest_step_1_single();
void Compare_pin();
void Compare_pin_32(uint32_t raw32, uint16_t *Lista_GPIOx, uint8_t gpst, char *outchar);
void resetgpio_char();
void CheckAllPass();
//void Tx_UART_Verita_Packet_u8(UART_HandleTypeDef *huart, uint8_t regis,uint8_t *pdata, uint8_t size);
//void Tx_UART_Verita_Packet_u32(UART_HandleTypeDef *huart, uint8_t regis,uint32_t pdata);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  CPUTemprdINIT();

  char temp[]="----------------- F411_Verita_Client --------------------\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)temp, strlen(temp),10);

  sprintf(uartTXBf, "Firmware ver: %08X \r\n ", FIRMWARE_VER);
  HAL_UART_Transmit(&huart2, (uint8_t*)uartTXBf, strlen(uartTXBf),10);

  ////  ------------- UART Recieve : Circular DMA here--------------------------
  HAL_UART_Receive_DMA(&huart6, &RxBufferMtCl[0], RxbufferSize_VRT);

  VR_Cli.Mark.FirmwareVer = FIRMWARE_VER;

   //GPIOB->MODER = 0x100680;

//   uint32_t tyyy = GPIOB->MODER;
//   tyyy &= ~( 0b11 << (5 * 2U));
//   tyyy &= ~( 0b11 << (10 * 2U));
//   tyyy |= ( GPIO_MODE_OUTPUT_PP << (5 * 2U));
//   tyyy |= ( GPIO_MODE_OUTPUT_PP << (10 * 2U));
//   GPIOB->MODER = tyyy;

#ifdef TIMx_PWM_En
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //rslt = Rx_Verita_engine(RxBufferMtCl, &VR_Cli);
	  //Tx_Rq_Verita_engine(&huart6, &VR_Cli);
	  ////  ------------- UART Recieve : Normal DMA --------------------------
	  //HAL_UART_Receive_DMA(&huart6, &RxBufferMtCl[0], 9);

	  if(HAL_GetTick() >= timestamp_one){
		  timestamp_one += 1000;

		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		  cputmpraw = CPUTempread();
		  VR_Cli.Mark.cputemp = CPUTempread();

		  cputempCC = TempEquat(ADCTVolta(cputmpraw));

		  sprintf(uartTXBf, "\r\n - - - - - - - - - - - - - - - - - - - - - - - - -\r\n");
		  HAL_UART_Transmit(&huart2, (uint8_t*)uartTXBf, strlen(uartTXBf),10);

		  sprintf(uartTXBf, "cputempraw = %d => %.3f C\r\n ",
				  cputmpraw,
				  cputempCC);
		  HAL_UART_Transmit(&huart2, (uint8_t*)uartTXBf, strlen(uartTXBf),10);

		  //// Print GPIO Test Result
		  if(counter_flagger){
			  if(cnt_allpass >= 9){
				  //// there're 9 pass
				  sprintf(uartTXBf, "\r\n+++ ALL PASS +++\r\n"); HAL_UART_Transmit(&huart2, (uint8_t*)uartTXBf, strlen(uartTXBf),10);
			  }else{
				  sprintf(uartTXBf, "\r\n--- unhealthy ---\r\n"); HAL_UART_Transmit(&huart2, (uint8_t*)uartTXBf, strlen(uartTXBf),10);
			  }


			  sprintf(uartTXBf, WR_A_PUPDR); HAL_UART_Transmit(&huart2, (uint8_t*)uartTXBf, strlen(uartTXBf),10);

			  sprintf(uartTXBf, WR_A_OPP); HAL_UART_Transmit(&huart2, (uint8_t*)uartTXBf, strlen(uartTXBf),10);

			  sprintf(uartTXBf, WR_A_OOD); HAL_UART_Transmit(&huart2, (uint8_t*)uartTXBf, strlen(uartTXBf),10);

			  sprintf(uartTXBf, WR_B_PUPDR); HAL_UART_Transmit(&huart2, (uint8_t*)uartTXBf, strlen(uartTXBf),10);

			  sprintf(uartTXBf, WR_B_OPP); HAL_UART_Transmit(&huart2, (uint8_t*)uartTXBf, strlen(uartTXBf),10);

			  sprintf(uartTXBf, WR_B_OOD); HAL_UART_Transmit(&huart2, (uint8_t*)uartTXBf, strlen(uartTXBf),10);

			  sprintf(uartTXBf, WR_C_PUPDR); HAL_UART_Transmit(&huart2, (uint8_t*)uartTXBf, strlen(uartTXBf),10);

			  sprintf(uartTXBf, WR_C_OPP); HAL_UART_Transmit(&huart2, (uint8_t*)uartTXBf, strlen(uartTXBf),10);

			  sprintf(uartTXBf, WR_C_OOD); HAL_UART_Transmit(&huart2, (uint8_t*)uartTXBf, strlen(uartTXBf),10);
		  }


//		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
//		  gpio_C_rd[3] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
		  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);

	  }

	  if(flag_gpioselftest){

		  //// ---------- Verita send 1 set --------------------------
//		  static uint8_t gg = 0x66;
//		  static uint8_t rg = 0x03;
//		  uint8_t ggg[4] = {0x00, 0x11, 0x33, gg};
//		  Tx_UART_Verita_Packet_u8(&huart6, rg, ggg, sizeof(ggg));
//
//		  gg++; rg++;
//
//		  Tx_UART_Verita_Packet_u32(&huart6, VR_FWID, (uint32_t)FIRMWARE_VER);
//
//		  Tx_UART_Verita_Packet_u32(&huart6, 0x92, (uint32_t)0x00FF00AA);
//		  Tx_UART_Verita_Packet_u32(&huart6, 0x13, 0x12); //// data request
		  //// ------- old script ------------------

		  //VR_Cli.Mark.Flag_ger = 0x02;
		  //// delay wait for button release
		  if (HAL_GetTick() >= timestamp_selftestdelay){
			  VR_Cli.Mark.Flag_ger = VRF_GPIO_Runalltest;
			  flag_gpioselftest = 0;
		  }

	  }

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

//		  uint32_t bbb = 0x12123333;
//		  for(register int i = 1;i < 9;i++){
//			  Tx_UART_Verita_Packet_u32(&huart6, i, bbb);
//			  bbb += 0xFF;
//		  }
		  Tx_UART_Verita_Command(&huart6, VRC_Next, 0x00);
		  VR_Cli.Mark.Flag_ger = VRF_SendALLTestData;

	  }

	  //// SEnd All Data Flag
	  if(VR_Cli.Mark.Flag_ger == VRF_SendALLTestData){

		  Tx_UART_Verita_Packet_u32(&huart6, VR_PA_PUPDR, VR_Cli.Mark.PA_PUPDR);
		  Tx_UART_Verita_Packet_u32(&huart6, VR_PB_PUPDR, VR_Cli.Mark.PB_PUPDR);
		  Tx_UART_Verita_Packet_u32(&huart6, VR_PC_PUPDR, VR_Cli.Mark.PC_PUPDR);

		  HAL_Delay(10);

		  Tx_UART_Verita_Packet_u32(&huart6, VR_PA_OUT_PP, VR_Cli.Mark.PA_OUT_PP);
		  Tx_UART_Verita_Packet_u32(&huart6, VR_PB_OUT_PP, VR_Cli.Mark.PB_OUT_PP);
		  Tx_UART_Verita_Packet_u32(&huart6, VR_PC_OUT_PP, VR_Cli.Mark.PC_OUT_PP);

		  HAL_Delay(10);

		  Tx_UART_Verita_Packet_u32(&huart6, VR_PA_OUT_OD, VR_Cli.Mark.PA_OUT_OD);
		  Tx_UART_Verita_Packet_u32(&huart6, VR_PB_OUT_OD, VR_Cli.Mark.PB_OUT_OD);
		  Tx_UART_Verita_Packet_u32(&huart6, VR_PC_OUT_OD, VR_Cli.Mark.PC_OUT_OD);

		  Tx_UART_Verita_Packet_u32(&huart6, VR_FWID, VR_Cli.Mark.FirmwareVer);

		  HAL_Delay(15);

		  Tx_UART_Verita_Command(&huart6, VRC_Flag_aa, 0xFF);
		  Tx_UART_Verita_Command(&huart6, VRC_Flag_ger, VRF_SendALLTestData);
		  VR_Cli.Mark.Flag_ger = 0;

	  }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void CPUTemprdINIT(){
	ADCChannell[0].Confix.Channel = ADC_CHANNEL_TEMPSENSOR;
	ADCChannell[0].Confix.Rank = 1;
	ADCChannell[0].Confix.SamplingTime = ADC_SAMPLETIME_3CYCLES;
}

uint16_t CPUTempread(){
	uint16_t tmpbf;

	HAL_ADC_ConfigChannel(&hadc1, &ADCChannell[0].Confix); //

	HAL_ADC_Start(&hadc1);

	if(HAL_ADC_PollForConversion(&hadc1, 10)==HAL_OK) //10mSec timeout
		{
			//ReadData to confix channel
			tmpbf = HAL_ADC_GetValue(&hadc1);
		}

	HAL_ADC_Stop(&hadc1);

	return tmpbf;
}

float ADCTVolta(uint16_t btt){
	// convert 0-4096 ADC bit -> 0-3.3V
	return (btt /4096.0) * 3.3;
}

float TempEquat(float Vs){
	//Vs = V tmp read , V25= 0.76V, Avg_slope = 2.5 mV
	return ((Vs - 0.76)/(0.0025)) + 25.0; //2.5*0.001
}


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
//void GPIO_Selftest_step_1_single(){
//	/* 1 - Input pullup read
//	 * 2 - Input pulldown read
//	 *
//	 * 3 - Output pushpull
//	 * 4 - Output opendrain
//	 *
//	 * 0xA800 0000 | Reset GPIOA_MODER
//	 * 0x6400 0000 | Reset GPIOA_PUPDR
//	 * */
//
//
//	//uint32_t temp_mode, temp_pupdr; //
//	//uint32_t posit = 0x00000001;
//
//	//GPIOC->MODER = 0x00000000U; //
//	//GPIOC->PUPDR = 0x05555555U; //  All pullup
//	//GPIOC->PUPDR = 0x0AAAAAAAU; //  All pulldown
//
//	temp_mode = GPIOC->MODER;
//	temp_pupdr = GPIOC->PUPDR;
//
//	uint8_t sizearr = sizeof(List_GPIOC); // / sizeof(List_GPIOC[0])
//
//
//	//// ------------------ Input PULLUP ------------------------------
//	for(register int i = 0;i < sizearr; i++){
//		temp_mode &= ~( 0b11 << (List_GPIOC[i] * 2U)); // clear only 2 register want to reconfig by shift 11 to prefer position then & it's invert to the previous read
//		temp_mode |= ( GPIO_MODE_INPUT << (List_GPIOC[i] * 2U));
//	}
//	GPIOC->MODER = temp_mode;
//
//
//	for(register int i = 0;i < sizearr; i++){
//		temp_pupdr &= ~( 0b11 << (List_GPIOC[i] * 2U)); // clear only 2 register want to reconfig by shift 11 to prefer position then & it's invert to the previous read
//		temp_pupdr |= ( GPIO_PULLUP << (List_GPIOC[i] * 2U));
//	}
//	GPIOC->PUPDR = temp_pupdr;
//	HAL_Delay(5);
//	gpio_C_rd[0] = GPIOC->IDR;
//
//	//// ------------------ Input PULLDOWN ------------------------------
//	for(register int i = 0;i < sizearr; i++){
//		temp_pupdr &= ~( 0b11 << (List_GPIOC[i] * 2U)); // clear only 2 register want to reconfig by shift 11 to prefer position then & it's invert to the previous read
//		temp_pupdr |= ( GPIO_PULLDOWN << (List_GPIOC[i] * 2U));
//	}
//	GPIOC->PUPDR = temp_pupdr;
//	HAL_Delay(5);
//	gpio_C_rd[1] = GPIOC->IDR;
//
//
//	////temp = GPIOx->PUPDR;
//	////temp &= ~(GPIO_PUPDR_PUPDR0 << (position * 2U));
//	////temp |= ((GPIO_Init->Pull) << (position * 2U));
//	////GPIOx->PUPDR = temp;
//}



//// ----------------GPIO_EXTI_Callback-----------------------------------------

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_13){
		bluecounter++;
		bluecounter%=4;


		//VR_Cli.Mark.Flag_ger = VRF_GPIO_Runalltest; // use timestamp delay to trig instead
		flag_gpioselftest = 1;
		timestamp_selftestdelay = HAL_GetTick() + 600;


//		Tx_UART_Verita_Command(&huart6, VRC_Flag_aa, 0xFF);
//		Tx_UART_Verita_Command(&huart6, VRC_Flag_ger, VRF_SendALLTestData);


#ifdef TIMx_PWM_En
		if(bluecounter == 0){
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 9000);
		}
		else if(bluecounter == 1){

			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 6000);
		}
		else if(bluecounter == 2){
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 3000);
		}
		else if(bluecounter == 3){
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		}else{
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 3000);
		}

//		TIM_OC_InitTypeDef sconff;
//		sconff.OCMode = TIM_OCMODE_FORCED_ACTIVE;
//		HAL_TIM_OC_ConfigChannel(&htim3, sconff, TIM_CHANNEL_1);

//// RM0383 p362 OC1M
//// 0b100 - low / 0b101 - high / 0b110 - pWM 1
//		uint32_t trd = TIM3->CCMR1;
//		trd &= ~(0b111 << 4);
//		trd |= ( GPIO_MODE_OUTPUT_PP << 4);

#endif

		}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	Rx_Verita_engine_callBak(RxBufferMtCl, &VR_Cli); //// try using only 1 slot 9 Buffer
	Tx_Rq_Verita_engine(&huart6, &VR_Cli);
	//HAL_UART_Receive_DMA(&huart6, &RxBufferMtCl[0], 9);
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
