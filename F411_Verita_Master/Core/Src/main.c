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
   * ================ VERITA: Nucleo F411RE Tester ====  --->>>  Master  >>>----
   *
   * Author: owl_hor, FRAB#7 @FIBO KMUTT
   * Advisor: Pittiwut Teerakittikul, Ph.D & Puttinart Archewawanich (AlphaP2712)
   * --------------------------------------------------------------------------
   *  ===== peripheral Usage (Last update: 7 Mar 2023) ===========
   *  - [I2C1]   -> 2x INA219 Power Monitors
   *  - [SPI2]   -> ILI9341 LCD (RobertoBenjami's stm32_graphics_display_drivers library
   *  				 / didn't use hardware SPI, use position of GPIO)
   *  - [SPI3]   -> MCP3208 12bit 0-5V Range ADC
   *  - [USART1] -> Bootloader to client
   *  - [USART2] -> UART Terminal report to PC / read by Tera term, or any.
   *  - [USART6] -> Verita protocol [Master <-> Client]
   *  - [TIM3] 	 -> QEI Rotary Encoder knob read
   *  - [TIM10]  -> Buzzer trigger
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
#include <stdarg.h>

#include "LCDrv_f4_spi/Fonts/fonts.h"
#include "LCDrv_f4_spi/ili9341.h"
#include "LCDrv_f4_spi/bmp.h"
#include "testimg.h"

#include "INA219.h"
#include "MCP320X.h"
#include "Verita_PTC.h"
#include "Client_bin.h"
#include "bootloader_UART.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INA219_Wrk
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
//// ___________________________________________________________________
//// --------------------UART Buffer -------------------
char TextDispBuffer[100] = {0}; // Display Text
char TextUARTBuffer[100] = {0}; // UART Console Text
uint8_t RxBufferMtCl[RxbufferSize_VRT] = {0}; // Recieved packet buffer

//// ------- UART Bootloader ---------------
uint8_t bootloop_n = 0;
//uint8_t bl_n_cr = 0;
UARTBootloader_state respo;
uint16_t boot_size = sizeof(F411_Verita_Client); // size of F411_Verita_Client
uint8_t BL_UARTBuffer[20] = {0};
uint8_t BL_MemBuffer[260] = {0};
//// ---------- Verita Register -------------------------

VRTPTC_StatusTypedef engst; // return engine state

//uint8_t flag_vrt_en = 0;
//// = = = = = register bank = = = = = = = =
uint32_t verita_regis[16] = {0};
Verita_Register_Bank VRB;


//// --------- INA219 ------------------------------------
union {
	uint8_t U8[12];
	uint16_t U16[6];
}INATT;

INA219_Read_Set inata; // read first point
INA219_Read_Set inatb; // read second point
INA219_Conf_Strc cofgra;

//// ---------------- MCP3208 ------------------------
struct _mcp_read{
	uint16_t raw[4];
	float cv[4];
}mcp_read;
//// -------------- Timestamp ---------------------------
uint64_t timestamp_one = 0;
uint64_t timestamp_buzbtn = 0; // in while
uint64_t timestamp_bz = 0;     // for buzzer function only

uint32_t _millis = 0;
//// --------------- Grandstate & flags & buttons------------------
uint8_t flagc_bz = 0; // flag counter for buzzer
uint8_t btn_read[4] = {0}; // use Btn 3 as last process val
uint16_t btn_cnt = 0;
uint8_t counter = 0;

uint16_t knobtick[2] = {0}; //// store val fron TIM QEI
uint8_t btn_K[2] = {0}; //// encoder knob btn
uint8_t btn_k_cnt = 0;

typedef enum{
	k_zero = 0x00U,
	k_up,
	k_down
} flag_k;
uint8_t flag_k_up = 0;
uint8_t flag_k_dn = 0;

static enum{
	init,
	pre_lobby,
	lobby,
	s_bootloader,
	pre_monitor,
	monitor

}GrandState = pre_lobby;
//// buzzer time period
uint16_t bzz_t_priod_up = 250;
uint16_t bzz_t_priod_dn = 100;

//// ----------- Display buffer ------------
typedef struct {
	uint16_t xp; // x axis start point
	uint16_t yp; // y axis start point
	uint16_t xsi; // x axis range
	uint16_t ysi; // y axis range
}disp_posixy;

uint8_t bois_xi = 0;
uint8_t bois_yi = 0;
uint16_t bosx[8] ={0, 40, 80, 120, 160};

uint16_t bosy[6] ={220, 60, 90, 120, 150};

uint8_t state_box_choice_n = 4;
//// can it be an array of specific position?
//uint8_t st_boxchoice_lobby[3] = {1,2,3,4};

int8_t state_box_choice_is = 1;
//static uint8_t an_boxpoint = 0;

//static enum {st1, st1s, st2,st2s, st3,st3s, st4, st4s} disb_state = st1;
static enum {a_wait, a_change, a_boxclr} a_boxpoint; //abd1, abd2, abd3, abd1s, abd2s, abd3s


uint8_t flag_boxpoint_start = 0; // in case start function
//// ________________________________________________________________
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void running_box();
void buzzer_scream_cnt();
void Button_machine();
void simple_scr();
void GrandState_Verita();
void box_pointer(uint16_t posx, uint16_t posy);
void knob_rotter();
//VRTPTC_StatusTypedef Rx_Verita_engine(uint8_t *Rxbffr, uint32_t *regisk);
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
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  MX_SPI3_Init();
  MX_TIM10_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /// Timers Start
  HAL_TIM_Base_Start_IT(&htim10); // buzzer timer
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  TIM3->CNT = 0x8000; //// start QEI counter from the center
  knobtick[0] = TIM3->CNT;

  ili9341_Init();
  ili9341_DisplayOn();

//  ili9341_FillRect(50, 20, 50, 20, cl_RED);
//  ili9341_FillRect(100, 20, 50, 20, cl_GREEN);
//  ili9341_FillRect(150, 20, 50, 20, cl_BLUE);


#ifdef INA219_Wrk
  INA219_INIT_Calibrate(&hi2c1, INA219_ADDR_1);
//  cofgra.INA219CF.reset = 0;
//  cofgra.INA219CF.BRNG = BRNG_FSR_16V;
//  cofgra.INA219CF.PGA = PGA_GainD8_320mv;
//  cofgra.INA219CF.BADC = ADCI_12bit_532uS;
//  cofgra.INA219CF.SADC = ADCI_10bit_148uS;
//  cofgra.INA219CF.Mode = INAM_ShuntBusV_Continuous;
//
//  INA219_INIT(&hi2c1, INA219_ADDR_1, cofgra);
//  INA219_Calibrate(&hi2c1, INA219_ADDR_1);
#endif

  char temp[]="----------------- F411_Verita_Master --------------------\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)temp, strlen(temp),10);


////  ------------- UART Recieve --------------------------
  HAL_UART_Receive_DMA(&huart6, &RxBufferMtCl[0], RxbufferSize_VRT);

  //// for storage test only
//  uint32_t flashboot[3000];
//  for(register int i = 0; i < 3000 ;i++){
//	  flashboot[i] = F411_Verita_Client[i];
//  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //// -------- buzzer & Button -----------------
	  Button_machine();
	  ////  ------------- Verita UART Recieve --------------------------
	  //HAL_UART_Receive_DMA(&huart6, &RxBufferMtCl[0], 9); // Normal DMA
	  //engst = Rx_Verita_engine(RxBufferMtCl, verita_regis);
	  Rx_Verita_engine(RxBufferMtCl, VRB.U32);
	  //// ----------------------------------------------------


	  if (HAL_GetTick() >= timestamp_buzbtn){
		timestamp_buzbtn += 200;

		knob_rotter();

		//running_box();

		//// box pointer try
		/* function runs here with speed
		 * can change the number of selection following to the GrandState
		 * state_box_choice = 3;
		*/

		if(state_box_choice_n){

		switch (a_boxpoint){

		default:
		case a_wait:

			if(flag_k_up){

				state_box_choice_is++;
				//state_box_choice_is %= state_box_choice_n; // don't be more than spec of Grandstate sub
				if(state_box_choice_is >= state_box_choice_n){state_box_choice_is = 0;}

				 flag_k_up = 0;
				 a_boxpoint = a_change;}
			if(flag_k_dn){

				state_box_choice_is--;
				if(state_box_choice_is < 0){state_box_choice_is = state_box_choice_n - 1;}
				//state_box_choice_is = (state_box_choice_is < 0) ? state_box_choice_n-1:state_box_choice_is;
				 flag_k_dn = 0;
				 a_boxpoint = a_change;}

			break;

		case a_change:

			box_pointer(20, bosy[state_box_choice_is]);
			a_boxpoint = a_wait;
			break;

		case a_boxclr:

			break;

			}
		}
 /////////////////////////////////////////////////////////////////////
//		  switch (disb_state){
//		  default:
//		  case st1:
//
//			  if(flag_k_up){disb_state = st2s; flag_k_up--;}
//			  if(flag_k_dn){disb_state = st4s; flag_k_dn--;}
//			  break;
//		  case st2:
//
//			  if(flag_k_up){ disb_state = st3s; flag_k_up--;}
//			  if(flag_k_dn){ disb_state = st1s; flag_k_dn--;}
//			  break;
//		  case st3:
//
//			  if(flag_k_up){ disb_state = st4s; flag_k_up--;}
//			  if(flag_k_dn){disb_state = st2s; flag_k_dn--;}
//			  break;
//
//		  case st4:
//
//			  if(flag_k_up){ disb_state = st1s; flag_k_up--;}
//			  if(flag_k_dn){ disb_state = st3s; flag_k_dn--;}
//			  break;
//
//
//		  case st1s:
//		  		box_pointer(20, 60);
//		  		disb_state = st1;
//		  	 break;
//		  case st2s:
//				box_pointer(20, 90);
//				disb_state = st2;
//			 break;
//		  case st3s:
//				box_pointer(20, 120);
//				disb_state = st3;
//			 break;
//		  case st4s:
//				box_pointer(20, 150);
//				disb_state = st4;
//			 break;
//		  }


	  }// timestamp_dis


	  if (HAL_GetTick() >= timestamp_one){
		  timestamp_one += 500;
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		  GrandState_Verita();

		  } // timestamp_one

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV2;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV2;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 4999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_RS_Pin|LCD_CS_Pin|LCD_MOSI_Pin|client_NRST_Pin
                          |Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RST_Pin|LD2_Pin|boot0_trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_SCK_GPIO_Port, LCD_SCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RS_Pin LCD_CS_Pin client_NRST_Pin Buzzer_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_CS_Pin|client_NRST_Pin|Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_MISO_Pin */
  GPIO_InitStruct.Pin = LCD_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_MISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_MOSI_Pin */
  GPIO_InitStruct.Pin = LCD_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_MOSI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RST_Pin LD2_Pin boot0_trigger_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LD2_Pin|boot0_trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_SCK_Pin */
  GPIO_InitStruct.Pin = LCD_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LCD_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Btn_1_Pin Btn_2_Pin Btn_3_Pin Btn_4_Pin */
  GPIO_InitStruct.Pin = Btn_1_Pin|Btn_2_Pin|Btn_3_Pin|Btn_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_CS_Pin */
  GPIO_InitStruct.Pin = SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI3_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KnobBTN_Pin */
  GPIO_InitStruct.Pin = KnobBTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KnobBTN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void running_box(){
   //// Running box ------
  int ratte = 1;
  int sizo = 30;
  int offs = 190;
  static uint16_t xsh = 0;
  ili9341_FillRect(xsh, offs, ratte ,sizo, cl_MAROON);
  xsh += ratte;
  ili9341_FillRect(xsh, offs, sizo, sizo, cl_CYAN); //// box
  if(xsh >= 400){ // clear
	  ili9341_FillRect(xsh, offs, sizo, sizo, cl_MAROON);
	  xsh = 0;
		  }

}

void box_pointer(uint16_t posx, uint16_t posy){
	/* write new box at the new position posx posy and erase the previous box*/
	static disp_posixy box1;

	//// erase previous box
	if(flag_boxpoint_start){

		ili9341_FillRect(box1.xp, box1.yp, 15, 15, cl_BLACK);
	}

	//// new box
	ili9341_FillRect(posx, posy, 15, 15, cl_YELLOW);

	//box[1] = box[0];
	box1.xp = posx;
	box1.yp = posy;
	//// trig the upper to erase the previous in the next call
	flag_boxpoint_start = 1;
}

void simple_scr(){
	  //ili9341_DrawRGBImage(60, 80, 128, 128, (uint16_t*)image_data_ImageoftestN2);
#ifdef INA219_Wrk

	  //INATT.U16[1] = INA219Read_cx(&hi2c1, INA219_ADDR_1, INA219_RG_Config);
	  //INATT.U16[2] = INA219Read_cx(&hi2c1, INA219_ADDR_1, INA219_RG_Current);

	  inata.Bus_V   = INA219Read_BusV(&hi2c1, INA219_ADDR_1);
	  inata.CURRENT = INA219Read_Current(&hi2c1, INA219_ADDR_1);
	  inata.POWER   = INA219Read_Power(&hi2c1, INA219_ADDR_1);
	  inata.SHUNT_V = INA219Read_ShuntV(&hi2c1, INA219_ADDR_1);

	  inata.Calibra =  INA219Read_cx(&hi2c1, INA219_ADDR_1, INA219_RG_Calibra);
	  inata.Config = INA219Read_cx(&hi2c1, INA219_ADDR_1, INA219_RG_Config);

	  sprintf(TextDispBuffer,"calibrator:%4X", inata.Calibra);
	  ili9341_WriteString(20, 50, TextDispBuffer, Font12, cl_GREENYELLOW, cl_BLACK);

	  sprintf(TextDispBuffer,"V mV: %d    ", inata.Bus_V);
	  ili9341_WriteString(20, 70, TextDispBuffer, Font16, cl_CYAN, cl_BLACK);

	  sprintf(TextDispBuffer,"I mA: %d    ", inata.CURRENT);
	  ili9341_WriteString(20, 95, TextDispBuffer, Font16, cl_CYAN, cl_BLACK);

	  sprintf(TextDispBuffer,"P mW: %.2f  ", inata.POWER);
	  ili9341_WriteString(20, 120, TextDispBuffer, Font16, cl_CYAN, cl_BLACK);
#endif

	  mcp_read.raw[0] = MCP3208_READ_8_DataSPI(&hspi3, M8_CH0);
	  mcp_read.cv[0] = MCP320x_ADCbit_to_Volt(mcp_read.raw[0]);
	  sprintf(TextDispBuffer,"MCP : %.2f  ", mcp_read.cv[0]);
	  ili9341_WriteString(20, 145, TextDispBuffer, Font16, cl_CYAN, cl_BLACK);

	  ////// 4x button
	  sprintf(TextDispBuffer,"btn %X %X %d",btn_read[1], btn_read[2], btn_cnt);
	  ili9341_WriteString(170, 50, TextDispBuffer, Font16, cl_YELLOW, cl_BLACK);

	  //// rortary encoder knob
	  sprintf(TextDispBuffer,"enc %d %d %d", knobtick[0], btn_k_cnt, flag_k_up);
	  ili9341_WriteString(170, 90, TextDispBuffer, Font16, cl_WHITE, cl_BLACK);

	  sprintf(TextDispBuffer, "%ld, %d", TIM3->CNT, state_box_choice_is);
	  ili9341_WriteString(190, 110, TextDispBuffer, Font16, cl_WHITE, cl_BLACK);


}


void Button_machine(){

//	static uint8_t counter_btn_deb = 0;
//
//	if( (0x0F & ~(GPIOB->IDR >> 12)) ){
//			counter_btn_deb++;
//
//		}else{
//			counter_btn_deb = 0;
//		}

//		if(counter_btn_deb >= 3){ //// act as debounce
//			btn_read[0] = (0x0F & ~(GPIOB->IDR >> 12));
//
//			//btn_read[1] = btn_read[0];
//			counter_btn_deb = 0;
//		}

	/* 4x btn_read{
	 * raw read,
	 * read from 1 as rising detect,
	 * read latest (bdebug),
	 * read latest & erased when fin}
	 */
		btn_read[1] = btn_read[0];
		btn_read[0] = (0x0F & ~(GPIOB->IDR >> 12)); //// available for PB 12 13 14 15 or which the same bank only

		//// rising edge counter
		if(btn_read[0] && btn_read[1] == 0){
			btn_cnt += btn_read[0]; //// plus at each hex pos
			btn_read[2] = btn_read[0]; //// read latest, debug
			btn_read[3] = btn_read[0]; //// read latest, clearable
		}

		//// Rotary Encoder knob Button----------------------------------
		btn_K[1] = btn_K[0];
		btn_K[0] = HAL_GPIO_ReadPin(KnobBTN_GPIO_Port, KnobBTN_Pin);

		if(btn_K[0] == 0 && btn_K[1]){
			btn_k_cnt++;
		}

//		knob_rotter();

		if(knobtick[0] <= 16 || knobtick[0] >= 0xFFF8){
			TIM3->CNT = 0x8000; // back to center
			knobtick[0] = 0x8000;
		}


}

void knob_rotter(){
	//// round up
	if((uint16_t)TIM3->CNT > knobtick[0]){  ////(uint16_t)TIM3->CNT - knobtick[0] >= 2
		flag_k_up = 1;
		knobtick[0] = TIM3->CNT;

	}
	//// round down
	else if((uint16_t)TIM3->CNT < knobtick[0]){ ////knobtick[0] - (uint16_t)TIM3->CNT >= 2
		flag_k_dn = 1;
		knobtick[0] = TIM3->CNT;

	}
	else{}

}

void buzzer_scream_cnt(){
	static enum {bz_init, bz_silent, bz_scream} bz_st = bz_init;
	//uint16_t tup = 100, tdn = 50;

		switch(bz_st){
		default:
		case bz_init:
			//HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

			if(flagc_bz){

				HAL_TIM_Base_Start_IT(&htim10);
				timestamp_bz = bzz_t_priod_up + HAL_GetTick(); //

				bz_st = bz_scream;
				/// down flag_counter every 1 scream
				flagc_bz--;
			}else{
				HAL_TIM_Base_Stop_IT(&htim10);
			}

			break;

		case bz_scream:
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);

			if(HAL_GetTick() >= timestamp_bz){
				timestamp_bz = bzz_t_priod_dn + HAL_GetTick();

				bz_st = bz_silent;
			}
			break;


		case bz_silent:
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

			if(HAL_GetTick() >= timestamp_bz){

				if(flagc_bz){
					timestamp_bz = bzz_t_priod_up + HAL_GetTick(); //

					flagc_bz--;
					bz_st = bz_scream;

				}else{
				bz_st = bz_init;
				}
			}

			break;
		}

}


void GrandState_Verita(){

	switch(GrandState){

	case pre_lobby:
		state_box_choice_n = 4;
		ili9341_FillRect(0, 0, 320, 240, cl_BLACK);

		ili9341_FillRect(300, 0, 20, 240, cl_ORANGE);
		ili9341_FillRect(0, 0, 10, 10, cl_ORANGE);

		sprintf(TextDispBuffer,"Verita: Nucleo-F411RE Tester");
		ili9341_WriteStringNoBG(10, 10, TextDispBuffer, Font20, cl_WHITE);

		sprintf(TextDispBuffer,"Full Script");
		ili9341_WriteStringNoBG(50, 60, TextDispBuffer, Font16, cl_CYAN);

		sprintf(TextDispBuffer,"PWR_Monitor");
		ili9341_WriteStringNoBG(50, 90, TextDispBuffer, Font16, cl_CYAN);

		sprintf(TextDispBuffer,"Analog, coming soon");
		ili9341_WriteStringNoBG(50, 120, TextDispBuffer, Font16, cl_CYAN);

		box_pointer(20, bosy[state_box_choice_is]);

		GrandState = lobby;
		break;

	default:
	case lobby:
		state_box_choice_n = 4;

		// debug
		sprintf(TextDispBuffer, "%ld, %d", TIM3->CNT, state_box_choice_is);
		ili9341_WriteString(120, 150, TextDispBuffer, Font16, cl_WHITE, cl_BLACK);
		//simple_scr();
		if(btn_k_cnt){

			if (state_box_choice_is == 2){GrandState = pre_monitor;}

		btn_k_cnt = 0;
		}

//		switch(a_boxpoint_lobby){
//
//		case abd1s:
//			box_pointer(20, 60); a_boxpoint_lobby= abd1; break;
//		case abd2s:
//			box_pointer(20, 90); a_boxpoint_lobby = abd1; break;
//		case abd3s:
//			box_pointer(20, 120); a_boxpoint_lobby = abd1; break;
//
//		default:
//		case abd1:
//
//		  if(flag_k_up){a_boxpoint_lobby = abd2s; flag_k_up--;}
//		  if(flag_k_dn){a_boxpoint_lobby = abd3s; flag_k_dn--;}
//		  break;
//		case abd2:
//
//		  if(flag_k_up){a_boxpoint_lobby = abd3s; flag_k_up--;}
//		  if(flag_k_dn){a_boxpoint_lobby = abd1s; flag_k_dn--;}
//		  break;
//		case abd3:
//
//		  if(flag_k_up){a_boxpoint_lobby = abd1s; flag_k_up--;}
//		  if(flag_k_dn){a_boxpoint_lobby = abd2s; flag_k_dn--;}
//		  break;
//
//		}

		break; // lobby

	case init:
		state_box_choice_n = 0;

//		//// test write bootloader
//		// find n times must be loop to upload all code
//		bootloop_n = (boot_size / 256) + ((boot_size % 256)>0 ? 1:0);
//		//bootloop_n = (uint8_t)ceil(boot_size / 256.0);
//
//		// case 31452 -> b must be loop 123 times
//		for(register int b = 0;b < bootloop_n - 1;b++){
//			BL_UART_WriteMem_d(&huart1, 0x08000000 + (b*0x100), 255, &F411_Verita_Client[0x100*b]);
//		}
//		//// last round: send only left bit (less 255)
//		HAL_Delay(2);
//		BL_UART_WriteMem_d(&huart1, 0x08000000 + ((bootloop_n-1)*0x100), boot_size % 256, &F411_Verita_Client[0x100*(bootloop_n-1)]);

		BL_UART_Start(&huart1);
		BL_UART_ExtendEraseMem_SP(&huart1, Erase_MASS_CMD);
		BL_UART_Finish();

		GrandState = lobby;
		break;

	case s_bootloader:
		state_box_choice_n = 0;

		//// find n times must be loop to upload all code
		bootloop_n = (boot_size / 256) + ((boot_size % 256)>0 ? 1:0);
		//bootloop_n = (uint8_t)ceil(boot_size / 256.0);

		BL_UART_Start(&huart1);

		//// Flash Memory Erase ============, Erase1_Mass_CMD makes bootloader not response to ALL write CMD / dont know why
		BL_UART_ExtendEraseMem_SP(&huart1, Erase_Bank1_CMD);
		BL_UART_ExtendEraseMem_SP(&huart1, Erase_Bank2_CMD);

		//// WriteMem Set  =========================================
		//// case 31452 -> b must be loop 123 times  ----------------------------------
		for(register int b = 0;b < bootloop_n - 1;b++){
			BL_UART_WriteMem(&huart1, 0x08000000 + (b*0x100), 255, &F411_Verita_Client[0x100*b]);
		}
		//// last round: send only left bit (less 255)
		BL_UART_WriteMem(&huart1, 0x08000000 + ((bootloop_n-1)*0x100), boot_size % 256, &F411_Verita_Client[0x100*(bootloop_n-1)]);
		//// WriteMem Set =========================================

		BL_UART_Finish();

		GrandState = lobby;

		break;

	case pre_monitor:
		state_box_choice_n = 0;
		ili9341_FillRect(0, 0, 320, 240, cl_BLACK);
		ili9341_FillRect(0, 0, 10, 10, cl_ORANGE);

		sprintf(TextDispBuffer,"<-Back (Knob press)");
		ili9341_WriteStringNoBG(60, 220, TextDispBuffer, Font16, cl_WHITE);
		GrandState = monitor;
		break;

	case monitor:
		state_box_choice_n = 1;
		simple_scr();

		if(btn_k_cnt){
			GrandState = pre_lobby;
			btn_k_cnt = 0;
			}
		break;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_13){
		//INA219_BitReset(&hi2c1, INA219_ADDR_1);
		flagc_bz = 12;
		buzzer_scream_cnt();

		//// bootloader test
		//GrandState = s_bootloader;
		//GrandState = init;
		}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim10){
		_millis++;
		buzzer_scream_cnt();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	counter++;
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
