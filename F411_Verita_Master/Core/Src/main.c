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
#include "Verita_PTC.h"
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

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
//// ___________________________________________________________________
//// --------------------UART Buffer -------------------
char TextDispBuffer[100] = {0}; // Display Text
char TextUARTBuffer[100] = {0}; // UART Console Text
uint8_t RxBufferMtCl[10] = {0}; // Recieved packet buffer

//// ---------- Verita Register -------------------------

//typedef enum{
//	VRT_ERROR  = 0x90U,
//	VRT_OK     = 0x91U,
//	VRT_Busy   = 0x92U,
//	VRT_Regain = 0x93U,
//	VRT_Next   = 0x94U,
//} VRTPTC_StatusTypedef;

VRTPTC_StatusTypedef engst;
//static enum {init, unpack, decode}verita_engine;
uint8_t flag_vrt_en = 0;
uint32_t verita_regis[16] = {0};

//// --------- INA219 ------------------------------------
union {
	uint8_t U8[12];
	uint16_t U16[6];
}INATT;

INA219_Read_Set inata;
INA219_Conf_Strc cofgra;

//// -------------- Timestamp ---------------------------
uint32_t timestamp_one = 0;
uint32_t timestamp_disp = 0;
//// ________________________________________________________________
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void running_box();
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
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  ili9341_Init();
  ili9341_DisplayOn();


//  ILI9341_Init();
//  ILI9341_FillScreen(ILI9341_BLACK);

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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  ////  ------------- UART Recieve --------------------------
	  HAL_UART_Receive_DMA(&huart6, &RxBufferMtCl[0], 10);
	  engst = Rx_Verita_engine(RxBufferMtCl, verita_regis);
	  //// ----------------------------------------------------

	  if (HAL_GetTick() >= timestamp_one){
		  timestamp_one += 1000;
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);


		  ili9341_FillRect(50, 20, 50, 20, cl_RED);
		  ili9341_FillRect(100, 20, 50, 20, cl_GREEN);
		  ili9341_FillRect(150, 20, 50, 20, cl_BLUE);

		  //ili9341_DrawRGBImage(60, 80, 128, 128, (uint16_t*)image_data_ImageoftestN2);

//		  ili9341_WriteString(50, 40, "Helios Terra Renai Kaliber Barx Maxon 129035"
//					 " __ --== + &&6.. [ ggg ]??? Rhivalia DIAR Barvarrian"
//					  "vicar nexus iICCTVS "
//				  , Font12, cl_YELLOW, cl_BLACK);


#ifdef INA219_Wrk
//		  HAL_I2C_Mem_Read(&hi2c1, INA219_ADDR_1, 0x00, I2C_MEMADD_SIZE_8BIT, &INATT.U8[0], 2, 10);
//		  INATT.U16[3] = (INATT.U8[0] << 8) | INATT.U8[1];

		  //INATT.U16[1] = INA219Read_cx(&hi2c1, INA219_ADDR_1, INA219_RG_Config);
		  //INATT.U16[2] = INA219Read_cx(&hi2c1, INA219_ADDR_1, INA219_RG_Current);

		  inata.Bus_V   = INA219Read_BusV(&hi2c1, INA219_ADDR_1);
		  inata.CURRENT = INA219Read_Current(&hi2c1, INA219_ADDR_1);
		  inata.POWER   = INA219Read_Power(&hi2c1, INA219_ADDR_1);
		  inata.SHUNT_V = INA219Read_ShuntV(&hi2c1, INA219_ADDR_1);

		  inata.Calibra =  INA219Read_cx(&hi2c1, INA219_ADDR_1, INA219_RG_Calibra);
		  inata.Config = INA219Read_cx(&hi2c1, INA219_ADDR_1, INA219_RG_Config);

		  sprintf(TextDispBuffer,"calibrator: %4X", inata.Calibra);
		  ili9341_WriteString(20, 50, TextDispBuffer, Font16, cl_GREENYELLOW, cl_BLACK);

		  sprintf(TextDispBuffer,"V mV: %d    ", inata.Bus_V);
		  ili9341_WriteString(20, 70, TextDispBuffer, Font20, cl_CYAN, cl_BLACK);

		  sprintf(TextDispBuffer,"I mA: %d    ", inata.CURRENT);
		  ili9341_WriteString(20, 95, TextDispBuffer, Font20, cl_CYAN, cl_BLACK);

		  sprintf(TextDispBuffer,"P mW: %.4f  ", inata.POWER);
		  ili9341_WriteString(20, 120, TextDispBuffer, Font20, cl_CYAN, cl_BLACK);

#endif

		  } // timestamp_one

	  if (HAL_GetTick() >= timestamp_disp){
	  		timestamp_disp += 10;
	  		running_box();

	  }// timestamp_dis
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|ili_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_RES_GPIO_Port, SPI2_RES_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ili_RES_GPIO_Port, ili_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin SPI2_CS_Pin ili_DC_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|SPI2_CS_Pin|ili_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_RES_Pin */
  GPIO_InitStruct.Pin = SPI2_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_RES_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ili_RES_Pin */
  GPIO_InitStruct.Pin = ili_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ili_RES_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_13){
		INA219_BitReset(&hi2c1, INA219_ADDR_1);
		}
}

void running_box(){
   //// Running box ------
  int ratte = 1;
  int sizo = 30;
  int offs = 180;
  static uint16_t xsh = 0;
  ili9341_FillRect(xsh, offs, ratte ,sizo, cl_MAROON);
  xsh += ratte;
  ili9341_FillRect(xsh, offs, sizo, sizo, cl_CYAN); //// box
  if(xsh >= 400){ // clear
	  ili9341_FillRect(xsh, offs, sizo, sizo, cl_MAROON);
	  xsh = 0;
		  }

}

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
//	//uint8_t chksum = 0;
//
//	switch (verita_engine){
//	default:
//	case init:
//
//		if(flag_vrt_en || Rxbffr[0] == 0x56){ //
//			verita_engine = unpack;
//		}
//		break;
//
//	case unpack:
//
//		if(Rxbffr[0] == 0x56 && Rxbffr[1] == 0x52 && Rxbffr[2] == 0x54){
//
//			//// log data first / prevent overwrite
//			for(register int k = 0; k < 7; k++){
//				logger[k] = Rxbffr[k+3];
//			}
//
//			//// checksum here
////			for(register int i = 0;i < 5; i++){
////				chksum += logger[i];
////			}
////			if(~chksum == logger[6]){
////				// pass
////			}
//
//			//// mark that this data is already read
//			Rxbffr[0] = 0xFF;
//			verita_engine = decode;
//		}
//		else{
//			verita_engine = init;
//			flag_vrt_en = 0;
//
//			//// destroy data
//			for(register int i = 0;i < sizeof(Rxbffr); i++){
//				Rxbffr[i] = 0x00;
//			}
//		}
//		break;
//
//	case decode:
//		verita_engine = init;
//
//		//// DATA phase, insert 32bit data into register box
//		if(logger[0] <= 0x20){
//
//			logu.U8[3] = logger[1];
//			logu.U8[2] = logger[2];
//			logu.U8[1] = logger[3];
//			logu.U8[0] = logger[4];
//
//			regisk[logger[0]] = logu.U32;
//			return VRT_OK;
//		}
//
//		//// CMD phase, return recieved Command
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
//		break;
//	}
//}

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
