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

#include "gpio_testscript.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define TIMx_PWM_En
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef struct{
	ADC_ChannelConfTypeDef Confix;
	uint16_t datt;
}ADCStructure;

ADCStructure ADCChannell[1];

uint16_t cputmpraw = 0;
float cputempCC = 0;

uint8_t bluecounter = 0;
uint32_t timestamp_one = 0;


//// --------- GPIO Read Buffer --------
uint32_t temp_mode, temp_pupdr;
uint16_t gpio_C_rd[4] = {0};
uint32_t gpio_xpupd_rd[4] = {0};
uint8_t flag_gpioselftest = 0;

//// lists All port - pin to inspect first // avoid special pin like osilators / UART
//// GPIO_PIN_x is in bit position format (0 2 4 8 16 ...) which loss if stored in that form and log2() to calculate back
uint16_t List_GPIOB[] = {0,1,2,  4,5,6,7,8,9,10,   12,13,14,15,  20}; // 11 is Vcap
uint16_t List_GPIOC[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,        20};

//// ---- -----------
char uartTXBf[100] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void CPUTemprdINIT();
uint16_t CPUTempread();
float ADCTVolta(uint16_t btt);
float TempEquat(float Vs);

void GPIO_Selftest_step_1_single();
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  CPUTemprdINIT();

  char temp[]="----------------- F411_Verita_Client --------------------\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)temp, strlen(temp),10);

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
	  if(HAL_GetTick() >= timestamp_one){
		  timestamp_one += 400;

		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		  cputmpraw = CPUTempread();

		  cputempCC = TempEquat(ADCTVolta(cputmpraw));

		  sprintf(uartTXBf, "cpuraw = %d  => %.3f C\r\n ",
				  cputmpraw,
				  cputempCC);
		  HAL_UART_Transmit(&huart2, (uint8_t*)uartTXBf, strlen(uartTXBf),10);

		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
		  gpio_C_rd[3] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
		  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);

	  }

	  if(flag_gpioselftest == 1){
		  //GPIO_Selftest_step_1_single();
		  gpio_xpupd_rd[2] = gpio_selftest_input_pupdr_1(GPIOC, List_GPIOC);
		  gpio_xpupd_rd[1] = gpio_selftest_input_pupdr_1(GPIOB, List_GPIOB);
		  flag_gpioselftest = 0;
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

void GPIO_Selftest_step_1_single(){
	/* 1 - Input pullup read
	 * 2 - Input pulldown read
	 *
	 * 3 - Output pushpull
	 * 4 - Output opendrain
	 *
	 * 0xA800 0000 | Reset GPIOA_MODER
	 * 0x6400 0000 | Reset GPIOA_PUPDR
	 * */


	//uint32_t temp_mode, temp_pupdr; //
	//uint32_t posit = 0x00000001;

	//GPIOC->MODER = 0x00000000U; //
	//GPIOC->PUPDR = 0x05555555U; //  All pullup
	//GPIOC->PUPDR = 0x0AAAAAAAU; //  All pulldown

	temp_mode = GPIOC->MODER;
	temp_pupdr = GPIOC->PUPDR;

	uint8_t sizearr = sizeof(List_GPIOC); // / sizeof(List_GPIOC[0])


	//// ------------------ Input PULLUP ------------------------------
	for(register int i = 0;i < sizearr; i++){
		temp_mode &= ~( 0b11 << (List_GPIOC[i] * 2U)); // clear only 2 register want to reconfig by shift 11 to prefer position then & it's invert to the previous read
		temp_mode |= ( GPIO_MODE_INPUT << (List_GPIOC[i] * 2U));
	}
	GPIOC->MODER = temp_mode;


	for(register int i = 0;i < sizearr; i++){
		temp_pupdr &= ~( 0b11 << (List_GPIOC[i] * 2U)); // clear only 2 register want to reconfig by shift 11 to prefer position then & it's invert to the previous read
		temp_pupdr |= ( GPIO_PULLUP << (List_GPIOC[i] * 2U));
	}
	GPIOC->PUPDR = temp_pupdr;
	HAL_Delay(5);
	gpio_C_rd[0] = GPIOC->IDR;

	//// ------------------ Input PULLDOWN ------------------------------
	for(register int i = 0;i < sizearr; i++){
		temp_pupdr &= ~( 0b11 << (List_GPIOC[i] * 2U)); // clear only 2 register want to reconfig by shift 11 to prefer position then & it's invert to the previous read
		temp_pupdr |= ( GPIO_PULLDOWN << (List_GPIOC[i] * 2U));
	}
	GPIOC->PUPDR = temp_pupdr;
	HAL_Delay(5);
	gpio_C_rd[1] = GPIOC->IDR;


	////temp = GPIOx->PUPDR;
	////temp &= ~(GPIO_PUPDR_PUPDR0 << (position * 2U));
	////temp |= ((GPIO_Init->Pull) << (position * 2U));
	////GPIOx->PUPDR = temp;
}

//// ----------------GPIO_EXTI_Callback-----------------------------------------

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_13){
		bluecounter++;
		bluecounter%=4;

		flag_gpioselftest = 1;


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
#endif


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
