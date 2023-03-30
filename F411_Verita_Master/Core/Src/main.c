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
#define Current_limit_mA 900

#define ff_runfull 3
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


//// ------- UART Bootloader ---------------
uint8_t bootloop_n = 0;
UARTBootloader_state respo;
uint16_t boot_size = sizeof(F411_Verita_Client); // size of F411_Verita_Client
uint8_t BL_UARTBuffer[20] = {0};
uint8_t BL_MemBuffer[260] = {0};
//// ---------- Verita Register -------------------------

VRTPTC_StatusTypedef engst; // return engine state
//// = = = = = register bank = = = = = = = =
Verita_Register_Bank VRB_CL;
uint8_t RxBufferMtCl[RxbufferSize_VRT] = {0}; // Recieved packet buffer
float client_temp_mcuCC;
//// --------- INA219 ------------------------------------
//union {
//	uint8_t U8[12];
//	uint16_t U16[6];
//}INATT;

INA219_Read_Set inata; // read MCU first point
INA219_Read_Set inatb; // read Brd second point
INA219_Conf_Strc cofgra;

//// ---------------- MCP3208 ---------------------------
struct _mcp_read{
	uint16_t raw[8];
	float cv[8];
}mcp_read;
//// -------------- Timestamp ------------------------------
uint64_t timestamp_one = 0;
uint64_t timestamp_sensors = 0;
uint64_t timestamp_buzbtn = 0; // in while

uint32_t _millis = 0;
//// ------------------- Grandstate & flags & buttons--------------------------
//uint8_t flagc_bz = 0; // flag counter for buzzer
uint8_t flag_manual_relay = 0;

struct _bzzr{
	uint8_t flag; // flag counter for buzzer
	uint16_t priod_up;
	uint16_t priod_dn;
	uint32_t timestamp;
}buzzr;

//// 4x button
uint8_t btn_read[4] = {0}; // use Btn_read[3] as last process val
uint16_t btn_cnt = 0;
uint8_t counter = 0;

//// Rotary encoder knob button
uint16_t knobtick[2] = {0}; //// store val fron TIM QEI
//uint8_t btn_K[2] = {0};  //// encoder knob btn edge detect, use NVIC GPIO7 instead

//// Rotary encoder rotor Knob Flag
struct _k_flag{
	uint8_t up;  // knob rotate CW
	uint8_t dn;  // knob rotate CCW
	uint8_t cnt; // Knob is pressed
} k_flag;

struct _grandScript{
	uint8_t fullflag; //// flag indicate full script run
	uint8_t counter_overcurrent;
	uint32_t timelog; // use to stamp time for like count up 3 sec
} gScr;

static enum _GrandState{
	init,
	pre_lobby,
	lobby,
	pre_hw_chk,
	hw_chk,
	pre_fw_lob,
	fw_lob,
	pre_bootloader,
	s_bootloader,
	pre_monitor,
	monitor,
	pre_gpio_chk,
	gpio_chk,
	pre_danger,
	danger,
	pre_about,
	about
}GrandState = pre_lobby;

//// lists All port - pin to inspect first // avoid special pin like osilators / UART
//// GPIO_PIN_x is in bit position format (0 2 4 8 16 ...) which loss if stored in that form and log2() to calculate back
uint16_t List_GPIOA[] = {0,1,    4,5,6,7,8,9,10,            15,  20}; // 2,3 STLK RXTX / 11 12 VRT MTxCL / 13 14 TMS TCK
uint16_t List_GPIOB[] = {0,1,2,  4,5,6,7,8,9,10,   12,13,14,15,  20}; // 11 is Vcap
uint16_t List_GPIOC[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,        20};

char WR_A_PUPDR[30] = "\r\nA_PUR: "; ////
char WR_B_PUPDR[30] = "\r\nB_PUR: ";
char WR_C_PUPDR[30] = "\r\nC_PUR: ";
char WR_A_OPP[30]   = "\r\nA_OPP: "; ////
char WR_B_OPP[30]   = "\r\nB_OPP: ";
char WR_C_OPP[30]   = "\r\nC_OPP: ";
char WR_A_OOD[30]   = "\r\nA_OOD: ";
char WR_B_OOD[30]   = "\r\nB_OOD: ";
char WR_C_OOD[30]   = "\r\nC_OOD: ";

uint32_t UA_BL_Break; //// buffer for write GPIOA->AFR

//// ------------------------ Display buffer -----------------------------
typedef struct _disp_posixy{
	uint16_t xp; // x axis start point
	uint16_t yp; // y axis start point
}disp_posixy;

// in case start function, use in box_pointer()
uint8_t flag_boxpoint_start = 0;
static enum _boxpoint_runner{a_wait, a_change, a_boxclr} boxpoint_runner; //abd1, abd2, abd3, abd1s, abd2s, abd3s


struct _boxpoint{
	uint8_t flag_start;
	uint8_t choice_set;  //// which set of boxpos array will be chosen
	uint8_t ch_is;
} stboxp;

typedef struct _bposxyt{
	uint8_t n_s;
	uint16_t x[10];
	uint16_t y[10];
}bposxyType;
// first index 320-240 is nonebox -> the box will overframe
const bposxyType bposxy_no = {
		1,
		{320},
		{240}
};

const bposxyType bposxy_def = {
		2,
		{320,  10},
		{240, 220}
};

const bposxyType bposxy_lobby = {
		7,
		{320, 20, 20,  20,  20,  20,  10},
		{240, 60, 90, 120, 150, 180, 220}
};

const bposxyType bposxy_lobba = {
		6,
		{320,   0, 40, 80, 120, 160},
		{240, 220, 60, 90, 120, 150}
};

bposxyType bposxy[4] = {
		bposxy_def,
		bposxy_lobby,
		bposxy_lobba,
		bposxy_no
};
const enum _bpoxy{
	bpoxy_def,
	bpoxy_lobby,
	bpoxy_lobba,
	bpoxy_no
}bpoxy;

/* use state_box_choice_is to point the index
 * in this arrayset
 1 - this state is (enum, uint)
 2 - number of box in this state (uint)
 3 - 2D array of x y pos()

 to call in state, box_runner will run up-down by knob not more than n of box
i = stnum(1-lobby, 2-mon, 3-boot, 4...);
 thisstate[i].n //num of positions

 stboxchis++--, stboxchis %= thisstate[i].n

 box_pointer( thisstate[i].pos[stboxchis].x, thisstate[i].pos[stboxchis].y)
 box_pointer( thisstate[i].pos[stboxchis][0], thisstate[i].pos[stboxchis][1])
 * */

////concept -------------------------------------
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
void Protection_machine();
void manual_relay();
void Compare_pin_32(uint32_t raw32, uint16_t *Lista_GPIOx, uint8_t gpst,char *outchar);

void gpio_BL_UART_activate();
void gpio_BL_UART_Deactivate();

float ADCTVolta(uint16_t btt);
float TempEquat(float Vs);
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
  //// start QEI counter from the center 32768
  TIM3->CNT = 0x8000;
  knobtick[0] = TIM3->CNT;

  //// buzzer raram setting
  buzzr.flag = 1;
  buzzr.priod_up = 250;
  buzzr.priod_dn = 100;
  buzzr.timestamp = 0;

  stboxp.flag_start = 0;
  stboxp.choice_set = bpoxy_lobby;
  stboxp.ch_is = 0;

  gScr.counter_overcurrent = 0;
  gScr.fullflag = 0;

  k_flag.cnt = 0; k_flag.dn = 0; k_flag.up = 0;


  ili9341_Init();
  ili9341_DisplayOn();

//  ili9341_FillRect(50, 20, 50, 20, cl_RED);
//  ili9341_FillRect(100, 20, 50, 20, cl_GREEN);
//  ili9341_FillRect(150, 20, 50, 20, cl_BLUE);

#ifdef INA219_Wrk
  INA219_INIT_Calibrate(&hi2c1, INA219_ADDR_1);
  INA219_INIT_Calibrate(&hi2c1, INA219_ADDR_2);
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
   HAL_UART_Receive_DMA(&huart6, RxBufferMtCl, RxbufferSize_VRT);

   gpio_BL_UART_Deactivate();


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
	  //engst = Rx_Verita_engine(RxBufferMtCl, verita_regis);
	  //Rx_Verita_engine(RxBufferMtCl, &VRB_CL); //  Use callback instead
	  //Tx_Rq_Verita_engine(&huart6, &VRB_CL);
	  //// ----------------------------------------------------


	  if (HAL_GetTick() >= timestamp_buzbtn){
		timestamp_buzbtn += 20;

		knob_rotter();
		//running_box();
		//if(stboxp.choice_set){

		switch (boxpoint_runner){

		default:
		case a_wait:

			if(k_flag.up){

				stboxp.ch_is++;

				k_flag.up = 0;
				boxpoint_runner = a_change;}
			if(k_flag.dn){

				//stboxp.ch_is--;
				//stboxp.ch_is = (stboxp.ch_is < 0) ? bposxy[stboxp.choice_set].n_s - 1 : stboxp.ch_is;
				//if(stboxp.ch_is  < 0){
				//	stboxp.ch_is  = bposxy[stboxp.choice_set].n_s - 1;}

				if(stboxp.ch_is == 0){
						stboxp.ch_is  = bposxy[stboxp.choice_set].n_s - 1;}
				else{stboxp.ch_is--;}

				 k_flag.dn = 0;
				 boxpoint_runner = a_change;}

			stboxp.ch_is %= bposxy[stboxp.choice_set].n_s; // don't be more than spec of Grandstate sub

			break;

		case a_change:

			box_pointer(bposxy[stboxp.choice_set].x[stboxp.ch_is], bposxy[stboxp.choice_set].y[stboxp.ch_is]);
			boxpoint_runner = a_wait;
			break;

			}
		//}

	  }// timestamp_dis


	  if (HAL_GetTick() >= timestamp_one){
		  timestamp_one += 500;
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		  GrandState_Verita();

		  } // timestamp_one


	  if(HAL_GetTick() >= timestamp_sensors + 250){
		  timestamp_sensors = HAL_GetTick(); // in case this func is temporary terminate, can continue

		  /* Request every dynamic parameters
		   *  - INA219#1 INA219#2
		   *  - MCP3208 CH[0..7]
		   *  - Client's MCU temp rwquest
		   * */

		  Tx_UART_Verita_Command(&huart6, VRC_Request, VR_CPU_Temp);// request first > pending > convert

		  inata.Bus_V   = INA219Read_BusV(&hi2c1, INA219_ADDR_1);
		  inata.CURRENT = INA219Read_Current(&hi2c1, INA219_ADDR_1);
		  inata.POWER   = INA219Read_Power(&hi2c1, INA219_ADDR_1);
		  inata.SHUNT_V = INA219Read_ShuntV(&hi2c1, INA219_ADDR_1);

		  inatb.Bus_V   = INA219Read_BusV(&hi2c1, INA219_ADDR_2);
		  inatb.CURRENT = INA219Read_Current(&hi2c1, INA219_ADDR_2);
		  inatb.POWER   = INA219Read_Power(&hi2c1, INA219_ADDR_2);
		  inatb.SHUNT_V = INA219Read_ShuntV(&hi2c1, INA219_ADDR_2);

		  mcp_read.raw[0] = MCP3208_READ_8_DataSPI(&hspi3, M8_CH0);
		  mcp_read.raw[1] = MCP3208_READ_8_DataSPI(&hspi3, M8_CH1);
		  mcp_read.raw[2] = MCP3208_READ_8_DataSPI(&hspi3, M8_CH2);
		  mcp_read.raw[3] = MCP3208_READ_8_DataSPI(&hspi3, M8_CH3);
		  mcp_read.raw[4] = MCP3208_READ_8_DataSPI(&hspi3, M8_CH4);
		  mcp_read.raw[5] = MCP3208_READ_8_DataSPI(&hspi3, M8_CH5);
		  mcp_read.raw[6] = MCP3208_READ_8_DataSPI(&hspi3, M8_CH6);
		  mcp_read.raw[7] = MCP3208_READ_8_DataSPI(&hspi3, M8_CH7);

		  for(register int i = 0;i < 7;i++){
			  mcp_read.cv[i] = MCP320x_ADCbit_to_Volt(mcp_read.raw[i]);
		  }

		  client_temp_mcuCC = TempEquat(ADCTVolta(VRB_CL.Mark.cputemp));


		  Protection_machine();
		  manual_relay();
	  }

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
                          |RelayClient_Pin|Buzzer_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : LCD_RS_Pin LCD_CS_Pin client_NRST_Pin RelayClient_Pin
                           Buzzer_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_CS_Pin|client_NRST_Pin|RelayClient_Pin
                          |Buzzer_Pin;
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

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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

	  //mcp_read.raw[0] = MCP3208_READ_8_DataSPI(&hspi3, M8_CH0);
	  //mcp_read.cv[0] = MCP320x_ADCbit_to_Volt(mcp_read.raw[0]);
	  //sprintf(TextDispBuffer,"MCP : %.2f  ", mcp_read.cv[0]);
	  //ili9341_WriteString(20, 155, TextDispBuffer, Font16, cl_CYAN, cl_BLACK);

	  ////// 4x button
	  sprintf(TextDispBuffer,"btn %X %X %d",btn_read[1], btn_read[2], btn_cnt);
	  ili9341_WriteString(220, 185, TextDispBuffer, Font12, cl_YELLOW, cl_BLACK);

	  //// rortary encoder knob
	  sprintf(TextDispBuffer,"enc %d %d %d", knobtick[0], k_flag.cnt, k_flag.up); //flag_k_up
	  ili9341_WriteString(220, 200, TextDispBuffer, Font12, cl_WHITE, cl_BLACK);

	  sprintf(TextDispBuffer, "%ld, %d", TIM3->CNT, stboxp.ch_is); //state_box_choice_is
	  ili9341_WriteString(250, 215, TextDispBuffer, Font12, cl_WHITE, cl_BLACK);

}

void Compare_pin(){
	uint16_t A_PUPDR_N = VRB_CL.Mark.PA_PUPDR & 0xFFFF;
	uint16_t A_PUPDR_P = (VRB_CL.Mark.PA_PUPDR >> 16) & 0xFFFF;
	uint8_t ia, ib;


	// A
	for(register int i = 0;i < sizeof(List_GPIOA);i++){
		ia = (A_PUPDR_N >> List_GPIOA[i]) & 0x01;
		ib = (A_PUPDR_P >> List_GPIOA[i]) & 0x01;
		 if(ia == ib){
			 // record ?
			 char aadd[4] = " PA";
			 char aade = + List_GPIOA[i] + '0';
			 strncat(aadd, &aade, 2);
			 strncat(WR_A_PUPDR, aadd, 4);
		 }
	}
}


void Button_machine(){

	/* 4x btn_read[4]{
	 * [0]raw read,
	 * [1]read from 1 as rising detect,
	 * [2]read latest (bdebug),
	 * [3]read latest & erased when fin}
	 */
		btn_read[1] = btn_read[0];
		btn_read[0] = (0x0F & ~(GPIOB->IDR >> 12)); //// available for PB 12 13 14 15 or which the same bank only

		//// rising edge counter
		if(btn_read[0] && btn_read[1] == 0){
			btn_cnt += btn_read[0]; //// plus at each hex pos
			btn_read[2] = btn_read[0]; //// read latest, debug
			btn_read[3] = btn_read[0]; //// read latest, clearable

			//// manual relay flag try
			flag_manual_relay = 1;
		}

		//// Rotary Encoder knob Button----------------------------------
		//// use NVIC GPIO_7 instead
//		btn_K[1] = btn_K[0];
//		btn_K[0] = HAL_GPIO_ReadPin(KnobBTN_GPIO_Port, KnobBTN_Pin);
//
//		if(btn_K[0] == 0 && btn_K[1]){
//			btn_k_cnt++;
//		}

//		knob_rotter();

		//// knob rotter overflow_resist
		if(knobtick[0] <= 16 || knobtick[0] >= 0xFFF8){
			TIM3->CNT = 0x8000; // back to center
			knobtick[0] = 0x8000;
		}


}

void knob_rotter(){
	//// round up
	if((uint16_t)TIM3->CNT > knobtick[0]){  ////(uint16_t)TIM3->CNT - knobtick[0] >= 2
		//flag_k_up = 1;
		k_flag.up = 1;
		knobtick[0] = TIM3->CNT;

	}
	//// round down
	else if((uint16_t)TIM3->CNT < knobtick[0]){ ////knobtick[0] - (uint16_t)TIM3->CNT >= 2
		//flag_k_dn = 1;
		k_flag.dn = 1;
		knobtick[0] = TIM3->CNT;

	}
	else{}

}

void Protection_machine(){

	//// overcurrent
	if (inata.CURRENT >= Current_limit_mA || inatb.CURRENT >= Current_limit_mA){
		gScr.counter_overcurrent++;

		if(gScr.counter_overcurrent >= 8){
			gScr.counter_overcurrent = 0;
			gScr.fullflag = 0;

			////Relay_cut
			HAL_GPIO_WritePin(RelayClient_GPIO_Port, RelayClient_Pin, GPIO_PIN_RESET);

			//// Buzzer scream
			buzzr.flag = 1;
			buzzr.priod_up = 1000;
			buzzer_scream_cnt();
			//// interrupt, go to state Client error.
			GrandState = pre_danger;
		}
	}else{gScr.counter_overcurrent = 0;}

}

void manual_relay(){
	if(flag_manual_relay){

		if(GrandState == monitor){
			HAL_GPIO_TogglePin(RelayClient_GPIO_Port, RelayClient_Pin);
		}
		flag_manual_relay = 0;
	}
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
			 sprintf(aadd, " ");
			 strncat(outchar, aadd, 1);
		 }
	}

	if(!cntr_w){
		sprintf(aadd, "_PASS");
		strncat(outchar, aadd, 7);
	}
}

void buzzer_scream_cnt(){
	static enum {bz_init, bz_silent, bz_scream} bz_st = bz_init;

		switch(bz_st){
		default:
		case bz_init:
			//HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

			if(buzzr.flag){

				HAL_TIM_Base_Start_IT(&htim10);
				buzzr.timestamp = buzzr.priod_up + HAL_GetTick();

				bz_st = bz_scream;
				/// down flag_counter every 1 scream
				buzzr.flag--;
			}else{
				HAL_TIM_Base_Stop_IT(&htim10);
			}

			break;

		case bz_scream:
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);

			if(HAL_GetTick() >= buzzr.timestamp){
				buzzr.timestamp = buzzr.priod_dn + HAL_GetTick();

				bz_st = bz_silent;
			}
			break;


		case bz_silent:
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

			if(HAL_GetTick() >= buzzr.timestamp){

				if(buzzr.flag){
					buzzr.timestamp = buzzr.priod_up + HAL_GetTick();

					buzzr.flag--;
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

		stboxp.choice_set = bpoxy_lobby; //state_box_choice_n = 4;
		ili9341_FillRect(0, 0, 320, 240, cl_BLACK);

		ili9341_FillRect(0, 0, 320, 32, cl_GRAY);

		ili9341_FillRect(305, 0, 15, 240, cl_ORANGE);

		sprintf(TextDispBuffer,"Nucleo-F411RE Tester");
		ili9341_WriteStringNoBG(10, 10, TextDispBuffer, Font20, cl_BLACK);
		ili9341_DrawHLine(cl_ORANGE, 0, 33, 320);

		sprintf(TextDispBuffer,"Full-Script");
		ili9341_WriteStringNoBG(50, 60, TextDispBuffer, Font16, cl_CYAN);

		sprintf(TextDispBuffer,"PWR_Monitor");
		ili9341_WriteStringNoBG(50, 90, TextDispBuffer, Font16, cl_CYAN);

		sprintf(TextDispBuffer,"Hardware-mode");
		ili9341_WriteStringNoBG(50, 120, TextDispBuffer, Font16, cl_CYAN);

		sprintf(TextDispBuffer,"Firmware-mode");
		ili9341_WriteStringNoBG(50, 150, TextDispBuffer, Font16, cl_CYAN);

		sprintf(TextDispBuffer,"About Verita");
		ili9341_WriteStringNoBG(50, 180, TextDispBuffer, Font16, cl_CYAN);

		GrandState = lobby;
		break; // pre-lobby

	default:
	case lobby:
		stboxp.choice_set = bpoxy_lobby; //state_box_choice_n = 4;

		// debug
		sprintf(TextDispBuffer, "%ld, %d", TIM3->CNT, stboxp.ch_is);
		ili9341_WriteString(200, 220, TextDispBuffer, Font12, cl_WHITE, cl_BLACK);
		//simple_scr();

		if(k_flag.cnt){


			if (stboxp.ch_is == 1) {
				gScr.fullflag = ff_runfull;
				GrandState = pre_hw_chk;
			}
			else if (stboxp.ch_is == 2){GrandState = pre_monitor;}
			else if (stboxp.ch_is == 3){GrandState = pre_hw_chk;}
			else if (stboxp.ch_is == 4){GrandState = pre_fw_lob;}
			else if (stboxp.ch_is == 5){GrandState = pre_about;}

		k_flag.cnt = 0;
		}

		break; // lobby

	case init:
		stboxp.choice_set = bpoxy_def;

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

	case pre_hw_chk:
		stboxp.choice_set = bpoxy_def;
		ili9341_FillRect(0, 30, 320, 210, cl_BLACK);
		ili9341_FillRect(0, 0, 320, 30, cl_ORANGE);

		//// Auto ON relay
		HAL_GPIO_WritePin(RelayClient_GPIO_Port, RelayClient_Pin, GPIO_PIN_SET);

		sprintf(TextDispBuffer,"Hardware_Chk");
		ili9341_WriteStringNoBG(60, 5, TextDispBuffer, Font20, cl_WHITE);


		sprintf(TextDispBuffer,"5Vin:");
		ili9341_WriteStringNoBG(15, 50, TextDispBuffer, Font16, cl_WHITE);

		sprintf(TextDispBuffer,"3V3:");
		ili9341_WriteStringNoBG(15, 75, TextDispBuffer, Font16, cl_WHITE);

		sprintf(TextDispBuffer,"3V3");
		ili9341_WriteStringNoBG(15, 95, TextDispBuffer, Font16, cl_WHITE);
		sprintf(TextDispBuffer,"STLink");
		ili9341_WriteStringNoBG(15, 113, TextDispBuffer, Font12, cl_WHITE);

		sprintf(TextDispBuffer,"I Brd:");
		ili9341_WriteStringNoBG(15, 125, TextDispBuffer, Font16, cl_WHITE);

		sprintf(TextDispBuffer,"I MCU:");
		ili9341_WriteStringNoBG(15, 150, TextDispBuffer, Font16, cl_WHITE);

		sprintf(TextDispBuffer,"mV"); ili9341_WriteStringNoBG(150, 54, TextDispBuffer, Font12, cl_WHITE);
		sprintf(TextDispBuffer,"mV"); ili9341_WriteStringNoBG(150, 79, TextDispBuffer, Font12, cl_WHITE);
		sprintf(TextDispBuffer,"mV"); ili9341_WriteStringNoBG(150, 104, TextDispBuffer, Font12, cl_WHITE);
		sprintf(TextDispBuffer,"mA"); ili9341_WriteStringNoBG(150, 129, TextDispBuffer, Font12, cl_WHITE);
		sprintf(TextDispBuffer,"mA"); ili9341_WriteStringNoBG(150, 154, TextDispBuffer, Font12, cl_WHITE);


		gScr.timelog = HAL_GetTick() + 3500;

		k_flag.cnt = 0; //// prevent over state jump
		GrandState = hw_chk;
		break; //pre_hw_chk
	case hw_chk:
		stboxp.choice_set = bpoxy_def;


		sprintf(TextDispBuffer,"<-Back");
		ili9341_WriteStringNoBG(30, 220, TextDispBuffer, Font16, cl_WHITE);


		/*condition to jump state from HW
		 * if currentchk ok
		 * 		if pressed or 3 sec pass -> go bootloader
		 * else force back lobby & turnoff relay.
		 * */
		if(k_flag.cnt){ //|| HAL_GetTick() >= gScr.timelog

			if(gScr.fullflag == ff_runfull){

			GrandState = pre_bootloader;
			}
			else{
				GrandState = pre_lobby;
			}
		}

		break; //hw_chk

	case pre_bootloader:
		stboxp.choice_set = bpoxy_no;

		ili9341_FillRect(0, 0, 320, 30, cl_YELLOW);
		ili9341_FillRect(0, 30, 320, 210, cl_BLACK);

		sprintf(TextDispBuffer,"- BOOTLOADER -");
		ili9341_WriteStringNoBG(60, 5, TextDispBuffer, Font20, cl_BLACK);


		sprintf(TextDispBuffer," .bin script is booting...");
		ili9341_WriteStringNoBG(20, 60, TextDispBuffer, Font16, cl_WHITE);
		sprintf(TextDispBuffer,"Don't pluck  off");
		ili9341_WriteStringNoBG(40, 110, TextDispBuffer, Font20, cl_WHITE);
		sprintf(TextDispBuffer,"the client board");
		ili9341_WriteStringNoBG(40, 135, TextDispBuffer, Font20, cl_ORANGE);

		k_flag.cnt = 0;
		GrandState = s_bootloader;
		break; // pre_bootloader

	case s_bootloader:
		stboxp.choice_set = bpoxy_def;

		//// enable UART, disable after endboot, prevent misunderstanding when GPIO test
		gpio_BL_UART_activate();

		//// find n times must be loop to upload all code
		bootloop_n = (boot_size / 256) + ((boot_size % 256)>0 ? 1:0);
		//bootloop_n = (uint8_t)ceil(boot_size / 256.0);

		BL_UART_Start(&huart1);

		//// Flash Memory Erase ============,
		//// Erase1_Mass_CMD makes bootloader not response to ALL write CMD / dont know why
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

			sprintf(TextDispBuffer,"Finish");
			ili9341_WriteStringNoBG(100, 160, TextDispBuffer, Font24, cl_GREEN);

		//// disable UART, disable after endboot, prevent misunderstanding when GPIO test
		gpio_BL_UART_Deactivate();

		k_flag.cnt = 0;//// prevent over state jump
		if(gScr.fullflag == ff_runfull){
			GrandState = pre_gpio_chk;
		}else{
			GrandState = pre_lobby;
		}

		break; ////s_bootloader

	case pre_gpio_chk:
			stboxp.choice_set = bpoxy_def;
			//// Send CMD to client to run GPIO testscript
			Tx_UART_Verita_Command(&huart6, VRC_Flag_ger, VRF_GPIO_Runalltest);

			//// Set UI
			ili9341_FillRect(0, 0, 320, 30, cl_DARKCYAN);
			ili9341_FillRect(0, 30, 320, 210, cl_BLACK);

			sprintf(TextDispBuffer,"GPIO Selftest");
			ili9341_WriteStringNoBG(60, 5, TextDispBuffer, Font20, cl_WHITE);

			sprintf(TextDispBuffer,"PUPDR:");
			ili9341_WriteStringNoBG(15, 50, TextDispBuffer, Font20, cl_WHITE);

			sprintf(TextDispBuffer,"PP:");
			ili9341_WriteStringNoBG(15, 85, TextDispBuffer, Font20, cl_WHITE);

			sprintf(TextDispBuffer,"OD:");
			ili9341_WriteStringNoBG(15, 120, TextDispBuffer, Font20, cl_WHITE);

			sprintf(TextDispBuffer,"MCU Temp:");
			ili9341_WriteStringNoBG(15, 160, TextDispBuffer, Font16, cl_WHITE);

			sprintf(TextDispBuffer,"FWID:");
			ili9341_WriteStringNoBG(15, 185, TextDispBuffer, Font16, cl_WHITE);

			sprintf(TextDispBuffer,"Finish >> ");
			ili9341_WriteStringNoBG(250, 220, TextDispBuffer, Font16, cl_WHITE);


			//// checkif GPIO test is finished ?
			if(VRB_CL.Mark.Flag_next){ // runalltest cplt
				Tx_UART_Verita_Command(&huart6, VRC_Flag_ger, VRF_SendALLTestData);
				HAL_Delay(100);

				if(VRB_CL.Mark.Flag_ger == VRF_SendALLTestData){ //// 'll send this flag back after cplt

					Compare_pin_32(VRB_CL.Mark.PA_PUPDR, List_GPIOA, 0, WR_A_PUPDR);
					Compare_pin_32(VRB_CL.Mark.PA_OUT_PP, List_GPIOA, 0, WR_A_OPP);
					Compare_pin_32(VRB_CL.Mark.PA_OUT_OD, List_GPIOA, 0, WR_A_OOD);

					Compare_pin_32(VRB_CL.Mark.PB_PUPDR, List_GPIOB, 1,  WR_B_PUPDR);
					Compare_pin_32(VRB_CL.Mark.PB_OUT_PP, List_GPIOB, 1, WR_B_OPP);
					Compare_pin_32(VRB_CL.Mark.PB_OUT_OD, List_GPIOB, 1, WR_B_OOD);

					Compare_pin_32(VRB_CL.Mark.PC_PUPDR, List_GPIOC, 2, WR_C_PUPDR);
					Compare_pin_32(VRB_CL.Mark.PC_OUT_PP, List_GPIOC, 2, WR_C_OPP);
					Compare_pin_32(VRB_CL.Mark.PC_OUT_OD, List_GPIOC, 2, WR_C_OOD);

					VRB_CL.Mark.Flag_ger = 0;
					VRB_CL.Mark.Flag_next = 0;
					GrandState = gpio_chk;
				}
			}
			break; //// pre_gpio_chk

		case gpio_chk:
			stboxp.choice_set = bpoxy_def;



			if(k_flag.cnt && stboxp.ch_is == 1){ //// Back to lobby
					GrandState = pre_lobby;
					k_flag.cnt = 0;
					HAL_GPIO_WritePin(RelayClient_GPIO_Port, RelayClient_Pin, GPIO_PIN_RESET);
					}
			break; //gpio_chk

	case pre_monitor:
		stboxp.choice_set = bpoxy_def;
		ili9341_FillRect(0, 30, 320, 210, cl_BLACK);
		ili9341_FillRect(0, 0, 320, 30, cl_BLUE);

		//// Auto ON relay
		HAL_GPIO_WritePin(RelayClient_GPIO_Port, RelayClient_Pin, GPIO_PIN_SET);

		sprintf(TextDispBuffer,"PWR_Monitor");
		ili9341_WriteStringNoBG(60, 5, TextDispBuffer, Font20, cl_WHITE);

		sprintf(TextDispBuffer,"<-Back");
		ili9341_WriteStringNoBG(30, 220, TextDispBuffer, Font16, cl_WHITE);

		//sprintf(TextDispBuffer,"calib:%4X", inata.Calibra);
		//ili9341_WriteString(20, 30, TextDispBuffer, Font12, cl_GREENYELLOW, cl_BLACK);

		sprintf(TextDispBuffer,"5Vin:");
		ili9341_WriteStringNoBG(15, 50, TextDispBuffer, Font16, cl_WHITE);

		sprintf(TextDispBuffer,"3V3:");
		ili9341_WriteStringNoBG(15, 75, TextDispBuffer, Font16, cl_WHITE);

		sprintf(TextDispBuffer,"I MCU:");
		ili9341_WriteStringNoBG(15, 100, TextDispBuffer, Font16, cl_WHITE);

		sprintf(TextDispBuffer,"I Brd:");
		ili9341_WriteStringNoBG(15, 125, TextDispBuffer, Font16, cl_WHITE);

		sprintf(TextDispBuffer,"PWR");
		ili9341_WriteStringNoBG(15, 150, TextDispBuffer, Font16, cl_WHITE);

		sprintf(TextDispBuffer,"MCU");
		ili9341_WriteStringNoBG(60, 150, TextDispBuffer, Font12, cl_WHITE);
		sprintf(TextDispBuffer,"Brd");
		ili9341_WriteStringNoBG(60, 170, TextDispBuffer, Font12, cl_WHITE);

		sprintf(TextDispBuffer,"MCP3208");
		ili9341_WriteStringNoBG(220, 40, TextDispBuffer, Font16, cl_WHITE);

		sprintf(TextDispBuffer,"mV"); ili9341_WriteStringNoBG(150, 54, TextDispBuffer, Font12, cl_WHITE);
		sprintf(TextDispBuffer,"mV"); ili9341_WriteStringNoBG(150, 79, TextDispBuffer, Font12, cl_WHITE);
		sprintf(TextDispBuffer,"mA"); ili9341_WriteStringNoBG(150, 104, TextDispBuffer, Font12, cl_WHITE);
		sprintf(TextDispBuffer,"mA"); ili9341_WriteStringNoBG(150, 129, TextDispBuffer, Font12, cl_WHITE);
		sprintf(TextDispBuffer,"mW"); ili9341_WriteStringNoBG(170, 154, TextDispBuffer, Font12, cl_WHITE);
		sprintf(TextDispBuffer,"mW"); ili9341_WriteStringNoBG(170, 174, TextDispBuffer, Font12, cl_WHITE);

		for(register int t = 0; t < 7; t++){
			sprintf(TextDispBuffer,"CH%d",t);
			ili9341_WriteStringNoBG(220, 65 + (12*t), TextDispBuffer, Font12, cl_YELLOW);
		}

		sprintf(TextDispBuffer,"MCU_Cl_Temp:");
		ili9341_WriteString(20, 190, TextDispBuffer, Font12, cl_WHITE, cl_BLACK);

		k_flag.cnt = 0; //// prevent over state jump
		GrandState = monitor;
		break; //// pre monitor


	case monitor:
		stboxp.choice_set = bpoxy_def;
		simple_scr();

		sprintf(TextDispBuffer,"%4d", inatb.Bus_V);
		if(inatb.Bus_V < 2000){
			ili9341_WriteString(90, 50, TextDispBuffer, Font16, cl_RED, cl_BLACK);
		}else{
			ili9341_WriteString(90, 50, TextDispBuffer, Font16, cl_GREEN, cl_BLACK);
		}

		sprintf(TextDispBuffer,"%4d", inata.Bus_V);
		if(inata.Bus_V < 2000){
			ili9341_WriteString(90, 75, TextDispBuffer, Font16, cl_RED, cl_BLACK);
		}else{
			ili9341_WriteString(90, 75, TextDispBuffer, Font16, cl_GREEN, cl_BLACK);
		}

		sprintf(TextDispBuffer,"%4d", inata.CURRENT);
		ili9341_WriteString(90, 100, TextDispBuffer, Font16, cl_CYAN, cl_BLACK);
		sprintf(TextDispBuffer,"%4d", inatb.CURRENT);
		ili9341_WriteString(90, 125, TextDispBuffer, Font16, cl_CYAN, cl_BLACK);

		sprintf(TextDispBuffer,"%4.1f", inata.POWER);
		ili9341_WriteString(95, 150, TextDispBuffer, Font16, cl_ORANGE, cl_BLACK);
		sprintf(TextDispBuffer,"%4.1f", inatb.POWER);
		ili9341_WriteString(95, 170, TextDispBuffer, Font16, cl_ORANGE, cl_BLACK);

		//// MCP3208 ADC Raw Read
		ili9341_FillRect(250, 65, 30, 84, cl_BLACK);
		for(register int t = 0; t < 7; t++){
			sprintf(TextDispBuffer,"%d",mcp_read.raw[t]);
		ili9341_WriteStringNoBG(250, 65 + (12*t), TextDispBuffer, Font12, cl_WHITE);
		}

		//// Client's CPU Temp
		sprintf(TextDispBuffer,"%2.1f C", client_temp_mcuCC);
			if(client_temp_mcuCC < 65){
				ili9341_WriteString(120, 190, TextDispBuffer, Font16, cl_LIGHTGREY, cl_BLACK);
			}else{
				ili9341_WriteString(120, 190, TextDispBuffer, Font16, cl_RED, cl_BLACK);
			}


		if(k_flag.cnt && stboxp.ch_is == 1){ //// Back to lobby
			GrandState = pre_lobby;
			k_flag.cnt = 0;
			}
		break; // monitor


	case pre_danger:
		stboxp.choice_set = bpoxy_def;
		ili9341_FillRect(0, 30, 320, 210, cl_BLACK);
		ili9341_FillRect(0, 0, 320, 30, cl_RED);

		sprintf(TextDispBuffer,"Danger!!!");
		ili9341_WriteStringNoBG(105, 5, TextDispBuffer, Font20, cl_WHITE);

		sprintf(TextDispBuffer,"<-Back to lobby");
		ili9341_WriteStringNoBG(30, 220, TextDispBuffer, Font16, cl_WHITE);

		sprintf(TextDispBuffer,"Overcurrent Detect!");
		ili9341_WriteStringNoBG(40, 70, TextDispBuffer, Font20, cl_WHITE);

		sprintf(TextDispBuffer,"I MCU:");
		ili9341_WriteString(20, 100, TextDispBuffer, Font16, cl_WHITE, cl_BLACK);

		sprintf(TextDispBuffer,"I Brd:");
		ili9341_WriteString(20, 130, TextDispBuffer, Font16, cl_WHITE, cl_BLACK);

		sprintf(TextDispBuffer,"%4d", inata.CURRENT);
		if(inata.CURRENT >= Current_limit_mA){
			ili9341_WriteString(120, 100, TextDispBuffer, Font20, cl_RED, cl_BLACK);
			sprintf(TextDispBuffer,"FAIL"); ili9341_WriteStringNoBG(220, 100, TextDispBuffer, Font20, cl_RED);
		}else{
			ili9341_WriteString(120, 100, TextDispBuffer, Font20, cl_WHITE, cl_BLACK);
			}

		sprintf(TextDispBuffer,"%4d", inatb.CURRENT);
		if(inatb.CURRENT >= Current_limit_mA){
			ili9341_WriteString(120, 130, TextDispBuffer, Font20, cl_RED, cl_BLACK);
			sprintf(TextDispBuffer,"FAIL"); ili9341_WriteStringNoBG(220, 130, TextDispBuffer, Font20, cl_RED);
		}else{
			ili9341_WriteString(120, 130, TextDispBuffer, Font20, cl_WHITE, cl_BLACK);
			}

		GrandState = danger;
		break;

	case danger:

		if(k_flag.cnt){ //// Back to lobby
			GrandState = pre_lobby;
			k_flag.cnt = 0;
			}
		break;
	}
}

float ADCTVolta(uint16_t btt){return (btt /4096.0) * 3.3;}
float TempEquat(float Vs){
	//Vs = V tmp read , V25= 0.76V, Avg_slope = 2.5 mV
	return ((Vs - 0.76)/(0.0025)) + 25.0; //2.5*0.001
}

void gpio_BL_UART_activate(){
	/* Change AFRH (AFR[1]) for PA9 PA10 from default GPIO to UART AF7, Only at Bootloader process
	 * PA9 PA10 is block 1, 2 in AFR[1]
	 * AF7(USART1) = 0x07 | AF0(System) = 0x0
	 *
	 * RM0383 P150 Fig 17 AFR Mux & P164 GPIOx_AFR register map
	 * */
	  UA_BL_Break = GPIOA->AFR[1];
	  UA_BL_Break &= ~( 0b1111 << (1 * 4U));
	  UA_BL_Break &= ~( 0b1111 << (2 * 4U));
	  UA_BL_Break |= ( 0x7 << (1 * 4U));
	  UA_BL_Break |= ( 0x7 << (2 * 4U));
	  GPIOA->AFR[1] = UA_BL_Break;

	  uint32_t tyyy = GPIOA->PUPDR;
	  tyyy &= ~( 0b11 << (9 * 2U));
	  tyyy &= ~( 0b11 << (10 * 2U));
	  tyyy |= ( GPIO_NOPULL << (9 * 2U));
	  tyyy |= ( GPIO_NOPULL << (10 * 2U));
	  GPIOA->PUPDR = tyyy;
}

void gpio_BL_UART_Deactivate(){
	/* Change AFRH (AFR[1]) for PA9 PA10 from Init UART to default GPIO, prevent UART High distrub the client
	 * PA9 PA10 is block 1, 2 in AFR[1]
	 * AF7(USART1) = 0x07 | AF0(System) = 0x0
	 * */
	  UA_BL_Break = GPIOA->AFR[1];
	  UA_BL_Break &= ~( 0b1111 << (1 * 4U));
	  UA_BL_Break &= ~( 0b1111 << (2 * 4U));
	  UA_BL_Break |= ( 0x0 << (1 * 4U));
	  UA_BL_Break |= ( 0x0 << (2 * 4U));
	  GPIOA->AFR[1] = UA_BL_Break;

		 uint32_t tyyy = GPIOA->PUPDR;
		 tyyy &= ~( 0b11 << (9 * 2U));
		 tyyy &= ~( 0b11 << (10 * 2U));
		 tyyy |= ( GPIO_NOPULL << (9 * 2U));
		 tyyy |= ( GPIO_NOPULL << (10 * 2U));
		 GPIOA->PUPDR = tyyy;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_13){
		//INA219_BitReset(&hi2c1, INA219_ADDR_1);
		buzzr.flag = 8;
		buzzr.priod_up = 250;
		buzzr.priod_dn = 100;
		buzzer_scream_cnt();
		//// bootloader test
		//GrandState = s_bootloader;
		//GrandState = init;

		Tx_UART_Verita_Command(&huart6, VRC_Flag_ger, VRF_GPIO_Runalltest);
		//Tx_UART_Verita_Command(&huart6, VRC_Request, VR_FWID);

		}

	//// knob rotter button pressed
	if(GPIO_Pin == GPIO_PIN_7){
		k_flag.cnt++;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim10){
		_millis++;
		//// Timer interrupt
		buzzer_scream_cnt();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	Rx_Verita_engine_callBak(RxBufferMtCl, &VRB_CL); //// try using only 1 slot 9 Buffer
	Tx_Rq_Verita_engine(&huart6, &VRB_CL);
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
