/* USER CODE BEGIN Header */

/*
 * HEPA/UV Test Code
*/
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for HEPA_UV PCBA
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "EEPROM.h"
#include "can.h"
#include "LEDs.h"
#include "voltage_monitors.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C2_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

void UVLEDTest(void);
void HEPALEDTest(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define TIMCLOCK   64000000
#define PRESCALAR  64


uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t IC_Val3 = 0;
uint32_t IC_Val4 = 0;
uint32_t IC_Val5 = 0;
uint32_t DifferenceT1 = 0;
uint32_t DifferenceT2 = 0;
uint32_t DifferenceT3 = 0;
uint32_t DifferenceT4 = 0;
int Is_First_Captured = 0;
int Is_Second_Captured = 0;
int Is_Third_Captured = 0;
int Is_Fourth_Captured = 0;
uint32_t usWidthT1 = 0;
uint32_t usWidthT2 = 0;
uint32_t usWidthT3 = 0;
uint32_t usWidthT4 = 0;
uint32_t usWidthT = 0;
float RPM = 0;
char buf[100];
int ready = 1;

/* CAN Configuration */
uint8_t debug_str[32];
uint8_t RxData[8];
uint8_t TxData[8];
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t EEPROM_CAN_Msg;

/* EEPROM */
uint8_t dataRead[50];
uint8_t dataWrite[8];
uint8_t dataw1[] = "HEPA/UV \r\n";
uint8_t dataw2[] = "PN: 913-00072 \r\n";
uint8_t dataw3[] = "SN: 0002 \r\n";
uint8_t datar1[100];
uint8_t datar2[100];
uint8_t datar3[100];
uint8_t line1[] = "\r\n";

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//	{
//	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
//	{
//		//HAL_GPIO_TogglePin(LED_DRIVE_GPIO_Port, LED_DRIVE_Pin);
//		//HAL_UART_Transmit(&hlpuart1, dataw3, sizeof(dataw3), HAL_MAX_DELAY);
//
//		if (Is_First_Captured==0) // if the first value is not captured yet
//		{
//			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
//			Is_First_Captured = 1;  // set the first captured as true
//		}
//
//		else if (Is_Second_Captured==0) // if the second value is not captured yet
//		{
//			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the second value
//			if (IC_Val2 > IC_Val1)
//				{
//					DifferenceT1 = IC_Val2-IC_Val1;
//				}
//
//			else if (IC_Val1 > IC_Val2)
//				{
//					DifferenceT1 = (0xffffffff - IC_Val1) + IC_Val2;
//				}
//
//			float refClock = TIMCLOCK/(PRESCALAR);
//			float mFactor = 1000000/refClock;
//
//			usWidthT1 = DifferenceT1*mFactor;
//			Is_Second_Captured = 1;  // set the second captured as true
//
//			sprintf(buf, "usWidthT1 = %hu\r\n", (int)usWidthT1);
//			HAL_UART_Transmit(&hlpuart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
//		}
//
//		else if (Is_Third_Captured==0) // if the third value is not captured yet
//		{
//			IC_Val3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the third value
//			if (IC_Val3 > IC_Val2)
//				{
//					DifferenceT2 = IC_Val3-IC_Val2;
//				}
//
//			else if (IC_Val2 > IC_Val3)
//				{
//					DifferenceT2 = (0xffffffff - IC_Val2) + IC_Val3;
//				}
//
//			float refClock = TIMCLOCK/(PRESCALAR);
//			float mFactor = 1000000/refClock;
//
//			usWidthT2 = DifferenceT2*mFactor;
//			Is_Third_Captured = 1;  // set the third captured as true
//
//			sprintf(buf, "usWidthT2 = %hu\r\n", (int)usWidthT2);
//			HAL_UART_Transmit(&hlpuart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
//		}
//		else if (Is_Fourth_Captured==0) // if the fourth value is not captured yet
//		{
//			IC_Val4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the fourth value
//			if (IC_Val4 > IC_Val3)
//			{
//				DifferenceT3 = IC_Val4-IC_Val3;
//			}
//
//			else if (IC_Val3 > IC_Val4)
//			{
//				DifferenceT3 = (0xffffffff - IC_Val3) + IC_Val4;
//			}
//
//			float refClock = TIMCLOCK/(PRESCALAR);
//			float mFactor = 1000000/refClock;
//
//			usWidthT3 = DifferenceT3*mFactor;
//			Is_Fourth_Captured = 1;  // set the fourth captured as true
//
//			sprintf(buf, "usWidthT3 = %hu\r\n", (int)usWidthT3);
//			HAL_UART_Transmit(&hlpuart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
//		}
//		else // read fifth value
//		{
//			IC_Val5 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the fifth value
//			if (IC_Val5 > IC_Val4)
//			{
//				DifferenceT4 = IC_Val5-IC_Val4;
//			}
//
//			else if (IC_Val4 > IC_Val5)
//			{
//				DifferenceT4 = (0xffffffff - IC_Val4) + IC_Val5;
//			}
//
//			float refClock = TIMCLOCK/(PRESCALAR);
//			float mFactor = 1000000/refClock;
//
//			usWidthT4 = DifferenceT4*mFactor;
//			usWidthT = usWidthT1 + usWidthT2 + usWidthT3 + usWidthT4;
//			//RPM = 60/(usWidthT/1000000);
//			RPM = round(60/((float)usWidthT/1000000));
//
//			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
//			Is_First_Captured = 0; // set it back to false
//			Is_Second_Captured = 0; // set it back to false
//			Is_Third_Captured = 0; // set it back to false
//			Is_Fourth_Captured = 0; // set it back to false
//
//			sprintf(buf, "usWidthT4 = %hu\r\n", (int)usWidthT4);
//			HAL_UART_Transmit(&hlpuart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
//
//			sprintf(buf, "usWidthT = %hu\r\n", (int)usWidthT);
//			HAL_UART_Transmit(&hlpuart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
//
//			sprintf(buf, "RPM = %.0f\r\n", RPM);
//			HAL_UART_Transmit(&hlpuart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
//
//			HAL_UART_Transmit(&hlpuart1, "\n", 1, HAL_MAX_DELAY);
//
//		}
//	}
//  }


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
  MX_FDCAN1_Init();
  MX_I2C2_Init();
  MX_LPUART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C3_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /* Test EEPROM */
//  HAL_GPIO_WritePin(nEEPROM_WP_GPIO_Port, nEEPROM_WP_Pin, GPIO_PIN_RESET);

  /* Erase EEPROM */
//  for (uint8_t i=0; i<251; i++)
//    {
//	  EEPROM_PageErase (i);
//    }

  /* Write to EEPROM */
//  EEPROM_Write(0, 0, dataw1, strlen((char *)dataw1));
//  EEPROM_Write(1, 0, dataw2, strlen((char *)dataw2));
//  EEPROM_Write(2, 0, dataw3, strlen((char *)dataw3));

  /* Read EEPROM */
//  EEPROM_Read(0, 0, datar1, 50);
  //EEPROM_Read(1, 0, datar1, 50);
  //EEPROM_Read(2, 0, datar1, 50);

  /* Start Tach Capture */
//  if(HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
  //HAL_GPIO_WritePin(HEPA_ON_OFF_GPIO_Port, HEPA_ON_OFF_Pin, GPIO_PIN_SET);

  //HAL_GPIO_WritePin(UV_ON_OFF_MCU_GPIO_Port, UV_ON_OFF_MCU_Pin, GPIO_PIN_SET);

  EEPROM_CAN_Msg = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	  /* LED Blink */
	  if (EEPROM_CAN_Msg == 1)
	  {
		  for (int i = 0; i < 8; i++)
		  {
			  dataWrite[i] = (uint8_t)(rand() % 20);
		  }
		  HAL_UART_Transmit(&hlpuart1, dataWrite, strlen((const char*)(dataWrite)), HAL_MAX_DELAY);
		  HAL_UART_Transmit(&hlpuart1, line1, sizeof(line1), HAL_MAX_DELAY);
		  /* Write EEPROM */
		  //EEPROM_Write(0, 0, dataw1, strlen((char *)dataw1));
		  send_msg(dataWrite, 8);
		  EEPROM_Write(0, 0, dataWrite, sizeof(dataWrite));

		  //EEPROM_Write(1, 0, dataw2, strlen((char *)dataw2));

		  /* Read EEPROM */
		  EEPROM_Read(0, 0, datar1, 8);
		  send_msg(datar1, 8);
		  //EEPROM_Read(1, 0, datar2, 50);

		  EEPROM_CAN_Msg = 0;

		  /* Erase EEPROM */
		  for (uint8_t i=0; i<251; i++)
		  {
			  EEPROM_PageErase (i);
		  }
		  //HAL_Delay(100);
	  }
	  /* CAN FD Test */
	  test_can_bus();
	  //uint8_t msg[4] = { 0x1, 0x2, 0x3, 0x4 };
	  //send_msg(msg, 4);
	  //can_listen();



	  if (HAL_GPIO_ReadPin(DOOR_OPEN_MCU_GPIO_Port, DOOR_OPEN_MCU_Pin))
	  {
		  HAL_GPIO_WritePin(UV_ON_OFF_MCU_GPIO_Port, UV_ON_OFF_MCU_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(UV_B_CTRL_GPIO_Port, UV_B_CTRL_Pin, GPIO_PIN_RESET);
		  ready = 1;
	  }
	  else if (HAL_GPIO_ReadPin(POS_SW_MCU_GPIO_Port, POS_SW_MCU_Pin))
	  	  {
	  		  HAL_GPIO_WritePin(UV_ON_OFF_MCU_GPIO_Port, UV_ON_OFF_MCU_Pin, GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(UV_B_CTRL_GPIO_Port, UV_B_CTRL_Pin, GPIO_PIN_RESET);
	  		  ready = 1;
	  	  }
	  else ready = 0;

	   /*Test UV ON/OFF Pin*/
	  if (ready == 0)
	  {
		  if (!HAL_GPIO_ReadPin(UV_NO_GPIO_Port, UV_NO_Pin))
	    {
	  	  HAL_GPIO_TogglePin(UV_ON_OFF_MCU_GPIO_Port, UV_ON_OFF_MCU_Pin);
	  	  if(!HAL_GPIO_ReadPin(UV_ON_OFF_MCU_GPIO_Port, UV_ON_OFF_MCU_Pin))
	  	  {
	  		  HAL_GPIO_WritePin(UV_B_CTRL_GPIO_Port, UV_B_CTRL_Pin, GPIO_PIN_SET);
	  	  }
	  	  else HAL_GPIO_WritePin(UV_B_CTRL_GPIO_Port, UV_B_CTRL_Pin, GPIO_PIN_RESET);
	  	  HAL_Delay(500);
	    }
	  }

	  /* Test HEPA ON/OFF Pin */
		if (!HAL_GPIO_ReadPin(HEPA_NO_GPIO_Port, HEPA_NO_Pin))
		{
		  HAL_GPIO_TogglePin(HEPA_ON_OFF_GPIO_Port, HEPA_ON_OFF_Pin);
		  HAL_GPIO_TogglePin(HEPA_B_CTRL_GPIO_Port, HEPA_B_CTRL_Pin);
		  HAL_Delay(500);
		}
	   HAL_Delay(100);

	  /* Get 24V and 3V3 ADC readings */
//		get_vin();
//		get_3V3();

	  /* Test UV LED */
//	    UVLEDTest();

	  /* Test HEPA LED */
//	    HEPALEDTest();

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_Delay(10);

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_Delay(10);

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */
	//Change TxHeader.Identifier, RxHeader.Identifier
	/* AUX1 ID */
	TxHeader.Identifier = 0x23; //0x555;
	/* AUX2 ID */
	//TxHeader.Identifier = 0xFAAA;
	TxHeader.IdType = FDCAN_EXTENDED_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_FD_CAN; // FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	/* AUX1 ID */
	RxHeader.Identifier = 0x23; //0x555;
	/* AUX2 ID */
	//RxHeader.Identifier = 0xFAAA; //0x555;
	RxHeader.IdType = FDCAN_EXTENDED_ID;
	RxHeader.RxFrameType = FDCAN_DATA_FRAME;
	RxHeader.DataLength = FDCAN_DLC_BYTES_8;
	RxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	RxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	RxHeader.FDFormat = FDCAN_FD_CAN; // FDCAN_CLASSIC_CAN;

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 110;
  hfdcan1.Init.NominalTimeSeg2 = 17;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 32;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  FDCAN_FilterTypeDef sFilterConfig;

  /* FD CAN Filter Config */
    /*sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
    //sFilterConfig.FilterConfig = FDCAN_FILTER_REJECT;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

     Use for AUX1 - Reject AUX2
    //sFilterConfig.FilterID1 = 0xFAAA;
    //sFilterConfig.FilterID2 = 0xFAAA;

     Use for AUX2 - Reject AUX1
    sFilterConfig.FilterID1 = 0x00;
	sFilterConfig.FilterID2 = 0x00;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
  	   Filter configuration Error
  	  Error_Handler();
    }
*/
/* Configure global filter to reject all non-matching frames */
    //HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

/* Start CAN */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	{
		// Couldn't start FDCAN
		Error_Handler();
	}

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00602173;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00602173;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4.294967295E9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 64-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 40-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 30;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // start pwm generation

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_DRIVE_Pin|HEPA_ON_OFF_Pin|HEPA_R_CTRL_Pin
                          |HEPA_W_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, UV_B_CTRL_Pin|HEPA_G_CTRL_Pin|nEEPROM_WP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, UV_G_CTRL_Pin|UV_R_CTRL_Pin|UV_W_CTRL_Pin|HEPA_B_CTRL_Pin, GPIO_PIN_RESET);

  /*UV_ON_OFF is set by default to keep UV bulb off */
  HAL_GPIO_WritePin(UV_ON_OFF_MCU_GPIO_Port, UV_ON_OFF_MCU_Pin, GPIO_PIN_SET);


  /*Configure GPIO pins : UV_NO_Pin DOOR_OPEN_MCU_Pin POS_SW__MCU_Pin */
  GPIO_InitStruct.Pin = UV_NO_Pin|DOOR_OPEN_MCU_Pin|POS_SW_MCU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_DRIVE_Pin */
  GPIO_InitStruct.Pin = LED_DRIVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_DRIVE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UV_ON_OFF_MCU_Pin HEPA_ON_OFF_Pin HEPA_R_CTRL_Pin HEPA_W_CTRL_Pin */
  GPIO_InitStruct.Pin = UV_ON_OFF_MCU_Pin|HEPA_ON_OFF_Pin|HEPA_R_CTRL_Pin|HEPA_W_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : UV_B_CTRL_Pin HEPA_G_CTRL_Pin */
  GPIO_InitStruct.Pin = UV_B_CTRL_Pin|HEPA_G_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : UV_G_CTRL_Pin UV_R_CTRL_Pin UV_W_CTRL_Pin HEPA_B_CTRL_Pin */
  GPIO_InitStruct.Pin = UV_G_CTRL_Pin|UV_R_CTRL_Pin|UV_W_CTRL_Pin|HEPA_B_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : HEPA_NO_Pin */
  GPIO_InitStruct.Pin = HEPA_NO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HEPA_NO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : nEEPROM_WP_Pin */
  GPIO_InitStruct.Pin = nEEPROM_WP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(nEEPROM_WP_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
