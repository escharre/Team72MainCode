/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
float lightSensor(void);
int moistureSensor(void);
void LightOutput(int);
void WaterOutput(int);
void Error_Handler(void);

/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  //Local Variables
    int mois;
    int basil = 1;
    int rosemary = 0;
    int thyme = 0;
    float light;
    int dec;
    int days;
    int days2;
    RTC_TimeTypeDef currTime = {0};
    RTC_TimeTypeDef currDate = {0};
    int timeCon;
    int hourtosmecs, mintosmecs, secstosmecs;

    HAL_RTC_GetTime(&hrtc, &currTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &currDate, RTC_FORMAT_BIN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  //Get Time from RTC
	  	    HAL_RTC_GetTime(&hrtc, &currTime, RTC_FORMAT_BIN);
	        char timeBuff[20];
	        sprintf(timeBuff,"%d,,,%d...%d\n", currTime.Hours, currTime.Minutes, currTime.Seconds);
	        HAL_UART_Transmit(&huart2, timeBuff, sizeof(timeBuff), 100);

	         //time conversion
	         hourtosmecs = currTime.Hours * 3600000000;
	         mintosmecs = currTime.Minutes * 60000000;
	         secstosmecs = currTime.Seconds * 1000000;
	         timeCon = hourtosmecs + mintosmecs + secstosmecs;

	         //HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	         //Day counter for thyme
	         if (timeCon == 86400000000 && days <= 10){
	             	days = days + 1;
	             }
	             else {
	             	days = 0;
	             }
	         //Day counter for rosemary
	             if (timeCon == 86400000000 && days2 <= 14){
	                 	days2 = days2 + 1;
	                 }
	                 else {
	                 	days2 = 0;
	                 }

	         //Sensor Function Calls
	         light = lightSensor();	//Light Sensor Function Call
	         mois = moistureSensor(); //Moisture Sensor Function Call

	         //Sensor Decisions
	         //Sensor Decision Basil
	         if (basil == 1 && thyme == 0 && rosemary == 0){
	           	if (light <3000 && timeCon < 28800000000){
	            		dec = 1;
	            		LightOutput(dec); //Light Output Function Call- ON
	            	}
	            	if (light >=3000 && timeCon < 28800000000){
	            	    dec = 0;
	            	    LightOutput(dec); //Light Output Function Call- OFF
	            	}
	            	if (mois <350){
	            	    dec = 1;
	            	    WaterOutput(dec); //Water Output Function Call- ON
	            	}
	            	if (mois >=350){
	            	    dec = 0;
	            	    WaterOutput(dec); //Water Output Function Call- OFF
	            	}
	         }

	         //Sensor Decision Thyme
	         if (basil == 0 && thyme == 1 && rosemary == 0){
	            	if (light <9000 && timeCon < 28800000000){
	            	    dec = 1;
	            	    LightOutput(dec); //Light Output Function Call- ON
	            	}
	            	if (light >=9000 && timeCon < 28800000000){
	            	    dec = 0;
	            	    LightOutput(dec); //Light Output Function Call- OFF
	            	}

	            	if (mois <350 && days >= 10){
	                 dec = 1;
	                 WaterOutput(dec); //Water Output Function Call- ON
	             }
	             if (mois >=350 && days <= 10){
	                 dec = 0;
	                 WaterOutput(dec); //Water Output Function Call- OFF
	             }
	         }

	         //Sensor Decision Rosemary
	         if (basil == 0 && thyme == 0 && rosemary == 1){
	             if (light <9000 && timeCon < 28800000000){
	             	dec = 1;
	                 LightOutput(dec); //Light Output Function Call- ON
	             }
	             if (light >=9000 && timeCon < 28800000000){
	                	dec = 0;
	                 LightOutput(dec); //Light Output Function Call- OFF
	             }

	             if (mois <350 && days2 <= 183){
	                	dec = 1;
	                 WaterOutput(dec); //Water Output Function Call- ON
	             }
	             if (mois >=350 && days2 >= 14){
	                	dec = 0;
	                	WaterOutput(dec); //Water Output Function Call- OFF
	             }
	         }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
   sTime.Hours = 10;
   sTime.Minutes = 15;
   sTime.Seconds = 0;
   sTime.TimeFormat = RTC_HOURFORMAT12_PM;
   sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
   sTime.StoreOperation = RTC_STOREOPERATION_RESET;
   if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
   {
     Error_Handler();
   }
   sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
   sDate.Month = RTC_MONTH_MAY;
   sDate.Date = 25;
   sDate.Year = 19;

   if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
   {
     Error_Handler();
   }
  /* USER CODE END RTC_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */
float lightSensor(void){
	 /*Function Description: Light Sensor Code
	 *Author: Elizabeth Scharre
	 *Date Updated: 3/30/21
	 *Inputs: None
	 *Outputs: float ambient light value in lux*/

	 //local variables
	 const uint8_t sensorAddress = 0x10;
	 const uint16_t CONFIG_VALUE = 0x0000;
	 float out;
	 float gain = 1.8432; //Gain for 1/8 gain & 25ms IT (lower resolution but more range)
	 uint16_t Resolution;
	 uint16_t WhiteChannel;
	 float WC;
	 uint8_t lightResolutionData[2];
	 uint8_t WhiteChannelData[2];

	 //Process
	 HAL_I2C_Mem_Write(&hi2c1, (sensorAddress<<1), 0x00, 1, &CONFIG_VALUE, 2, 50);
	 HAL_I2C_Mem_Write(&hi2c1, (sensorAddress<<1), 0x03, 1, &CONFIG_VALUE, 2, 50);
	 HAL_I2C_Mem_Read(&hi2c1, (sensorAddress << 1) | 0x01, 0x04, 1, &lightResolutionData, 2, 50);
	 HAL_I2C_Mem_Read(&hi2c1, (sensorAddress << 1) | 0x01, 0x05, 1, &WhiteChannelData, 2, 50);
	 Resolution = (lightResolutionData[1]<<8) | lightResolutionData[0];
	 WhiteChannel = (WhiteChannelData[1]<<8) | WhiteChannelData[0];

	 //Conversion
	 WC = WhiteChannel; //Int to float conversion

	 out = gain * WC; //Ambient light calculation

	 return out;
}

int moistureSensor(void){
	 /*Function Description: Moisture Sensor Code
	 *Author: Elizabeth Scharre
	 *Date Updated: 3/30/21
	 *Inputs: None
	 *Outputs: float moisture value and int cap*/

	 //local variables
	 HAL_StatusTypeDef ret;
	 uint8_t buf[12];
	 uint16_t capac = 0;

	 //Process
	 buf[0] = 0x00;
	 buf[1] = 0x04;
	 ret = HAL_I2C_Master_Transmit(&hi2c1,0x36<<1, buf, 2, HAL_MAX_DELAY);
	 if ( ret != HAL_OK ) {
		 strcpy((char*)buf, "Error Tx\r\n");
	 	 } else {
	 		 ret = HAL_I2C_Master_Receive(&hi2c1, 0x36<<1, buf, 4, HAL_MAX_DELAY); //previously 2

	 		 if ( ret != HAL_OK ) {
	 			 strcpy((char*)buf, "Error Rx\r\n");
	 		 	 } else {

	 		 		 //Combine the bytes
	 		 		 capac = ((uint16_t)buf[0] <<8) | buf[1];


	 		 	 }
	 	 }


	 return capac;
}

void LightOutput(int decision){
	 /*Function Description: Light Output Code
	 *Author: Elizabeth Scharre & Sophia Korner
	 *Date Updated: 3/30/21
	 *Inputs: int decision from main due to sensor code
	 *Outputs: none*/

	 //local variables


	 //Toggle Light On/Off

		if (decision == 1){
			HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_12); //Pin assigned to GPIO Output by IDE
		}
		if (decision == 0){
			HAL_Delay (1800000);   /*Insert delay 30 minutes*/
		}

		 return;
}

void WaterOutput(int decision){
	 /*Function Description: Light Output Code
	 *Author: Sophia Korner
	 *Date Updated: 3/30/21
	 *Inputs: int decision from main due to sensor code
	 *Outputs: none*/

	 //local variables

	 //Toggle water on/off

		if (decision == 1){
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);      //Turning the motor on (all 4 signal pins
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); 	   //are turned on here)
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);

			HAL_Delay(2000);
		}
		if (decision == 0){
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);   //Turning all 4 motor pins off
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);   //The motor will only be hooked up to two of them
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);

		}

	 return;
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
