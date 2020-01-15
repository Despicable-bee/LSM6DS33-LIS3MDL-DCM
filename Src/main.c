/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "LSM6DS33.h"
#include "LIS3MDL.h"
#include "AHRS.h"
#include "DCM.h"
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

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
bool loopInitiate;
bool loopComplete;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  LSM6DS33 device;
  LIS3MDL device2;
  DCMStruct dcmData;
  LIS3MDLSRegConfig sreg;
  uint8_t buffer[350];
  uint8_t spaceBuff[200];
  spaceBuff[0] = '\r';
  for(int i = 1; i < 199; i++) {
	  spaceBuff[i] = ' ';
  }
  spaceBuff[199] = '\r';

  // Initialise the Gyro and Accelerometer device
  device.device = LSM6DS33_DEVICE_TYPE_DS33;
  device.address = LSM6DS33_SA0_HIGH_ADDR;
  device.hi2c = &hi2c1;
  LSM6DS33_init(&device, device.device, LSM6DS33_SA0_HIGH);

  // Initialise the Magnetometer device
  device2.address = LIS3MDL_SA0_HIGH;
  device2.hi2c = &hi2c1;
  LIS3MDL_init_reg(&device2);

  // Initialise the buffer
  for(int i = 0; i < 100; i++) {
	  buffer[i] = 0;
  }

  // Initialise the DCM data
  dcm_init(&dcmData, &device);

  // Start the global loops
  loopInitiate = false;
  loopComplete = true;

  dcmData.timer = HAL_GetTick();
  // Enable the clock interrupt
  HAL_TIM_Base_Start_IT(&htim2);

  HAL_Delay(20); // delay the first 20 ms

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// Start doing stuff
	if(loopInitiate) {
		loopComplete = false;
		HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
		dcmData.counter++;
		dcmData.timer_old = dcmData.timer;
		dcmData.timer = HAL_GetTick();
		if(dcmData.timer > dcmData.timer_old) {

			dcmData.matrixData.G_Dt = ((float)(dcmData.timer - dcmData.timer_old) / 1000);
			if(dcmData.matrixData.G_Dt > 0.2)
				dcmData.matrixData.G_Dt = 0; // Ignore integrations over 200ms
		} else {
			dcmData.matrixData.G_Dt = 0; // Ignore overlaps
		}

		dcm_loop(&dcmData, &device, &device2);

		// Clear the screen
		sprintf(buffer,"\033[27\033[HEuler:\n\r"
				"\tRoll:\t\t%.5f\n\r"
				"\tPitch:\t\t%.5f\n\r"
				"\tYaw:\t\t%.5f\n\r\n\r"
				"Gyroscope:\n\r"
				"\tX:\t\t%d\n\r"
				"\tY:\t\t%d\n\r"
				"\tZ:\t\t%d\n\r\n\r"
				"Accelerometer:\n\r"
				"\tX:\t\t%d\n\r"
				"\tY:\t\t%d\n\r"
				"\tZ:\t\t%d\n\r\n\r"
				"Magnetometer:\n\r"
				"\tX:\t\t%d\n\r"
				"\tY:\t\t%d\n\r"
				"\tZ:\t\t%d\n\r",
				ToDeg(dcmData.angleData.roll),
				ToDeg(dcmData.angleData.pitch),
				ToDeg(dcmData.angleData.yaw),
				dcmData.sensorData.gyro_x,
				dcmData.sensorData.gyro_y,
				dcmData.sensorData.gyro_z,
				dcmData.sensorData.accl_x,
				dcmData.sensorData.accl_y,
				dcmData.sensorData.accl_z,
				dcmData.sensorData.magn_x,
				dcmData.sensorData.magn_y,
				dcmData.sensorData.magn_z);
		HAL_UART_Transmit(&huart2, buffer, strlen(buffer), HAL_MAX_DELAY);
		loopComplete = true;
	}
	// Done


	// Compute the angles.
	//AHRSData data = MadgwickAHRSupdate(gyroX_f, gyroY_f, gyroZ_f, acclX_f, acclY_f, acclZ_f, magnX_f, magnY_f, magnZ_f);

	  // Print
//	  sprintf(buffer,
//			  "\033[27\033[HQ:\n\rw: %.5f\n\rx: %.5f\n\ry: %5.f\n\rz: %.5f\n\r\n\r"
//			  "Euler:\n\rYaw: %.5f\n\rPitch: %.5f\n\rRoll: %.5f\n\r\n\r"
//			  "Accl:\n\rx: %.5f\n\ry: %.5f\n\rz: %.5f\n\r\n\r"
//			  "Gyro:\n\rx: %.5f\n\ry: %.5f\n\rz: %.5f\n\r\n\r"
//			  "Magn:\n\rx: %.5f\n\ry: %.5f\n\rz: %.5f\n\r\n\r"
//			  "Temp 1:\n\r(Celcius): %.5f\n\r",
//			  data.q.w, data.q.x, data.q.y, data.q.z,
//			  data.angles.yaw, data.angles.pitch, data.angles.roll,
//			  acclX_f, acclY_f, acclZ_f,
//			  gyroX_f, gyroY_f, gyroZ_f,
//			  magnX_f, magnY_f, magnZ_f,
//			  temp_f);

	 // HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), HAL_MAX_DELAY);

//	  device.acclX = 0;
//	  device.acclY = 0;
//	  device.acclZ = 0;
//	  device.gyroX = 0;
//	  device.gyroY = 0;
//	  device.gyroZ = 0;
//	  device2.magX = 0;
//	  device2.magY = 0;
//	  device2.magZ = 0;

//	  HAL_UART_Transmit(&huart2, spaceBuff, strlen(spaceBuff), HAL_MAX_DELAY);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x2000090E;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RED_LED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if(loopComplete) {
		loopInitiate = true;
	} else {
		loopInitiate = false;
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
