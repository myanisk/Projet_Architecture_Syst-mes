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
#include "YNV_LIS3DSH.h"
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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
LIS3DSH_init_t LIS3DSH_initHandle_REG4;
LIS3DSH_init_t LIS3DSH_initHandle_REG5;
LIS3DSH_Status_t LIS3DSH_status;
uint8_t try_reg4 = 0;
uint8_t try_reg5 = 0;
uint8_t *acclX = 0;
uint8_t *acclY = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  LIS3DSH_initHandle_REG4.xEnable = LIS3DSH_CTRL_REG4_X_ENABLE;
  LIS3DSH_initHandle_REG4.yEnable = LIS3DSH_CTRL_REG4_Y_ENABLE;
  LIS3DSH_initHandle_REG4.zEnable = LIS3DSH_CTRL_REG4_Z_ENABLE;
  LIS3DSH_initHandle_REG4.bduOn = LIS3DSH_CTRL_REG4_BDU_ON;
  LIS3DSH_initHandle_REG4.odr = LIS3DSH_CTRL_REG4_ODR_3_125_HZ;
  LIS3DSH_initHandle_REG4.bw = LIS3DSH_CTRL_REG5_BW_800_HZ;
  LIS3DSH_initHandle_REG4.fscale = LIS3DSH_CTRL_REG5_FSCALE_2_G;


  LIS3DSH_initHandle_REG5.xEnable = LIS3DSH_CTRL_REG4_X_ENABLE;
  LIS3DSH_initHandle_REG5.yEnable = LIS3DSH_CTRL_REG4_Y_ENABLE;
  LIS3DSH_initHandle_REG5.zEnable = LIS3DSH_CTRL_REG4_Z_ENABLE;
  LIS3DSH_initHandle_REG5.bduOn = LIS3DSH_CTRL_REG4_BDU_ON;
  LIS3DSH_initHandle_REG5.odr = LIS3DSH_CTRL_REG4_ODR_3_125_HZ;
  LIS3DSH_initHandle_REG5.bw = LIS3DSH_CTRL_REG5_BW_800_HZ;
  LIS3DSH_initHandle_REG5.fscale = LIS3DSH_CTRL_REG5_FSCALE_2_G;


  do
   	{
   		try_reg4 += 1;
   		LIS3DSH_status = LIS3DSH_Init_CTRL_REG4(&hspi1, LIS3DSH_CTRL_REG4, &LIS3DSH_initHandle_REG4);

   	}while(try_reg4 < 3 && LIS3DSH_status == LIS3DSH_ERROR);

   	// si la configuration n'est pas passe, blocage dans la boucle
   	while(LIS3DSH_status == LIS3DSH_ERROR)
   	{
   		HAL_GPIO_TogglePin(GPIOD, LD6_Pin);
   		HAL_Delay(500);
   	}

   do
   		{
   			try_reg5 += 1;
   			LIS3DSH_status = LIS3DSH_Init_CTRL_REG5(&hspi1, LIS3DSH_CTRL_REG5, &LIS3DSH_initHandle_REG5);

   		}while(try_reg5 < 3 && LIS3DSH_status == LIS3DSH_ERROR);

   		// si la configuration n'est pas passe, blocage dans la boucle
   		while(LIS3DSH_status == LIS3DSH_ERROR)
   		{
   			HAL_GPIO_TogglePin(GPIOD, LD6_Pin);
   			HAL_Delay(500);
   		}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  while (1)
//  	{
//	  LIS3DSH_status = LIS3DSH_Get_X(&hspi1, LIS3DSH_OUT_X, &acclX);
//  		if(LIS3DSH_status == LIS3DSH_ERROR)
//  		{
//  			HAL_GPIO_TogglePin(GPIOD, LD6_Pin);
//  		}
//  		if (acclX < 0)
//  		  	{
//  			HAL_GPIO_TogglePin(GPIOD, LD3_Pin);
//  		  	}
//  	  LIS3DSH_status = LIS3DSH_Get_X(&hspi1, LIS3DSH_OUT_X, &acclX);
//  		if(LIS3DSH_status == LIS3DSH_ERROR)
//  		{
//  		  	HAL_GPIO_TogglePin(GPIOD, LD6_Pin);
//  		}
//  		if (acclX > 0)
//  		{
//  		  	HAL_GPIO_TogglePin(GPIOD, LD4_Pin);
//
//  		}
//
//	  LIS3DSH_status = LIS3DSH_Get_Y(&hspi1, LIS3DSH_OUT_Y, &acclY);
//  		 if(LIS3DSH_status == LIS3DSH_ERROR)
//  		  		{
//  		  		  	HAL_GPIO_TogglePin(GPIOD, LD6_Pin);
//  		  		}
//  		  		if (acclY < 0)
//  		  		{
//  		  		  	HAL_GPIO_TogglePin(GPIOD, LD5_Pin);
//
//  		  		}
//
// 	  LIS3DSH_status = LIS3DSH_Get_Y(&hspi1, LIS3DSH_OUT_Y, &acclY);
//   		 if(LIS3DSH_status == LIS3DSH_ERROR)
//   		  		{
//   		  		  	HAL_GPIO_TogglePin(GPIOD, LD6_Pin);
//   		  		}
//   		  		if (acclY > 0)
//   		  		{
//   		  		  	HAL_GPIO_TogglePin(GPIOD, LD5_Pin);
//
//   		  		}
//   }

   		while (1)
   		{
   					HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_RESET);
   			        HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET);
   			        HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_RESET);
   			        HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_RESET);

   			        LIS3DSH_status = LIS3DSH_Get_X( &hspi1,LIS3DSH_OUT_X , &acclX);
   			        if(acclX > 50)
   			        {
   			            HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);
   			        }
   			        else if(acclX < -50)
   			        {
   			            HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
   			        }

   			        LIS3DSH_status = LIS3DSH_Get_Y( &hspi1,LIS3DSH_OUT_Y , &acclY);

   			        if(acclY > 50)
   			        {
   			            HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
   			        }
   			        else if(acclY <-50)
   			        {
   			            HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET);
   			        }
   			        HAL_Delay(10);
   		}



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
