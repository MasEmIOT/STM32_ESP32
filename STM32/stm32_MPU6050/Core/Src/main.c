/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "CLCD_I2C.h"
#include "stdio.h"
#include "string.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MPU6050_ADDR 0xD0
#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW, Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
float Ax, Ay, Az, Gx, Gy, Gz;
CLCD_I2C_Name LCD1;

void MPU6050_Init(void)
{
	uint8_t check, data;
	//Mem: memory
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 100);
	if(check == 104)
	{
		data = 0;
		HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,PWR_MGMT_1, 1, &data, 1, 100);
	  
		data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,SMPLRT_DIV, 1, &data, 1, 100);
	  
		//acceleration: g
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,ACCEL_CONFIG, 1, &data, 1, 100);
		
		//gyro:  �/s 
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,GYRO_CONFIG, 1, &data, 1, 100);
	}
}

void MPU6050_Read_Accel(void)
{
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 6, 100);
	
	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	
	Ax =(float)Accel_X_RAW/16384.0;
	Ay =(float)Accel_Y_RAW/16384.0;
  Az =(float)Accel_Z_RAW/16384.0;
}

void MPU6050_Read_Gyro(void)
{
	uint8_t Rec_Data[6];
	
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_Data, 6, 1000);
	
	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	
	Gx =(float)Gyro_X_RAW/131.0;
	Gy =(float)Gyro_Y_RAW/131.0;
  Gz =(float)Gyro_Z_RAW/131.0;
}

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
  /* USER CODE BEGIN 2 */
	MPU6050_Init();
  CLCD_I2C_Init(&LCD1, &hi2c1, 0x4e, 16,2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		char buf[20];
		MPU6050_Read_Accel();
		MPU6050_Read_Gyro();
		
		CLCD_I2C_SetCursor(&LCD1,0,0);
		sprintf(buf,"%.1f", Ax);
		CLCD_I2C_WriteString(&LCD1,buf);
		
		CLCD_I2C_SetCursor(&LCD1,5,0);
		sprintf(buf,"%.1f", Ay);
		CLCD_I2C_WriteString(&LCD1,buf);
		
		CLCD_I2C_SetCursor(&LCD1,10,0);
		sprintf(buf,"%.1f", Az);
		CLCD_I2C_WriteString(&LCD1,buf);
		
		CLCD_I2C_SetCursor(&LCD1,0,1);
		sprintf(buf,"%.1f", Gx);
		CLCD_I2C_WriteString(&LCD1,buf);
		
		CLCD_I2C_SetCursor(&LCD1,5,1);
		sprintf(buf,"%.1f", Gy);
		CLCD_I2C_WriteString(&LCD1,buf);
		
		CLCD_I2C_SetCursor(&LCD1,10,1);
		sprintf(buf,"%.1f", Gz);
		CLCD_I2C_WriteString(&LCD1,buf);
		
		HAL_Delay(500);
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
