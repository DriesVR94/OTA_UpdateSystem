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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INITIALIZATION_FLAG_ADDRESS	0x0805FFFC

#define STARTING_FLAG_VALUE			0XFFFFFFFF
#define CORRECT_UPDATE_FLAG_VALUE 	0xBBBBBBBB
#define ERROR_UPDATE_FLAG_VALUE		0xCCCCCCCC
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAJOR 0
#define MINOR 1


#define SECTOR_2_START_ADDRESS 0x08008000  // Starting address of sector 2
#define SECTOR_2_SIZE 0x4000               // Size of sector 2 (16KB)

#define SECTOR_3_START_ADDRESS 0x0800C000  // Starting address of sector 3
#define SECTOR_3_SIZE 0x4000               // Size of sector 3 (16KB)

#define SECTOR_4_START_ADDRESS 0x08010000  // Starting address of sector 4
#define SECTOR_4_SIZE 0x10000               // Size of sector 4 (64KB)

#define SECTOR_5_START_ADDRESS 0x08020000  // Starting address of sector 5
#define SECTOR_5_SIZE 0x20000               // Size of sector 5 (128KB)

#define SECTOR_6_START_ADDRESS 0x08040000  // Starting address of sector 6
#define SECTOR_6_SIZE 0x20000            // Size of sector 6 (128KB)


#define SECTOR_7_START_ADDRESS 0x08060000  // Starting address of sector 7 FREERTOS SECTOR
#define SECTOR_7_SIZE 0x20000            // Size of sector 7 (128KB)



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
const uint8_t BL_Version[2] = {MAJOR, MINOR};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void JumpToFreeRTOS( void );
static void JumpToUpdate( void );
void SystemReset(void);
static uint32_t checkInitFlag(void);
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
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  //STARTING message
  printf("Starting Bootloader (%d.%d)\r\n", BL_Version[0], BL_Version[1]);
  HAL_GPIO_WritePin( GPIOB, GPIO_PIN_0,GPIO_PIN_SET);
  HAL_Delay(2000);


  //flash init

  if(!Flash_Init())
  {
	  printf("External memory not initialized. Check if it is connected properly.  \r\n");
  }



  printf("initflag: %lX \r\n", checkInitFlag());
/*
  if (checkInitFlag() != INITIALIZATION_FLAG_VALUE){
	  JumpToFreeRTOS();
  }
  else{
	  JumpToUpdate();
  }
*/

  if (checkInitFlag() == CORRECT_UPDATE_FLAG_VALUE )
  {
	  JumpToUpdate();
  }
  else if(checkInitFlag() == ERROR_UPDATE_FLAG_VALUE) // The update had an error
  {
	  JumpToFreeRTOS();
  }
  else
  {
	  // We perform a safety coppy of all the sectors in external SPI memory
	  uint8_t *ptr1 = (uint8_t *)SECTOR_2_START_ADDRESS; //
	  SPI_Flash_Write(0x90008000 ,ptr1,SECTOR_2_SIZE);
	  printf("Sector 2 copied\r\n");

	  ptr1 = (uint8_t *)SECTOR_3_START_ADDRESS; //
	  SPI_Flash_Write(0x9000C000,ptr1,SECTOR_3_SIZE);
	  printf("Sector 3 copied\r\n");

	  ptr1 = (uint8_t *)SECTOR_4_START_ADDRESS; //
	  SPI_Flash_Write(0x90010000,ptr1,SECTOR_4_SIZE);
	  printf("Sector 4 copied\r\n");


	  // Copy sector 5 in two rounds of 64KB each
	  ptr1 = (uint8_t *)SECTOR_5_START_ADDRESS;
	  SPI_Flash_Write(0x90020000, ptr1, 0x10000);  // First 64KB
	  printf("Sector 5 part 1 copied\r\n");
	  SPI_Flash_Write(0x90030000, ptr1 + 0x10000, 0x10000);  // Second 64KB
	  printf("Sector 5 part 2 copied\r\n");

	  // Copy sector 6 in two rounds of 64KB each
	  ptr1 = (uint8_t *)SECTOR_6_START_ADDRESS;
	  SPI_Flash_Write(0x90040000, ptr1, 0x10000);  // First 64KB
	  printf("Sector 6 part 1 copied\r\n");
	  SPI_Flash_Write(0x90050000, ptr1 + 0x10000, 0x10000);  // Second 64KB
	  printf("Sector 6 part 2 copied\r\n");

	  // Copy sector 7 in two rounds of 64KB each			RTOS INSIDE
	  ptr1 = (uint8_t *)SECTOR_7_START_ADDRESS;
	  SPI_Flash_Write(0x90060000, ptr1, 0x10000);  // First 64KB
	  printf("Sector 7 part 1 copied\r\n");
	  SPI_Flash_Write(0x90070000, ptr1 + 0x10000, 0x10000);  // Second 64KB
	  printf("Sector 7 part 2 copied\r\n");

	  JumpToFreeRTOS();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FLASH_CS_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(FLASH_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__

 /* With GCC, small printf (option LD Linker->Libraries->Small printf set to 'Yes') calls __io_putchar() */
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}



static void JumpToFreeRTOS(void)
{
	printf("Jumping to FreeRTOS \r\n");

	void (*app_reset_handler)(void) = (void*) ( *(volatile uint32_t *)(0x08060000 + 4));

	HAL_GPIO_WritePin( GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

	app_reset_handler(); //call the app reset handler
}

static void JumpToUpdate(void)
{
	printf("Jumping to Update \r\n");

	void (*app_reset_handler)(void) = (void*) ( *(volatile uint32_t *)(0x08040000 + 4));

	HAL_GPIO_WritePin( GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

	app_reset_handler(); //call the app reset handler
}

static uint32_t checkInitFlag(){
	return *(uint32_t*)INITIALIZATION_FLAG_ADDRESS;
}

// Function to trigger a system reset
void SystemReset(void)
{
    printf("System resetting...\r\n");
    HAL_NVIC_SystemReset();
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
