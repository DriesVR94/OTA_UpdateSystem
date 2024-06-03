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
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// CAN Related
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;
// LED Status...
uint8_t				  LDSTAT;
uint8_t			      CNT;
uint8_t			      TX;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN1_Init(void);
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
	TX = 0; // Stel hier in of het zender (1) of ontvanger (0) is.

	LDSTAT = 0;
	CNT = 0;
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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (TX) {
					CNT = CNT + 1;
					TxData[0] = CNT;
					TxData[1] = LDSTAT;

					if (LDSTAT == 0) {
						HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOB, LD3_Pin | LD2_Pin, GPIO_PIN_RESET);
						LDSTAT = 1;
					} else if (LDSTAT == 1) {
						HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOB, LD3_Pin | LD1_Pin, GPIO_PIN_RESET);
						LDSTAT = 2;
					} else {
						HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD2_Pin, GPIO_PIN_RESET);
						LDSTAT = 0;
					}


					if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox)
							!= HAL_OK) {
						/* Transmission request Error */
						Error_Handler();
					}

					HAL_Delay(500);
				}
//					else {
//					HAL_Delay(100);
//
//					if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
//						/* Get RX message */
//						if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader,
//								RxData) != HAL_OK) {
//							/* Reception Error */
//							Error_Handler();
//						}
//						/* Display LEDx */
//						if ((RxHeader.IDE == CAN_ID_STD)
//								&& (RxHeader.DLC == 2)) {
//							LDSTAT = RxData[1];
//							if (LDSTAT == 0) {
//								HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOB, LD3_Pin | LD2_Pin,
//										GPIO_PIN_RESET);
//							} else if (LDSTAT == 1) {
//								HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOB, LD3_Pin | LD1_Pin,
//										GPIO_PIN_RESET);
//							} else {
//								HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD2_Pin,
//										GPIO_PIN_RESET);
//							}
//						}
//					}
//				}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
  CAN_FilterTypeDef  sFilterConfig0;
  CAN_FilterTypeDef  sFilterConfig1;
  CAN_FilterTypeDef  sFilterConfig2;
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  // Configuring the filter for Application 0.
  sFilterConfig0.FilterBank = 0;
  sFilterConfig0.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig0.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig0.FilterIdHigh = 0x0000;
  sFilterConfig0.FilterIdLow = 0x0000;
  sFilterConfig0.FilterMaskIdHigh = 0x0000;
  sFilterConfig0.FilterMaskIdLow = 0x0000;
  sFilterConfig0.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig0.FilterActivation = ENABLE;
  sFilterConfig0.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig0) != HAL_OK) {
	/* Filter configuration Error */
	Error_Handler();
  }

  // Configuring the filter for Application 1.
  sFilterConfig1.FilterBank = 1;
  sFilterConfig1.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig1.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig1.FilterIdHigh = 0x0020;
  sFilterConfig1.FilterIdLow = 0x0000;
  sFilterConfig1.FilterMaskIdHigh = 0x0000;
  sFilterConfig1.FilterMaskIdLow = 0x0000;
  sFilterConfig1.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig1.FilterActivation = ENABLE;
  sFilterConfig1.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig1) != HAL_OK) {
	/* Filter configuration Error */
	Error_Handler();
  }

  // Configuring the filter for Application 2.
  sFilterConfig2.FilterBank = 2;
  sFilterConfig2.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig2.FilterIdHigh = 0x0040;
  sFilterConfig2.FilterIdLow = 0x0000;
  sFilterConfig2.FilterMaskIdHigh = 0x0000;
  sFilterConfig2.FilterMaskIdLow = 0x0000;
  sFilterConfig2.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig2.FilterActivation = ENABLE;
  sFilterConfig2.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig2) != HAL_OK) {
	/* Filter configuration Error */
	Error_Handler();
  }
  /* CAN RX Notification activeren */
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
    /* Notification Error */
      Error_Handler();
    }

  /*##-5- Configure Transmission process ##*/
  TxHeader.StdId = 0b00000000010;
  TxHeader.ExtId = 0x00;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 2; // Datalengte is gelijk aan 2
  TxHeader.TransmitGlobalTime = DISABLE;


  /* USER CODE END CAN1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Rx Fifo 0 message pending callback
  * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }
  /* Display LEDx */
	if ((RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 2)) {
		LDSTAT = RxData[1];
		if (LDSTAT == 0) {
			HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, LD3_Pin | LD2_Pin, GPIO_PIN_RESET);
	    } else if (LDSTAT == 1) {
			HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, LD3_Pin | LD1_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD2_Pin, GPIO_PIN_RESET);
		}
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
