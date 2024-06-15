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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAJOR 0
#define MINOR 0

/* Before transmitting, make sure that there is an 'update' available in the sector defined below.
 * This update will then be sent from one F446 board to another through the CAN bus.
 * One board simulates the telemetry module (= responsible from communication with the ground station) on a CubeSat,
 * the other board simulates the on-board computer (OBC) of the CubeSat. */

//#define FLASH_USER_START_ADDR_RX   ADDR_FLASH_SECTOR_5_START		/* Start @ of user Flash area */
//#define FLASH_USER_END_ADDR_RX     ADDR_FLASH_SECTOR_5_END 			/* End @ of user Flash area */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart3;

/* Definitions for app0 */
osThreadId_t app0Handle;
const osThreadAttr_t app0_attributes = {
  .name = "app0",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for app1 */
osThreadId_t app1Handle;
const osThreadAttr_t app1_attributes = {
  .name = "app1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for app2 */
osThreadId_t app2Handle;
const osThreadAttr_t app2_attributes = {
  .name = "app2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for app3 */
osThreadId_t app3Handle;
const osThreadAttr_t app3_attributes = {
  .name = "app3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */


const uint8_t			APP_Version[2]={MAJOR, MINOR};

// CAN variables
CAN_RxHeaderTypeDef   	RxHeader;
uint8_t               	TxData[8];
uint8_t               	RxData[8];
uint32_t              	TxMailbox;
QueueHandle_t canRxQueue;

// Board status variables
enum board_status 	  	{NO_UPDATE_AVAILABLE, SENDING_UPDATE, RECEIVING_UPDATE, UPDATE_FINISHED};
enum board_status 	  	board_status = NO_UPDATE_AVAILABLE;

// Flash Operation variables
uint32_t FirstSector = 0, NbOfSectors = 0;
uint32_t Address = 0, SECTORError = 0;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;
uint8_t TxBuffer;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN1_Init(void);
void ShowBoardStatus(void *argument);
void StartApp1(void *argument);
void StartApp2(void *argument);
void StartApp3(void *argument);
void StartApp1_1(void *argument);

/* USER CODE BEGIN PFP */
void SystemReset(void);
void StopAllThreads(void);
void RebootToApplication(void);


/* Function prototypes for Flash Operations */


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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  /* Start the CAN and enable interrupts*/
 HAL_CAN_Start(&hcan1);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */


  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of app0 */
  app0Handle = osThreadNew(ShowBoardStatus, NULL, &app0_attributes);

  /* creation of app1 */
  app1Handle = osThreadNew(StartApp1, NULL, &app1_attributes);

  /* creation of app2 */
  app2Handle = osThreadNew(StartApp2, NULL, &app2_attributes);

  /* creation of app3 */
  app3Handle = osThreadNew(StartApp3, NULL, &app3_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */


  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  printf("in the while \r\n");

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
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  sFilterConfig0.FilterBank = 0;
  sFilterConfig0.FilterMode = CAN_FILTERMODE_IDMASK;
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

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
    /* Notification Error */
      Error_Handler();
    }

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
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	printf("msg callback \r\n");
    // Get the message
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
    	printf("Got msg \r\n");
    }

}


/* Enabling a print function for Putty. */
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


uint32_t Read_Message_and_Write_in_FLASH(uint32_t Address, uint32_t EndSectorAddress, uint8_t *data, uint8_t length)
{
    //printf("Entering Read_Message_and_Write_in_FLASH\r\n");
    HAL_FLASH_Unlock();
    //printf("Flash unlocked\r\n");
    board_status = RECEIVING_UPDATE;

    for (uint8_t i = 0; i < length; i++)
    {
        if (Address > EndSectorAddress)
        {
            // Handle error: Flash address out of range
            printf("Flash address out of range\r\n");
            break;
        }

        HAL_StatusTypeDef flash_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, Address, data[i]);
        if (flash_status == HAL_OK)
        {
            //printf("Flash write successful: %02X at address %lu\r\n", data[i], Address);
            Address++;
        }
        else
        {
            printf("Flash write failed: status=%d\r\n", flash_status);
            HAL_FLASH_Lock();
            Error_Handler();
        }
    }

    printf("Address %lu\r\n", Address);
    HAL_FLASH_Lock();
    //printf("Flash locked\r\n");

    //printf("Exiting Read_Message_and_Write_in_FLASH\r\n");
    return Address;
}





// Function to trigger a system reset
void SystemReset(void)
{
    printf("System resetting...\r\n");
    HAL_NVIC_SystemReset();
}

void StopAllThreads(void)
{
    vTaskSuspend(app1Handle);
    vTaskSuspend(app2Handle);
    vTaskSuspend(app3Handle);
    // Add suspensions for any other threads if necessary
}


void StartApp1_1(void *argument)
{
  for(;;)
  {
      //printf("Hello from app 1.1\r\n");

	  printf("About to try to relaunch with a new task \r\n");
      //application1();
      osDelay(1000);
  }
}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_ShowBoardStatus */
/**
  * @brief  Function implementing the app0 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ShowBoardStatus */
void ShowBoardStatus(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(100);
	  if (board_status == NO_UPDATE_AVAILABLE){
		  HAL_GPIO_WritePin(GPIOB, LD2_Pin | LD1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
		  osDelay(1000);
	  }
	  else if (board_status == RECEIVING_UPDATE){
		  HAL_GPIO_WritePin(GPIOB, LD3_Pin | LD1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
		  osDelay(100);
	  }
	  else if (board_status == UPDATE_FINISHED){
		  HAL_GPIO_WritePin(GPIOB, LD2_Pin | LD3_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
		  osDelay(1000);
	  }
	  osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartApp1 */
/**
  * @brief  Function implementing the app1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartApp1 */
void StartApp1(void *argument)
{
  for(;;)
  {

	  application1();
	  osDelay(1000);

  }
  /* USER CODE END StartApp1 */
}


/* USER CODE BEGIN Header_StartApp2 */
/**
* @brief Function implementing the app2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartApp2 */
void StartApp2(void *argument)
{
  /* USER CODE BEGIN StartApp2 */
  /* Infinite loop */
  for(;;)
  {
	  application2();
	  osDelay(1000);
  }
  /* USER CODE END StartApp2 */
}

/* USER CODE BEGIN Header_StartApp3 */
/**
* @brief Function implementing the app3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartApp3 */
void StartApp3(void *argument)
{
  /* USER CODE BEGIN StartApp3 */
  /* Infinite loop */
  for(;;)
  {
	  application3();
	  osDelay(1000);
  }
  /* USER CODE END StartApp3 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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