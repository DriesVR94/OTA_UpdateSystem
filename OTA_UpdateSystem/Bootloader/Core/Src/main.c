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
#include "F446ZE_FLASH_Sector_Addresses.h"
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CUSTOM_SECTION_0			0x08008000
#define CUSTOM_SECTION_1			0x0800C000
#define CUSTOM_SECTION_2			0x08010000
#define CUSTOM_SECTION_3			0x08020000
#define CUSTOM_SECTION_4			0x08040000

#define INITIALIZATION_FLAG_ADDRESS	0x08007FFC
#define INITIALIZATION_FLAG_VALUE 	0xAAAAAAAA
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
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
const uint8_t BL_Version[2] = {MAJOR, MINOR};

// CAN variables
CAN_RxHeaderTypeDef   	RxHeader;
uint8_t               	TxData[8];
uint8_t               	RxData[8];
uint32_t              	TxMailbox;

// Flash Operation variables
uint32_t FirstSector = 0, NbOfSectors = 0;
uint32_t Start_Address = 0, Write_Address = 0, SECTORError = 0;
uint8_t TxBuffer;

/*Variable used for Flash Erase procedure*/
//static FLASH_EraseInitTypeDef EraseInitStruct;

bool flashUpdateDone = false;
bool flashErased = false;  // Add this line to declare the flashErased variable
bool updateComplete = false;
bool taskCreated = false;
bool flag1;

// Board status variables
enum board_status 	  	{NO_UPDATE_AVAILABLE, SENDING_UPDATE, RECEIVING_UPDATE, UPDATE_FINISHED};
enum board_status 	  	board_status = NO_UPDATE_AVAILABLE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
static void JumpToFreeRTOS( void );
void CANRxTask(void *argument);
void SystemReset(void);
/* Function prototypes for Flash Operations */
uint32_t GetSector(uint32_t Start_Address);
uint32_t GetSectorSize(uint32_t Sector);
uint32_t Read_Message_and_Write_in_FLASH(uint32_t StartSectorAddress, uint32_t EndSectorAddress, uint8_t *data, uint8_t length);
static uint32_t GetFilterMatchingIndex(CAN_RxHeaderTypeDef *RxHeader);
static uint32_t SetFlashSectorForWritingUpdate(uint32_t FilterMatchIndex);
static uint32_t checkInitFlag(void);
static void writeInitFlag(uint32_t initFlag);
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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  /* Start the CAN and enable interrupts*/
  HAL_CAN_Start(&hcan1);
  //STARTING message
  printf("Starting Bootloader (%d.%d)\r\n", BL_Version[0], BL_Version[1]);
  HAL_GPIO_WritePin( GPIOB, GPIO_PIN_0,GPIO_PIN_SET);
  HAL_Delay(2000);


  //flash init

  if(!Flash_Init())
  {
	  printf("It does not init \r\n");
  }


  // We perform a safety coppy of all the sectors in external SPI memory
  uint8_t *ptr1 = (uint8_t *)SECTOR_2_START_ADDRESS; //
  SPI_Flash_Write(0x90008000 ,ptr1,SECTOR_2_SIZE);
  printf("Sector 2 coppied\r\n");

  ptr1 = (uint8_t *)SECTOR_3_START_ADDRESS; //
  SPI_Flash_Write(0x9000C000,ptr1,SECTOR_3_SIZE);
  printf("Sector 3 coppied\r\n");

  ptr1 = (uint8_t *)SECTOR_4_START_ADDRESS; //
  SPI_Flash_Write(0x90010000,ptr1,SECTOR_4_SIZE);
  printf("Sector 4 coppied\r\n");


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


  /*  */
  uint32_t initFlag = checkInitFlag();
  if (initFlag != INITIALIZATION_FLAG_VALUE){
	  writeInitFlag(INITIALIZATION_FLAG_VALUE);
	  JumpToFreeRTOS();
  }
  else{
	  printf("Init completed. \r\n");
  }




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(100);
	  if (board_status == NO_UPDATE_AVAILABLE){
		  HAL_GPIO_WritePin(GPIOB, LD2_Pin | LD1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
		  HAL_Delay(1000);
	  }
	  else if (board_status == RECEIVING_UPDATE){
		  HAL_GPIO_WritePin(GPIOB, LD3_Pin | LD1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
		  HAL_Delay(100);
	  }
	  else if (board_status == UPDATE_FINISHED){
		  HAL_GPIO_WritePin(GPIOB, LD2_Pin | LD3_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
		  HAL_Delay(1000);
	  }
	  HAL_Delay(1);
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
  hcan1.Init.AutoRetransmission = DISABLE;
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
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
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

static uint32_t checkInitFlag(){
	return *(uint32_t*)INITIALIZATION_FLAG_ADDRESS;
}

static void writeInitFlag(uint32_t initFlag){
	HAL_FLASH_Unlock();

    if (HAL_FLASH_Program(TYPEPROGRAM_WORD, INITIALIZATION_FLAG_ADDRESS, initFlag) != HAL_OK) {
        // Error handling
    	printf("Flag program error. \r\n");
    }
    HAL_FLASH_Lock();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	printf("Message callback in BL. \r\n");

}

static uint32_t GetFilterMatchingIndex(CAN_RxHeaderTypeDef *RxHeader)
{
	if (RxHeader->FilterMatchIndex == 0){
		printf("This is for App0. \r\n");
	}
	else if (RxHeader->FilterMatchIndex == 2){
		printf("This is for App1. \r\n");
	}
	else if (RxHeader->FilterMatchIndex == 4){
			printf("This is for App2. \r\n");
		}
	else if (RxHeader->FilterMatchIndex == 6){
			printf("This is for App3. \r\n");
		}
	else if (RxHeader->FilterMatchIndex == 8){
			printf("This is for App4. \r\n");
		}
	else {
		printf("Rejected. \r\n");
	}

	return RxHeader->FilterMatchIndex;
}

static uint32_t SetFlashSectorForWritingUpdate(uint32_t FilterMatchIndex){
	if (FilterMatchIndex == 0){
		Start_Address = CUSTOM_SECTION_0;
		Write_Address = Start_Address;
	}
	else if (FilterMatchIndex == 2){
		Start_Address = CUSTOM_SECTION_1;
		Write_Address = Start_Address;
	}
	else if (FilterMatchIndex == 4){
		Start_Address = CUSTOM_SECTION_2;
		Write_Address = Start_Address;
	}
	else if (FilterMatchIndex == 6){
		Start_Address = CUSTOM_SECTION_3;
		Write_Address = Start_Address;
	}
	else if (FilterMatchIndex == 8){
		Start_Address = CUSTOM_SECTION_4;
		Write_Address = Start_Address;
	}
	else {
		printf("No Filter Match. \r\n");
	}
	return Start_Address;
}


static void JumpToFreeRTOS (void)
{
	printf("Gonna Jump to Application \n");

	void (*app_reset_handler)(void) = (void*) ( *(volatile uint32_t *)(0x08060000 +4));

	HAL_GPIO_WritePin( GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

	app_reset_handler(); //call the app reset handler
}

/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address < ADDR_FLASH_SECTOR_1_START) && (Address >= ADDR_FLASH_SECTOR_0_START))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < ADDR_FLASH_SECTOR_2_START) && (Address >= ADDR_FLASH_SECTOR_1_START))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < ADDR_FLASH_SECTOR_3_START) && (Address >= ADDR_FLASH_SECTOR_2_START))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < ADDR_FLASH_SECTOR_4_START) && (Address >= ADDR_FLASH_SECTOR_3_START))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < ADDR_FLASH_SECTOR_5_START) && (Address >= ADDR_FLASH_SECTOR_4_START))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < ADDR_FLASH_SECTOR_6_START) && (Address >= ADDR_FLASH_SECTOR_5_START))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < ADDR_FLASH_SECTOR_7_START) && (Address >= ADDR_FLASH_SECTOR_6_START))
  {
    sector = FLASH_SECTOR_6;
  }
  else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_7) */
  {
    sector = FLASH_SECTOR_7;
  }
  return sector;
}

/**
  * @brief  Gets sector Size
  * @param  None
  * @retval The size of a given sector
  */
uint32_t GetSectorSize(uint32_t Sector)
{
  uint32_t sectorsize = 0x00;
  if((Sector == FLASH_SECTOR_0) || (Sector == FLASH_SECTOR_1) || (Sector == FLASH_SECTOR_2) || (Sector == FLASH_SECTOR_3))
  {
    sectorsize = 16 * 1024;
  }
  else if(Sector == FLASH_SECTOR_4)
  {
    sectorsize = 64 * 1024;
  }
  else
  {
    sectorsize = 128 * 1024;
  }
  return sectorsize;
}

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//
//	  /* Preparing some variables */
//	  uint32_t sector = GetSector(Start_Address);
//	  uint32_t sectorsize = GetSectorSize(sector);
//
//    // Erase the sector only once before writing the data
//    if (!flashErased)
//    {
//        /* Get the 1st sector to erase */
//        FirstSector = sector;
//        /* Get the number of sectors to erase */
//        NbOfSectors = 1; // Assuming only one sector needs to be erased
//        /* Fill EraseInit structure */
//        EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
//        EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
//        EraseInitStruct.Sector        = FirstSector;
//        EraseInitStruct.NbSectors     = NbOfSectors;
//
//        if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
//        {
//            printf("Error erasing flash sector\r\n");
//            HAL_FLASH_Lock();
//        }
//        else
//        {
//            printf("Flash sector erased successfully\r\n");
//            flashErased = true;
//        }
//    }
//    // Get the message
//    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
//    {
//    	printf("got msg \r\n");
//    	Read_Message_and_Write_in_FLASH(Write_Address, Start_Address + sectorsize, RxData, RxHeader.DLC);
//
//    }
//
//    // Set the update complete flag when the last chunk is received and written
//    if (Write_Address == Start_Address + sectorsize)
//    {
//        updateComplete = true;
//
//    }
//    // Check if update is complete and reset the system
//    if (updateComplete)
//    {
//    	printf("Update completed \r\n");
//        SystemReset();
//    }
//}

uint32_t Read_Message_and_Write_in_FLASH(uint32_t Address, uint32_t EndSectorAddress, uint8_t *data, uint8_t length)
{
    printf("Entering Read_Message_and_Write_in_FLASH\r\n");
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

//Task that handles RX
// Updated CANRxTask
void CANRxTask(void *argument)
{

    while (1)
    {



    }
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
