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
#include "applications.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Before transmitting, we will program the update in Sector 7 ( = its last sector) of one of the F446 boards.
 * Programming a .bin file in the Flash memory is done through the STM32CubeProgrammer application.
 * This update will then be send to the second F446 board through the CAN bus. Here, we use wires to simulate this process.
 * For a real space application, a wireless CAN transceiver has to be used.
 * The receiving board will also store the update in sector 7. The sector choice is just an arbitrary decision.
 * Any other sector can be chosen too, as long as it doesn't conflict with firmware code. */

#define FLASH_USER_START_ADDR_TX   ADDR_FLASH_SECTOR_3_START		/* Start @ of user Flash area */
#define FLASH_USER_START_ADDR_RX   ADDR_FLASH_SECTOR_3_START
#define FLASH_USER_END_ADDR_TX     ADDR_FLASH_SECTOR_3_END 			/* End @ of user Flash area */
#define FLASH_USER_END_ADDR_RX	   ADDR_FLASH_SECTOR_3_END

#define DATA_32                    ((uint32_t)0x12345678)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

// Define whether the board will be sending or receiving.
uint8_t			      	TX;

// CAN variables
CAN_TxHeaderTypeDef   	TxHeader_GroundStation;
CAN_TxHeaderTypeDef   	TxHeader_CubeSat;
CAN_RxHeaderTypeDef   	RxHeader_GroundStation;
CAN_RxHeaderTypeDef   	RxHeader_CubeSat;
uint8_t               	TxData[8];
uint8_t               	RxData[8];
uint32_t              	TxMailbox;
enum board_status 	  	{NO_UPDATE_AVAILABLE, SENDING_UPDATE, RECEIVING_UPDATE, UPDATE_FINISHED};
enum board_status 	  	board_status = NO_UPDATE_AVAILABLE;
enum ACK_status		  	{ACK, NACK};
enum ACK_status			ack_status = NACK;

// Flash Operation variables
uint32_t FirstSector = 0, NbOfSectors = 0;
uint32_t Address = FLASH_USER_START_ADDR_RX, SECTORError = 0;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;
uint8_t TxBuffer;

/*Variable used for Flash Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

/* Function prototypes for Flash Operations */
static uint32_t GetSector(uint32_t Address);
static uint32_t GetSectorSize(uint32_t Sector);
static void Read_FLASH_and_Prepare_Data_for_CAN(uint32_t Sector, uint32_t StartSectorAddress);
static uint32_t Read_Message_and_Write_in_FLASH(uint32_t StartSectorAddress, uint32_t EndSectorAddress);
//static void sendACK(void);
//static enum ACK_status receiveACK(void);
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

	/* Transmitting board: 	TX = 1
	 * Receiving board: 	TX = 0 */

	TX = 1;

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

  /* As mentioned in 'stm32f4xx_hal_can.c', if you want to change any of the callback functions defined in 'stm32f4xx_hal_can.h',
   * you first need to set the macro 'USE_HAL_CAN_REGISTER_CALLBACKS' to 1 in 'stm32f4xx_hal_conf.h'.
   * This can also be done in the .ioc file GUI by going to Project Manager > Advanced Settings > Register Callback > CAN: ENABLE
   * Secondly, you need to register this callback using the function below. */
  //HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, HAL_CAN_RxFifo0MsgPendingCallback);

  /* Before receiving an update, we need to make sure that the receiving board erases the sector where the update will be stored. */
  if (!TX){

	  /* Unlock the Flash to enable the flash control register access */
	  HAL_FLASH_Unlock();

	  /* Erase the user Flash area defined by FLASH_USER_START_ADDR_RX and FLASH_USER_END_ADDR_RX) */

	  /* Get the 1st sector to erase */
	  FirstSector = GetSector(FLASH_USER_START_ADDR_RX);
	  /* Get the number of sector to erase from 1st sector*/
	  NbOfSectors = GetSector(FLASH_USER_END_ADDR_RX) - FirstSector + 1;
	  /* Fill EraseInit structure*/
	  EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	  EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	  EraseInitStruct.Sector        = FirstSector;
	  EraseInitStruct.NbSectors     = NbOfSectors;

	  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	  {
	    /*
	      Error occurred while sector erase.
	      User can add here some code to deal with this error.
	      SECTORError will contain the faulty sector and then to know the code error on this sector,
	      user can call function 'HAL_FLASH_GetError()'
	    */
	    /* Infinite loop */
	    while (1)
	    {
	    	HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
	    }
	  }

	  /* Add a small delay so the Flash erase can be completed before the CAN is started. */
	  //HAL_Delay(2000);
	  HAL_FLASH_Lock();
    }

  /* Start the CAN and enable interrupts*/
  //HAL_CAN_Start(&hcan1);

  app1();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //printf("in the while \r\n");
	  if (TX){
		  if (board_status == NO_UPDATE_AVAILABLE){
			  HAL_GPIO_WritePin(GPIOB, LD2_Pin | LD1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
			  HAL_Delay(1000);
		  }
		  else if (board_status == SENDING_UPDATE){
			  HAL_GPIO_WritePin(GPIOB, LD3_Pin | LD1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
			  HAL_Delay(100);
		  }
		  else if (board_status == UPDATE_FINISHED){
			  HAL_GPIO_WritePin(GPIOB, LD2_Pin | LD3_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
			  HAL_Delay(1000);
		  }
	  }
	  else {
		  HAL_Delay(100);

		  if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0){
			  /* Get RX message */
			  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader_CubeSat, RxData) != HAL_OK) {
				  /* Reception Error */
				  Error_Handler();
			  }
		  }
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

  TxHeader_GroundStation.StdId = 0b00000000000;
  TxHeader_GroundStation.ExtId = 0x00;
  TxHeader_GroundStation.RTR = CAN_RTR_DATA;
  TxHeader_GroundStation.IDE = CAN_ID_STD;
  TxHeader_GroundStation.DLC = 8; // Data length is equal to 8
  TxHeader_GroundStation.TransmitGlobalTime = DISABLE;

  TxHeader_CubeSat.StdId = 0b00000000000;
  TxHeader_CubeSat.ExtId = 0x00;
  TxHeader_CubeSat.RTR = CAN_RTR_DATA;
  TxHeader_CubeSat.IDE = CAN_ID_STD;
  TxHeader_CubeSat.DLC = 1; // Data length is equal to 1
  TxHeader_CubeSat.TransmitGlobalTime = DISABLE;

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

/* We will read the full sector. We read a byte each time and pass it to the TxData[i] of the CAN.
 * The maximum amount of data the CAN protocol can send per message is 8 bytes,
 * so we will repeat the read operation 8 times. */

void Read_FLASH_and_Prepare_Data_for_CAN(uint32_t Sector, uint32_t StartSectorAddress){

	board_status = SENDING_UPDATE;
	HAL_GPIO_WritePin(GPIOB, LD3_Pin | LD1_Pin, GPIO_PIN_RESET);

	/* Unlock the Flash to enable the flash control register access. */
	HAL_FLASH_Unlock();
	/* Since sectors can have different length, we need to calculate how many bytes need to be read. */
	uint32_t nrOfBytes = GetSectorSize(Sector);
	//uint32_t sector = Sector;

 	while (nrOfBytes != 0){
 		HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
 		for (int i = 0; i < 8; i++){
 			/* Read the first byte from Flash and store it in TxBuffer. */
 			TxBuffer = *(__IO uint8_t *)StartSectorAddress;

 			/* Put the content of TxBuffer in TxData[i]. */
 			TxData[i] = TxBuffer;

 			/* Move to the next byte in Flash memory. */
 			StartSectorAddress += sizeof(uint8_t);

 			/* Decrement the number of bytes that still have to be read. */
 			nrOfBytes--;
 			//printf("Sector: %lu \r\n", sector);
 			printf("nrOfBytes: %lu \r\n", nrOfBytes);
 			printf("TxData: %02X \r\n", TxData[i]);
 		}

 		/* Now that we have 8 bytes of data, we send a message through the CAN bus! */
 		HAL_CAN_AddTxMessage(&hcan1 , &TxHeader_GroundStation, TxData, &TxMailbox);
	}

 	/* When all data is read and sent, we lock the Flash memory again to protect it from unwanted operations. */
 	HAL_FLASH_Lock();
 	board_status = UPDATE_FINISHED;
}

/* When a message is received, we need to read it and write it in the Flash memory of the receiving board. */
uint32_t Read_Message_and_Write_in_FLASH(uint32_t StartSectorAddress, uint32_t EndSectorAddress){

	printf("reading message \r\n");

	HAL_FLASH_Unlock();

	board_status = RECEIVING_UPDATE;

//	/* Get RX message */
//	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
//	{
//		/* Reception Error */
//	    Error_Handler();
//	}


	/* Program the user Flash area byte by byte. */
	Address = StartSectorAddress;
	if (Address < EndSectorAddress){

		HAL_GPIO_TogglePin(GPIOB, LD2_Pin);

		/* Get RX message */
		if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader_CubeSat, RxData) != HAL_OK){

			/* Reception Error */
			Error_Handler();
		}

		for (int i = 0; i < 8; i++){
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, Address, RxData[i]) == HAL_OK){
				//Address = Address + 4;
				Address++;
				//printf("RxData: %02X \r\n", RxData[i]);
			}
		}
		//sendACK();
	}

	 /* Lock the Flash to disable the flash control register access
	  * (recommended to protect the FLASH memory against possible unwanted operation). */
	HAL_FLASH_Lock();
	board_status = UPDATE_FINISHED;
	return Address;
}

//void sendACK(){
//
//	TxData[0] = 0x11;
//	HAL_CAN_AddTxMessage(&hcan1 , &TxHeader_CubeSat, TxData, &TxMailbox);
//}
//
//enum ACK_status receiveACK(){
//
//	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader_GroundStation, RxData) != HAL_OK){
//		/* Reception Error */
//		Error_Handler();
//	}
//
//	if ((RxHeader_GroundStation.DLC == 1) && RxData[0] == 0x11){
//		printf("ACK \r\n");
//		ack_status = ACK;
//	}
//	else {
//		ack_status = NACK;
//	}
//	return ack_status;
//}

/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
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
static uint32_t GetSectorSize(uint32_t Sector)
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == GPIO_PIN_13){

		// Start reading the Flash sector where we stored the .bin file of the update.
		Read_FLASH_and_Prepare_Data_for_CAN(FLASH_SECTOR_3, FLASH_USER_START_ADDR_TX);
	}
}

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//  /* Get RX message */
//  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
//  {
//    /* Reception Error */
//    Error_Handler();
//  }
//
//  /* Display LEDx */
//  if ((RxHeader.StdId == 0b00000000000) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 8))
//  {
//    printf("message received \r\n");
//  }
//}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
//	if (TX){
//		printf("in the transmitter's callback \r\n");
//		receiveACK();

//	}
//	else {
		printf("in the receiver's callback \r\n");
		Read_Message_and_Write_in_FLASH(Address, FLASH_USER_END_ADDR_RX);
//	}

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
