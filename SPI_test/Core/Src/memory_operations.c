#include "stm32f4xx_hal.h"
#include "memory_operations.h"
#include <stdio.h>

#define FLASH_ROW_SIZE          32

/* @note All the executable code is mapped in SRAM1 area */
#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_2   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_2  +  GetSectorSize(ADDR_FLASH_SECTOR_7) -1 /* End @ of user Flash area : sector start address + sector size -1 */

#define DATA_32		((uint32_t)0x12345678)

uint32_t FirstSector = 0, NbOfSectors = 0;
uint32_t Address = 0, SECTORError = 0;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;

static FLASH_EraseInitTypeDef EraseInitStruct;

void Flash_Erase_Write()
{
	 /* Unlock the Flash to enable the flash control register access *************/
	  HAL_FLASH_Unlock();

	  /* Erase the user Flash area
	    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	  /* Get the 1st sector to erase */
	  FirstSector = GetSector(FLASH_USER_START_ADDR);
	  NbOfSectors = GetSector(FLASH_USER_END_ADDR) - FirstSector + 1;
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
	      printf("Error while sector erase\r\n");
	    }
	  }

	  /* Program the user Flash area word by word
	    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	  Address = FLASH_USER_START_ADDR;
	  uint32_t Sector2Start = ADDR_FLASH_SECTOR_7;
	  uint32_t Sector2End = ADDR_FLASH_SECTOR_7 + GetSectorSize(FLASH_SECTOR_2) - 1;

	  /*

	  while (Address < FLASH_USER_END_ADDR)
	  {
	    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, DATA_32) == HAL_OK)
	    {
	      Address = Address + 4;
	    }

	    */

	  while (Address < FLASH_USER_END_ADDR && Sector2Start <= Sector2End)
	  {
		  uint32_t data32 = *(__IO uint32_t *)Sector2Start;

		  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, data32) == HAL_OK)
		  {
			  Address += 4;
			  Sector2Start += 4;
		  }
		  else
		  {
			  /* Error occurred while writing data in Flash memory.
	         User can add here some code to deal with this error */
			  while (1)
			  {
				  printf("error while writing data in flash memory \r\n");
			  }
		  }
	  }
	  /* Lock the Flash to disable the flash control register access (recommended
	      to protect the FLASH memory against possible unwanted operation) *********/
	   HAL_FLASH_Lock();

	   /* Check if the programmed data is OK
	       MemoryProgramStatus = 0: data programmed correctly
	       MemoryProgramStatus != 0: number of words not programmed correctly ******/
	   Address = FLASH_USER_START_ADDR;
	   MemoryProgramStatus = 0x0;

	   while (Address < FLASH_USER_END_ADDR)
	   {
	     data32 = *(__IO uint32_t *)Address;

	     if (data32 != DATA_32)
	     {
	       MemoryProgramStatus++;
	     }
	     Address = Address + 4;
	   }

	   /*Check if there is an issue to program data*/
	   if (MemoryProgramStatus == 0)
	   {
	   }
	   else
	   {
	     /* Error detected. Switch on LED2*/
	     printf("Error Detected\r\n");
	   }


}

uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_7) */
  {
    sector = FLASH_SECTOR_7;
  }
  return sector;
}


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

