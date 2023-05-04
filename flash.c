/*
 * flash.c
 *
 *  Created on: Jun 30, 2022
 *      Author: hien
 */

#include "flash.h"

/**
 * @brief  Gets the page of a given address
 * @param  Addr: Address of the FLASH Memory
 * @retval The page of a given address
 */
static uint32_t GetPageFromAddr(uint32_t Addr) {
	return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;;
}

// Return 0: No error
// Return 1: Error
uint8_t WriteFlash(uint32_t Address, uint32_t data_32) {

	uint32_t old_data = *(__IO uint32_t*) Address;

	if (old_data != data_32) {

		uint64_t data_64 = 0;
		FLASH_EraseInitTypeDef EraseInitStruct;
		uint32_t FirstPage = 0, NbOfPages = 0;
		uint32_t PageError = 0;

		HAL_FLASH_Unlock(); // Unlock the Flash

		// For debugging
		//printf("Reading from address 0x%lx: %ld\n", Address, *(__IO uint32_t *)Address);

		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR); // Clear OPTVERR bit set on virgin samples

		FirstPage = GetPageFromAddr(Address); //Get the 1st page to erase
		NbOfPages = GetPageFromAddr(Address) - FirstPage + 1; // Get the number of pages to erase from 1st page

		/* Fill EraseInit structure*/
		EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.Page = FirstPage;
		EraseInitStruct.NbPages = NbOfPages;

		if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
			printf("Flash erase error\n");
			uint32_t error_code = HAL_FLASH_GetError();
			printf("Error code: %ld\n", error_code);
			return 1;
		}
		printf("Page %ld was erased\n", (Address - 0x08000000) / 0x800);

		data_64 = ((uint64_t) (data_32) << 32) | data_32;
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, data_64)
				== HAL_OK) {
		} else {
			printf("Flash write error\n");
			return 1;
		}

		HAL_FLASH_Lock();

		uint32_t read_data = *(__IO uint32_t*) Address;

		if (data_32 == read_data) {
			//printf("Wrote %ld to address 0x%ld\n", read_data, Address);
			return 0;
		} else {
			printf("Write failed\n");
			return 1;
		}
	} else {
		return 0;
	}
}

// Return data
int32_t ReadFlash(uint32_t Address) {
	int32_t read_data = *(__IO uint32_t*) Address;
	return read_data;
}

