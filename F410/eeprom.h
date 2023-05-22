/**
  ******************************************************************************
  * @file    EEPROM/EEPROM_Emulation/inc/eeprom.h 
  * @author  MCD Application Team
  * @brief   This file contains all the functions prototypes for the EEPROM 
  *          emulation firmware library.
  ******************************************************************************
  * @attention
  *
  * Always check user manual to adjust EEPROM_START_ADDRESS (always use the last flash sector)
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdio.h>

/* My application definitions --------------------------------------------------------*/
#define ADDR_ANCHOR_ID ((uint16_t)0x0001) // One page: 0x0000 to 0x7FFF



/* Exported constants --------------------------------------------------------*/
/* EEPROM emulation firmware error codes */
#define EE_OK      (uint32_t)HAL_OK
#define EE_ERROR   (uint32_t)HAL_ERROR
#define EE_BUSY    (uint32_t)HAL_BUSY
#define EE_TIMEOUT (uint32_t)HAL_TIMEOUT

/* Define the size of the sectors to be used */
#define PAGE_SIZE               (uint32_t)0x4000  /* Page size = 16KByte */

/* Device voltage range supposed to be [2.7V to 3.6V], the operation will 
   be done by word  */
#define VOLTAGE_RANGE           (uint8_t)VOLTAGE_RANGE_3

///* EEPROM start address in Flash */
//#define EEPROM_START_ADDRESS  ((uint32_t)0x08008000) /* EEPROM emulation start address:
//                                                  from sector2 : after 16KByte of used
//                                                  Flash memory */

/* HV: New EEPROM start address in Flash based on amount of flash occupied by the program*/
// https://www.st.com/resource/en/reference_manual/rm0401-stm32f410-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
// Page 46
#define EEPROM_START_ADDRESS  ((uint32_t)0x0801B000) /* EEPROM emulation start address: after 112KByte of used Flash memory */
// Flash start address 	0x0800 0000
// Flash end address   	0x0802 0000
// Sector 2 address		0x0800 8000
// Sector 4 address		0x0801 1000 to 0x0801 FFFF 64 KB
// Each 4000 of the address corresponding to 16 kB (One page)
// 4k: cal: 0x0801C000, worked: 0x0801B000
// 2k: 0x0801E000
// Problem: This address is inside sector 4 which is being used by the program. Thus it gets erased during flashing a new program.

/* Pages 0 and 1 base and end addresses */
#define PAGE0_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + 0x0000))
#define PAGE0_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (PAGE_SIZE - 1)))
#define PAGE0_ID               FLASH_SECTOR_2

#define PAGE1_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + 0x4000))
#define PAGE1_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (2 * PAGE_SIZE - 1)))
#define PAGE1_ID               FLASH_SECTOR_3

/* Used Flash pages for EEPROM emulation */
#define PAGE0                 ((uint16_t)0x0000)
#define PAGE1                 ((uint16_t)0x0001) /* Page nb between PAGE0_BASE_ADDRESS & PAGE1_BASE_ADDRESS*/

/* No valid page define */
#define NO_VALID_PAGE         ((uint16_t)0x00AB)

/* Page status definitions */
#define ERASED                ((uint16_t)0xFFFF)     /* Page is empty */
#define RECEIVE_DATA          ((uint16_t)0xEEEE)     /* Page is marked to receive data */
#define VALID_PAGE            ((uint16_t)0x0000)     /* Page containing valid data */

/* Valid pages in read and write defines */
#define READ_FROM_VALID_PAGE  ((uint8_t)0x00)
#define WRITE_IN_VALID_PAGE   ((uint8_t)0x01)

/* Page full define */
#define PAGE_FULL             ((uint8_t)0x80)

/* Variables' number */
#define NB_OF_VAR             ((uint8_t)0x03)

/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint16_t EE_Init(void);
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data);
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data);

/* HV: My functions */
uint8_t WriteFlash(uint16_t addr, uint16_t data);
uint16_t ReadFlash(uint16_t addr);

#endif /* __EEPROM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
