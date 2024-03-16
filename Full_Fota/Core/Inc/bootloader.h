#ifndef _BOOTLOADER_H_
#define _BOOTLOADER_H_

/**
  ******************************************************************************
  * @file    bootloader.h
  * @brief   This file provides code for the configuration
  *          of the BOOTLOADER instances.
  ******************************************************************************
  **/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include "usart.h"
#include "crc.h"

/* Macro Declaration ---------------------------------------------------------*/
#define BL_HOST_BUFFER_RX_LENGTH             200

/* Debug Info */
#define BL_DEBUG_UART_PC                     &huart1
#define BL_DEBUG_UART_HOST                   &huart3

#define DEBUG_INFO_DISABLE          				 0
#define DEBUG_INFO_ENABLE            				 1
#define BL_DEBUG_ENABLE             				 DEBUG_INFO_DISABLE

#define BL_ENABLE_UART_DEBUG_MESSAGE 				 0x00
#define BL_ENABLE_SPI_DEBUG_MESSAGE 				 0x01
#define BL_ENABLE_CAN_DEBUG_MESSAGE  				 0x02
#define BL_DEBUG_METHOD                      BL_ENABLE_UART_DEBUG_MESSAGE

/* Bootloader Commands ID */
#define CBL_GET_VER_CMD                      0x10   /* Gets the protocol version */
#define CBL_GET_HELP_CMD                     0x11   /* Gets help with Commands ID */
#define CBL_GET_CID_CMD                      0x12   /* Gets the chip ID */
#define CBL_GET_RDP_STATUS_CMD               0x13   /* Read protection level */
#define CBL_GO_TO_ADDR_CMD                   0x14   /* Jumps to specific address located in the flash memory */
#define CBL_FLASH_ERASE_CMD                  0x15   /* Erases some pages from flash memory */
#define CBL_MEM_WRITE_CMD                    0x16   /* Writes up to 256 bytes FM from address specified by the application */
#define CBL_GO_TO_USER_APP                   0x22   /* Jumps to user application code located in the flash memory */

/* Added in future bootloader versions */
#define CBL_EN_RW_PROTECT_CMD                0x17
#define CBL_MEM_READ_CMD                     0x18
#define CBL_OTP_READ_CMD                     0x20
#define CBL_READ_SECTOR_STATUS_CMD           0x19
#define CBL_CHANGE_ROP_Level_CMD             0x21

/* Correspond to Bootloader_Get_Version function */
#define CBL_VENDOR_ID               			   100
#define CBL_SW_MAJOR_VERSION        			   1
#define CBL_SW_MINOR_VERSION        			   0
#define CBL_SW_PATCH_VERSION        			   1

/* CRC_VERIFICATION */
#define CRC_ENGINE_OBJ                       &hcrc
#define CRC_TYPE_SIZE_BYTE                   4
#define CRC_VERIFICATION_FAILED              0x00
#define CRC_VERIFICATION_PASSED              0x01

/* ACK/NACK Info */
#define CBL_SEND_NACK                        0xAB
#define CBL_SEND_ACK                         0xCD

/* Address Verification */
#define ADDRESS_IS_INVALID                   0x00
#define ADDRESS_IS_VALID                     0x01

#define STM32F103_SRAM_SIZE        				   (20  * 1024)
#define STM32F103_FLASH_SIZE        				 (64 * 1024)
#define STM32F103_SRAM_END         				   (SRAM_BASE  + STM32F103_SRAM_SIZE)
#define STM32F103_FLASH_END         				 (FLASH_BASE + STM32F103_FLASH_SIZE)

/* CBL_FLASH_ERASE_CMD */
#define CBL_USER_FLASH_START_ADDRESS          0x08000000
#define CBL_FLASH_MAX_PAGES_NUMBER  					64
#define CBL_FLASH_MASS_ERASE         					0xFF   

#define INVALID_PAGE_NUMBER        				  	0x00
#define VALID_PAGE_NUMBER         				  	0x01
#define UNSUCCESSFUL_ERASE          					0x02
#define SUCCESSFUL_ERASE            					0x03
#define SUCCESSFUL_MASS_ERASE          		  	0x04
#define UNSUCCESSFUL_MASS_ERASE          			0x05
#define CHECK_SUCCESSFUL_ERASE            	  0xFFFFFFFFU

#define HAL_SUCCESSFUL_ERASE         					0xFFFFFFFFU

/* CBL_MEM_WRITE_CMD */
#define FLASH_PAYLOAD_WRITE_FAILED   					0x00
#define FLASH_PAYLOAD_WRITE_PASSED   					0x01

#define FLASH_LOCK_WRITE_FAILED               0x00
#define FLASH_LOCK_WRITE_PASSED               0x01

/* CBL_GET_RDP_STATUS_CMD */
#define ROP_LEVEL_READ_INVALID                0x00
#define ROP_LEVE_READL_VALID                  0X01

/* CBL_CHANGE_ROP_Level_CMD */
#define ROP_LEVEL_CHANGE_INVALID              0x00
#define ROP_LEVEL_CHANGE_VALID                0X01

#define CBL_ROP_LEVEL_0            				    0x00
#define CBL_ROP_LEVEL_1              				  0x01
#define CBL_ROP_LEVEL_2              					0x02

/* Macro Function Declaration ------------------------------------------------*/

/* Data Types Declaration ----------------------------------------------------*/
typedef enum
{
	BL_OK,
	BL_NOT_OK
}BL_Status;

typedef void (*pMainApp) (void);
typedef void (*Jump_Ptr) (void);
/* Function Declaration ------------------------------------------------------*/
BL_Status BL_UART_Fetch_Host_Command (void);
void      BL_Print_Message           (char *format, ...);
#endif /* _BOOTLOADER_H_ */
