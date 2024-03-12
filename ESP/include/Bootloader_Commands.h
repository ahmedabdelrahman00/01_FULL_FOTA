#ifndef COMMANDS_H_
#define COMMANDS_H_

/**
  ******************************************************************************
  * @file    Bootloader Commands.h
  * @brief   This file provides code for the configuration
  *          of the Commands instances.
  ******************************************************************************
**/
/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <stdio.h>
#include "Debug.h"
#include "FileSystemHandler.h"

/* Macro Declaration ---------------------------------------------------------*/
#define CBL_GET_VER_CMD                      0x10   /* Gets the protocol version */
#define CBL_GET_HELP_CMD                     0x11   /* Gets help with Commands ID */
#define CBL_GET_CID_CMD                      0x12   /* Gets the chip ID */
#define CBL_GET_RDP_STATUS_CMD               0x13   /* Read protection level */
#define CBL_GO_TO_ADDR_CMD                   0x14   /* Jumps to specific address located in the flash memory */
#define CBL_FLASH_ERASE_CMD                  0x15   /* Erases some pages from flash memory */
#define CBL_MEM_WRITE_CMD                    0x16   /* Writes up to 256 bytes FM from address specified by the application */
#define CBL_GO_TO_USER_APP                   0x22   /* Jumps to user application code located in the flash memory */

#define PAYLOAD_LENGTH                       64
#define BL_Host_Buffer_LENGTH                400
#define APP1_BASE_ADDRESS                    0x08008000

#define CBL_GET_VER_CMD_LEN                  6   
#define CBL_GET_HELP_CMD_LEN                 6   
#define CBL_GET_CID_CMD_LEN                  6   
#define CBL_GET_RDP_STATUS_CMD_LEN           6  
#define CBL_GO_TO_USER_APP_LEN               6  
#define CBL_FLASH_ERASE_CMD_LEN              8  
#define CBL_MEM_WRITE_CMD_LEN                11 + PAYLOAD_LENGTH

#define SUCCESS                              0x01
#define FAILD                                0x00
#define ACK                                  0xCD
#define NACK                                 0xAB
#define BOOTLOADER_MODE                      0xC1
#define APPLICATION_MODE                     0xC2
#define BOOTLOADER_START_KEY                 0x7F
/* Macro Function Declaration ------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/
void     UART2_Init                          (void);
/**
 * @brief Calculate CRC32 checksum for a given buffer.
 *
 * This function calculates the CRC32 checksum for the specified buffer of data.
 *
 * @param[in] Buffer Pointer to the input buffer containing the data.
 * @param[in] Buffer_Length Length of the input buffer in bytes.
 * @return uint32_t Calculated CRC32 checksum.
 */
uint32_t Calculate_CRC32                     (char *Buffer, uint32_t Buffer_Length);

/**
 * @brief Convert a 32-bit word value to an 8-bit byte value.
 *
 * This function extracts the byte value at the specified index from a 32-bit word.
 *
 * @param[in] Word_Value The 32-bit word value.
 * @param[in] Byte_Index Index of the byte to extract (0-3).
 * @return uint8_t The extracted 8-bit byte value.
 */
uint8_t Word_Value_To_Byte_Value(uint32_t Word_Value, uint8_t Byte_Index);

/**
 * @brief Wait for acknowledgment.
 *
 * This function waits for an acknowledgment signal.
 */
void waitAck(void);

/**
 * @brief Send a host command over UART.
 *
 * This function sends a specified command ID to the host over UART.
 *
 * @param[in] command_ID The ID of the command to be sent.
 */
void BL_UART_Send_Host_Command(uint8_t command_ID);

/**
 * @brief Send a write command to the bootloader.
 *
 * This function sends a write command along with the memory address to the bootloader over UART.
 *
 * @param[in] memoryAddress The memory address for the write operation.
 * @return uint8_t Status indicating success (1) or failure (0).
 * @note This function is common for these commands:
 *        1- Gets the protocol version
 *        2- Gets help with Commands ID 
 *        3- Gets the chip ID
 *        4- Read protection level
 *       
 */
uint8_t BL_UART_Send_Write_Command(uint32_t memoryAddress);

/**
 * @brief Send an application jump command to the bootloader.
 *
 * This function sends a command to the bootloader to initiate a jump to the application.
 */
void BL_UART_Send_AppJump_Command(void);
S
/**
 * @brief Send a bootloader jump command to the host.
 *S
 * This function sends a command to the host to initiate a jump to the bootloader.
 *
 * @return uint8_t Status indicating success (1) or failure (0).
 */
uint8_t BL_UART_Send_BootloaderJump_Command(void);

/**
 * @brief Send an erase command to the bootloader.
 *
 * This function sends an erase command to the bootloader over UART.
 *
 * @param[in] pageNum Page number to be erased.
 * @param[in] NumsofPages Number of pages to be erased.
 */
void BL_UART_Send_Erase_Command(uint8_t pageNum, uint8_t NumsofPages);

#endif