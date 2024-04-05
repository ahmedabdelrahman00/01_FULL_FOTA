/**
  ******************************************************************************
  * @file    Bootloader Commands.cpp
  * @brief   This file provides code for the configuration
  *          of the Commands instances.
  ******************************************************************************
  **/

/* Includes ------------------------------------------------------------------*/
#include "Bootloader_Commands.h"
#include <SoftwareSerial.h>
SoftwareSerial SerialPort(4,5);


void displayProgressBar(uint32_t bytesRead, uint32_t fileSize); 
/* Data Types Declaration ----------------------------------------------------*/

/* Global Definitions --------------------------------------------------------*/  
char     BL_Host_Buffer        [BL_Host_Buffer_LENGTH];
char     BL_Buffer             [20];
uint32_t CRC32_Value;

/* API Definitions -----------------------------------------------------------*/
 void UART2_Init (void){
   SerialPort.begin(9600);
 }

/**
 * @brief Calculate CRC32 checksum for a given buffer.
 *
 * This function calculates the CRC32 checksum for the specified buffer of data.
 */
uint32_t Calculate_CRC32 (char* Buffer, uint32_t Buffer_Length){
    uint32_t CRC_Value = 0xFFFFFFFF;
    for (uint32_t i = 0; i < Buffer_Length; i++) {
        CRC_Value = CRC_Value ^ Buffer[i];
        for (uint32_t DataElemBitLen = 0; DataElemBitLen < 32; DataElemBitLen++){
            if(CRC_Value & 0x80000000){
                CRC_Value = (CRC_Value << 1) ^ 0x08C71CC1;
            } else {
                CRC_Value = (CRC_Value << 1);
            }
        }
    }
    return CRC_Value;
}

/**
 * @brief Convert a 32-bit word value to an 8-bit byte value.
 *
 * This function extracts the byte value at the specified index from a 32-bit word.
 */   
uint8_t Word_Value_To_Byte_Value(uint32_t Word_Value, uint8_t Byte_Index) {
    uint8_t Byte_Value = (Word_Value >> (8 * (Byte_Index - 1))) & 0x000000FF;
    return Byte_Value;
}

/**
 * @brief Wait for acknowledgment.
 *
 * This function waits for an acknowledgment signal.
 */
void waitAck (void){
    uint8_t recVal = NACK;
    do {
        recVal = NACK;
        if (SerialPort.available()) {
            recVal = SerialPort.read();
        }
    } while (recVal == NACK);
    //Serial.println("\nReceived ACK\n");
}

/**
 * @brief Send a host command over UART.
 *
 * This function sends a specified command ID to the host over UART.
 */
void BL_UART_Send_Host_Command (uint8_t command_ID){
  /* Format the Command Packet */
  BL_Host_Buffer[0] = CBL_GET_VER_CMD_LEN - 1;
  BL_Host_Buffer[1] = command_ID;
  CRC32_Value = Calculate_CRC32(BL_Host_Buffer, CBL_GET_VER_CMD_LEN - 4);
  CRC32_Value = CRC32_Value & 0xFFFFFFFF;
  Serial.print("\n\n");
  Serial.println(CRC32_Value, HEX);
  BL_Host_Buffer[2] = Word_Value_To_Byte_Value(CRC32_Value, 1);
  BL_Host_Buffer[3] = Word_Value_To_Byte_Value(CRC32_Value, 2);
  BL_Host_Buffer[4] = Word_Value_To_Byte_Value(CRC32_Value, 3);
  BL_Host_Buffer[5] = Word_Value_To_Byte_Value(CRC32_Value, 4);
  /* Inform STM32 to Start Fetching the Packet */
  Serial.println("Starting Process...");
  SerialPort.write(BOOTLOADER_START_KEY);
  /* Send the Packet */
  Serial.print(BL_Host_Buffer[0]);
  Serial.print(' ');
  SerialPort.write(BL_Host_Buffer[0]);
  for (int i = 1; i < CBL_GET_VER_CMD_LEN; ++i) {
     Serial.print(BL_Host_Buffer[i], HEX);
     Serial.print(' ');
     SerialPort.write(BL_Host_Buffer[i]);
  }
  Serial.println();
  /* Read the Command Response */
  while(!SerialPort.available());
  SerialPort.readBytes(BL_Buffer, 2);
  Serial.println(BL_Buffer[0], HEX);
  Serial.print(BL_Buffer[1], HEX);
  if (BL_Buffer[0] == ACK){
    Serial.println("\nReceived Acknowledge");
    uint8_t packet_received_length = BL_Buffer[1];
    SerialPort.readBytes(BL_Buffer, packet_received_length);
     for (int i = 0; i < packet_received_length; ++i){
     Serial.print(BL_Buffer[i], DEC);
     Serial.print(' ');
     }
  }
  else{
   Serial.println("\nReceived Not-acknowledge");
  }
}

/**
 * @brief Send a write command to the bootloader.
 *
 * This function sends a write command along with the memory address to the bootloader over UART.
 *
 * @param[in] memoryAddress The memory address for the write operation.
 * @return uint8_t Status indicating success (1) or failure (0).
 */

uint8_t BL_UART_Send_Write_Command (uint32_t memoryAddress){
 char     Local_dummyBuf [2];
 uint16_t shiftingTimes = 0;
 uint32_t bytesRead = 0;
 uint8_t  CBL_MEM_WRITE_CMD_Len = 11 + PAYLOAD_LENGTH;
 uint32_t memoryAddressShift;
 /* Display some Infos */   
 Serial.println("Write data into different memories of the MCU command");
 Serial.print("Base Address: ");
 Serial.println(memoryAddress, HEX);
 /* Open file */
 if (!LittleFS.begin()) {
    Serial.println("Failed to mount LittleFS");
    return 0;
  }
  Serial.println("LittleFS mounted successfully");
  File file = LittleFS.open("/fileupdate.bin", "r");
  if (!file) {
    Serial.println("Failed to open file for reading");
  } 
  else {
       Serial.print("Preparing writing a binary file with length: ");
       Serial.println(file.size());
       while(file.available()){
          /* Format the Command Packet */
          BL_Host_Buffer[0]           = CBL_MEM_WRITE_CMD_Len - 1;
          BL_Host_Buffer[1]           = CBL_MEM_WRITE_CMD;
          memoryAddressShift = memoryAddress + (shiftingTimes * PAYLOAD_LENGTH);
          BL_Host_Buffer[2] = Word_Value_To_Byte_Value(memoryAddressShift, 1);
          BL_Host_Buffer[3] = Word_Value_To_Byte_Value(memoryAddressShift, 2);
          BL_Host_Buffer[4] = Word_Value_To_Byte_Value(memoryAddressShift, 3);
          BL_Host_Buffer[5] = Word_Value_To_Byte_Value(memoryAddressShift, 4);
          BL_Host_Buffer[6]           = PAYLOAD_LENGTH;
          uint8_t itrator = 0;
          while (itrator < PAYLOAD_LENGTH){
              BL_Host_Buffer[7 + itrator] = (char)file.read();;
              //Serial.printf("Packet[%d] = ", 7 + itrator);
              //Serial.printf("%x\n", BL_Host_Buffer[7 + itrator]);
              itrator++;
          }
          bytesRead += PAYLOAD_LENGTH;
          displayProgressBar(bytesRead, file.size());
          CRC32_Value = Calculate_CRC32(BL_Host_Buffer, CBL_MEM_WRITE_CMD_Len - 4);
          CRC32_Value = CRC32_Value & 0xFFFFFFFF;
          //Serial.print("\n\n");
          //Serial.println(CRC32_Value, HEX);
          BL_Host_Buffer[(7 + PAYLOAD_LENGTH)]  = Word_Value_To_Byte_Value(CRC32_Value, 1);
          BL_Host_Buffer[(8 + PAYLOAD_LENGTH)]  = Word_Value_To_Byte_Value(CRC32_Value, 2);
          BL_Host_Buffer[(9 + PAYLOAD_LENGTH)]  = Word_Value_To_Byte_Value(CRC32_Value, 3);
          BL_Host_Buffer[(10 + PAYLOAD_LENGTH)] = Word_Value_To_Byte_Value(CRC32_Value, 4);
          /* Display whole packet 
          for (uint8_t i = 0; i < CBL_MEM_WRITE_CMD_Len; i++){
               Serial.print(BL_Host_Buffer[i], HEX);
               Serial.print(' ');
          }
          */
          //Serial.print("\n\n");
          /* Inform STM32 to Start Fetching the Packet */
          //Serial.println("Starting Process...");
          SerialPort.write(BOOTLOADER_START_KEY);
          /* Send the Packet */
          //Serial.print(BL_Host_Buffer[0]);
          //Serial.print(' ');
          SerialPort.write(BL_Host_Buffer[0]);
          for (uint16_t i = 1; i < CBL_MEM_WRITE_CMD_Len; ++i){
                //Serial.print(BL_Host_Buffer[i], HEX);
                //Serial.print(' ');
                SerialPort.write(BL_Host_Buffer[i]);
           }
           //Serial.println();
           /* Read the Command Response */
           while(!SerialPort.available());
           SerialPort.readBytes(BL_Buffer, 2);
           //Serial.println(BL_Buffer[0], HEX);
           //Serial.print(BL_Buffer[1], HEX);
           if (BL_Buffer[0] == ACK){
                /* Hence, The Packet transfering successed */
                //Serial.println("\nReceived Acknowledge");
                uint8_t packet_received_length = BL_Buffer[1];
                SerialPort.readBytes(BL_Buffer, packet_received_length);
                /*
                for (int i = 0; i < packet_received_length; ++i){
                      Serial.print(BL_Buffer[i], DEC);
                      Serial.print(' ');
                }
                */
                shiftingTimes++;
                //Serial.println("Flushing Host Buffer...");
                memset(BL_Host_Buffer, 0, sizeof(BL_Host_Buffer)); 
                //Serial.println("Waiting for Acknowledge...");
                waitAck();
            }
            else{
               Serial.println("\nReceived Not-acknowledge");
               Serial.println("\nOut of Function");
               return 0;
            }
        }   
    }
    return 1;     
}

/**
 * @brief Send an application jump command to the bootloader.
 *
 * This function sends a command to the bootloader to initiate a jump to the application.
 */
void BL_UART_Send_AppJump_Command (void){
  /* Format the Command Packet */
  BL_Host_Buffer[0] = CBL_GO_TO_USER_APP_LEN - 1;
  BL_Host_Buffer[1] = CBL_GO_TO_USER_APP;
  CRC32_Value = Calculate_CRC32(BL_Host_Buffer, CBL_GO_TO_USER_APP_LEN - 4);
  CRC32_Value = CRC32_Value & 0xFFFFFFFF;
  Serial.print("\n\n");
  Serial.println(CRC32_Value, HEX);
  BL_Host_Buffer[2] = Word_Value_To_Byte_Value(CRC32_Value, 1);
  BL_Host_Buffer[3] = Word_Value_To_Byte_Value(CRC32_Value, 2);
  BL_Host_Buffer[4] = Word_Value_To_Byte_Value(CRC32_Value, 3);
  BL_Host_Buffer[5] = Word_Value_To_Byte_Value(CRC32_Value, 4);
  /* Inform STM32 to Start Fetching the Packet */
  Serial.println("Starting Process...");
  SerialPort.write(BOOTLOADER_START_KEY);
  /* Send the Packet */
  Serial.println("---> Send JUMP TO USER APPLICATION command");
  Serial.print(BL_Host_Buffer[0]);
  Serial.print(' ');
  SerialPort.write(BL_Host_Buffer[0]);
  for (int i = 1; i < CBL_GO_TO_USER_APP_LEN; ++i) {
     Serial.print(BL_Host_Buffer[i], HEX);
     Serial.print(' ');
     SerialPort.write(BL_Host_Buffer[i]);
  }
  Serial.println();
  /* Read the Command Response */
  while(!SerialPort.available());
  SerialPort.readBytes(BL_Buffer, 2);
  Serial.println(BL_Buffer[0], HEX);
  Serial.print(BL_Buffer[1], HEX);
  if (BL_Buffer[0] == ACK){
    Serial.println("\nReceived Acknowledge");
    uint8_t packet_received_length = BL_Buffer[1];
    SerialPort.readBytes(BL_Buffer, packet_received_length);
     for (int i = 0; i < packet_received_length; ++i){
     Serial.print(BL_Buffer[i], DEC);
     Serial.print(' ');
     }
  }
  else{
   Serial.println("\nReceived Not-acknowledge");
  }
}

/**
 * @brief Send a bootloader jump command to the host.
 *
 * This function sends a command to the host to initiate a jump to the bootloader.
 *
 * @return uint8_t Status indicating success (1) or failure (0).
 */

uint8_t  BL_UART_Send_BootloaderJump_Command (void){
  char bte = 0x00;
  /* Inform STM32 to Start Fetching the Packet */
  Serial.println("Starting send Jump to bootloader Process...");
  SerialPort.write(BOOTLOADER_START_KEY);
  SerialPort.write(BOOTLOADER_START_KEY);
  /* Read Trigger key status */
  while(!SerialPort.available());
  bte = SerialPort.read();
  debug("bte = ");
  Serial.println(bte, HEX);
  if (ACK == bte){
        debugln("Bootloader Command Packet has been Acknowledged");
        return 1;
  }
  else{
        Serial.println("Bootloader Command Packet Not Acknowledged");
        return 0; 
  }
}

/**
 * @brief Send an erase command to the bootloader.
 *
 * This function sends an erase command to the bootloader over UART.
 *
 * @param[in] pageNum Page number to be erased.
 * @param[in] NumsofPages Number of pages to be erased.
 */
void BL_UART_Send_Erase_Command (uint8_t pageNum, uint8_t NumsofPages){
  memset(BL_Host_Buffer, 0, sizeof(BL_Host_Buffer));  
  /* Format the Command Packet */
  BL_Host_Buffer[0] = CBL_FLASH_ERASE_CMD_LEN - 1;
  BL_Host_Buffer[1] = CBL_FLASH_ERASE_CMD;
  BL_Host_Buffer[2] = pageNum;
  BL_Host_Buffer[3] = NumsofPages;
  CRC32_Value = Calculate_CRC32(BL_Host_Buffer, CBL_FLASH_ERASE_CMD_LEN - 4);
  CRC32_Value = CRC32_Value & 0xFFFFFFFF;
  Serial.print("\n\n");
  Serial.println(CRC32_Value, HEX);
  BL_Host_Buffer[4] = Word_Value_To_Byte_Value(CRC32_Value, 1);
  BL_Host_Buffer[5] = Word_Value_To_Byte_Value(CRC32_Value, 2);
  BL_Host_Buffer[6] = Word_Value_To_Byte_Value(CRC32_Value, 3);
  BL_Host_Buffer[7] = Word_Value_To_Byte_Value(CRC32_Value, 4);
  /* Inform STM32 to Start Fetching the Packet */
  Serial.println("Starting Process...");
  SerialPort.write(BOOTLOADER_START_KEY);
  /* Send the Packet */
  Serial.print(BL_Host_Buffer[0]);
  Serial.print(' ');
  SerialPort.write(BL_Host_Buffer[0]);
  for (int i = 1; i < CBL_FLASH_ERASE_CMD_LEN; ++i) {
     Serial.print(BL_Host_Buffer[i], HEX);
     Serial.print(' ');
     SerialPort.write(BL_Host_Buffer[i]);
  }
  Serial.println();
  /* Read the Command Response */
  while(!SerialPort.available());
  SerialPort.readBytes(BL_Buffer, 2);
  Serial.println(BL_Buffer[0], HEX);
  Serial.print(BL_Buffer[1], HEX);
  if (BL_Buffer[0] == ACK){
    Serial.println("\nReceived Acknowledge");
    uint8_t packet_received_length = BL_Buffer[1];
    SerialPort.readBytes(BL_Buffer, packet_received_length);
     for (int i = 0; i < packet_received_length; ++i){
     Serial.print(BL_Buffer[i], DEC);
     Serial.print(' ');
     }
     Serial.println("\nErased Successed");
  }
  else{
   Serial.println("\nReceived Not-acknowledge");
   Serial.println("\nErased Failed");
  }
}

void displayProgressBar(uint32_t bytesRead, uint32_t fileSize) {
    uint8_t PROGRESS_BAR_WIDTH = 80;
    int progress = (bytesRead * PROGRESS_BAR_WIDTH) / fileSize;
    int percentage = (bytesRead * 100) / fileSize;

    Serial.printf("\rProgress: [");
    for (int i = 0; i < PROGRESS_BAR_WIDTH; ++i) {
        if (i < progress) {
            Serial.print("=");
        } else {
            Serial.print(" ");
        }
    }
    Serial.printf("] %d%%", percentage);
    for (int i = 0; i < PROGRESS_BAR_WIDTH + 15; ++i) {
        Serial.print("\b");
    }
}






uint8_t determineBootMode() {
    char response = 0x00;
    
    Serial.println("Requesting mode state...");
    SerialPort.write(0x66); // Command byte to STM
    
    long startMillis = millis();
    while (!SerialPort.available()) {
        if (millis() - startMillis > 5000) { // Timeout of 5 seconds
            Serial.println("Timeout waiting for response");
            return 0xFF; // Indicate timeout
        }
    }
    
    response = SerialPort.read();
    
    Serial.print("Response received: ");
    Serial.println(response, HEX);
    
    return response;
}