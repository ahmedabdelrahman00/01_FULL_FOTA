/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <HardwareSerial.h>
#include "Firebase_Handler.h"
#include "FileSystemHandler.h"
#include "Bootloader_Commands.h"
#include "Debug.h"

/* Global Definitions --------------------------------------------------------*/  
unsigned long PrevMillis;
uint8_t       bootloaderJumpStatues;
bool          writeState             = true;
uint8_t       writeStatus            = FAILD;
uint8_t       stateMode              = BOOTLOADER_MODE;


void setup() { 
    
 UART2_Init();
 Wifi_Connect();
 Server_Connect();
 /* Test the Bootloader is Active and make a response */
 //BL_UART_Send_Host_Command(CBL_GET_VER_CMD);

  uint8_t mode = determineBootMode();
  if (mode == 0x01) {
      Serial.println("STM32 in Application Mode");
      stateMode = APPLICATION_MODE;
  } else if (mode == 0x02) {
      Serial.println("STM32 in Bootloader Mode");
      stateMode = BOOTLOADER_MODE;
  } else {
      Serial.println("Unknown Mode or Timeout");
  }
}
void loop() {
  // Check for update periodically
  if (millis() - PrevMillis > 5000) { // Every 5 seconds
    PrevMillis = millis();
    bool isUpdate = UpdateCheck(); // Checks if a new firmware version is available

    // If the STM32 is in application mode and no update is needed, do nothing
    if (APPLICATION_MODE == stateMode && !isUpdate) {
      Serial.println("Application mode detected with no updates available. Waiting...");
      return; // Wait until the next loop iteration to check again
    }

    // If an update is detected or STM32 is in bootloader mode, proceed with flashing
    if (isUpdate || BOOTLOADER_MODE == stateMode) {
      Server_Download("file.bin");
      debugln("Read The File to Serial Monitor");
      readFile("/fileupdate.bin");
      debugln("Send The File to STM32");

      // Only try jumping to bootloader mode if we're in application mode
      if (APPLICATION_MODE == stateMode) {
        debugln("Send Jump to Bootloader Command");
        bootloaderJumpStatues = BL_UART_Send_BootloaderJump_Command();
        if (bootloaderJumpStatues) {
          debugln("Waiting seconds...");
          delay(4000); // Ensure STM32 is ready to receive the erase command
          debugln("Send Erase Command");
          BL_UART_Send_Erase_Command(2, 3); // Example erase command
          writeState = true;
        }
      }

      // Proceed with flashing process
      if (writeState) {
        writeStatus = BL_UART_Send_Write_Command(APP1_BASE_ADDRESS);
        if (SUCCESS == writeStatus) {
          debugln("Write process succeeded, jumping to application");
          BL_UART_Send_AppJump_Command();
          stateMode = APPLICATION_MODE; // Update the mode to application
          writeState = false; // Reset the write state for future updates
        } else {
          debugln("Write process failed");
        }
        isUpdate = false; // Reset update flag
      }
    }
  }
}
