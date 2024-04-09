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
uint8_t Active = 0 ;
uint8_t ValidApp =0;

void setup() { 
    
 UART2_Init();

 Wifi_Connect();
 Server_Connect();
 /* Test the Bootloader is Active and make a response */
//  BL_UART_Send_Host_Command(CBL_GET_VER_CMD);

   Active = get_Active_Bank_no();//0x66 cmd
  if (Active == 1) {
      Serial.println("Bank2 is Active Bank");
     
  } else if (Active == 0) {
      Serial.println("Bank1 is Active Bank");
     
  } else {
      Serial.println("Unknown error or Timeout");
  }
}

void loop() {
  // Check for update periodically
  if (millis() - PrevMillis > 5000) { // Every 5 seconds
    PrevMillis = millis();
    bool isUpdate = UpdateCheck(); // Checks if a new firmware version is available

  
    // If an update is detected or STM32 is in bootloader mode, proceed with flashing
    if (isUpdate ) {
       writeState             = true;
      Server_Download("file.bin");
      debugln("Read The File to Serial Monitor");
      readFile("/fileupdate.bin");
      debugln("Send The File to STM32");
      // Proceed with flashing process
      if (writeState) {

       Active = get_Active_Bank_no();//0x66 cmd
         if (Active == 1) {
         Serial.println("Bank2 is Active Bank");
            BL_UART_Send_Erase_Command(0, 3); // Example erase command
        } 

        else if (Active == 0) {
        Serial.println("Bank1 is Active Bank");
            BL_UART_Send_Erase_Command(12, 3); // Example erase command
       } 

        writeStatus = BL_UART_Send_Write_Command(APP1_BASE_ADDRESS);
        if (SUCCESS == writeStatus) {
          debugln("Write process succeeded, verifiy application is completed");

         
           ValidApp =  CheckValidityMarker();
           if (ValidApp ==0x01)
           {
             Serial.println("Valid App");
             SWAB_BANK();//swap bank
           }
           else 
           {
             Serial.println("not valid app  ");
              
             writeState             = true;
           }
    



          // writeState = false; // Reset the write state for future updates



        } else {
          debugln("Write process failed");
        }
       isUpdate = false; // Reset update flag
      }
    }
  }
}
