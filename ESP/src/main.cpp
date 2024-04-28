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
  if (Active == 1)
  {
      Serial.println("Bank2");
     
  }
   else if (Active == 0)
  {
      Serial.println("Bank1");
  }
   else
  {
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
      // Server_Download("file2.bin");
      Server_Download("file3.bin");

      
      debugln("Read The File to Serial Monitor");
      readFile("/fileupdate.bin");
      debugln("Send The File to STM32");
      // Proceed with flashing process
      if (writeState) {

       Active = get_Active_Bank_no();//0x66 cmd
         if (Active == 1) {
         Serial.println("Bank2");
            BL_UART_Send_Erase_Command(0, 3); //  erase command (bank1)
        } 

        else if (Active == 0) {
        Serial.println("Bank1");
            BL_UART_Send_Erase_Command(12, 3); //  erase command (bank2)
       } 

        writeStatus = BL_UART_Send_Write_Command(APP1_BASE_ADDRESS);
        if (SUCCESS == writeStatus) {
          debugln("Write process succeeded, verifiy application is completed");


/////////////////////////////////////
ValidApp  = CheckValidityMarker();

 if (ValidApp == 0x01)
  {
      Serial.println("the app is valid");
         Display_choose_swaping();//swap bank
     
  }
   else if (ValidApp == 0x02)
  {
      Serial.println("the app is not valid ");
          writeState = true; // Reset the write state for future updates
           isUpdate = true; // Reset update flag
  }
  

//////////////////////////////////////
          // SWAB_BANK();//swap bank
          writeState = false; // Reset the write state for future updates
        

        } else {
          debugln("Write process failed");
        }
       isUpdate = false; // Reset update flag
      }
    }
  }
}
