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
 BL_UART_Send_Host_Command(CBL_GET_VER_CMD);
}

void loop() {
  if (millis() - PrevMillis > 5000){
      PrevMillis = millis();
      bool isUpdate = UpdateCheck();
      if (isUpdate){
         Server_Download("file.bin");
         debugln("Read The File to Serial Monitor");
         readFile("/fileupdate.bin");
         debugln("Send The File to STM32f");
         if (APPLICATION_MODE == stateMode){
            /* Send Jump to Bootloader Command */
            debugln("Send Jump to Bootloader Command");
            bootloaderJumpStatues = BL_UART_Send_BootloaderJump_Command();
            if (bootloaderJumpStatues){
                debugln("wait seconds ....");
                delay(4000);
                /* Send Erase Command */
                debugln("Send Erase Command");
                BL_UART_Send_Erase_Command(2, 6);
                writeState = true;
            }
         }
         if (writeState){
            writeStatus = BL_UART_Send_Write_Command(APP1_BASE_ADDRESS);
            if (SUCCESS == writeStatus){
               /* Jump to User Application */
               debugln("\n\nWrite process successed, then Jumping to Application");
               BL_UART_Send_AppJump_Command();
               stateMode = APPLICATION_MODE;
               writeState = false;
            }
            else{
               /* faild while write data to STM32 */
               debugln("Write process faild");
            }
            debugln();
            isUpdate = false;
         }
      }
   }
}

