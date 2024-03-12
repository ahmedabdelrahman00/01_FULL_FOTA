/**
  ******************************************************************************
  * @file    bootloader.c
  * @brief   This file provides code for the configuration
  *          of the BOOTLOADER instances.
  ******************************************************************************
  **/

/* Includes ------------------------------------------------------------------*/
#include "bootloader.h"

/* Helper API Declarations ---------------------------------------------------*/
static void Bootloader_Get_Version											(uint8_t *Host_Buffer);
static void Bootloader_Get_Help													(uint8_t *Host_Buffer);
static void Bootloader_Get_Chip_Identification_Number		(uint8_t *Host_Buffer);
static void Bootloader_Read_Protection_Level						(uint8_t *Host_Buffer);
static void Bootloader_Jump_To_Address									(uint8_t *Host_Buffer);
static void Bootloader_Erase_Flash											(uint8_t *Host_Buffer);
static void Bootloader_Memory_Write											(uint8_t *Host_Buffer);
/* Added in future bootloader versions */
static void Bootloader_Enable_RW_Protection							(uint8_t *Host_Buffer);
static void Bootloader_Memory_Read											(uint8_t *Host_Buffer);
static void Bootloader_Get_Sector_Protection_Status			(uint8_t *Host_Buffer);
static void Bootloader_Read_OTP													(uint8_t *Host_Buffer);
static void Bootloader_Change_Read_Protection_Level  		(uint8_t *Host_Buffer);


static uint8_t     Bootloader_CRC_Verify          (uint8_t *pData, uint32_t Data_Len, uint32_t Host_CRC);
static uint8_t     Calculate_CRC32                (uint8_t* Buffer, uint32_t Buffer_Length, uint32_t Host_CRC);
static inline void Bootloader_Send_Data_To_Host   (uint8_t *Host_Buffer, uint32_t Data_Len);
static inline void Bootloader_Send_ACK					  (uint8_t Replay_Len);
static inline void Bootloader_Send_NACK				    (void);
static void        Bootloader_Jump_To_UserApp     (uint8_t *Host_Buffer);
static uint8_t     Host_Address_Verification      (uint32_t Jump_Address);
static uint8_t     Perform_Flash_Erase            (uint8_t Sector_Numebr, uint8_t Number_Of_Sectors);
static uint8_t     Flash_Memory_Write_Payload     (uint8_t *Host_Payload, uint32_t Payload_Start_Address, 
	                                                 uint16_t Payload_Len);
static inline uint8_t CBL_STM32F_Get_RDP_Level    (void);
/* Global Definitions --------------------------------------------------------*/
/* To prevent the compiler from optimizing out the Bootloader_Jump_To_UserApp() */
typedef void (*Bootloader_Jump_To_UserApp_Ptr)(void);
uint8_t BL_Host_Buffer [BL_HOST_BUFFER_RX_LENGTH];
static uint8_t Bootloader_Supported_CMDs[13] = {
    CBL_GET_VER_CMD,
    CBL_GET_HELP_CMD,
    CBL_GET_CID_CMD,
    CBL_GET_RDP_STATUS_CMD,
    CBL_GO_TO_ADDR_CMD,
    CBL_FLASH_ERASE_CMD,
    CBL_MEM_WRITE_CMD,
	  CBL_GO_TO_USER_APP,
    CBL_EN_RW_PROTECT_CMD,
    CBL_MEM_READ_CMD,
    CBL_READ_SECTOR_STATUS_CMD,
    CBL_OTP_READ_CMD,
    CBL_CHANGE_ROP_Level_CMD
};

/* API Definitions -----------------------------------------------------------*/
void BL_Print_Message(char *format, ...){
	char Messsage[100] = {0};
	/* holds the information needed by va_start, va_arg, va_end */
	va_list args;
	/* Enables access to the variable arguments */
	va_start(args, format);
	/* Write formatted data from variable argument list to string */
	vsprintf(Messsage, format, args);
#if (BL_DEBUG_METHOD == BL_ENABLE_UART_DEBUG_MESSAGE)
	/* Trasmit the formatted data through the defined UART */
	HAL_UART_Transmit(BL_DEBUG_UART_PC, (uint8_t *)Messsage, sizeof(Messsage), HAL_MAX_DELAY);
#elif (BL_DEBUG_METHOD == BL_ENABLE_SPI_DEBUG_MESSAGE)
	/* Trasmit the formatted data through the defined SPI */
#elif (BL_DEBUG_METHOD == BL_ENABLE_CAN_DEBUG_MESSAGE)
	/* Trasmit the formatted data through the defined CAN */
#endif
	/* Performs cleanup for an ap object initialized by a call to va_start */
	va_end(args);
}

BL_Status BL_UART_Fetch_Host_Command (void){
	/* Local Definations Scope */
	BL_Status Status = BL_OK;
	HAL_StatusTypeDef HAL_Status = HAL_ERROR;
	uint16_t Data_length = 0;
	/* Clear BL_Host_Buffer */
	memset(BL_Host_Buffer, 0, BL_HOST_BUFFER_RX_LENGTH);
	/* Read the length of the command packet received from the HOST */
	HAL_Status = HAL_UART_Receive(&huart3, BL_Host_Buffer, 1, HAL_MAX_DELAY);
	if (HAL_Status != HAL_OK){
		  Status = BL_NOT_OK;
	}
	else{
		Data_length = BL_Host_Buffer[0];
		HAL_Status = HAL_UART_Receive(&huart3, &BL_Host_Buffer[1], Data_length, HAL_MAX_DELAY);
		if (HAL_Status != HAL_OK){
		  Status = BL_NOT_OK;
	   }
		else{	
			switch (BL_Host_Buffer[1]){
				case CBL_GET_VER_CMD:
					BL_Print_Message("Gets the protocol version \r\n");
					Bootloader_Get_Version(BL_Host_Buffer);
					Status = BL_OK;
					break;
				case CBL_GET_HELP_CMD:
					BL_Print_Message("Gets help with Commands ID \r\n");
					Bootloader_Get_Help(BL_Host_Buffer);
					Status = BL_OK;
					break;
				case CBL_GET_CID_CMD:
					BL_Print_Message("Gets the chip ID \r\n");
					Bootloader_Get_Chip_Identification_Number(BL_Host_Buffer);
					Status = BL_OK;
					break;
				case CBL_GET_RDP_STATUS_CMD:
					BL_Print_Message("Read protection level \r\n");
					Bootloader_Read_Protection_Level(BL_Host_Buffer);
					Status = BL_OK;
					break;
				case CBL_GO_TO_ADDR_CMD:
					BL_Print_Message("Jumps to specific address located in the flash memory \r\n");
					Bootloader_Jump_To_Address(BL_Host_Buffer);
					Status = BL_OK;
					break;
				case CBL_FLASH_ERASE_CMD:
					BL_Print_Message("Erases some pages in flash memory \r\n");
				    Bootloader_Erase_Flash(BL_Host_Buffer);
					Status = BL_OK;
					break;
				case CBL_MEM_WRITE_CMD:
					BL_Print_Message("Write bytes in Flash Memory from address specified by the application \r\n");
					Bootloader_Memory_Write(BL_Host_Buffer);
					Status = BL_OK;
					break;
				case CBL_GO_TO_USER_APP:
					BL_Print_Message("Jumps to user application code located in the flash memory \r\n");
					Bootloader_Jump_To_UserApp(BL_Host_Buffer);
					Status = BL_OK;
					break;
				case CBL_EN_RW_PROTECT_CMD:
					BL_Print_Message("Enable or Disable write protect on different sectors of the user flash \r\n");
					Bootloader_Enable_RW_Protection(BL_Host_Buffer);
					Status = BL_OK;
					break;
				case CBL_MEM_READ_CMD:
					BL_Print_Message("Read data from different memories of the microcontroller \r\n");
					Bootloader_Memory_Read(BL_Host_Buffer);
					Status = BL_OK;
					break;
				case CBL_READ_SECTOR_STATUS_CMD:
					BL_Print_Message("Read all the sector protection status \r\n");
					Bootloader_Get_Sector_Protection_Status(BL_Host_Buffer);
					Status = BL_OK;
					break;
				case CBL_OTP_READ_CMD:
					BL_Print_Message("Read the OTP contents \r\n");
					Bootloader_Read_OTP(BL_Host_Buffer);
					Status = BL_OK;
					break;
				case CBL_CHANGE_ROP_Level_CMD:
					BL_Print_Message("Read the ROP Level \r\n");
					Bootloader_Change_Read_Protection_Level(BL_Host_Buffer);
					Status = BL_OK;
					break;
				default:
					BL_Print_Message("Invalid command code received from host !! \r\n");
					break;
			}
		}
	}
  return Status;
}

/*************************************************************************************
  * @brief  Send Data To Host.
  * @param  Host_Buffer : Data.
  * @param  Data_Len    : Data Length.
  * @retval no return.
**************************************************************************************/

 static inline void Bootloader_Send_Data_To_Host (uint8_t *Host_Buffer, uint32_t Data_Len){
	HAL_UART_Transmit(&huart3, Host_Buffer, Data_Len, HAL_MAX_DELAY);
}

/*************************************************************************************
  * @brief  Send ACK.
  * @param  no params.
  * @retval no return.
**************************************************************************************/

 static inline void Bootloader_Send_ACK (uint8_t Replay_Len){
	uint8_t Ack_Info[2] = {0, 0};
	Ack_Info[0] = CBL_SEND_ACK;
	Ack_Info[1] = Replay_Len; /* Length of the following packet (Response) from STM32 to the Host*/
	Bootloader_Send_Data_To_Host(Ack_Info, 2);
}

/*************************************************************************************
  * @brief  Send NACK.
  * @param  no params.
  * @retval no return.
**************************************************************************************/

static inline void Bootloader_Send_NACK	(void){
	uint8_t Ack_Info = CBL_SEND_NACK;
	Bootloader_Send_Data_To_Host(&Ack_Info, 1);
}

/*************************************************************************************
  * @brief  CRC Verification.
  * @param  *pData   : Pointer to Verified Data.
  * @param  Data_Len : Data Length.
  * @param  Host_CRC : CRC Delivered by Host.
  * @retval CRC_Status.
**************************************************************************************/
static uint8_t Bootloader_CRC_Verify (uint8_t *pData, uint32_t Data_Len, uint32_t Host_CRC){
	uint8_t CRC_Status = CRC_VERIFICATION_FAILED;
	uint32_t MCU_CRC_Calculated = 0;
	uint8_t Data_Counter = 0;
	uint32_t Data_Buffer = 0;
	for (Data_Counter = 0; Data_Counter < Data_Len; Data_Counter++){
		Data_Buffer = (uint32_t)(pData[Data_Counter]);
		MCU_CRC_Calculated = HAL_CRC_Accumulate(CRC_ENGINE_OBJ, &Data_Buffer, 1);
	}
	/* Reset the CRC Calculation Unit */
  __HAL_CRC_DR_RESET(CRC_ENGINE_OBJ);
	/* Compare the Host CRC and Calculated CRC */
	if(MCU_CRC_Calculated == Host_CRC){
		CRC_Status = CRC_VERIFICATION_PASSED;
	}
	else{
		CRC_Status = CRC_VERIFICATION_FAILED;
	}
	return CRC_Status;
}
/* Custom CRC Verification with different Polynomial (0x08C71CC1) */
static uint8_t Calculate_CRC32 (uint8_t* Buffer, uint32_t Buffer_Length, uint32_t Host_CRC){
	uint8_t CRC_Status = CRC_VERIFICATION_FAILED;
	uint32_t MCU_CRC_Calculated = 0;
  uint32_t CRC_Value = 0xFFFFFFFF;
  for (uint32_t i = 0; i < Buffer_Length; i++){
      CRC_Value = CRC_Value ^ Buffer[i];
      for (uint32_t DataElemBitLen = 0; DataElemBitLen < 32; DataElemBitLen++){
          if(CRC_Value & 0x80000000){
              CRC_Value = (CRC_Value << 1) ^ 0x08C71CC1;
          } 
					else{
              CRC_Value = (CRC_Value << 1);
          }
      }
  }
	/* Compare the Host CRC and Calculated CRC */
	if(CRC_Value == Host_CRC){
		CRC_Status = CRC_VERIFICATION_PASSED;
	}
	else{
		CRC_Status = CRC_VERIFICATION_FAILED;
	}
	return CRC_Status;
}
/*************************************************************************************
  * @brief  Get Bootloader Version.
  * @param  Pointer to Host_Buffer (Command Frame).
  * @retval no return.
**************************************************************************************/

static void Bootloader_Get_Version	(uint8_t *Host_Buffer){
	/* Local Definations Scope */
	uint8_t BL_Version [4] = {CBL_VENDOR_ID, CBL_SW_MAJOR_VERSION, CBL_SW_MINOR_VERSION, CBL_SW_PATCH_VERSION};
	uint16_t Host_CMD_Packet_Len = 0;
  uint32_t Host_CRC32 = 0;
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("Read the bootloader version from the MCU \r\n");
#endif	
	/* CRC Verification */
	Host_CMD_Packet_Len = Host_Buffer[0] + 1;
	Host_CRC32 = *((uint32_t* )((Host_Buffer + Host_CMD_Packet_Len) - CRC_TYPE_SIZE_BYTE));	
	if (CRC_VERIFICATION_PASSED == 
		                     Calculate_CRC32 (Host_Buffer, Host_CMD_Packet_Len -  CRC_TYPE_SIZE_BYTE, Host_CRC32)){
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
		BL_Print_Message("CRC Verification Passed \r\n");
#endif										 
		Bootloader_Send_ACK(4);
		Bootloader_Send_Data_To_Host(BL_Version, 4);
													 
//    volatile Bootloader_Jump_To_UserApp_Ptr jumpToUserApp = Bootloader_Jump_To_UserApp;
//    jumpToUserApp();												 
	}
	else{
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	  BL_Print_Message("CRC Verification Failed \r\n");
#endif
	  Bootloader_Send_NACK();
	}	
}

/*************************************************************************************
  * @brief  Get Commands Help.
  * @param  Pointer to Host_Buffer (Command Frame).
  * @retval no return.
**************************************************************************************/

static void Bootloader_Get_Help	(uint8_t *Host_Buffer){
  /* Local Definations Scope */
	uint16_t Host_CMD_Packet_Len = 0;
  uint32_t Host_CRC32 = 0;
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("Read the commands supported by the bootloader \r\n");
#endif
	/* CRC Verification */
	Host_CMD_Packet_Len = Host_Buffer[0] + 1;
	Host_CRC32 = *((uint32_t* )((Host_Buffer + Host_CMD_Packet_Len) - CRC_TYPE_SIZE_BYTE));
	if (CRC_VERIFICATION_PASSED == 
		                     Calculate_CRC32(Host_Buffer, Host_CMD_Packet_Len - CRC_TYPE_SIZE_BYTE, Host_CRC32)){
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
		BL_Print_Message("CRC Verification Passed \r\n");
#endif
		Bootloader_Send_ACK(12);
		Bootloader_Send_Data_To_Host(Bootloader_Supported_CMDs, 12);
	}
	else{
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("CRC Verification Failed \r\n");
#endif
	Bootloader_Send_NACK();
	}	
}

/*************************************************************************************
  * @brief  Get Chip Identification Number.
  * @param  *pData   : Pointer to Host_Buffer (Command Frame).
  * @retval no return.
**************************************************************************************/

static void Bootloader_Get_Chip_Identification_Number	(uint8_t *Host_Buffer){
  /* Local Definations Scope */
	uint16_t Host_CMD_Packet_Len = 0;
  uint32_t Host_CRC32 = 0;
	uint16_t MCU_Identification_Number = 0;
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("Read the MCU chip identification number \r\n");
#endif
	/* CRC Verification */
	Host_CMD_Packet_Len = Host_Buffer[0] + 1;
	Host_CRC32 = *((uint32_t* )((Host_Buffer + Host_CMD_Packet_Len) - CRC_TYPE_SIZE_BYTE));
	if (CRC_VERIFICATION_PASSED == 
		                     Calculate_CRC32(Host_Buffer, Host_CMD_Packet_Len -  CRC_TYPE_SIZE_BYTE, Host_CRC32)){
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
		BL_Print_Message("CRC Verification Passed \r\n");
#endif
		MCU_Identification_Number = (uint16_t)((DBGMCU->IDCODE) & 0x00000FFF);
		Bootloader_Send_ACK(2);
		Bootloader_Send_Data_To_Host((uint8_t* )&MCU_Identification_Number, 2);
	}
	else{
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("CRC Verification Failed \r\n");
#endif
	Bootloader_Send_NACK();
	}	
}


/*************************************************************************************
  * @brief  Jump To User Application.
  * @param  no params.
  * @retval no return.
**************************************************************************************/
static void Bootloader_Jump_To_UserApp (uint8_t *Host_Buffer){
	uint16_t Host_CMD_Packet_Len = 0;
  uint32_t Host_CRC32 = 0;
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("Read the bootloader version from the MCU \r\n");
#endif	
	/* CRC Verification */
	Host_CMD_Packet_Len = Host_Buffer[0] + 1;
	Host_CRC32 = *((uint32_t* )((Host_Buffer + Host_CMD_Packet_Len) - CRC_TYPE_SIZE_BYTE));	
	if (CRC_VERIFICATION_PASSED == 
		                     Calculate_CRC32 (Host_Buffer, Host_CMD_Packet_Len -  CRC_TYPE_SIZE_BYTE, Host_CRC32)){
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
		BL_Print_Message("CRC Verification Passed \r\n");
#endif										 

		Bootloader_Send_ACK(1);

		Bootloader_Send_Data_To_Host((uint8_t*)CRC_VERIFICATION_PASSED, 1);										 
		/* ------> Jump to user Application <------ */
    // Read the MSP (Main Stack Pointer) value from the user application's vector table
		Bootloader_Jump_To_Application();
    uint32_t MSP_value   = *((volatile uint32_t* )0x08008000U);
	  // Read the reset handler address from the user application's vector table
	  uint32_t MainAppAdd  = *((volatile uint32_t* )(0x08008000U + 4));
	  // Define a function pointer to the reset handler address
	  pMainApp ResetHandler_Address = (pMainApp)MainAppAdd;
	  // Set the MSP to the value obtained from the user application's vector table
	  __set_MSP(MSP_value);
	  // Deinitialize the RCC (Reset and Clock Control) peripheral (Block any External Interrupts)
	  HAL_RCC_DeInit();
	  // Jump to the reset handler address in the user application
	  ResetHandler_Address();													 											 											 
	}
	else{
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	  BL_Print_Message("CRC Verification Failed \r\n");
#endif
	  Bootloader_Send_NACK();
	}	


}
void Bootloader_Jump_To_Application()
  {

	  //check_ValidityMarker();
	    void(*App_Reset_Handler)(void);

	  	uint32_t ResetHandlerAddress ;

	  	/*configure MSP of user APP by reading value form base address of sector2*/
	  	uint32_t Local_u32MSPval =*((volatile uint32_t * )(0x08008000));

	  	/*write the user MSP value inside into msp register*/
	  	__asm volatile("msr MSP,%0"::"r"(Local_u32MSPval));

	  	/*Get reset Handler Address of user app*/
	  	ResetHandlerAddress = *((volatile uint32_t * ) (0x08008000 + 4)) ;

	  	App_Reset_Handler=(void*)ResetHandlerAddress;

	  	/*jump to the user app handler */
	  	App_Reset_Handler();



  }
/*************************************************************************************
  * @brief  Host Address Verification.
  * @param  Jump_Address.
  * @retval Address_Verification.
**************************************************************************************/

static uint8_t Host_Address_Verification (uint32_t Jump_Address){
	uint8_t Address_Verification = ADDRESS_IS_INVALID;
	if((Jump_Address >= SRAM_BASE) && (Jump_Address <= STM32F103_SRAM_END)){
		  Address_Verification = ADDRESS_IS_VALID;
	}
	else if((Jump_Address >= FLASH_BASE) && (Jump_Address <= STM32F103_FLASH_END)){
		  Address_Verification = ADDRESS_IS_VALID;
	}
	else{
		  Address_Verification = ADDRESS_IS_INVALID;
	}
	return Address_Verification;
}	

/*************************************************************************************
  * @brief  Jump To Specific Address.
  * @param  *Host_Buffer: Pointer to Host_Buffer (Command Frame).
  * @retval no return.
**************************************************************************************/

static void Bootloader_Jump_To_Address (uint8_t *Host_Buffer){
  /* Local Definations Scope */
	uint16_t Host_CMD_Packet_Len = 0;
  uint32_t Host_CRC32 = 0;
	uint32_t HOST_Jump_Address = 0;
	uint8_t Address_Verification = ADDRESS_IS_INVALID;
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("Read the MCU chip identification number \r\n");
#endif
	/* CRC Verification */
	Host_CMD_Packet_Len = Host_Buffer[0] + 1;
	Host_CRC32 = *((uint32_t* )((Host_Buffer + Host_CMD_Packet_Len) - CRC_TYPE_SIZE_BYTE));
	if (CRC_VERIFICATION_PASSED == 
		                     Bootloader_CRC_Verify(Host_Buffer, Host_CMD_Packet_Len -  CRC_TYPE_SIZE_BYTE, Host_CRC32)){
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
		BL_Print_Message("CRC Verification Passed \r\n");
#endif
		Bootloader_Send_ACK(1);
		HOST_Jump_Address = *((uint32_t* )&Host_Buffer[2]);
		Address_Verification = Host_Address_Verification(HOST_Jump_Address);				
    if( ADDRESS_IS_VALID == Address_Verification ){
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
			BL_Print_Message("Address verification succeeded \r\n");
#endif
			/* Report address verification succeeded */
			Bootloader_Send_Data_To_Host((uint8_t *)&Address_Verification, 1);
			/* Prepare the address to jump (1 --> state for thumb architecture) */
			Jump_Ptr Jump_Address = (Jump_Ptr)(HOST_Jump_Address + 1);
			Jump_Address();
		}			
    else{
			/* Report address verification failed */
			Bootloader_Send_Data_To_Host((uint8_t *)&Address_Verification, 1);
		}
	}
	else{
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("CRC Verification Failed \r\n");
#endif
	Bootloader_Send_NACK();
	}	
}

/*************************************************************************************
  * @brief  Flash Erase.
  * @param  Sector_Numebr.
  * @param  Number_Of_Sectors.
  * @retval Validity_Status.
**************************************************************************************/

//static uint8_t Perform_Flash_Erase (uint8_t Page_Numebr, uint8_t Number_Of_Pages){
//	uint8_t Sector_Validity_Status = INVALID_PAGE_NUMBER;
//	uint8_t Validity_Status        = UNSUCCESSFUL_ERASE;
//	uint32_t Erase_Validity;
//  HAL_StatusTypeDef HAL_Status   = HAL_ERROR;
//	FLASH_EraseInitTypeDef pEraseInit;
//
//	if (CBL_FLASH_MASS_ERASE == Page_Numebr){
//#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
//	BL_Print_Message("Mass Erase Verified \r\n");
//#endif
//	    pEraseInit.TypeErase    = FLASH_TYPEERASE_MASSERASE;
//		  pEraseInit.Banks        = 1;
//		  pEraseInit.PageAddress  = CBL_USER_FLASH_START_ADDRESS;
//		  pEraseInit.NbPages      = CBL_FLASH_MAX_PAGES_NUMBER;
//
//		  HAL_FLASH_Unlock();
//		  HAL_FLASHEx_Erase(&pEraseInit, &Erase_Validity);
//		  HAL_FLASH_Lock();
//
//      if (CHECK_SUCCESSFUL_ERASE ==  Erase_Validity){
//#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
//	BL_Print_Message("Mass Erase Successed \r\n");
//#endif
//       Validity_Status = SUCCESSFUL_MASS_ERASE;
//			}
//      else{
//#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
//	BL_Print_Message("Mass Erase Failed \r\n");
//#endif
//       Validity_Status = UNSUCCESSFUL_MASS_ERASE;
//			}
//	}
//	else if (Page_Numebr <= (CBL_FLASH_MAX_PAGES_NUMBER - 1)){
//		if ((Number_Of_Pages >= CBL_FLASH_MAX_PAGES_NUMBER) ||
//			  (Number_Of_Pages > (CBL_FLASH_MAX_PAGES_NUMBER - Page_Numebr))){
//			     Number_Of_Pages = CBL_FLASH_MAX_PAGES_NUMBER - Page_Numebr;
//		}
//    pEraseInit.TypeErase    =  FLASH_TYPEERASE_PAGES;
//    pEraseInit.Banks        =  1;
//		pEraseInit.NbPages      =  Number_Of_Pages;
//		pEraseInit.PageAddress  =  CBL_USER_FLASH_START_ADDRESS + (0x400 * Page_Numebr);
//
//		HAL_FLASH_Unlock();
//		HAL_FLASHEx_Erase(&pEraseInit, &Erase_Validity);
//		HAL_FLASH_Lock();
//
//    if (CHECK_SUCCESSFUL_ERASE ==  Erase_Validity){
//#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
//	BL_Print_Message("Sectors Erase Successed \r\n");
//#endif
//       Validity_Status = SUCCESSFUL_ERASE;
//			}
//      else{
//#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
//	BL_Print_Message("Sectors Erase Failed \r\n");
//#endif
//       Validity_Status = UNSUCCESSFUL_ERASE;
//			}
//	}
//	else{
//#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
//	BL_Print_Message("Sectors Erase Failed \r\n");
//#endif
//    Validity_Status  = INVALID_PAGE_NUMBER;
//	}
//	return 	Validity_Status;
//}
//
#define INVALID_SECTOR            0x12
  uint8_t Perform_Flash_Erase(uint8_t initial_sector_number, uint8_t number_of_sector)
  {
  	//we have totally 12 sectors in one bank .. sector[0 to 11]
  	//number_of_sector has to be in the range of 0 to 11
  	// if sector_number = 0xff , that means mass erase !

  	FLASH_EraseInitTypeDef flashErase_handle;
  	uint32_t sectorError = 0;
  	uint8_t erase_status = 0x01;

  	if (number_of_sector > 23)
  		return (uint8_t) INVALID_SECTOR;

  	if ((initial_sector_number == 0xFFFFFFFF) || (number_of_sector <= 23)) {
  		if (number_of_sector == (uint32_t) 0xFFFFFFFF) {
  			flashErase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE;
  			flashErase_handle.Banks = FLASH_BANK_1;
  		} else {
  			/*Here we are just calculating how many sectors needs to erased */
  			uint32_t remanining_sector = 24 - number_of_sector;
  			if (number_of_sector > remanining_sector) {
  				number_of_sector = remanining_sector;
  			}
  			flashErase_handle.TypeErase = FLASH_TYPEERASE_SECTORS;
  			flashErase_handle.Sector = initial_sector_number; // this is the initial sector
  			flashErase_handle.NbSectors = number_of_sector;
  		}

  		/*Get access to touch the flash registers */
  		HAL_FLASH_Unlock();
  		flashErase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3; // our MCU will work on this voltage range
  		erase_status = (uint8_t) HAL_FLASHEx_Erase(&flashErase_handle,
  				&sectorError);
  		HAL_FLASH_Lock();

  		return (uint8_t) erase_status;
  	}

  	return (uint8_t) INVALID_SECTOR;
  }


static void Bootloader_Erase_Flash	(uint8_t *Host_Buffer){
   /* Local Definations Scope */
	uint16_t Host_CMD_Packet_Len = 0;
  uint32_t Host_CRC32 = 0;
	uint8_t Erase_Status = 0;
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("Read the MCU chip identification number \r\n");
#endif
	/* CRC Verification */
	Host_CMD_Packet_Len = Host_Buffer[0] + 1;
	Host_CRC32 = *((uint32_t* )((Host_Buffer + Host_CMD_Packet_Len) - CRC_TYPE_SIZE_BYTE));
	if (CRC_VERIFICATION_PASSED ==Calculate_CRC32(Host_Buffer, Host_CMD_Packet_Len -  CRC_TYPE_SIZE_BYTE, Host_CRC32)){
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
		BL_Print_Message("CRC Verification Passed \r\n");
#endif
		Perform_Flash_Erase(2,6);
		Bootloader_Send_ACK(1);
		Erase_Status = Perform_Flash_Erase(Host_Buffer[2], Host_Buffer[3]);
		Bootloader_Send_Data_To_Host((uint8_t *)&Erase_Status, 1);
/* Report address verification successed */
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
			BL_Print_Message("Erase Status: %d \r\n", Erase_Status);
#endif
  }
	else{
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("CRC Verification Failed \r\n");
#endif
	Bootloader_Send_NACK();
	}
}

/*************************************************************************************
  * @brief  Flash Write.
  * @param  *Host_Payload          :Pointer to the Data to be Written.
  * @param  *Payload_Start_Address :Data Start Address.
  * @param  *Payload_Len           :Data Length.
  * @param  Number_Of_Sectors.
  * @retval Flash_Payload_Write_Status.
**************************************************************************************/

uint8_t Flash_Memory_Write_Payload (uint8_t *Host_Payload, uint32_t Payload_Start_Address, uint16_t Payload_Len){
	HAL_StatusTypeDef HAL_Status = HAL_ERROR;
	uint8_t Flash_Payload_Write_Status = FLASH_PAYLOAD_WRITE_FAILED;
	uint16_t Payload_Counter = 0;
	uint32_t Address = 0;
	/* Unlock the FLASH control register access */
  HAL_Status = HAL_FLASH_Unlock();
	if(HAL_Status != HAL_OK){
		Flash_Payload_Write_Status = FLASH_PAYLOAD_WRITE_FAILED;
	}
	else{
	for (uint16_t Payload_Counter = 0; Payload_Counter < Payload_Len; Payload_Counter += 2) {
            /* Use uint16_t pointer to properly handle 16-bit data */
            uint16_t* dataToWrite = (uint16_t*)&Host_Payload[Payload_Counter];
            HAL_Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                                           Payload_Start_Address + Payload_Counter,
                                           *dataToWrite);
            if (HAL_Status != HAL_OK){
                Flash_Payload_Write_Status = FLASH_PAYLOAD_WRITE_FAILED;
                break;
            }
            else {
                Flash_Payload_Write_Status = FLASH_PAYLOAD_WRITE_PASSED;
            }
        }
  }
	
	if((FLASH_PAYLOAD_WRITE_PASSED == Flash_Payload_Write_Status) && (HAL_OK == HAL_Status)){
		/* Locks the FLASH control register access */
		HAL_Status = HAL_FLASH_Lock();
		if(HAL_Status != HAL_OK){
			Flash_Payload_Write_Status = FLASH_PAYLOAD_WRITE_FAILED;
		}
		else{
			Flash_Payload_Write_Status = FLASH_PAYLOAD_WRITE_PASSED;
		}
	}
	else{
		Flash_Payload_Write_Status = FLASH_PAYLOAD_WRITE_FAILED;
	}
	
	return Flash_Payload_Write_Status;
}

static void Bootloader_Memory_Write	(uint8_t *Host_Buffer){
 /* Local Definations Scope */
	uint16_t Host_CMD_Packet_Len = 0;
  uint32_t Host_CRC32 = 0;
  uint32_t HOST_Address = 0;
	uint8_t Address_Verification = ADDRESS_IS_INVALID;
	uint8_t Payload_Len = 0;
	uint8_t Flash_Payload_Write_Status = FLASH_PAYLOAD_WRITE_FAILED;
	
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("Write data into different memories of the MCU \r\n");
#endif
	/* CRC Verification */
	Host_CMD_Packet_Len = Host_Buffer[0] + 1;
	Host_CRC32 = *((uint32_t* )((Host_Buffer + Host_CMD_Packet_Len) - CRC_TYPE_SIZE_BYTE));
	if (CRC_VERIFICATION_PASSED == 
		                     Calculate_CRC32(Host_Buffer, Host_CMD_Packet_Len -  CRC_TYPE_SIZE_BYTE, Host_CRC32)){
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
		BL_Print_Message("CRC Verification Passed \r\n");
#endif
		Bootloader_Send_ACK(1);
		/* Extract the payload length from the Host packet */
		Payload_Len  = Host_Buffer[6];
		/* Extract the start address from the Host packet */
    HOST_Address = *((uint32_t *)(&Host_Buffer[2]));
    /* Verify the Extracted address to be valid address */
		Address_Verification = Host_Address_Verification(HOST_Address);
		if(ADDRESS_IS_VALID == Address_Verification){
         Flash_Payload_Write_Status = Flash_Memory_Write_Payload((uint8_t *)&Host_Buffer[7], HOST_Address, Payload_Len);
			   if(FLASH_PAYLOAD_WRITE_PASSED == Flash_Payload_Write_Status){
				 /* Report payload write passed */
				 Bootloader_Send_Data_To_Host((uint8_t *)&Flash_Payload_Write_Status, 1);
				 /* ------> Send Acknowledge <------ */
         Bootloader_Send_Data_To_Host((uint8_t* )CBL_SEND_ACK, 1);						 
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
				BL_Print_Message("Payload Valid \r\n");
#endif
			   }
			  else{
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
				BL_Print_Message("Payload InValid \r\n");
#endif
				/* Report payload write failed */
				Bootloader_Send_Data_To_Host((uint8_t *)&Flash_Payload_Write_Status, 1);
			  }
		}
    else{
			/* Report address verification failed */
			Address_Verification = ADDRESS_IS_INVALID;
			Bootloader_Send_Data_To_Host((uint8_t *)&Address_Verification, 1);
    }	
	}
	else{
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("CRC Verification Failed \r\n");
#endif
	Bootloader_Send_NACK();
	}	
}

/*************************************************************************************
  * @brief  Read Protection Level.
  * @param  no param
  * @retval Protection Level.
**************************************************************************************/

static inline uint8_t CBL_STM32F_Get_RDP_Level (void){
	FLASH_OBProgramInitTypeDef FLASH_OBProgram;
	/* Get the Option byte configuration */
	HAL_FLASHEx_OBGetConfig(&FLASH_OBProgram);
	
	return (uint8_t)(FLASH_OBProgram.RDPLevel);
}

static void Bootloader_Read_Protection_Level(uint8_t *Host_Buffer){
	uint16_t Host_CMD_Packet_Len = 0;
  uint32_t Host_CRC32 = 0;
	uint8_t RDP_Level = 0;
	
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("Read the FLASH Read Protection Out level \r\n");
#endif
	/* Extract the CRC32 and packet length sent by the HOST */
	Host_CMD_Packet_Len = Host_Buffer[0] + 1;
	Host_CRC32 = *((uint32_t *)((Host_Buffer + Host_CMD_Packet_Len) - CRC_TYPE_SIZE_BYTE));	
/* CRC Verification */
	if(CRC_VERIFICATION_PASSED == Calculate_CRC32((uint8_t *)&Host_Buffer[0] , Host_CMD_Packet_Len - 4, Host_CRC32)){
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
		BL_Print_Message("CRC Verification Passed \r\n");
#endif
		Bootloader_Send_ACK(1);
		/* Read Protection Level */
		RDP_Level = CBL_STM32F_Get_RDP_Level();
		/* Report Valid Protection Level */
		Bootloader_Send_Data_To_Host((uint8_t *)&RDP_Level, 1);
	}
	else{
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
		BL_Print_Message("CRC Verification Failed \r\n");
#endif
		Bootloader_Send_NACK();
	}
}

/* Additional Features ----------------------------------------------------------*/
static uint8_t Change_ROP_Level                             (uint32_t ROP_Level){}
static void    Bootloader_Change_Read_Protection_Level  		(uint8_t *Host_Buffer){}
static void    Bootloader_Enable_RW_Protection							(uint8_t *Host_Buffer){}
static void    Bootloader_Memory_Read										  	(uint8_t *Host_Buffer){}
static void    Bootloader_Get_Sector_Protection_Status			(uint8_t *Host_Buffer){}
static void    Bootloader_Read_OTP													(uint8_t *Host_Buffer){}
