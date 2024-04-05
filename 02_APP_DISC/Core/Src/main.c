/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
uint8_t tx_buffer[27]="HELLO FROM app\n\r";

uint8_t rx_index;
uint8_t rx_data[2];
uint8_t rx_buffer[100];
uint8_t transfer_cplt;
uint8_t  ahmed = 39 ;
uint8_t Host_Buffer2[100];

//uint32_t Programming_Marker __attribute__((section(".Global_Marker")));
//uint32_t Validity_Marker __attribute__((section(".marker_data")))=0xA0A0;
volatile uint8_t RecVal;
volatile uint8_t FIRMWARE_UPDATE_TRIGGER = 0x7F;
unsigned char dummyBuffer [4];

int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}


uint32_t get_Active_Bank_no(void) {
	uint32_t checkBank = READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_UFB_MODE);
	if (checkBank == 0) {
		 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, 1);
	} else {
		 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 1);
	}
	return checkBank;
}
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();


  /* USER CODE BEGIN 2 */
  //toggleBankAndReset();
//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 1);
//  HAL_UART_Transmit(&huart3, (uint8_t *)"hello starting\n\r", 25, HAL_MAX_DELAY);
HAL_UART_Receive_IT(&huart3, &RecVal, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
  while (1)
  {
    /* USER CODE END WHILE */
	//  HAL_UART_Transmit(&huart3, (uint8_t *)"hello starting\n\r", 25, HAL_MAX_DELAY);

	 HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
	// HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	  HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14);

	  HAL_Delay(1000);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//    UNUSED(huart);
//    uint8_t i;
//
//    if (huart->Instance == USART3) {
//        if (rx_index == 0) {
//            for (i = 0; i < 100; i++)
//                rx_buffer[i] = 0;
//        }
//
//        if (rx_data[0] != 13) {
//            if (rx_data[0] == 'C') {
//                // Clear the buffer when 'C' is received
//                rx_index = 0;
//            } else {
//                rx_buffer[rx_index++] = rx_data[0];
//            }
//        } else {
//            rx_index = 0;
//            transfer_cplt = 1;
//            HAL_UART_Transmit(&huart3 ,"\n\r", 2, 100);
//
//            // Compare ignoring case
//            if (strncasecmp(rx_buffer, "LED ON", strlen("LED ON")) == 0) {
//                HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
//            } else if (strncasecmp(rx_buffer, "UPDATE", strlen("UPDATE")) == 0) {
//
//                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
//                Programming_Marker =1;
//                NVIC_SystemReset();
//            }
//        }
//
//        HAL_UART_Receive_IT(&huart3 ,rx_data, 1);
//        HAL_UART_Transmit(&huart3, rx_data, 1, 100);
//    }
	HAL_UART_Receive_IT(&huart3, &RecVal, 1);
		    if (FIRMWARE_UPDATE_TRIGGER == RecVal){
					  /* Send Acknowledge */
					  uint8_t ackValue = 0xCD;
					  HAL_UART_Transmit(&huart3, &ackValue, 1, HAL_MAX_DELAY);
					  /* System reset (Jump to Bootloader) */
					  HAL_NVIC_SystemReset();
			  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
