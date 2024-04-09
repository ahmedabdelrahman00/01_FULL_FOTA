/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
SemaphoreHandle_t SimpleMutex;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId Task2Handle;
osThreadId Task3Handle;
osMutexId Mutex1Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//volatile uint8_t indecator_Var, dummy;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Task2_init(void const * argument);
void Task3_init(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of Mutex1 */
  osMutexDef(Mutex1);
  Mutex1Handle = osMutexCreate(osMutex(Mutex1));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, -1, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Task2 */
  osThreadDef(Task2, Task2_init, osPriorityNormal, 0, 128);
  Task2Handle = osThreadCreate(osThread(Task2), NULL);

  /* definition and creation of Task3 */
//  osThreadDef(Task3, Task3_init, osPriorityIdle, 0, 128);
//  Task3Handle = osThreadCreate(osThread(Task3), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */

volatile uint8_t indecator_Var, dummy;
volatile uint8_t RecVal;
volatile uint8_t FIRMWARE_UPDATE_TRIGGER = 0x7F;
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
   HAL_UART_Receive_IT(&huart1, &RecVal, 1);
   HAL_UART_Receive(&huart3, &indecator_Var, 1, HAL_MAX_DELAY);
  	    if (0x7F == indecator_Var)
  	     BL_UART_Fetch_Host_Command();
 	     osDelay(65);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Task2_init */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task2_init */
void Task2_init(void const * argument)
{
  /* USER CODE BEGIN Task2_init */
  /* Infinite loop */
  for(;;)
  {
      HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
//      HAL_Delay(100);
      osDelay(500);
  }
  /* USER CODE END Task2_init */
}

/* USER CODE BEGIN Header_Task3_init */
/**
* @brief Function implementing the Task3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task3_init */
void Task3_init(void const * argument)
{
  /* USER CODE BEGIN Task3_init */
  /* Infinite loop */
  for(;;)
  {
    //osDelay(1);
  }
  /* USER CODE END Task3_init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//    UNUSED(huart);



    	HAL_UART_Receive_IT(&huart1, &RecVal, 1);
    			    if (FIRMWARE_UPDATE_TRIGGER == RecVal)
    			    {
    						  /* Send Acknowledge */
    						  uint8_t ackValue = 0xCD;
    						  HAL_UART_Transmit(&huart1, &ackValue, 1, HAL_MAX_DELAY);
    						  /* System reset (Jump to Bootloader) */
    						  HAL_NVIC_SystemReset();
    				 }

    			    if (RecVal == 0x66)
    			    { // Example command byte from ESP
    			   	            // get active bank here..
    			   	            uint8_t mode = get_Active_Bank_no();
    			   	            HAL_UART_Transmit(&huart1, &mode, 1, 10); // Send  active bank
    			   	 }

    			    if (RecVal == 0x8C)
    			     {
    			    	toggleBankAndReset();
    			     }

    			    if (RecVal == 0x4C)
    			     {
    			    	/*  check validity marker  */
    			    	check_ValidityMarker();
    			     }


}
/* USER CODE END Application */
