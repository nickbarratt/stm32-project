/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"  // Ensures types like UART_HandleTypeDef are known
#include <string.h> // For strlen() if you're using it in serial tasks

#include <stdio.h>  // Fixes the printf error

// Link hardware handles from main.c
extern TIM_HandleTypeDef htim1;  
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;

// Link your logic variables from main.c
extern uint32_t lastBlinkTime;
extern uint8_t  ledIsOn;
extern uint32_t duty_cycle;
extern uint8_t  k0_last_state;
extern uint8_t  k1_last_state;
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
/* USER CODE BEGIN Variables */
extern SPI_HandleTypeDef hspi2;
#define RF95_REG_FIFO 0x00
#define RF95_REG_OP_MODE 0x01
#define RF95_MODE_TX 0x03
/* USER CODE END Variables */
/* Definitions for MainLogicTask */
osThreadId_t MainLogicTaskHandle;
const osThreadAttr_t MainLogicTask_attributes = {
  .name = "MainLogicTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SerialTask */
osThreadId_t SerialTaskHandle;
const osThreadAttr_t SerialTask_attributes = {
  .name = "SerialTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for LoRaTask */
osThreadId_t LoRaTaskHandle;
const osThreadAttr_t LoRaTask_attributes = {
  .name = "LoRaTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMainLogicTask(void *argument);
void StartSerialTask(void *argument);
void StartLoRaTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  /* creation of MainLogicTask */
  MainLogicTaskHandle = osThreadNew(StartMainLogicTask, NULL, &MainLogicTask_attributes);

  /* creation of SerialTask */
  SerialTaskHandle = osThreadNew(StartSerialTask, NULL, &SerialTask_attributes);

  /* creation of LoRaTask */
  LoRaTaskHandle = osThreadNew(StartLoRaTask, NULL, &LoRaTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMainLogicTask */
/**
  * @brief  Function implementing the MainLogicTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainLogicTask */
void StartMainLogicTask(void *argument)
{
  /* USER CODE BEGIN StartMainLogicTask */
  /* Infinite loop */
  for(;;)
  {
  	// 1. BLINK LOGIC (Non-blocking)
    if (HAL_GetTick() - lastBlinkTime >= 500) 
    {
        lastBlinkTime = HAL_GetTick();
        ledIsOn = !ledIsOn; // Flip the state

        if (ledIsOn) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_cycle);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        } else {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty_cycle);
        }
    }
  	
  	// Check K0: Increase Duty
    uint8_t k0_current = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
    if (k0_current == 0 && k0_last_state == 1) {
        if (duty_cycle <= 950) duty_cycle += 50; 
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle); // updates PE9
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_cycle); // Updates Onboard LED
        printf("Duty Increased: %lu\r\n", duty_cycle);
        osDelay(50); // Debounce
    }
    k0_last_state = k0_current;

    // Check K1: Decrease Duty
    uint8_t k1_current = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);
    if (k1_current == 0 && k1_last_state == 1) {
        if (duty_cycle >= 50) duty_cycle -= 50; 
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle); // updates PE9
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_cycle); // Updates Onboard LED        
        printf("Duty Decreased: %lu\r\n", duty_cycle);
        osDelay(50); // Debounce
    }
    k1_last_state = k1_current;
    
    
    osDelay(10);
  }
  /* USER CODE END StartMainLogicTask */
}

/* USER CODE BEGIN Header_StartSerialTask */
/**
* @brief Function implementing the SerialTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSerialTask */
void StartSerialTask(void *argument)
{
  /* USER CODE BEGIN StartSerialTask */
  /* Infinite loop */
  for(;;)
  {
    char msg[] = "RTOS Heartbeat: System Stable\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    osDelay(2000); // Send a message every 2 seconds
  }
  /* USER CODE END StartSerialTask */
}

/* USER CODE BEGIN Header_StartLoRaTask */
/**
* @brief Function implementing the LoRaTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLoRaTask */
void StartLoRaTask(void *argument)
{
  /* USER CODE BEGIN StartLoRaTask */
  
    // 1. Reset Radio
  HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_RESET);
  osDelay(10);
  HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_SET);
  
  /* Infinite loop */
  for(;;)
  {
  	// Range Test Ping logic will go here tomorrow
    osDelay(5000);
  }
  /* USER CODE END StartLoRaTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Lora_WriteReg(uint8_t addr, uint8_t val) {
    uint8_t data[2] = { (uint8_t)(addr | 0x80), val }; // Cast and format for SPI
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, data, 2, 100);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}
/* USER CODE END Application */

