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
#include "lmic.h"  // 
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

#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_PAYLOAD_LENGTH       0x22
#define REG_DIO_MAPPING_1        0x40

// Mode bits
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03

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
void Lora_WriteReg(uint8_t addr, uint8_t val);
uint8_t Lora_ReadReg(uint8_t addr);
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
  /* USER CODE BEGIN StartLoraTask */
  // Expose the global functions and tracking variables instantiated in main.c
  extern void os_init(void);
  extern void LMIC_reset(void);
  extern void do_send(void* job);
  extern osjob_t txJob;

  // 1. Initialize the internal LMIC clock and scheduler metrics
  os_init();    
    
  // 2. Clear out tracking parameters and prep the SX1276 chip registers
  LMIC_reset(); 
    
  // 3. Queue up the initial network handshake join sequence pass
  do_send(&txJob);

  /* Infinite processing loop executed concurrently by FreeRTOS */
  for(;;)
  {
    // Continuously advance the LoRaWAN execution engine state machine
    os_runloop_once(); 
        
    // Yield 2 milliseconds to protect your other task (StartSerialTask) from starving
    osDelay(2);        
  }
  /* USER CODE END StartLoraTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// Write to a register
void Lora_WriteReg(uint8_t addr, uint8_t val) {
    uint8_t data[2] = { (uint8_t)(addr | 0x80), val }; // Cast and format for SPI
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, data, 2, 100);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}

// Read from a register
uint8_t Lora_ReadReg(uint8_t addr) {
    uint8_t addr_byte = addr & 0x7F; // MSB low for Read
    uint8_t val = 0;
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &addr_byte, 1, 100);
    HAL_SPI_Receive(&hspi2, &val, 1, 100);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
    return val;
}

// This function automatically fires the exact millisecond a packet leaves the antenna
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == LORA_DIO0_Pin) // Check if the interrupt came from PE0
  {
    // Clear the radio's internal IRQ flags so it can transmit again next time
    // Register 0x12 is RegIrqFlags. Writing 0x08 clears the TxDone flag.
    Lora_WriteReg(0x12, 0x08);
    
    // Print immediate confirmation to your serial monitor
    HAL_UART_Transmit(&huart1, (uint8_t*)"[LoRa] -> TXDone Interrupt Received! Packet is in the air.\r\n", 60, 10);
  }
}

/* USER CODE END Application */

