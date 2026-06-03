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
#include "lmic.h"
#include "hal.h"


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
extern void radio_irq_handler(u1_t dio);
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

/* USER CODE BEGIN Header_StartLoraTask */
/**
  * @brief Function implementing the loraTask thread.
  * @param argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLoraTask */
// Change "StartLoraTask" to "StartLoRaTask" (Capital O and Capital R)
void StartLoRaTask(void *argument) 
{
  /* USER CODE BEGIN StartLoRaTask */
  
  LMIC_startJoining();
  LMIC_setDrTxpow(DR_SF7, 14);
  LMIC_setAdrMode(0);

  /* Infinite loop */
  for(;;)
  {
    extern void lmic_hal_processPendingIRQs(void);
    lmic_hal_processPendingIRQs();

    os_runloop_once();

    osDelay(2); 
  }
  /* USER CODE END StartLoRaTask */
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    
}


void onEvent (ev_t ev) {
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            HAL_UART_Transmit(&huart1, (uint8_t*)"[LMIC Event] Scan Timeout\r\n", 27, 100);
            break;
        case EV_BEACON_FOUND:
            HAL_UART_Transmit(&huart1, (uint8_t*)"[LMIC Event] Beacon Found\r\n", 27, 100);
            break;
        case EV_BEACON_MISSED:
            HAL_UART_Transmit(&huart1, (uint8_t*)"[LMIC Event] Beacon Missed\r\n", 28, 100);
            break;
        case EV_BEACON_TRACKED:
            HAL_UART_Transmit(&huart1, (uint8_t*)"[LMIC Event] Beacon Tracked\r\n", 29, 100);
            break;
        case EV_JOINING:
            HAL_UART_Transmit(&huart1, (uint8_t*)"[LMIC Event] Event 17: EV_JOINING (Looking for gateway...)\r\n", 60, 100);
            break;
        case EV_JOINED:
            HAL_UART_Transmit(&huart1, (uint8_t*)"[LMIC Event] EV_JOINED (Connected to network!)\r\n", 48, 100);
            break;
        case EV_JOIN_FAILED:
            HAL_UART_Transmit(&huart1, (uint8_t*)"[LMIC Event] Join Failed\r\n", 26, 100);
            break;
        case EV_REJOIN_FAILED:
            HAL_UART_Transmit(&huart1, (uint8_t*)"[LMIC Event] Rejoin Failed\r\n", 28, 100);
            break;
        case EV_TXCOMPLETE:
            HAL_UART_Transmit(&huart1, (uint8_t*)"[LMIC Event] Transmission Complete (RX Windows Open)\r\n", 54, 100);
            break;
        case EV_RESET:
            HAL_UART_Transmit(&huart1, (uint8_t*)"[LMIC Event] Radio Reset\r\n", 26, 100);
            break;
        case EV_LINK_DEAD:
            HAL_UART_Transmit(&huart1, (uint8_t*)"[LMIC Event] Link Dead\r\n", 24, 100);
            break;
        case EV_LINK_ALIVE:
            HAL_UART_Transmit(&huart1, (uint8_t*)"[LMIC Event] Link Alive\r\n", 25, 100);
            break;
        case EV_TXSTART:
            HAL_UART_Transmit(&huart1, (uint8_t*)"[LMIC Event] Event 20: EV_TXSTART (Uplink/Join Frame Airbound)\r\n", 64, 100);
            break;
        case EV_RXSTART:
            HAL_UART_Transmit(&huart1, (uint8_t*)"[LMIC Event] EV_RXSTART (Downlink Window Active)\r\n", 50, 100);
            break;

        default:
            // FIXED: Cleaned up code to explicitly print any unknown codes safely
            {
                char log_buffer[50];
                int len = snprintf(log_buffer, sizeof(log_buffer), "[LMIC Event] Unmapped Code: %d\r\n", (int)ev);
                HAL_UART_Transmit(&huart1, (uint8_t*)log_buffer, len, 100);
            }
            break;
    }
}

/* USER CODE END Application */

