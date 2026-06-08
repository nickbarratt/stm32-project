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
#include <string.h> // For strlen() and standard string handling
#include <stdio.h>  // For sprintf() formatting tracking utilities

// Hardware Handle Linkage from main.c
extern TIM_HandleTypeDef htim1;  
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi2;

// Shared Hardware State Variables from main.c
extern uint32_t lastBlinkTime;
extern uint8_t  ledIsOn;
extern uint32_t duty_cycle;
extern uint8_t  k0_last_state;
extern uint8_t  k1_last_state;

// Single Source-of-Truth LoRaWAN Frame Counter Variable
extern uint16_t frame_counter;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// LoRa RF95 Transceiver Register Map Definitions
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

// Radio Functional Operational Modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03

// Legacy Alias Support
#define RF95_REG_FIFO            REG_FIFO
#define RF95_REG_OP_MODE         REG_OP_MODE
#define RF95_MODE_TX             MODE_TX
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for MainLogicTask */
osThreadId_t MainLogicTaskHandle;
const osThreadAttr_t MainLogicTask_attributes = {
  .name = "MainLogicTask",
  .stack_size = 1536 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SerialTask */
osThreadId_t SerialTaskHandle;
const osThreadAttr_t SerialTask_attributes = {
  .name = "SerialTask",
  .stack_size = 1536 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for LoRaTask */
osThreadId_t LoRaTaskHandle;
const osThreadAttr_t LoRaTask_attributes = {
  .name = "LoRaTask",
  .stack_size = 1536 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// Core Radio Driver Commands
void Lora_WriteReg(uint8_t addr, uint8_t val);
uint8_t Lora_ReadReg(uint8_t addr);
void Lora_ReadFIFOVerify(uint8_t *buffer, uint8_t length);

// Wear-Leveled Flash Counter Module Drivers
uint16_t Load_Frame_Counter_Wear_Leveled(void);
void Save_Frame_Counter_Wear_Leveled(uint16_t counter);

// LoRaWAN Cryptographic Libraries Linkage
extern void aes_encrypt_block(const uint8_t *key, const uint8_t *input, uint8_t *output);
extern void calculate_lorawan_mic(uint8_t *packet, uint8_t payload_len, const uint8_t *nwkSKey, uint8_t *mic_out);
extern void encrypt_lorawan_payload(uint8_t *payload, uint8_t payload_len, const uint8_t *appSKey, uint32_t devAddr, uint16_t fCnt);
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
   for(;;); // Trap context execution here for debugging stack breaks
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

  // 1. Copy your secret 16-byte key straight out of your TTN Console dashboard settings
  const uint8_t nwkSKey[16] = { 0x15, 0x95, 0x75, 0x32, 0x81, 0xC7, 0x92, 0x28, 0x75, 0x4D, 0x76, 0xA0, 0x1D, 0xA2, 0xFD, 0x7C };
  const uint8_t appSKey[16] = { 0x29, 0x93, 0x62, 0x1E, 0x0C, 0x03, 0xE1, 0xCA, 0x6B, 0x28, 0x45, 0x81, 0x40, 0x45, 0xFB, 0x46 };

  /* 1. Hardware Reset */
  HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_RESET);
  osDelay(10);
  HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_SET);
  osDelay(10);

  /* 2. SPI Communication Test */
  // Register 0x42 is RegVersion. On RFM95W/SX1276, it always returns 0x12.
  uint8_t version = Lora_ReadReg(0x42);
  
  // Use version to protect the system
  if (version != 0x12) {
      HAL_UART_Transmit(&huart1, (uint8_t*)"[LoRa] ERROR: Radio core not found!\r\n", 37, 100);
      for(;;) { osDelay(1000); } // Loop forever here and protect the radio
  }
  
  // 1. Put radio in Sleep mode to allow changing to LoRa mode
	Lora_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
	osDelay(10);
	
	// 2. Set Frequency to 868.1 MHz (Formula: F_RF / (32MHz / 2^19))
	// 868100000 / 61.03515625 = 14223122 = 0xD90712
	Lora_WriteReg(REG_FRF_MSB, 0xD9);
	Lora_WriteReg(REG_FRF_MID, 0x07);
	Lora_WriteReg(REG_FRF_LSB, 0x12);
	
	// 3. Configure Base FIFO addresses
	Lora_WriteReg(REG_FIFO_TX_BASE_ADDR, 0x00);
	Lora_WriteReg(REG_FIFO_ADDR_PTR, 0x00);
	
	// 4. Configure Modem for Standard LoRaWAN settings (SF7, 125kHz Bandwidth)
	// RegModemConfig1: BW=125kHz (0x70), Coding Rate=4/5 (0x02), Explicit Header (0x00), LowDataRateOptimize=Off (0x00)
	Lora_WriteReg(REG_MODEM_CONFIG_1, 0x70);
	
	// RegModemConfig2: Spreading Factor 7 (0x70), CRC On (0x04)
	Lora_WriteReg(REG_MODEM_CONFIG_2, 0x74);

	// 5. Power Configuration (Max output power for legal UK unlicensed band)
	// PA_BOOST pin enabled (0x80) + Max power output setting
	Lora_WriteReg(REG_PA_CONFIG, 0x8F); 
	
	// 6. Map DIO0 to trigger on TXDone (Transmission Complete)
	// This will physically pull your PE0 (or interrupt pin) High when finished transmitting
	Lora_WriteReg(REG_DIO_MAPPING_1, 0x40);
	
	// 7. Bring radio into Standby mode to ready the synthesiser
	Lora_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
	
	    // RegInvertInq (Register 0x33). Set bit 6 to 0 to disable InvertIQ on TX
    // Standard default configuration value for basic uploops is 0x27
    Lora_WriteReg(0x33, 0x27); 
    

	osDelay(10);


	for(;;)
	{
	    // ==========================================================
	    // TRANSMIT LOOP 
	    // ==========================================================
	    
	    uint8_t packet[64]; 
	    uint8_t idx = 0;    
	
	    // 1. Define your Device parameters
	    uint8_t dev_addr[4] = { 0x2D, 0x61, 0x00, 0x27 }; // 2700602C LSB array format
	//    uint32_t dev_addr_int = 0x2C600027;               // Numeric format for crypto block math

    
	    // 2. Assemble the LoRaWAN Header
	    packet[idx++] = 0x80;        // MHDR: Confirmed Uplink Type
	    packet[idx++] = dev_addr[0]; // DevAddr Byte 0
	    packet[idx++] = dev_addr[1]; // DevAddr Byte 1
	    packet[idx++] = dev_addr[2]; // DevAddr Byte 2
	    packet[idx++] = dev_addr[3]; // DevAddr Byte 3
	    packet[idx++] = 0x00;        // FCtrl
	    packet[idx++] = (frame_counter & 0xFF);        // FCnt Low Byte
	    packet[idx++] = ((frame_counter >> 8) & 0xFF); // FCnt High Byte
	    packet[idx++] = 0x01;        // FPort
	
	    // ==========================================================
	    // INCORPORATE APPKEY (APPSKEY) HERE
	    // ==========================================================
	    
    // Setup temporary array for string data
  //  uint8_t app_payload[16];
  //  strcpy((char*)app_payload, "TESTX");
      uint8_t app_payload[4] = { 0xDE, 0xAD, 0xBE, 0xEF};
   // uint8_t app_payload_len = 4;
    	uint8_t app_payload_len = 4; // Use exact explicit length
	    
	    // B. Encrypt the raw string inline using your AppSKey token function

    // CHANGE THAT LINE TO FORCE AN EXPLICIT CONSTANT INSTEAD:
    	encrypt_lorawan_payload(app_payload, app_payload_len, appSKey, *(uint32_t*)dev_addr, frame_counter);

	    
	    // C. Append the newly encrypted scrambled bytes into your main routing frame
    	for(uint8_t i = 0; i < 4; i++) {
     	   packet[idx++] = app_payload[i];
    	}
	    
	    // ==========================================================
	    
	    // 2. Calculate length BEFORE the trailing MIC slots
	    uint8_t payload_len_before_mic = idx;
	
	    // 3. Compute the legitimate math signature over the ENCRYPTED data block
	    uint8_t real_mic[4];
	    calculate_lorawan_mic(packet, payload_len_before_mic, nwkSKey, real_mic);
	
	    // 4. Safely load the real math outputs where the zeros used to sit
	    packet[idx++] = real_mic[0];
	    packet[idx++] = real_mic[1];
	    packet[idx++] = real_mic[2];
	    packet[idx++] = real_mic[3];
	
	    // 5. Send this newly signed payload out via your standard FIFO loop
	    Lora_WriteReg(REG_FIFO_ADDR_PTR, 0x00);
	    
	    Lora_WriteReg(0x22, idx); // 0x22 is RegPayloadLength. Explicitly use total byte count 'idx'
	    
	    Lora_WriteReg(REG_PAYLOAD_LENGTH, idx);
	    for(uint8_t i = 0; i < idx; i++) {
	        Lora_WriteReg(REG_FIFO, packet[i]);
	    }





    // ==========================================================
    // 💾 PASTE THE FIFO READBACK DIAGNOSTIC BLOCK HERE:
    // ==========================================================
    
    // 1. Point the radio's memory pointer back to the start of the FIFO to read it
    Lora_WriteReg(REG_FIFO_ADDR_PTR, 0x00);
    
    // 2. Set up a verification array block on the stack
    uint8_t readback_verify[64] = {0};
    
    // 3. Read back what the chip actually stored
    Lora_ReadFIFOVerify(readback_verify, idx);
    
    // 4. Print the comparison table out to your UART terminal
    char uart_buf[100];
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n--- [LoRa FIFO Check] ---\r\n", 29, 100);
    for(uint8_t i = 0; i < idx; i++) {
        sprintf(uart_buf, "Byte [%02d]: Written=0x%02X  |  RadioHas=0x%02X\r\n", i, packet[i], readback_verify[i]);
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)"---------------------------\r\n\r\n", 31, 100);

    // 5. CRITICAL STEP: Point the hardware pointer back to 0x00 so it transmits from the start
    Lora_WriteReg(REG_FIFO_ADDR_PTR, 0x00);
    
    // ==========================================================



	    HAL_UART_Transmit(&huart1, (uint8_t*)"[LoRa] Broadcasting Styled Frame...\r\n", 37, 100);
	    Lora_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
	    
	    Save_Frame_Counter_Wear_Leveled(frame_counter);
	
	    frame_counter++; // Safely advance packet sequence tracker
	
	    osDelay(30000); // Wait 30 seconds
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

/**
 * @brief  Reads back the contents of the radio's TX FIFO to verify exactly 
 *         what bytes are staged for transmission.
 * @param  buffer: Pointer to an array where the read-back bytes will be stored
 * @param  length: Total number of bytes to read back (should equal your 'idx' variable)
 */
void Lora_ReadFIFOVerify(uint8_t *buffer, uint8_t length) {
    uint8_t addr_byte = 0x00 & 0x7F; // REG_FIFO (0x00) with Read Bit (MSB = 0)
    
    // Use your actual working LoRa CS labels suggested by the compiler
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET); 
    
    // Send register address byte over your operational SPI link
    HAL_SPI_Transmit(&hspi2, &addr_byte, 1, HAL_MAX_DELAY);
    
    // Read the array block out of the radio memory
    HAL_SPI_Receive(&hspi2, buffer, length, HAL_MAX_DELAY);
    
    // Pull CS back High to end transaction
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}



/* USER CODE END Application */

