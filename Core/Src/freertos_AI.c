/* USER CODE BEGIN Header */                                                                           
/**                                                                                                    
  * ******************************************************************************                       
  * @file    freertos.c                                                                    
  * @brief   Code for freertos applications                                                
  * ******************************************************************************                       
  */                                                                                                     
/* USER CODE END Header */                                                                             
                                                                                                       
/* Includes ------------------------------------------------------------------*/                       
#include "FreeRTOS.h"                                                                                  
#include "task.h"                                                                                      
#include "main.h"                                                                                      
#include "cmsis_os.h"                                                                                  
                                                                                                       
/* Private includes ----------------------------------------------------------*/                       
/* USER CODE BEGIN Includes */                                                                         
#include "main.h"   
#include <string.h> 
#include <stdio.h>  
                                                                                                       
extern TIM_HandleTypeDef htim1;                                                                        
extern TIM_HandleTypeDef htim3;                                                                        
extern UART_HandleTypeDef huart1;                                                                      
                                                                                                       
extern uint32_t lastBlinkTime;                                                                         
extern uint8_t  ledIsOn;                                                                               
extern uint32_t duty_cycle;                                                                            
extern uint8_t  k0_last_state;                                                                         
extern uint8_t  k1_last_state;                                                                         
/* USER CODE END Includes */                                                                           
                                                                                                       
/* Private variables ---------------------------------------------------------*/                       
/* USER CODE BEGIN Variables */                                                                        
extern SPI_HandleTypeDef hspi2;                                                                        
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

#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03

// Crypto bindings from your updated aes.c file
extern void calculate_lorawan_mic(uint8_t *packet, uint8_t payload_len, const uint8_t *nwkSKey, uint8_t *mic_out);
extern void encrypt_lorawan_payload(uint8_t *payload, uint8_t payload_len, const uint8_t *appSKey, uint32_t devAddr, uint16_t fCnt);
extern void Lora_WriteReg(uint8_t addr, uint8_t value);
extern uint8_t Lora_ReadReg(uint8_t addr);
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN GFP */
void StartLoRaTask(void *argument);
/* USER CODE END GFP */
/* USER CODE BEGIN PrivateCode */

/**
  * @brief  FreeRTOS initialization function hook expected by main.c
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* 
   * Note: If you are initializing your FreeRTOS tasks dynamically via the CubeMX 
   * interface, task creation handles (like osThreadNew) will sit here.
   * If you are launching StartLoRaTask manually, this block can remain empty.
   */
}

void StartLoRaTask(void *argument)
{
  /* USER CODE BEGIN StartLoRaTask */
  // 1. Keys copied from the TTN Dashboard (Ensure both are kept MSB first)
  const uint8_t nwkSKey[16] = { 0x15, 0x95, 0x75, 0x32, 0x81, 0xC7, 0x92, 0x28, 0x75, 0x4D, 0x76, 0xA0, 0x1D, 0xA2, 0xFD, 0x7C };
  // CRITICAL: Replace these 16 bytes with your explicit AppSKey token from your device panel
  const uint8_t appSKey[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  /* 2. Hardware Pulse Reset */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); 
  osDelay(10);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);   
  osDelay(10);

  /* 3. Operational Integrity Test */
  if (Lora_ReadReg(0x42) != 0x12) {
      HAL_UART_Transmit(&huart1, (uint8_t*)"[LoRa] ERROR: Chip not found!\r\n", 31, 100);
      for(;;) { osDelay(1000); } 
  }
  
  // Shift to sleep mode to unlock configurations registers
  Lora_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
  osDelay(10);
	
  // Set EU868 Frequency to 868.1 MHz
  Lora_WriteReg(REG_FRF_MSB, 0xD9);
  Lora_WriteReg(REG_FRF_MID, 0x1E);
  Lora_WriteReg(REG_FRF_LSB, 0x12);
	
  // Setup FIFO bounds and Modem Parameters (SF7, 125kHz, CRC)
  Lora_WriteReg(REG_FIFO_TX_BASE_ADDR, 0x00);
  Lora_WriteReg(REG_FIFO_ADDR_PTR, 0x00);
  Lora_WriteReg(REG_MODEM_CONFIG_1, 0x72); 
  Lora_WriteReg(REG_MODEM_CONFIG_2, 0x74); 
  Lora_WriteReg(REG_PA_CONFIG, 0xFF); // Boost on
  Lora_WriteReg(REG_DIO_MAPPING_1, 0x40); // DIO0 mappings to TXDone
  Lora_WriteReg(0x39, 0x34); // Set Public Sync Word
	
  Lora_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
  osDelay(10);

  for(;;)
  {
      uint8_t packet[64]; 
      uint8_t idx = 0; 
  
      uint8_t dev_addr[4] = { 0x2C, 0x60, 0x00, 0x27 }; // Converted to LSB layout
      uint32_t dev_addr_crypto = 0x2700602C;           // Converted to MSB integer format
      static uint16_t frame_counter = 0;              
  
      // Pack MHDR, DevAddr, FCtrl, FCnt, and FPort
      packet[idx++] = 0x40;                           
      packet[idx++] = dev_addr[0]; packet[idx++] = dev_addr[1];
      packet[idx++] = dev_addr[2]; packet[idx++] = dev_addr[3];
      packet[idx++] = 0x00;                           
      packet[idx++] = (frame_counter & 0xFF);         
      packet[idx++] = ((frame_counter >> 8) & 0xFF);  
      packet[idx++] = 0x01;                           

      // Handle isolated Encryption Payload
      uint8_t app_payload[16] = {0};
      uint8_t payload_len = 5; 
      memcpy(app_payload, "HELLO", payload_len);
      
      encrypt_lorawan_payload(app_payload, payload_len, appSKey, dev_addr_crypto, frame_counter);
      
      // Append encrypted bytes into transmission packet frame
      for(uint8_t i = 0; i < payload_len; i++) {
          packet[idx++] = app_payload[i];
      }
      
      uint8_t payload_len_before_mic = idx;
      uint8_t real_mic[4] = {0};
  
      // Generate the signature block
      calculate_lorawan_mic(packet, payload_len_before_mic, nwkSKey, real_mic);
  
      packet[idx++] = real_mic[0]; packet[idx++] = real_mic[1];
      packet[idx++] = real_mic[2]; packet[idx++] = real_mic[3];
  
      // Stream packet to FIFO and Transmit
      Lora_WriteReg(REG_FIFO_ADDR_PTR, 0x00);
      Lora_WriteReg(REG_PAYLOAD_LENGTH, idx);
      for(uint8_t i = 0; i < idx; i++) {
          Lora_WriteReg(REG_FIFO, packet[i]);
      }

      HAL_UART_Transmit(&huart1, (uint8_t*)"[LoRa] Packet Transmitting...\r\n", 31, 100);
      Lora_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
  
      frame_counter++; 
      osDelay(10000); // Wait 10 seconds before next transmission
  }
  /* USER CODE END StartLoRaTask */
}
/* USER CODE END PrivateCode */
