#undef US_PER_OSTICK
#undef OSTICKS_PER_SEC
#define US_PER_OSTICK 30
#define OSTICKS_PER_SEC 32768

#include "lmic.h"
#include "hal.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

extern SPI_HandleTypeDef hspi2; 
static uint8_t irq_lock_cnt = 0;

static uint8_t dio0_last_state = 0;
static uint8_t dio1_last_state = 0;

void hal_spi_init (void) {}

void hal_spi_select (u1_t on) {
    if (on) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); 
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);   
    }
}

// Add this bridge function so the MCCI core can control the NSS pin
void hal_pin_nss(u1_t val) {
    // In LMIC, 0 means activate (low), 1 means deactivate (high)
    if (val == 0) {
        hal_spi_select(1); // Pulls PC5 LOW
    } else {
        hal_spi_select(0); // Pulls PC5 HIGH
    }
}


void lmic_hal_spi_write(u1_t cmd, const u1_t* buf, size_t len) {
    uint8_t command_byte = (uint8_t)(cmd | 0x80); 
    uint8_t dummy_rx;
    
    hal_spi_select(1);
    
    HAL_SPI_TransmitReceive(&hspi2, &command_byte, &dummy_rx, 1, 10);
    if (len > 0) {
        HAL_SPI_Transmit(&hspi2, (uint8_t*)buf, len, 50);
    }
    
    hal_spi_select(0);
    
}
void hal_spi_write(u1_t outval) {
    uint8_t val = (uint8_t)outval;
    
        // === FIX: Explicitly frame the single-byte transmission ===
    hal_spi_select(1); // Pull PC5 LOW
    
    HAL_SPI_Transmit(&hspi2, &val, 1, 10);
    

    
    HAL_SPI_Transmit(&hspi2, &val, 1, 10);
    
        hal_spi_select(0); // Pull PC5 HIGH
}

void lmic_hal_spi_read(u1_t cmd, u1_t* buf, size_t len) {
    uint8_t command_byte = (uint8_t)(cmd & 0x7F); 
    uint8_t dummy_rx;
    
    hal_spi_select(1);
       
    HAL_SPI_TransmitReceive(&hspi2, &command_byte, &dummy_rx, 1, 10);
    if (len > 0) {
        uint8_t dummy_tx = 0x00;
        for (size_t i = 0; i < len; i++) {
            HAL_SPI_TransmitReceive(&hspi2, &dummy_tx, (uint8_t*)&buf[i], 1, 10);
        }
    }
    
    hal_spi_select(0);
}
u1_t hal_spi_read(void) {
    uint8_t dummy = 0, inval = 0;
    HAL_SPI_TransmitReceive(&hspi2, &dummy, &inval, 1, 10);
    return (u1_t)inval;
}

void lmic_hal_pin_rst (u1_t val) {
    if (val == 0) {
        // Drive PC4 LOW to reset the radio module
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
    } else {
        // Drive PC4 HIGH for both 1 and 2, keeping it stable
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
        
        // Ensure the pin remains a solid Digital Output
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    }
}
void hal_pin_rst (u1_t val) { lmic_hal_pin_rst(val); }


u4_t lmic_hal_waitUntil (u4_t time) {
    u4_t now = lmic_hal_ticks();
    while ((s4_t)(time - now) > 0) {
        now = lmic_hal_ticks();
    }
    return now;
}
void hal_waitUntil (u4_t time) { (void)lmic_hal_waitUntil(time); }




// 1. Keep the timing function pointed to the continuous hardware clock
u4_t lmic_hal_ticks (void) {
    uint32_t ms = HAL_GetTick(); // Runs continuously
    return (u4_t)((ms * 32768ULL) / 1000ULL);
  //    return ms2osticks(xTaskGetTickCount());
}
u4_t hal_ticks (void) { return lmic_hal_ticks(); }

// 2. Empty out the IRQ disabling macros so they don't freeze the SysTick clock
void lmic_hal_disableIRQs (void) {
    // Leave this blank! Your single FreeRTOS task prevents memory race conditions.
    irq_lock_cnt++;
}
void hal_disableIRQs (void) {
	taskENTER_CRITICAL();
//	lmic_hal_disableIRQs();
}

void lmic_hal_enableIRQs (void) {
    if (irq_lock_cnt > 0) {
        irq_lock_cnt--;
    }
}
void hal_enableIRQs (void) {
	//lmic_hal_enableIRQs();
	taskEXIT_CRITICAL(); 
}


void lmic_hal_sleep (void) {}
void hal_sleep (void) {}

void lmic_hal_processPendingIRQs (void) {
    // Read your real DIO0 pin (GPIOB, PIN 2) safely from the RTOS thread
    uint8_t current_dio0 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET) ? 1 : 0;
    
    // Read your real DIO1 pin (GPIOE, PIN 1) safely from the RTOS thread
    uint8_t current_dio1 = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == GPIO_PIN_SET) ? 1 : 0;
    
    if (current_dio0 == 1 && dio0_last_state == 0) {
        radio_irq_handler(0); // Safely process TX Done
    }
    dio0_last_state = current_dio0;
    
    if (current_dio1 == 1 && dio1_last_state == 0) {
        radio_irq_handler(1); // Safely process RX Timeout / Window
    }
    current_dio1 = dio1_last_state;
}

u1_t lmic_hal_checkTimer (u4_t targettime) { (void)targettime; return 0; }
ostime_t lmic_hal_setModuleActive (bit_t val) { (void)val; return 0; }

void lmic_hal_pin_rxtx (u1_t val) { (void)val; }
bit_t lmic_hal_queryUsingTcxo (void) { return 0; }
s1_t lmic_hal_getRssiCal (void) { return 0; }
void lmic_hal_failed (const char *file, u2_t line) { (void)file; (void)line; }

void lmic_hal_init_ex (const void *pBoards) {
    (void)pBoards;
    hal_pin_rst(1); 
}

u1_t lmic_hal_getTxPowerPolicy (u1_t policy, s1_t requestedPower, u4_t frequency) {
    (void)policy; (void)frequency;
    return requestedPower;
}

int lmic_secure_element_init (void) { return 0; }
bit_t LMIC_SecureElement_Default_initialize(void) { return 1; }
bit_t LMIC_SecureElement_Default_setAppKey(const u1_t *pKey) { (void)pKey; return 1; }
bit_t LMIC_SecureElement_Default_setAppEUI(const u1_t *pEui) { (void)pEui; return 1; }
bit_t LMIC_SecureElement_Default_setDevEUI(const u1_t *pEui) { (void)pEui; return 1; }
bit_t LMIC_SecureElement_Default_createJoinRequest(void *pFrame, u1_t fmt) { (void)pFrame; (void)fmt; return 1; }
bit_t LMIC_SecureElement_Default_decodeJoinAccept(void *pFrame, u1_t len) { (void)pFrame; (void)len; return 1; }
bit_t LMIC_SecureElement_Default_encodeMessage(void *pFrame, u1_t len, u1_t port, u1_t *pOut, u1_t conf) { (void)pFrame; (void)len; (void)port; (void)pOut; (void)conf; return 1; }
bit_t LMIC_SecureElement_Default_verifyMIC(void *pFrame, u1_t len, u1_t *pMic) { (void)pFrame; (void)len; (void)pMic; return 1; }
bit_t LMIC_SecureElement_Default_decodeMessage(void *pFrame, u1_t len, u1_t port, u1_t *pOut, u1_t conf) { (void)pFrame; (void)len; (void)port; (void)pOut; (void)conf; return 1; }
bit_t LMIC_SecureElement_Default_aes128Encrypt(u1_t *pData, const u1_t *pKey) { (void)pData; (void)pKey; return 1; }

// FIXED: Restored explicit array sizing brackets to preserve all indices cleanly
typedef struct {
    uint8_t nss;
    uint8_t rxtx;
    uint8_t rst;
    uint8_t dio[3]; 
} lmic_pinmap;

// Inside lmic_hal_boards.c (around line 171)
const lmic_pinmap lmic_pins = {
    .nss = 4,       //  Change from '1' or hardcoded values to your macro
    .rxtx = 0,
    .rst = 0,     //  Point this to your PC4 Reset pin macro
    .dio = {
        0,         //  Point this to your PB2 interrupt macro
        1,         //  Point this to your PE1 orange wire macro
        0
    }
};

//const lmic_pinmap lmic_pins = {
//    .nss = 5,       //  Change from '1' or hardcoded values to your macro
//    .rxtx = 0,
//    .rst = 4,     //  Point this to your PC4 Reset pin macro
//    .dio = {
//        2,         //  Point this to your PB2 interrupt macro
//        1,         //  Point this to your PE1 orange wire macro
//        0
//    }
//};

//const lmic_pinmap lmic_pins = {
 //   .nss  = 1,
 //   .rxtx = 0xff,
 //   .rst  = 0xff,
//    .dio  = { 0, 1, 0xff } 
//};
