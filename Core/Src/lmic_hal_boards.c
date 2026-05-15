#include "lmic.h"
#include "hal.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

extern SPI_HandleTypeDef hspi2; 
static uint8_t irq_lock_cnt = 0;

void hal_spi_init (void) {}

// SPI Write
void lmic_hal_spi_write(u1_t cmd, const u1_t* buf, size_t len) {
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 10);
    if (len > 0) {
        HAL_SPI_Transmit(&hspi2, (uint8_t*)buf, len, 50);
    }
}
void hal_spi_write(u1_t outval) {
    HAL_SPI_Transmit(&hspi2, &outval, 1, 10);
}

// SPI Read
void lmic_hal_spi_read(u1_t cmd, u1_t* buf, size_t len) {
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 10);
    if (len > 0) {
        HAL_SPI_Receive(&hspi2, buf, len, 50);
    }
}
u1_t hal_spi_read(void) {
    u1_t dummy = 0, inval = 0;
    HAL_SPI_TransmitReceive(&hspi2, &dummy, &inval, 1, 10);
    return inval;
}

void hal_spi_select (u1_t on) {
    if (on) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    }
}

// Pin Reset
void lmic_hal_pin_rst (u1_t val) {
    if (val == 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
    } else if (val == 1) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
    } else {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    }
}
void hal_pin_rst (u1_t val) { lmic_hal_pin_rst(val); }

u4_t lmic_hal_ticks (void) {
    uint32_t ms = osKernelGetTickCount();
    return (u4_t)(ms * 1000);
}
u4_t hal_ticks (void) { return lmic_hal_ticks(); }

// Fixed type: Returns u4_t to match hal.h exactly
u4_t lmic_hal_waitUntil (u4_t time) {
    u4_t now = lmic_hal_ticks();
    while ((s4_t)(time - now) > 0) {
        now = lmic_hal_ticks();
    }
    return now;
}
void hal_waitUntil (u4_t time) { (void)lmic_hal_waitUntil(time); }

void lmic_hal_disableIRQs (void) {
    if (irq_lock_cnt == 0) {
        taskENTER_CRITICAL();
    }
    irq_lock_cnt++;
}
void hal_disableIRQs (void) { lmic_hal_disableIRQs(); }

void lmic_hal_enableIRQs (void) {
    if (irq_lock_cnt > 0) {
        irq_lock_cnt--;
        if (irq_lock_cnt == 0) {
            taskEXIT_CRITICAL();
        }
    }
}
void hal_enableIRQs (void) { lmic_hal_enableIRQs(); }

void lmic_hal_sleep (void) {}
void hal_sleep (void) {}

void lmic_hal_processPendingIRQs (void) {}
u1_t lmic_hal_checkTimer (u4_t targettime) { (void)targettime; return 0; }
ostime_t lmic_hal_setModuleActive (bit_t val) { (void)val; return 0; }

void lmic_hal_pin_rxtx (u1_t val) { (void)val; }
bit_t lmic_hal_queryUsingTcxo (void) { return 0; }
s1_t lmic_hal_getRssiCal (void) { return 0; }
void lmic_hal_failed (const char *file, u2_t line) { (void)file; (void)line; }
void lmic_hal_init_ex (const void *pBoards) { (void)pBoards; }

u1_t lmic_hal_getTxPowerPolicy (u1_t policy, s1_t requestedPower, u4_t frequency) {
    (void)policy; (void)frequency;
    return requestedPower;
}

// ============================================================================
// Secure Element Library Driver Stubs
// ============================================================================
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

#ifndef LMIC_UNUSED_PIN
#define LMIC_UNUSED_PIN 0xff
#endif

// Fixed type mapping configuration
struct my_lmic_pinmap {
    u1_t nss;
    u1_t rxtx;
    u1_t rst;
    u1_t dio[3]; // Fixed: Added standard length boundary to resolve scalar brace warnings
};

const struct my_lmic_pinmap lmic_pins = {
    .nss  = 1,
    .rxtx = LMIC_UNUSED_PIN,
    .rst  = LMIC_UNUSED_PIN,
    .dio  = { 2, 13, LMIC_UNUSED_PIN } 
};
