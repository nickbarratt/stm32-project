#include "lmic.h"
#include "hal.h"
#include "main.h"

// FreeRTOS Native Headers
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

extern SPI_HandleTypeDef hspi2; 
static uint8_t irq_lock_cnt = 0;

void hal_spi_init (void) {}

void hal_spi_select (u1_t on) {
    if (on) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    }
}

u1_t hal_spi_write (u1_t outval) {
    u1_t inval = 0;
    HAL_SPI_TransmitReceive(&hspi2, &outval, &inval, 1, 10);
    return inval; 
}

void hal_pin_rst (u1_t val) {
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

u4_t hal_ticks (void) {
    uint32_t ms = osKernelGetTickCount();
    return (u4_t)(ms * 1000);
}

void hal_waitUntil (u4_t time) {
    u4_t now = hal_ticks();
    while ((s4_t)(time - now) > 0) {
        now = hal_ticks();
    }
}

void hal_disableIRQs (void) {
    if (irq_lock_cnt == 0) {
        taskENTER_CRITICAL();
    }
    irq_lock_cnt++;
}

void hal_enableIRQs (void) {
    if (irq_lock_cnt > 0) {
        irq_lock_cnt--;
        if (irq_lock_cnt == 0) {
            taskEXIT_CRITICAL();
        }
    }
}

void hal_sleep (void) {}
void lmic_hal_pin_rxtx (u1_t val) { (void)val; }
bit_t lmic_hal_queryUsingTcxo (void) { return 0; }
s1_t lmic_hal_getRssiCal (void) { return 0; }

// ============================================================================
// Robust Pinmap Definition Override
// ============================================================================

#ifndef LMIC_UNUSED_PIN
#define LMIC_UNUSED_PIN 0xff
#endif

struct my_lmic_pinmap {
    u1_t nss;
    u1_t rxtx;
    u1_t rst;
    u1_t dio[3];
};

// Declaring using basic types ensures the compiler evaluates the size instantly
const struct my_lmic_pinmap lmic_pins = {
    .nss  = 1,
    .rxtx = LMIC_UNUSED_PIN,
    .rst  = LMIC_UNUSED_PIN,
    .dio  = { 2, 13, LMIC_UNUSED_PIN } 
};
