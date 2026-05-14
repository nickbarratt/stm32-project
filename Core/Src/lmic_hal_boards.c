#include "lmic.h"
#include "hal.h"

// The Adafruit RFM95W handles RF switching internally via hardware.
// Leave this function empty.
void lmic_hal_pin_rxtx (u1_t val) {
    (void)val; 
}

// The Adafruit module standardly uses a regular crystal (XTAL), not a TCXO.
bit_t lmic_hal_queryUsingTcxo (void) {
    return 0; 
}

// Default RSSI calibration offset for SX1276 chip architectures.
s1_t lmic_hal_getRssiCal (void) {
    return 0; 
}
