#ifndef LED_H
#define LED_H

#include <Arduino.h>

// LED control functions
void setupLED();
void loopLED();
void setLED(int num, uint8_t r, uint8_t g, uint8_t b);

#endif