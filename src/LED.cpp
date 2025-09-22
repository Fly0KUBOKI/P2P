#include <Arduino.h>
#include <FastLED.h>
#include "LED.hpp"

// Configuration: change these as needed
#ifndef LED_PIN
#define LED_PIN 5
#endif

#ifndef NUM_LEDS
#define NUM_LEDS 2
#endif

CRGB leds[NUM_LEDS];

void setupLED() {
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.clear(true);
}

// Helper: set single LED by number (1-based).
// If `num` is 1, change the first LED; if 2, change the second, etc.
void setLED(int num, uint8_t r, uint8_t g, uint8_t b) {
  int index = num - 1; // convert 1-based to 0-based
  if (index < 0 || index >= NUM_LEDS) return;
  leds[index] = CRGB(r, g, b);
}

// Demo: set each LED independently and rotate colors (minimal, no extra helpers)
void loopLED() {
  // Start with all off
  for (int i = 0; i < NUM_LEDS; ++i) leds[i] = CRGB::Black;

  setLED(1, 0, 50, 50);
  setLED(2, 0, 50, 50); 
  FastLED.show();

  delay(100);

  setLED(1, 0, 0, 0);
  setLED(2, 0, 0, 0);
  FastLED.show();

  delay(100);

}
