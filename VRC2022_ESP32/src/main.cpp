#include "Arduino.h"
#include <HCSR04.h>
#include <FastLED.h>
#define LED_PIN 13
#define NUM_LEDS 5

byte triggerPin = 16;
byte echoCount = 1;
byte* echoPins = new byte[echoCount] { 17 };
CRGB  led[NUM_LEDS];

void led_random_test(void){
  for(int i=0;i<10;i++){
    led[i] = CRGB(random(0,255), random(0,255), random(0,255));
    FastLED.show();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup () {
  FastLED.addLeds<WS2812, LED_PIN, GRB>(led, NUM_LEDS);
  Serial.begin(115200);
  HCSR04.begin(triggerPin, echoPins, echoCount);
}

void loop () {
  led_random_test();
  double* distances = HCSR04.measureDistanceCm();
  
  for (int i = 0; i < echoCount; i++) {
    if (i > 0) Serial.print(" | ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(distances[i]);
    Serial.print(" cm");
  }
  
  Serial.println("");
  //delay(500);
}