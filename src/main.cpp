#include <Arduino.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "radio/radio.h"

// Define SPI pins for ESP32-S2
#define PIN_NUM_MOSI 11  // Also used as MISO in 3-wire mode
#define PIN_NUM_CLK 12
#define PIN_NUM_CS 10


void setup() {
    Serial.begin(115200);
    RF_Init();
}

void loop() {
    // Add your continuous operations here
    delay(1000);
}
