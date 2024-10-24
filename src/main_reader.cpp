#include <Arduino.h>
#include <Wire.h>
// #include "AS5600.h"

#include <stdlib.h>
// #include <Adafruit_LSM9DS1.h>
// #include <Adafruit_Sensor.h>
// #include <cmath>

// #include "headers.hpp"

void setup() {
  Serial1.setTX(12);
  Serial1.setRX(13);
  Serial1.begin(9600);
  Serial.begin(9600);
}



void loop() {
  if(Serial1.available()) {
    char read = Serial1.read();
    Serial.print(read);
  }
}