#include <Wire.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LIS3MDL.h>

Adafruit_LSM6DS3TRC lsm6ds;
Adafruit_LIS3MDL lis3mdl;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin();

  Serial.println("Adafruit LSM6DS+LIS3MDL test!");

  if (!lsm6ds.begin_I2C()) {
    Serial.println("Failed to find LSM6DS chip");
    while (1) delay(10);
  }
  Serial.println("LSM6DS found");

  
  if (!lis3mdl.begin_I2C(0x1E)) {
    Serial.println("Failed to find LIS3MDL chip");
    while (1) delay(10);
  }
}

  Serial.println("LIS3MDL found!");
}

void loop() {
}
