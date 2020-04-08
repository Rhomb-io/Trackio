/*
Tiny9's LIS2HH12 Library Main Source File
Tiny9_LIS2HH12.cpp

T.J. Schramm @ Tiny9
Created: Nov 14, 2017
Updated: Nov 14, 2017

Modified Bildr LIS2HH12 Source File @
to support I2C and SPI Communication (not working yet)

Hardware Resources:
- Arduino Development Board
- Tiny9 Triple Access Accelerometer LIS2HH12

Development Environment Specifics:
Arduino 1.6.8
Tiny9 Triple Axis Accelerometer Breakout - LIS2HH12
Arduino Uno
Arduino Nano
*/

//#include "Arduino.h"
#include "rhio-LIS2HH12.h"
#include <Wire.h>

#define LIS2HH12_DEVICE (0x1D) // Device Address for LIS2HH12
#define LIS2HH12_TO_READ (1)   // Number of Bytes Read - Two Bytes Per Axis

LIS2HH12::LIS2HH12() {
  status = LIS2HH12_OK;
  error_code = LIS2HH12_NO_ERROR;

  gains[0] = 1; // 0.00376390; // Original gain 0.00376390
  gains[1] = 1; // 0.00376009; // Original gain 0.00376009
  gains[2] = 1; // 0.00349265; // Original gain 0.00349265
}

void LIS2HH12::powerOn() {
  Wire.begin();

  writeTo(LIS2HH12_CTRL1, 23);
  writeTo(LIS2HH12_CTRL3, 128);
  writeTo(LIS2HH12_ACT_THS, 0); // Activity/Inactivity detection function disabled
  writeTo(LIS2HH12_ACT_DUR, 0); // Activity/Inactivity detection function disabled
  writeTo(LIS2HH12_FIFO_CTRL, 0);
}

void LIS2HH12::readAccel(int *x, int *y, int *z) {
  readFrom(40, 6, _buff); // Read Accel Data from LIS2HH12

  // Each Axis @ All g Ranges: 10 Bit Resolution (2 Bytes)
  *x = (((int)_buff[1]) << 8) | _buff[0];
  *y = (((int)_buff[3]) << 8) | _buff[2];
  *z = (((int)_buff[5]) << 8) | _buff[4];
}

void LIS2HH12::readFrom (byte address, int num, byte _buff[]) {
  Wire.beginTransmission(LIS2HH12_DEVICE);
  Wire.write(address);
  Wire.endTransmission();

  Wire.beginTransmission(LIS2HH12_DEVICE);
  Wire.requestFrom(LIS2HH12_DEVICE, num);  // Request 6 Bytes

  int i = 0;
  while (Wire.available()) {
    _buff[i] = Wire.read();
    i++;
  }

  if (i != num) {
    status = LIS2HH12_ERROR;
    error_code = LIS2HH12_READ_ERROR;
  }
  Wire.endTransmission();
}

void LIS2HH12::writeTo(byte address, byte val) {
  Wire.beginTransmission(LIS2HH12_DEVICE);
  Wire.write(address);
  Wire.write(val);
  Wire.endTransmission();
}
