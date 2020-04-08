#include <Wire.h>
#include "Rhio-RGB.h"

int rhio_rgbled_i2c_address = 0x43;

void rh_rgbSetup () {
  Wire.begin();

  Wire.beginTransmission(rhio_rgbled_i2c_address);
  Wire.write(0x03);
  Wire.write(0x3E);
  Wire.endTransmission();

  Wire.beginTransmission(rhio_rgbled_i2c_address);
  Wire.write(0x07);
  Wire.write(0x00);
  Wire.endTransmission();
}

void rh_rgbSetup (int address) {
  rh_setI2CLedAddress(address);
  rh_rgbSetup();
}

void rh_setI2CLedAddress (int address) {
  rhio_rgbled_i2c_address = address;
}

void rh_redLedOn () {
  Wire.beginTransmission(rhio_rgbled_i2c_address);
  Wire.write(RH_LED_REG);
  Wire.write(RH_LED_REG_RED);
  Wire.endTransmission();
}

void rh_greenLedOn () {
  Wire.beginTransmission(rhio_rgbled_i2c_address);
  Wire.write(RH_LED_REG);
  Wire.write(RH_LED_REG_GREEN);
  Wire.endTransmission();
}

void rh_blueLedOn () {
  Wire.beginTransmission(rhio_rgbled_i2c_address);
  Wire.write(RH_LED_REG);
  Wire.write(RH_LED_REG_BLUE);
  Wire.endTransmission();
}

void rh_ledOff () {
  Wire.beginTransmission(rhio_rgbled_i2c_address);
  Wire.write(RH_LED_REG);
  Wire.write(RH_LED_REG_OFF);
  Wire.endTransmission();
}
