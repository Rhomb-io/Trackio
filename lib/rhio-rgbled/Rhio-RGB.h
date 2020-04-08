#ifndef RHIO_RGB
#define RHIO_RGB

extern int rhio_rgbled_i2c_address;

#define RH_LED_REG 0x05
#define RH_LED_REG_RED 0x08
#define RH_LED_REG_GREEN 0x10
#define RH_LED_REG_BLUE 0x20
#define RH_LED_REG_OFF 0x00

void rh_rgbSetup (int address);
void rh_rgbSetup ();
void rh_setI2CLedAddress (int address);
void rh_redLedOn();
void rh_greenLedOn();
void rh_blueLedOn();
void rh_ledOff();

#endif
