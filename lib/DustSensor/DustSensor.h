#ifndef DUSTSENSOR_h
#define DUSTSENSOR_h

#define COV_RATIO 0.2 // ug/mmm / mv
#define NO_DUST_VOLTAGE 400 // mv
#define SYS_VOLTAGE 5000 // 5V (or 3300 for 3v3)

class DustSensor{
public:
  float density;
  float voltage;
  int Filter(int m);
  // Convert raw ADC voltage reading (0-1023) into dust density in ug/m^3
  float Conversion(int rawVoltageADC);
};

#endif
