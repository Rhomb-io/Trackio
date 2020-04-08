#include "Arduino.h"

#ifndef LIS2HH12_h
#define LIS2HH12_h

/*************************** REGISTER MAP ***************************/
#define LIS2HH12_RESERVED0    0x00  // Reserved. Do Not Access.
#define LIS2HH12_RESERVED1    0x01  // Reserved. Do Not Access.
#define LIS2HH12_RESERVED2    0x02  // Reserved. Do Not Access.
#define LIS2HH12_RESERVED3    0x03  // Reserved. Do Not Access.
#define LIS2HH12_RESERVED4    0x04  // Reserved. Do Not Access.
#define LIS2HH12_RESERVED5    0x05  // Reserved. Do Not Access.
#define LIS2HH12_RESERVED6    0x06  // Reserved. Do Not Access.
#define LIS2HH12_RESERVED7    0x07  // Reserved. Do Not Access.
#define LIS2HH12_RESERVED8    0x08  // Reserved. Do Not Access.
#define LIS2HH12_RESERVED9    0x09  // Reserved. Do Not Access.
#define LIS2HH12_RESERVEDA    0x0A  // Reserved. Do Not Access.
#define LIS2HH12_TEMP_L       0x0B  // Temperature Low Significant Byte (R)
#define LIS2HH12_TEMP_H       0x0C  // Temperature High Significant Byte (R)
#define LIS2HH12_RESERVEDE    0x0E  // Reserved. Do Not Access.
#define LIS2HH12_WHO_AM_I     0x0F  // Device ID. (R)
#define LIS2HH12_ACT_THS      0x1E  // ? (R/W)
#define LIS2HH12_ACT_DUR      0x1F  // ? (R/W)
#define LIS2HH12_CTRL1        0x20  // Control Register (R/W)
#define LIS2HH12_CTRL2        0x21  // Control Register (R/W)
#define LIS2HH12_CTRL3        0x22  // Control Register (R/W)
#define LIS2HH12_CTRL4        0x23  // Control Register (R/W)
#define LIS2HH12_CTRL5        0x24  // Control Register (R/W)
#define LIS2HH12_CTRL6        0x25  // Control Register (R/W)
#define LIS2HH12_CTRL7        0x26  // Control Register (R/W)
#define LIS2HH12_STATUS       0x27  // Status Data Register (R)
#define LIS2HH12_OUT_X_L      0x28  // X-Axis_Low BYTE (R)
#define LIS2HH12_OUT_X_H      0x29  // X-Axis_High BYTE (R)
#define LIS2HH12_OUT_Y_L      0x2A  // X-Axis_Low BYTE (R)
#define LIS2HH12_OUT_Y_H      0x2B  // X-Axis_High BYTE (R)
#define LIS2HH12_OUT_Z_L      0x2C  // X-Axis_Low BYTE (R)
#define LIS2HH12_OUT_Z_H      0x2D  // X-Axis_High BYTE (R)
#define LIS2HH12_FIFO_CTRL    0x2E  // FIFI Control (R/W)
#define LIS2HH12_FIFO_SRC     0x2F  // FIFO ? (R)
#define LIS2HH12_IG_CFG1      0x30  // Interrupt Generator 1 configuration (R/W)
#define LIS2HH12_IG_SRC1      0x31  // Interrupt Generator 1 status Register (R)
#define LIS2HH12_IG_THS_X1    0x32  // Interrupt generator 1 X Threshold (R/W)
#define LIS2HH12_IG_THS_Y1    0x33  // Interrupt Generator 1 Y Threshold (R/W)
#define LIS2HH12_IG_THS_Z1    0x34  // Interrupt Generator 1 Z Threshold (R/W)
#define LIS2HH12_IG_DUR1      0x35  // Interrupt Generator 1 Duration (R/W)
#define LIS2HH12_IG_CFG2      0x36  // Interrupt Generator 2 configuration (R/W)
#define LIS2HH12_IG_SRC2      0x37  // Interrupt Generator 2 status Register (R)
#define LIS2HH12_IG_THS2      0x38  // Interrupt generator 2 Threshold (R/W)
#define LIS2HH12_IG_DUR2      0x39  // Interrupt Generator 2 Duration (R/W)

#define LIS2HH12_XL_REFERENCE 0x3A  // Reference X Low (R/W)
#define LIS2HH12_XH_REFERENCE 0x3B  // Reference X High (R/W)
#define LIS2HH12_YL_REFERENCE 0x3C  // Reference Y Low (R/W)
#define LIS2HH12_YH_REFERENCE 0x3D  // Reference Y High (R/W)
#define LIS2HH12_ZL_REFERENCE 0x3E  // Reference Z Low (R/W)
#define LIS2HH12_ZH_REFERENCE 0x3F  // Reference Z High (R/W)


 /************************** INTERRUPT PINS **************************/
#define LIS2HH12_INT1_PIN     0x00  //INT1: 0
#define LIS2HH12_INT2_PIN     0x01  //INT2: 1


 /********************** INTERRUPT BIT POSITION **********************/
#define LIS2HH12_INT_DATA_READY_BIT  0x07
#define LIS2HH12_INT_SINGLE_TAP_BIT  0x06
#define LIS2HH12_INT_DOUBLE_TAP_BIT  0x05
#define LIS2HH12_INT_ACTIVITY_BIT    0x04
#define LIS2HH12_INT_INACTIVITY_BIT  0x03
#define LIS2HH12_INT_FREE_FALL_BIT   0x02
#define LIS2HH12_INT_WATERMARK_BIT   0x01
#define LIS2HH12_INT_OVERRUNY_BIT    0x00

#define LIS2HH12_DATA_READY    0x07
#define LIS2HH12_SINGLE_TAP    0x06
#define LIS2HH12_DOUBLE_TAP    0x05
#define LIS2HH12_ACTIVITY      0x04
#define LIS2HH12_INACTIVITY    0x03
#define LIS2HH12_FREE_FALL     0x02
#define LIS2HH12_WATERMARK     0x01
#define LIS2HH12_OVERRUNY      0x00

#define LIS2HH12_OK    1  // No Error
#define LIS2HH12_ERROR 0  // Error Exists

#define LIS2HH12_NO_ERROR   0  // Initial State
#define LIS2HH12_READ_ERROR 1  // Accelerometer Reading Error
#define LIS2HH12_BAD_ARG    2  // Bad Argument

class LIS2HH12 {
public:
  bool status;     // Set When Error Exists

  byte error_code; // Initial State
  double gains[3]; // Counts to Gs

  LIS2HH12();

  void powerOn();
  void readAccel(int* x, int* y, int* z);

private:
  byte _buff[6];
  void writeTo(byte address, byte val);
  void readFrom(byte address, int num, byte buff[]);
};
void print_byte(byte val);
#endif
