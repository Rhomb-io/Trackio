#ifndef __TRACKIO_COMMANDS__
#define __TRACKIO_COMMANDS__

bool valueIsBool(char *value) {
  int number = atoi(value);
  return number == 1 || number == 0;
}

bool valueIsNumber(const char *value) {
  int len = strlen(value);
  for (int i = 0; i < len; i++)
    if (value[i] < 48 || value[i] > 57)
      return false;

  return true;
}

bool valueInArray(const char *value, int *arr) {
  if (valueIsNumber(value) == false) return 0;
  int number = atoi(value);

  for (int i = 0; arr[i] != 0; i++)
    if (arr[i] == number)
      return true;

  return false;
}

bool valueIsBetween(char *value, unsigned long from, unsigned long to) {
  if (valueIsNumber(value) == false) return 0;
  unsigned long number = atoi(value);
  return number >= from || number <= to;
}

// V >= 0.3.0
bool cmd_0(char *value) { return true; }  // reserved

// Deep Sleep 1|0
bool cmd_1(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.deepSleep = atoi(value);
  return true;
}

// Sleep 1|0
bool cmd_2(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.sleep = atoi(value);
  return true;
}

// opmode
// 0=OP_STARTUP (safe reset) 2=OP_RST (hard reset with watchdog)
bool cmd_3(char *value) {
  int arr[] = {0, 2};
  if (!valueInArray(value, arr)) return false;
  cfg.opmode = atoi(value);
  return true;
}

// primary opmode
// 1=OP_AUTO (normal mode) 3=OP_LOW (force low mode)
bool cmd_4(char *value) {
  int arr[] = {1, 3};
  if (!valueInArray(value, arr)) return false;
  cfg.primaryOpMode = atoi(value);
  return true;
}

// V >= 0.3.2

// minimum VBAT value >3600 & <4100
bool cmd_5(char *value) {
  if (valueIsBetween(value, 3600, 4100) == false) return false;
  cfg.requiredVbat = atoi(value);
  return true;
}

// minimum VIN value >6000 & <70000
bool cmd_6(char *value) {
  if (valueIsBetween(value, 6000, 70000) == false) return false;
  cfg.requiredVin = atoi(value);
  return true;
}

// minimum VSYS value >4000 & <5000
bool cmd_7(char *value) {
  if (valueIsBetween(value, 4000, 5000) == false) return false;
  cfg.requiredVsys5v = atoi(value);
  return true;
}

// >= 0.4.0

// tickTimer (message time interval) in seconds
bool cmd_8(char *value) {
  if (valueIsNumber(value) == false) return false;
  cfg.tickTimer = atoi(value);
  return true;
}

// aliveTimer in seconds
bool cmd_9(char *value) {
  if (valueIsNumber(value) == false) return false;
  cfg.aliveTimer = atoi(value);
  return true;
}

// keep TCP open 0|1
bool cmd_10(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.keepTCPOpen = atoi(value);
  return true;
}

// use temp 0|1
bool cmd_11(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useTemp = atoi(value);
  return true;
}

// use pressure 0|1
bool cmd_12(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.usePressure = atoi(value);
  return true;
}

// use CO (carmon monoxide)  0|1
bool cmd_13(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useCO = atoi(value);
  return true;
}

// ue humidity  0|1
bool cmd_14(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useHumidity = atoi(value);
  return true;
}

// use VOC (CS811) 0|1
bool cmd_15(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useVOC = atoi(value);
  return true;
}

// use GPS time 0|1
bool cmd_16(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useGpsTime = atoi(value);
  return true;
}

// use GPS modem 0|1
bool cmd_17(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useGps = atoi(value);
  return true;
}

// use GEO Location (GPS) 0|1
bool cmd_18(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useLocation = atoi(value);
  return true;
}

// use altitude 0|1
bool cmd_19(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useAltitude = atoi(value);
  return true;
}

// use speed 0|1
bool cmd_20(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useSpeed = atoi(value);
  return true;
}

// use cog (course over ground) 0|1
bool cmd_21(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useCog = atoi(value);
  return true;
}

// use sats 0|1
bool cmd_22(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useSats = atoi(value);
  return true;
}

// use HDOP 0|1
bool cmd_23(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useHDOP = atoi(value);
  return true;
}

// use GSM Signal Quality 0|1
bool cmd_24(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useGSMSignal = atoi(value);
  return true;
}

// use time from GSM 0|1
bool cmd_25(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useGsmTime = atoi(value);
  return true;
}

// use GSM Location 0|1
bool cmd_26(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useGSMLocation = atoi(value);
  return true;
}

// use transmit with SMS 0|1
bool cmd_27(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.transmitWithSMS = atoi(value);
  return true;
}

// set SMS number (string)
bool cmd_28(char *value) {
  strcpy(cfg.SMSNumber, value);
  return true;
}

// use transmit log if full 0|1
bool cmd_29(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.transmitLogIfFull = atoi(value);
  return true;
}

// use persist log with mcu reset 0|1
bool cmd_30(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.persistLog = atoi(value);
  return true;
}

// use batt 0|1
bool cmd_31(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useBatt = atoi(value);
  return true;
}

// use use VBAT 0|1
bool cmd_32(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useVBAT = atoi(value);
  return true;
}

// use VSYS 0|1
bool cmd_33(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useVSYS = atoi(value);
  return true;
}

// use VIN 0|1
bool cmd_34(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useVIN = atoi(value);
  return true;
}

// set sim card apn (string)
bool cmd_35(char *value) {
  if (strlen(value) < 3) return false;
  strcpy(cfg.apn, value);
  return true;
}

// set apn user (string)
bool cmd_36(char *value) {
  if (strcmp(value, "0")) strcpy(value, "");
  strcpy(cfg.apnUser, value);
  return true;
}

// set apn pass (string)
bool cmd_37(char *value) {
  if (strcmp(value, "0")) strcpy(value, "");
  strcpy(cfg.apnPass, value);
  return true;
}

// set remote server (string)
bool cmd_38(char *value) {
  strcpy(cfg.remoteServer, value);
  return true;
}

// set remote port (number as string)
bool cmd_39(char *value) {
  if (valueIsBetween(value, 3000, 65000) == false) return false;
  strcpy(cfg.remotePort, value);
  return true;
}

// Save conf
bool cmd_40(char *value) {
  Trackio::saveConf();
  return true;
}

// set opmode OP_RST
bool cmd_41(char *value) {
  cfg.opmode = OP_RST;
  return true;
}

// set master GPIO (arduino ide pin number)
bool cmd_42(char *value) {
  if (valueIsBool(value) == false) return false;
  digitalWrite(RH_masterGPIO, atoi(value));
  return true;
}

// enable GPIO
bool cmd_43(char *value) {
  if (valueIsBetween(value, 0, 100) == false) return false;
  digitalWrite(atoi(value), HIGH);
  return true;
}

// disable GPIO
bool cmd_44(char *value) {
  if (valueIsBetween(value, 0, 100) == false) return false;
  digitalWrite(atoi(value), LOW);
  return true;
}

// set GPIO as input
bool cmd_45(char *value) {
  if (valueIsBetween(value, 0, 100) == false) return false;
  pinMode(atoi(value), INPUT);
  return true;
}

// set GPIO as output
bool cmd_46(char *value) {
  if (valueIsBetween(value, 0, 100) == false) return false;
  pinMode(atoi(value), OUTPUT);
  return true;
}

// use gas  0|1
bool cmd_47(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useGas = atoi(value);
  return true;
}

// use dust sensor  0|1
bool cmd_48(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useDustSensor = atoi(value);
  return true;
}

// use sonar  0|1
bool cmd_49(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useSonar = atoi(value);
  return true;
}

// use serial ext  0|1
bool cmd_50(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useSerialExt = atoi(value);
  return true;
}

// send serial string to SerialExt (string)
bool cmd_51(char *value) { return Trackio::sendSerialExt(value); }

// use accelerometer  0|1
bool cmd_52(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.useAccel = atoi(value);
  return true;
}

// set accel threshold time (milliseconds)
bool cmd_53(char *value) {
  if (valueIsBetween(value, 100, 20000) == false) return false;
  cfg.accelTime = atoi(value);
  return true;
}

// transmit accel now 0|1
bool cmd_54(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.transmitAccelNow = atoi(value);
  return true;
}

// accel GPIO 0|1
bool cmd_55(char *value) {
  if (valueIsBool(value) == false) return false;
  cfg.accelGPIO = atoi(value);
  return true;
}

// sleep now
bool cmd_56(char *value) {
  int secs = atoi(value);
  Trackio::sleepSecs = secs;
  cfg.opmode = OP_LOW;
  return true;
}

typedef bool (*FP)(char *x);
const int maxCmdId = 57;
const FP availableCommands[maxCmdId] = {
    &cmd_0,  &cmd_1,  &cmd_2,  &cmd_3,  &cmd_4,  &cmd_5,  &cmd_6,  &cmd_7,
    &cmd_8,  &cmd_9,  &cmd_10, &cmd_11, &cmd_12, &cmd_13, &cmd_14, &cmd_15,
    &cmd_16, &cmd_17, &cmd_18, &cmd_19, &cmd_20, &cmd_21, &cmd_22, &cmd_23,
    &cmd_24, &cmd_25, &cmd_26, &cmd_27, &cmd_28, &cmd_29, &cmd_30, &cmd_31,
    &cmd_32, &cmd_33, &cmd_34, &cmd_35, &cmd_36, &cmd_37, &cmd_38, &cmd_39,
    &cmd_40, &cmd_41, &cmd_42, &cmd_43, &cmd_44, &cmd_45, &cmd_46, &cmd_47,
    &cmd_48, &cmd_49, &cmd_50, &cmd_51, &cmd_52, &cmd_53, &cmd_54, &cmd_55,
    &cmd_56
};

#endif