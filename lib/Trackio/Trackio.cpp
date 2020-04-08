/*
 * @brief Trackio Class
 *
 * Copyright 2020 M2M System Source S.L
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file Trackio.cpp
 * @author Jordi Enguídanos <jenguidanos@rhomb.io>
 */

#include <Arduino.h>
#include "Trackio.h"
#include <FlashStorage.h>
#include <Adafruit_SleepyDog.h>
#include <Wire.h>
#include <SPI.h>
#include "battery.h"
#include "Readline.h"
#include "Rhio-RGB.h"
#include "TimeLib.h"

#include "commands.h"

// declare static members from Trackio class
unsigned long Trackio::sleepSecs = 0;

// Utilizado en Trackio::sendAt()
ReadLine readLine;

#if RH_useBME680 == true
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
Adafruit_BME680 bme;
#endif

#if RH_useBMP280 == true
#include "Adafruit_BMP280.h"
Adafruit_BMP280 bmp;  // I2C
#endif

#if RH_useBME280 == true
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;
#endif

#if RH_useCCS811 == true
#include "Adafruit_CCS811.h"
Adafruit_CCS811 ccs;
#endif

#if RH_useDHT == true
#include "DHT.h"
DHT dht;
#endif

#if RH_useCAN == true
#include <OBD.h>
COBDI2C obd;
#endif

#if RH_useSHT20 == true
#include "DFRobot_SHT20.h"
DFRobot_SHT20 sht20;
#endif

#if RH_useDustSensor == true
#include "DustSensor.h"
DustSensor ds;
#endif

#if RH_useSonar == true
#include <NewPing.h>
// NewPing setup of pins and maximum distance.
NewPing sonar(RH_SonarTriggerPin, RH_SonarEchoPin, RH_SonarMAX_DIST);
#endif

#if RH_useGpsL86 == true
#include <TinyGPS++.h>
TinyGPSPlus tinygps;
#endif

#if RH_useRTC == true
#if RH_useDS3231 == true
#include "RTClib.h"
RTC_DS3231 rtc;
#endif
#endif

#if RH_useSDI12 == true
#include <SDI12.h>
SDI12 sdi12(RH_SDI12Pin);

const char SDI_CMD_ID[] = "0I!";         // read probe ID
const char SDI_CMD_ADDR[] = "?!";        // read probe address
const char SDI_CMD_READ[] = "0D0!";      // read command
const char SDI_CMD_SALI[] = "0C1!";      // salinity
const char SDI_CMD_TEMP[] = "0C2!";      // temp in Cº
const char SDI_CMD_HUMI[] = "0C3!";      // moisture without salinity
const char SDI_CMD_HUMI_SALI[] = "0C!";  // moisture with salinity

bool sdiHasBeenSent = false;
char sdiId[30];
char sdiBuffer[100];
#endif

#if RH_useAccel == true
#if RH_useLIS2HH12 == true
#include <rhio-LIS2HH12.h>
LIS2HH12 lis = LIS2HH12();
#endif
int gravity = 1;
int AcX, AcY, AcZ;
#endif

#include "wiring_private.h"  // pinPeripheral() function
Uart SerialExt(&sercom2, RH_serialExtRX, RH_serialExtTX, SERCOM_RX_PAD_1,
               UART_TX_PAD_0);
void SERCOM2_Handler() { SerialExt.IrqHandler(); }

// --
#define OK (char *)"OK"
#define ERROR (char *)"ERROR"
#define READY (char *)"READY"
#define CONNECT_OK (char *)"CONNECT OK"

/**
 * @brief Comprueba si una cadena está compuesta por números
 *
 * @param number
 * @return true
 * @return false
 */
bool charIsValidNumber(const char number[]) {
  int i = 0;
  int len = strlen(number);
  for (; i < len; i++) {
    if (!isdigit(number[0])) {
      return false;
    }
  }

  return true;
}

// Configuración general
struct Conf cfg;
FlashStorage(confStore, Conf);

/**
 * @brief Almacena la lectura del Serial del modem. Se utiliza principalmente
 * en Trackio::sendCommand() y Trackio::sendAt()
 */
char buffer[UART_BUFFER_SIZE];

/**
 * @brief Cada vez que un comando AT falla se suma 1. Al llegar a X se hará
 * un reinicio del microcontrolador a través de Watchdog
 *
 * El valor de cuantos fallos se admite está en el método Trackio::sendCommand()
 */
uint8_t modemSerialFails = 0;

/**
 * @brief Lleva un conteo de cuantas veces falla una conexión TCP.
 *
 * A partir de determinado número se reiniciará el micro utilizando watchdog.
 * El valor de cuantos fallos se admite está en el método Trackio::openTcp()
 */
uint8_t openTcpFails = 0;

Trackio::Trackio() { Trackio::blinkTimes = 0; }

bool Trackio::begin() {
  Trackio::configureIOs();
  Trackio::blink(4, 300);

  SerialMon.begin(SerialMonBauds);
  while (!SerialMon) {
  }
  SerialSim.begin(SerialSimBauds);
  while (!SerialSim) {
  }

  __(F(""));
  __(F("#############################################"));
  _(F("           - Trackio "));
  _(F(VERSION));
  __(F(" START -"));
  __(F("#############################################"));
  __(F(""));

#if RH_useRGBLed
  rh_rgbSetup(0x43);
#endif

  Trackio::loadConf();

  if (cfg.useSerialExt) {
    // configure SerialExt
    SerialExt.begin(RH_serialExtBauds);
    while (!SerialExt) {
    }
    pinPeripheral(RH_serialExtRX, PIO_SERCOM_ALT);
    pinPeripheral(RH_serialExtTX, PIO_SERCOM_ALT);
    SerialExt.println("SerialExt Ready");
  }

#if RH_battMode == 3
  Trackio::configureTLA2024();
#endif

#if RH_useBatt
  Trackio::getBattery();
  if (Trackio::checkLowBattery()) {
    // salimos del begin, el startup del main.cpp inciará modo OP_LOW
    return true;
  }
#endif

#if RH_useRTC
  Trackio::setupRTC();
#endif

  // encendemos el modem
  if (!Trackio::powerOn()) {
    return false;
  }

  // comprobamos que se envian comandos AT
  if (!Trackio::checkModem()) {
    return false;
  }

  if (cfg.useGps) {
    if (!Trackio::powerOnGps()) {
      // revisar pin GPS_EN y conexión del módulo esclavo
      return false;
    }
  }

  Trackio::setupSensors();
  Trackio::printInfo();  // Requerido. NO BORRAR!
  Trackio::enableGSMClock();

  Trackio::ready = true;

  return true;
}

void Trackio::configureIOs() {
  // Disable PCB LED
  if (RH_useLED) {
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);
  }

  if (RH_useBuzzer) {
    pinMode(RH_buzzPin, OUTPUT);
    digitalWrite(RH_buzzPin, LOW);
  }

  // SIMCOM PINS
  pinMode(GSM_PWRKEY, OUTPUT);
  pinMode(GSM_PWREN, OUTPUT);
  pinMode(GPS_EN, OUTPUT);
  pinMode(GSM_STATUS, INPUT);

  // Master GPIO
  pinMode(RH_masterGPIO, OUTPUT);  // actuador externo (rele, led...)

  // rhomb.io Duino Zero mux control. Should be put to INPUT to use some GPIOS
  // See rhio-pinmap.h for details
  pinMode(MUX_SW, OUTPUT);

  if (RH_useNMIForWakeup) {
    pinMode(NMI, INPUT_PULLUP);
  }
}

void Trackio::loadConf() {
  _title("Cargar Configuración");
  Conf conf;
  conf = confStore.read();

  if (conf.eeprom != 1) {
    __(F("Cargamos flash por primera vez"));
    // RECUERDA! No modifiques nada aquí, revisa `lib/trackio/static-conf.h`
    cfg.eeprom = 1;
    cfg.battMode = RH_battMode;
    cfg.opmode = RH_opmode;
    cfg.primaryOpMode = RH_primaryOpMode;
    cfg.deepSleep = RH_deepSleep;
    cfg.sleep = RH_sleep;
    cfg.requiredVbat = RH_requiredVbat;
    cfg.requiredVin = RH_requiredVin;
    cfg.requiredVsys5v = RH_requiredVsys5v;
    cfg.useAccel = RH_useAccel;
    cfg.accelTime = RH_accelTime;
    cfg.transmitAccelNow = RH_transmitAccelNow;
    cfg.accelGPIO = RH_accelGPIO;

    cfg.aliveTimer = RH_aliveTimer;
    cfg.tickTimer = RH_tickTimer;
    cfg.keepTCPOpen = RH_keepTCPOpen;
    cfg.transmitAlways = RH_transmitAlways;
    cfg.transmitWithSMS = RH_transmitWithSMS;
    strcpy(cfg.SMSNumber, RH_SMSNumber);

    cfg.persistLog = RH_persistLog;
    cfg.transmitLogIfFull = RH_transmitLogIfFull;

    cfg.useBl = RH_useBl;

    cfg.useTemp = RH_useTemp;
    cfg.useHumidity = RH_useHumidity;
    cfg.usePressure = RH_usePressure;
    cfg.useCO = RH_useCO;
    cfg.useCO2 = RH_useCO2;
    cfg.useVOC = RH_useVOC;
    cfg.useGpsTime = RH_useGpsTime;
    cfg.useGas = RH_useGas;
    cfg.useDustSensor = RH_useDustSensor;
    cfg.useSonar = RH_useSonar;
    cfg.useGps = RH_useGps;
    cfg.useLocation = RH_useLocation;
    cfg.useAltitude = RH_useAltitude;
    cfg.useSpeed = RH_useSpeed;
    cfg.useCog = RH_useCog;
    cfg.useSats = RH_useSats;
    cfg.useHDOP = RH_useHDOP;
    cfg.useGSMSignal = RH_useGSMSignal;
    cfg.useGsmTime = RH_useGsmTime;
    cfg.useGSMLocation = RH_useGSMLocation;
    cfg.useBatt = RH_useBatt;
    cfg.useVBAT = RH_useVBAT;
    cfg.useVSYS = RH_useVSYS;
    cfg.useVIN = RH_useVIN;

    strcpy(cfg.apn, RH_apn);
    strcpy(cfg.apnUser, RH_apnUser);
    strcpy(cfg.apnPass, RH_apnPass);
    strcpy(cfg.remoteServer, RH_remoteServer);
    strcpy(cfg.remotePort, RH_remotePort);

    cfg.useCAN = RH_useCAN;
    cfg.useCANSpeed = RH_useCANSpeed;
    cfg.useCANBattery = RH_useCANBattery;
    cfg.useCANRPM = RH_useCANRPM;
    cfg.useCANFuel = RH_useCANFuel;

    cfg.useRTC = RH_useRTC;
    cfg.useSDI12 = RH_useSDI12;

    cfg.useSerialExt = RH_useSerialExt;

    Trackio::saveConf();

  } else {
    __(F("La configuración ya existe."));
    cfg = conf;
  }

  if (cfg.masterGPIO == HIGH) {
    digitalWrite(RH_masterGPIO, HIGH);
  } else {
    digitalWrite(RH_masterGPIO, LOW);
  }

  Trackio::showLog();

  __(F("Configuración cargada..."));
}

void Trackio::saveConf() {
  // avisamos en consola de que vamos a guardar en memoría flash
  _title("OJO! SAVE CONF");
  confStore.write(cfg);
}

bool Trackio::powerOn() {
  _title(F("Power On")) uint8_t gsm_status;

  Trackio::gps.fix = false;
  Trackio::gprsOk = false;
  Trackio::cregOk = false;
  Trackio::tcpOk = false;

  Trackio::powerOff();
  Trackio::_delay(1000);
  digitalWrite(GSM_PWREN, HIGH);

  // 1 -> pulso powerKey
  for (char i = 0; i < 5; i++) {
    gsm_status = digitalRead(GSM_STATUS);
    if (gsm_status == HIGH) {
      __(F("  == GSM HIGH"));
      break;
    } else {
      __(F("  -> GSM LOW"));
      digitalWrite(GSM_PWRKEY, HIGH);
      Trackio::_delay(1000);
      digitalWrite(GSM_PWRKEY, LOW);
      Trackio::_delay(1000);
    }
  }

  // 2
  if (!gsm_status) {
    // No se ha podido encender el modem. Revisar que GSM_PWREN, GSM_STATUS y
    // GSM_PWRKEY son los pines correctos.
    return false;
  }

  // 3
  return true;
}

void Trackio::powerOff() {
  _title(F("Power OFF"));
  Trackio::ledStatus(0);

  digitalWrite(GSM_PWREN, LOW);
  Trackio::_delay(1000);
  digitalWrite(GPS_EN, LOW);
  Trackio::_delay(1000);
}

// #############################################################################

void Trackio::printInfo() {
  Trackio::getImei();
  Trackio::getIccid();
  Trackio::printPin();
  Trackio::sendCommand((char *)"AT+GSV");
  Trackio::getBattery();
}

void Trackio::getImei() {
  if (Trackio::sendAt((char *)"AT+CGSN", 1)) {
    if (strlen(buffer) == 15) {
      strcpy(Trackio::imei, buffer);
      ___(F("  == IMEI: "), Trackio::imei);
      return;
    }
  }

  strcpy(Trackio::imei, "x");
  ___(F("   == IMEI: "), F("FAIL"));
}

void Trackio::printPin() { Trackio::sendAt((char *)"AT+CPIN?", 1); }

void Trackio::getIccid() {
  if (!Trackio::sendAt((char *)"AT+CCID", 1, ERROR)) {
    ___(F("  == ICCID: "), buffer);
    if (strlen(buffer) < 23) {
      strcpy(Trackio::iccid, buffer);
      return;
    }
  }

  __(F("  == ERROR ICCID"));
}

// #############################################################################

void Trackio::getBattery() {
  Trackio::vbat = 0;
  Trackio::vin = 0;
  Trackio::vsys_5v = 0;

  if (cfg.battMode == 1) {
    Trackio::getSimcomBattery();
  } else if (cfg.battMode == 2) {
    Trackio::getAnalogBattery();
  } else if (cfg.battMode == 3) {
    Trackio::getTLA2024Battery();
  }

  Trackio::checkLowBattery();
}

void Trackio::getSimcomBattery() {
  _title(F("Get MODEM Battery"));

  if (Trackio::sendAt((char *)"AT+CBC", 2)) {
    // buffer = +CBC: 0,21,3571
    char *split;
    split = strtok(buffer, ",");
    split = strtok(NULL, ",");  // porcentaje de batería
    split = strtok(NULL, ",");  // batería

    Trackio::vbat = atoi(split);
  }
}

bool Trackio::checkLowBattery() {
  char low = 0;

  if (cfg.requiredVbat > 0 && Trackio::vbat < cfg.requiredVbat) {
    low = 1;
    __(F("  -> WARNING LOW VBAT"));
  }
  if (cfg.requiredVin > 0 && Trackio::vin < cfg.requiredVin) {
    low = 1;
    __(F("  -> WARNING LOW VIN"));
  }
  if (cfg.requiredVsys5v > 0 && Trackio::vsys_5v < cfg.requiredVsys5v) {
    low = 1;
    __(F("  -> WARNING LOW VSYS"));
  }

  if (low == 1) {
    __(F("  ***** Entramos en LOW MODE *****"));
    // reset serial fails, evita reinicio antes de deep sleep
    // establecemos el modo de bajo consumo y apagamos el simcom
    modemSerialFails = 0;
    cfg.opmode = OP_LOW;
    Trackio::powerOff();
    return true;
  }

  cfg.opmode = cfg.opmode == OP_LOW ? OP_STARTUP : cfg.opmode;
  return false;
}

void Trackio::getAnalogBattery() {
  _title(F("Get Analog Battery"));

  uint16_t lectura_mV;

  digitalWrite(MUX_SW, LOW);
  delay(10);
  lectura_mV = (float)Trackio::readAnalogBatt(VBAT_PIN);
  Trackio::vbat = (float)(lectura_mV / VBAT_aux);
  _(F("  == VBAT: "));
  __(Trackio::vbat);

  lectura_mV = (float)Trackio::readAnalogBatt(VSYS_PIN);
  Trackio::vsys_5v = (float)(lectura_mV / VSYS_aux);
  _(F("  == VSYS: "));
  __(Trackio::vsys_5v);

  digitalWrite(MUX_SW, HIGH);
  delay(10);
  lectura_mV = (float)Trackio::readAnalogBatt(VIN_PIN);
  Trackio::vin = (float)(lectura_mV / VIN_aux);
  _(F("  == VIN: "));
  __(Trackio::vin);
}

uint16_t Trackio::readAnalogBatt(byte adc_pin) {
  int readingADC;
  float reading_mV;
  uint16_t result;

  readingADC = analogRead(adc_pin);
  readingADC = (int)readingADC;
  reading_mV = (float)readingADC * (float)mV_step_used;
  reading_mV = (float)reading_mV * 1000;
  result = (uint16_t)reading_mV;
  return (result);
}

void Trackio::getTLA2024Battery() {
#if RH_battMode == 3
  _title(F("Get TLA2024 Battery"));

  Trackio::vbat = (int)Trackio::readTLA2024Battery(AIN0, VBAT_aux_HALLEY);
  ___(F("  == VBAT: "), Trackio::vbat);

  Trackio::vsys_5v = (int)Trackio::readTLA2024Battery(AIN2, VSYS_aux_HALLEY);
  ___(F("  == VSYS: "), Trackio::vsys_5v);

  Trackio::vin = (int)Trackio::readTLA2024Battery(AIN1, VIN_aux_HALLEY);
  ___(F("  == VIN: "), Trackio::vin);
#else
  __(F("ERROR: RH_battMode debe ser =3 para lecturas con TLA2024"));
#endif
}

void Trackio::configureTLA2024() {
#if RH_battMode == 3
  adc.begin();
  adc.setFSR(20);  //-> ± 2.048 (default)
  adc.setMux(4);   // point to channel AIN0
  adc.setDR(1);    // DR = 250 SPS
  adc.setMode(CONT);
#endif
}

float Trackio::readTLA2024Battery(uint8_t channel, float aux) {
#if RH_battMode == 3
  float result;
  adc.setMux(channel);
  float val = adc.analogRead();
  result = (float)(val / aux);
  return result;
#else
  return 0;
#endif
}

// #############################################################################
bool Trackio::checkStatus() {
  if (!Trackio::checkCreg()) {
    __(F("  == FAIL checkCreg"));
    return false;
  }

  return true;
}

void Trackio::getSignalStrength() {
  char x[10] = "AT+CSQ";

  if (Trackio::sendAt(x, 1)) {
    char *split;
    split = strtok(buffer, " ");
    split = strtok(NULL, ",");
    strcpy(Trackio::gsmSignal, split);
    ___(F("  == signal: "), Trackio::gsmSignal);
  }
}

bool Trackio::checkCreg() {
  if (!Trackio::atOk) return false;
  Trackio::cregOk = false;

  // Verificamos el creg 20 veces
  for (int i = 0; i < 20; i++) {
    Trackio::getSignalStrength();
    if (!Trackio::sendAt((char *)"AT+CREG?", 1)) continue;
    if (strstr(buffer, ",5") || strstr(buffer, ",1")) {
      Trackio::cregOk = true;
      return true;
    }

    Trackio::_delay(1000);
  }

  return false;
}

bool Trackio::checkModem() {
  _title("checkModem") Trackio::atOk = false;

  Trackio::sendAt((char *)"ATE0", 1);  // no echo

  Trackio::sendCommand((char *)"AT+CFUN=0");
  Trackio::_delay(600);
  Trackio::sendCommand((char *)"AT+CFUN=1");
  Trackio::_delay(600);

  if (Trackio::sendAt((char *)"AT", 1, OK, 200)) {
    Trackio::atOk = true;
    __(F("  -> modem ok"));
  } else {
    __(F("  -> ERROR modem"));
  }

  Trackio::sendCommand((char *)"AT");

  return Trackio::atOk;
}

// #############################################################################
bool Trackio::openTcp() {
  Trackio::tcpOk = false;  // reset

  // Obtener el estado del servicio GPRS
  if (!Trackio::gprsOk) {
    if (!Trackio::openGprs()) {
      return false;
    }
  }

  char cmd[100];
  sprintf(cmd, "AT+CIPSTART=\"%s\",\"%s\",\"%s\"", RH_transmissionProtocol,
          cfg.remoteServer, cfg.remotePort);

  if (Trackio::sendAt(cmd, 2)) {
    ___(F("  == CIPSTART: "), buffer);
    Trackio::_delay(1000);

    // si ya está abierto Sim868 devolverá OK, sino READY CONNECT
    if (strstr(buffer, OK) || strstr(buffer, READY)) {
      openTcpFails = 0;
      __(F("  == TCP OK"));
      Trackio::ledStatus(1);
      Trackio::tcpOk = true;

      return true;
    }
  }

  __(F("  == ERROR TCP"));
  openTcpFails++;
  if (openTcpFails == 3) {
    __(F("El TCP ha fallado en multiples ocasiones - Hard Reset!"));
    Trackio::hardReset();
  }

  return false;
}

void Trackio::closeTcp(int mode) {
  char cmd[15];
  sprintf(cmd, "AT+CIPCLOSE=%i", mode);
  Trackio::sendAt(cmd, 1);
  Trackio::tcpOk = false;
  Trackio::ledStatus(0);
}

bool Trackio::tcpIsOpen() {
  Trackio::tcpOk = false;
  bool open = Trackio::sendAt((char *)"AT+CIPSTATUS", 2, CONNECT_OK);
  if (open) {
    Trackio::tcpOk = true;
  } else {
    if (strstr(buffer, "PDP DEACT")) {
      __(F("  == Hemos perdido conexión GPRS -> OP_STARTUP"));
      cfg.opmode = OP_STARTUP;
    }
  }

  return open;
}

bool Trackio::sayHello() {
  char hello[120];

  if (Trackio::firstGreetingsSent) {
    sprintf(hello, "@id:%s$", Trackio::imei);
  } else {
    sprintf(hello, "@id:%s,v:%s,iccid:%s,io:%d,iostatus:%d", Trackio::imei,
            VERSION, Trackio::iccid, RH_masterGPIO, digitalRead(RH_masterGPIO));

#if RH_useSDI12
    strcat(hello, ",sdi:");
    strcat(hello, sdiId);
#endif

    // caracter de cierre
    strcat(hello, "$");
  }

  if (Trackio::transmit(hello)) {
    Trackio::firstGreetingsSent = true;
    return true;
  }

  return false;
}

bool Trackio::prepareForTransmission() {
  if (Trackio::tcpIsOpen()) return true;
  if (!Trackio::checkCreg()) {
    __(F("  == prepareForTransmission: NO CREG"));
    return false;
  }

  if (!Trackio::gprsIsOpen()) {
    if (!Trackio::openGprs()) {
      __(F("  == prepareForTransmission: No se ha podido iniciar GPRS"));
      return false;
    }
  }

  if (!Trackio::openTcp()) {
    __(F("  == prepareForTransmission: Fallo al abrir TCP"));
    return false;
  }

  return true;
}

bool Trackio::transmit(char *msg) { return Trackio::transmit(msg, true); }

bool Trackio::transmit(char *msg, bool processResponse) {
  // comando para el mondo
  char cipsend[30];
  sprintf(cipsend, "AT+CIPSEND=%i", strlen(msg));

  // enviamos cipsend
  Trackio::sendCommand(cipsend);
  if (strstr(buffer, ERROR)) {
    __(F("  == CIPSEND FAIL - abort"));
    return false;
  }

  // enviamos los datos (trama)
  Trackio::sendCommand(msg);

  // verificamos respuesta
  if (strstr(buffer, "SEND OK")) {
    Trackio::afterTransmit(msg, processResponse);
    return true;
  }

  __(F("  == ERROR SEND"));
  return false;
}

void Trackio::afterTransmit(char *msg, bool processResponse) {
  // Procesamos la respuesta. `Trackio::processBuffer` accederá a la variable
  // `buffer`, la cual contiene la respuesta devuelta por el servidor tras
  // la transmisión
  if (processResponse) {
    Trackio::processBuffer();
  }

  // reseteamos flag de movimiento solo si se ha enviado su estado
  if (Trackio::moving && strstr(msg, "vbra:1")) {
    Trackio::moving = false;
  }
}

// #############################################################################
void Trackio::readDebugBuffer() {
  if (SerialMon.available()) {
    Trackio::emptyBuffer();
    int count = 0;
    while (SerialMon.available() && count < SERIAL_BUFFER_SIZE) {
      buffer[count] = (char)SerialMon.read();
      count++;
      Trackio::_delay(10);
    }

    ___(F("buffer: "), buffer);
    Trackio::processBuffer();
  }
}

void Trackio::readModemBuffer() {
  Trackio::emptyBuffer();

  if (SerialSim.available()) {
    while (SerialSim.available()) {
      char *serialBuffer = readLine.feed(&SerialSim);
      if (strlen(buffer)) {
        strcat(buffer, "\n");
      }
      strcat(buffer, serialBuffer);
    }
  }

  if (strlen(buffer)) {
    Trackio::processBuffer();
  }
}

void Trackio::processBuffer() {
  char *bufferLine;

  bufferLine = strtok(buffer, "\n");
  while (bufferLine != NULL) {
    ___(F("LINE: "), bufferLine);
    if (strlen(bufferLine) < 2) {
      bufferLine = strtok(NULL, "\n");
      continue;
    }

    // Comprobamos si es un comando
    if (Trackio::isCommand(bufferLine)) {
      // extraemos el comando de la respuesta tcp en la variable
      // Trackio::command
      Trackio::parseCommand(bufferLine);
      if (Trackio::processCommand()) {
        ___(F("  == Command PROCESSED: "), Trackio::command.id);
        char ack[150];
        sprintf(ack, "#ack|id:%s;io:%i;props:%s$", Trackio::command.id,
                digitalRead(RH_masterGPIO), Trackio::command.propsProcessed);
        ___(F("ID: "), Trackio::command.id);
        if (!strstr(Trackio::command.id, "noack")) {
          Trackio::transmit(ack, false);
        }
      }

      // El servidor ha cerrado la conexión por una razón que desconocemos
    } else if (strstr(bufferLine, "CLOSED")) {
      Trackio::closeTcp(1);
      __(F("  == CONEXION CERRADA POR SERVER"));
      // Tratamos de reconectar
      if (cfg.keepTCPOpen) {
        if (Trackio::prepareForTransmission()) {
          Trackio::sayHello();
        }
        // si no lo conseguimos op_auto() seguirá intentándolo
      }

      // Se está intentado realizar una conexión bluetooth
    } else if (strstr(bufferLine, "+BTCONNECTING")) {
      if (Trackio::sendAt((char *)"AT+BTACPT=1", 1, OK)) {
        __(F("Conexión aceptada"));
      }

      // Entra texto desde bluetooth
    } else if (strstr(bufferLine, "+BTSPPDATA:")) {
      Trackio::processBlData(bufferLine);

    } else if (strstr(bufferLine, "KO")) {
      // el dispositivo ha sido "expulsado" de la plataforma.
      // establecemos el modo OP_STARTUP para iniciar nuevo registro
      __(F("El servidor nos ha expulsado :("));
      cfg.opmode = OP_STARTUP;
    } else {
      __(F("Nada que procesar..."));
    }

    // siguiente línea del buffer
    bufferLine = strtok(NULL, "\n");
  }
}

void Trackio::emptyBuffer() { memset(buffer, 0, sizeof buffer); }

bool Trackio::isCommand(char *command) {
  char cmd[120];
  strcpy(cmd, command);

  char first = cmd[0];
  char last = cmd[strlen(cmd) - 1];

  if (first != '$') return false;
  if (last != '#') return false;
  if (!strstr(cmd, "|")) return false;

  __(F("IS COMMAND"));
  return true;
}

void Trackio::parseCommand(char *cmd) {
  char command[CMD_BUFFER_SIZE];

  // clear previous
  memset(Trackio::command.id, 0, sizeof Trackio::command.id);
  memset(Trackio::command.props, 0, sizeof Trackio::command.props);

  uint8_t len = strlen(cmd);
  int i;
  int count = 0;
  char *splitCmd;

  for (i = 1; i < len - 1; i++) {
    if (cmd[i] != '#' && cmd[i] != '$' && count < CMD_BUFFER_SIZE) {
      command[count] = cmd[i];
      count++;
    }
  }

  command[count] = '\0';

  splitCmd = strtok(command, "|");  // ID
  if (splitCmd != NULL) strcpy(Trackio::command.id, splitCmd);

  splitCmd = strtok(NULL, "|");  // COMMAND
  if (splitCmd != NULL) strcpy(Trackio::command.props, splitCmd);

  splitCmd = strtok(NULL, "|");  // size (number)
  if (splitCmd != NULL) Trackio::command.size = atoi(splitCmd);

  ___(F("COMMAND ID: "), Trackio::command.id);
  ___(F("COMMAND PROPS: "), Trackio::command.props);
  ___(F("COMMAND SIZE: "), Trackio::command.size);
}

bool Trackio::processCommand() {
  ___(F("Size of command: "), strlen(Trackio::command.props));
  strcpy(Trackio::command.propsProcessed, "");

  if (strlen(Trackio::command.props) != Trackio::command.size) {
    __(F("Command validation failed"));
    return false;
  } else {
    __(F("Command size is valid"));
  }

  int countProps = 0;
  char props[MAX_PARAMS_PER_CMD][CMD_PARAM_SIZE];

  char *splitProps = strtok(Trackio::command.props, ";");
  while (splitProps) {
    ___(F("propValue: "), splitProps);  // splitProps = 1:0
    strcpy(props[countProps], splitProps);
    splitProps = strtok(NULL, ";");
    countProps++;
  }

  for (int i = 0; i < MAX_PARAMS_PER_CMD; i++) {
    if (!strlen(props[i])) break;

    char *splitPropValue = strtok(props[i], ":");

    char prop[3];
    strcpy(prop, splitPropValue);
    ___(F("prop: "), prop);
    if (!charIsValidNumber(prop)) {
      // CRITICAL FAIL
      // las propiedades deben tener valores númericos para corresponder
      // uno de los métodos cmd_X() definidos al inicio de Trackio.cpp
      ___(F("PROP NO ES UNA PROPIEDAD VALIDA: "), prop);
      continue;
    }

    char value[30];
    splitPropValue = strtok(NULL, ":");
    strcpy(value, splitPropValue);
    ___(F("value: "), value);

    if (Trackio::applyConf(prop, value)) {
      strcat(Trackio::command.propsProcessed, prop);
      strcat(Trackio::command.propsProcessed, ",");
      _(F("Commando property processed: "));
      _(prop);
      ___(F(":"), value);
    } else {
      _(F("Fail processing: "));
      _(prop);
      ___(F(":"), value);
    }
  }

  return true;
}

bool Trackio::applyConf(char *confId, char *value) {
  const int id = atoi(confId);
  if (id < 0 || id > maxCmdId) return false;
  return availableCommands[id](value);
}

// #############################################################################

void Trackio::createMessage() {
  _title(F("Create Message"));

  // clear previous message
  memset(Trackio::message, 0, sizeof Trackio::message);

  // Headers to transmit with SMS or TCP
  if (cfg.transmitWithSMS) {
    sprintf(Trackio::message, "%s|s|", Trackio::imei);
  } else {
    strcpy(Trackio::message, "s|");
  }

  // Add data to the message. The order of the factors matters.
  Trackio::addGPSToMessage();
  Trackio::addGSMToMessage();
  Trackio::addTimeToMessage();
  Trackio::addBattToMessage();
  Trackio::addSensingToMessage();
  Trackio::addCANToMessage();

  ___(F("  == Trackio::message: "), Trackio::message);
  ___(F("  == msg size: "), strlen(Trackio::message));
}

void Trackio::addGSMToMessage() {
  char msg[40];

  if (cfg.useGSMSignal) {
    Trackio::getSignalStrength();
    strcpy(msg, "gsm:");
    strcat(msg, Trackio::gsmSignal);
    Trackio::addToMessage(msg);
  }

  if (cfg.useGSMLocation) {
    if (!Trackio::gps.fix && Trackio::getGsmLocation()) {
      strcpy(msg, "gloc:");
      strcat(msg, Trackio::gsmLoc.lat);
      strcat(msg, ",");
      strcat(msg, Trackio::gsmLoc.lon);
      strcat(msg, ",");
      strcat(msg, Trackio::gsmLoc.precission);
      Trackio::addToMessage(msg);
    }
  }
}

void Trackio::addGPSToMessage() {
  if (!cfg.useGps) return;

  // The sum of all GPS parameters does not exceed this value.
  char msg[30];
  // Get GPS data
  Trackio::getGps();

  // If there's no fix, there's nothing to do.
  if (!Trackio::gps.fix) return;

  // Add data to the message
  if (cfg.useLocation) {
    strcpy(msg, "loc:");
    strcat(msg, Trackio::gps.lat);
    strcat(msg, ",");
    strcat(msg, Trackio::gps.lon);
    Trackio::addToMessage(msg);
  }

  if (cfg.useAltitude) {
    strcpy(msg, "alt:");
    strcat(msg, Trackio::gps.alt);
    Trackio::addToMessage(msg);
  }

  if (cfg.useSpeed) {
    strcpy(msg, "sog:");
    strcat(msg, Trackio::gps.sog);
    Trackio::addToMessage(msg);
  }
}

void Trackio::addTimeToMessage() {
  char msg[20];

  if (cfg.useGpsTime && Trackio::gps.fix) {
    // Si no hay fix es posible que no haya podido fijar la hora.
    // Se necesitan al menos un par de satelites (aparecerá año 1980 si es el
    // caso)
    __(F("  Get GPS time"));
    strcpy(msg, "time:");
    strcat(msg, Trackio::gps.time);
    Trackio::addToMessage(msg);
  } else if (cfg.useGsmTime) {
    __(F("  Get GSM time"));
    Trackio::getGSMTime();
    sprintf(msg, "time:%ld", Trackio::getTimeNow());
    Trackio::addToMessage(msg);
  } else if (cfg.useRTC) {
    __(F("  Get RTC time"));
    long now = Trackio::getTimeNow();
    ___(F("  -> Time: "), now);
    sprintf(msg, "time:%ld", now);
    Trackio::addToMessage(msg);
  }
}

void Trackio::addCANToMessage() {
#if RH_useCAN == true
  if (!Trackio::CANEnabled) {
    if (obd.init()) {
      Trackio::CANEnabled = true;
    } else {
      return;
    }
  }

  if (cfg.useCAN && Trackio::CANEnabled) {
    char msg[20];

    if (cfg.useCANSpeed) {
      int speed;
      obd.readPID(PID_SPEED, speed);
      sprintf(msg, "cs:%d", speed);
      Trackio::addToMessage(msg);
    }

    if (cfg.useCANRPM) {
      int rpm;
      obd.readPID(PID_RPM, rpm);
      sprintf(msg, "cr:%d", rpm);
      Trackio::addToMessage(msg);
    }

    if (cfg.useCANBattery) {
      float volt = obd.getVoltage();
      sprintf(msg, "cb:%3.1f", volt);
      Trackio::addToMessage(msg);
    }

    if (!Trackio::CANVINSent) {
      char vinBuff[64];
      obd.getVIN(vinBuff, sizeof(vinBuff));
      if (strlen(vinBuff)) {
        strcpy(msg, "cvin:");
        strcat(msg, vinBuff);
        Trackio::addToMessage(msg);
      }
    }
  }
#endif
}

void Trackio::addBattToMessage() {
  if (!cfg.useBatt) return;

  char msg[20];

  Trackio::getBattery();
  if (cfg.useVBAT) {
    char vbat[10];
    sprintf(vbat, "%u", Trackio::vbat);
    strcpy(msg, "vb:");
    strcat(msg, vbat);
    Trackio::addToMessage(msg);
  }

  if (cfg.useVIN) {
    char vin[10];
    sprintf(vin, "%u", Trackio::vin);
    strcpy(msg, "vi:");
    strcat(msg, vin);
    Trackio::addToMessage(msg);
  }

  if (cfg.useVSYS) {
    char vsys[10];
    sprintf(vsys, "%u", Trackio::vsys_5v);
    strcpy(msg, "vs:");
    strcat(msg, vsys);
    Trackio::addToMessage(msg);
  }
}

void Trackio::addSensingToMessage() {
  char msg[20];
#if RH_useBME680 == true
  bme.performReading();
#endif

  if (cfg.useTemp) {
    Trackio::getTemperature();
    strcpy(msg, "temp:");
    strcat(msg, Trackio::sensors.temperature);
    Trackio::addToMessage(msg);
  }

  if (cfg.useHumidity) {
    Trackio::getHumidity();
    strcpy(msg, "humi:");
    strcat(msg, Trackio::sensors.humidity);
    Trackio::addToMessage(msg);
  }

  if (cfg.usePressure) {
    Trackio::getPressure();
    strcpy(msg, "pres:");
    strcat(msg, Trackio::sensors.pressure);
    Trackio::addToMessage(msg);
  }

  if (cfg.useVOC) {
    Trackio::getVoc();
    strcpy(msg, "voc:");
    strcat(msg, Trackio::sensors.voc);
    Trackio::addToMessage(msg);
  }

  if (cfg.useCO) {
    Trackio::getCO();
    strcpy(msg, "co:");
    strcat(msg, Trackio::sensors.co);
    Trackio::addToMessage(msg);
  }

  if (cfg.useCO2) {
    Trackio::getCO2();
    strcpy(msg, "co2:");
    strcat(msg, Trackio::sensors.co2);
    Trackio::addToMessage(msg);
  }

  if (cfg.useAccel && Trackio::moving) {
    strcpy(msg, "vbra:1");
    Trackio::addToMessage(msg);
  }

  if (cfg.useGas) {
#if RH_useMQ7 == true
    char mq7[8];
    sprintf(mq7, "mq7:%d", analogRead(RH_useMQ7Pin));
    Trackio::addToMessage(mq7);
#endif

#if RH_useMQ135 == true
    char mq135[12];
    sprintf(mq135, "mq135:%d", analogRead(RH_useMQ135Pin));
    Trackio::addToMessage(mq135);
#endif
  }

  if (cfg.useRaindrops > 0) {
    Trackio::getRaindrops();
    strcpy(msg, "rain:");
    strcat(msg, Trackio::sensors.raindrops);
    Trackio::addToMessage(msg);
  }

  if (cfg.useDustSensor) {
    Trackio::getDustSensor();
    strcpy(msg, "dust:");
    strcat(msg, Trackio::sensors.airParticles);
    Trackio::addToMessage(msg);
  }

  if (cfg.useSonar) {
    Trackio::getSonarDist();
    strcpy(msg, "dist:");
    strcat(msg, Trackio::sensors.distance);
    Trackio::addToMessage(msg);
  }

#if RH_useSDI12 == true
  if (cfg.useSDI12 && strlen(sdiId) > 1) {
    Trackio::addSDIToMessage();
  }
#endif
}

void Trackio::addToMessage(char part[]) {
  if ((strlen(part) + strlen(Trackio::message)) < MAX_MESSAGE_SIZE) {
    strcat(Trackio::message, part);
    strcat(Trackio::message, "$");
  } else {
    ___(F("El mensaje es muy largo para añadir: "), part);
  }
}

void Trackio::saveMessage() { Trackio::saveMessage(Trackio::message); }

void Trackio::saveMessage(char *message) {
  _title(F("Save Message"))

      if (cfg.transmitLogIfFull == 0) {
    __(F(" == Log desactivado (cfg.transmitLogIfFull == 0)"));
    return;
  }

  if (sizeof(message) >= RH_logBytes) {
    ___(F(" == ERROR El mensaje supera RH_logBytes: "), sizeof(message));
    _(F("b"));
    return;
  }

  int nextIndex = Trackio::getLogNextIndex();
  ___(F("  == Index: "), nextIndex);
  if (nextIndex < 0) return;
  strcpy(cfg.log[nextIndex], message);

  if (cfg.persistLog) {
    Trackio::saveConf();
  }
}

// #############################################################################

// not working... :(
bool Trackio::sendSMS(char *message) {
  if (!Trackio::sendAt((char *)"AT+CMGF=1", 1, OK)) {
    __(F("  == ERROR send SMS AT+CMGF"));
    return false;
  }
  delay(1000);

  Trackio::sendCommand((char *)"AT+CSMP?");
  delay(1000);

  Trackio::sendCommand((char *)"AT+CSCS=\"GSM\"");
  delay(1000);

  Trackio::sendCommand((char *)"AT+CMGS=\"+34691612793\"");
  delay(1000);

  SerialSim.write(message);
  Trackio::_delay(200);
  while (SerialSim.available()) SerialMon.write(SerialSim.read());
  SerialSim.write((byte)0x1A);
  Trackio::_delay(200);
  while (SerialSim.available()) SerialMon.write(SerialSim.read());
  SerialSim.println();

  if (!Trackio::sendCommand(message)) {
    __(F(" == ERROR Esto es un pedazo texto"));
    return false;
  }

  return true;
}

// #############################################################################

bool Trackio::openGprs() {
  if (cfg.transmitWithSMS) {
    return false;
  }

  // Obtener el estado del servicio GPRS
  if (Trackio::gprsIsOpen()) {
    // GPRS ya está abierto
    return false;
  }

  char apnCmd[150];
  sprintf(apnCmd, "AT+CSTT=\"%s\",\"%s\",\"%s\"", cfg.apn, cfg.apnUser,
          cfg.apnPass);

  if (!Trackio::sendAt(apnCmd, 1, OK)) return false;
  Trackio::_delay(1000);

  if (!Trackio::sendCommand((char *)"AT+CIICR")) {
    return Trackio::openGprs();
  }
  Trackio::_delay(1000);

  if (cfg.useGsmTime) {
    if (!Trackio::sendAt((char *)"AT+SAPBR=1,1", 1)) {
      __(F("  == ERROR al activar BEARER"));
      return Trackio::openGprs();
    } else {
      Trackio::sendCommand((char *)"AT+SAPBR=2,1");
    }
  }

  return Trackio::gprsIsOpen();
}

bool Trackio::gprsIsOpen() {
  if (!Trackio::cregOk) return false;
  Trackio::gprsOk = false;

  if (Trackio::sendAt((char *)"AT+CGATT?", 1, (char *)"+CGATT: 1")) {
    if (!Trackio::sendAt((char *)"AT+CIFSR", 1, ERROR)) {
      __(F("  == GPRS OK"));
      Trackio::gprsOk = true;
      return true;
    }
  }

  __(F("  == GPRS CLOSED"));
  return false;
}

// #############################################################################

bool Trackio::powerOnGps() {
  if (!cfg.useGps) return false;

#if RH_useGpsL86 == true
  // el módulo esclavo L86 en Socket 2 no tiene función poweron, siempre ON
  SerialL86.begin(9600);
  return true;
#else
  // Poweron en GPS SIM868
  if (!Trackio::sendAt((char *)"AT+CGNSPWR=1", 1, OK)) {
    return false;
  }
  Trackio::_delay(1500);

  // 2
  digitalWrite(GPS_EN, HIGH);

  return true;
#endif
}

void Trackio::getGps() {
  _title("getGps()");
  Trackio::resetGpsData();

  unsigned int now = millis();

  while ((millis() - now) < RH_waitForGps) {
#if RH_useGpsL86 == true
    // GPS WITH QUECTEL L86
    unsigned int now = millis();
    while (SerialL86.available() > 0 && (millis() - now) < 2000) {
      tinygps.encode(SerialL86.read());
    }

    if (tinygps.location.isValid()) {
      Trackio::gps.fix = true;
      sprintf(Trackio::gps.lat, "%.6f", tinygps.location.lat());
      sprintf(Trackio::gps.lon, "%.6f", tinygps.location.lng());
      sprintf(Trackio::gps.cog, "%.0f", tinygps.course.deg());
      sprintf(Trackio::gps.sats, "%lu", tinygps.satellites.value());
      sprintf(Trackio::gps.hdop, "%3.1f", tinygps.hdop.hdop());
      sprintf(Trackio::gps.alt, "%.0f", tinygps.altitude.meters());
      sprintf(Trackio::gps.sog, "%lu", tinygps.speed.value());
    }
#else
    // GPS WITH SIM868
    if (Trackio::sendAt((char *)"AT+CGNSINF", 1)) {
      char *split = strtok(buffer, " ");
      split = strtok(NULL, " ");
      Trackio::parseGps(split);
    }
#endif

    if (Trackio::gps.fix) {
      break;
    }

    delay(1000);
    __(F("  -> retry GPS"));
  }
}

void Trackio::resetGpsData() { Trackio::gps.fix = 0; }

void Trackio::parseGps(char *gps) {
  char *split = strtok(gps, ",");

  split = strtok(NULL, ",");
  Trackio::gps.fix = atoi(split);
  split = strtok(NULL, ",");
  strcpy(Trackio::gps.time, split);
  // si hay fix seguimos parseando
  if (Trackio::gps.fix) {
    split = strtok(NULL, ",");
    strcpy(Trackio::gps.lat, split);
    split = strtok(NULL, ",");
    strcpy(Trackio::gps.lon, split);
    split = strtok(NULL, ",");
    strcpy(Trackio::gps.alt, split);
    split = strtok(NULL, ",");
    strcpy(Trackio::gps.sog, split);
    split = strtok(NULL, ",");
    strcpy(Trackio::gps.cog, split);
    split = strtok(NULL, ",");
    split = strtok(NULL, ",");
    strcpy(Trackio::gps.hdop, split);
    split = strtok(NULL, ",");
    split = strtok(NULL, ",");
    split = strtok(NULL, ",");
    strcpy(Trackio::gps.sats, split);
    split = strtok(NULL, ",");
    split = strtok(NULL, ",");
  }

  Trackio::clearGps();
}

void Trackio::clearGps() {
  if (!Trackio::gps.fix) return;

  char *split;

  split = strtok(Trackio::gps.alt, ".");
  strcpy(Trackio::gps.alt, split);

  split = strtok(Trackio::gps.sog, ".");
  strcpy(Trackio::gps.sog, split);

  split = strtok(Trackio::gps.cog, ".");
  strcpy(Trackio::gps.cog, split);
}

void Trackio::downloadEpoFile() {
  Trackio::sendCommand((char *)"AT+CNTP?");
  delay(1000);
  Trackio::sendCommand((char *)"AT+CNTP");
  delay(1000);
  Trackio::sendCommand((char *)"AT+CCLK?");
  delay(1000);

  Trackio::sendCommand((char *)"AT+FTPSERV=\"116.247.119.165\"");
  delay(1000);
  Trackio::sendCommand((char *)"AT+FTPUN=\"customer\"");
  delay(1000);
  Trackio::sendCommand((char *)"AT+FTPPW=\"111111\"");
  delay(1000);
  Trackio::sendCommand((char *)"AT+FTPGETNAME=\"MTK3.EPO\"");
  delay(1000);
  Trackio::sendCommand((char *)"AT+FTPGETPATH=\"/\"");
  delay(1000);
  Trackio::sendCommand((char *)"AT+FTPEXTGET=1");
  delay(1000);
  Trackio::sendCommand((char *)"AT+FTPEXTGET=4,\"epo\"");
  delay(1000);
  Trackio::sendCommand((char *)"AT+FSLS=C:\\User\\");
  delay(1000);
  Trackio::sendCommand((char *)"AT+CGNSCHK=3,1");
  delay(1000);
  Trackio::sendCommand((char *)"AT+CGNSAID=31,1,1");
  delay(1000);
  Trackio::sendCommand((char *)"AT+CGNSINF");
  delay(1000);
}

void Trackio::enableSimFasFix() {
  Trackio::sendCommand((char *)"AT+CGNSPWR=0");
  Trackio::_delay(1000);
  Trackio::sendCommand((char *)"AT+SAPBR=2,1");
  Trackio::_delay(1000);

  Trackio::sendCommand((char *)"AT+CNTPCID=1");
  Trackio::_delay(1000);
  Trackio::sendCommand((char *)"AT+CNTP?");
  Trackio::_delay(1000);
  Trackio::sendCommand((char *)"AT+CNTP");
  Trackio::_delay(1000);
  Trackio::sendCommand((char *)"AT+CCLK?");
  Trackio::_delay(1000);
  Trackio::sendCommand((char *)"AT+CLBS=1,1");
  Trackio::_delay(1000);
  Trackio::sendCommand((char *)"AT+FTPSERV=\"116.247.119.165\"");
  Trackio::_delay(1000);
  Trackio::sendCommand((char *)"AT+FTPUN=\"customer\"");
  Trackio::_delay(1000);
  Trackio::sendCommand((char *)"AT+FTPPW=\"111111\"");
  Trackio::_delay(1000);
  Trackio::sendCommand((char *)"AT+FTPGETNAME=\"MTK3.EPO\"");
  Trackio::_delay(1000);
  Trackio::sendCommand((char *)"AT+FTPGETPATH=\"/\"");
  Trackio::_delay(1000);
  Trackio::sendCommand((char *)"AT+FTPEXTGET=1");
  Trackio::_delay(6000);
  int x = 10;
  while (x--) {
    Trackio::sendCommand((char *)"AT+FTPEXTGET?");
    Trackio::_delay(1000);
  }
  Trackio::sendCommand((char *)"AT+FTPEXTGET=4,\"epo\"");
  Trackio::_delay(1000);

  Trackio::sendCommand((char *)"AT+FSLS=C:\\User\\");
  Trackio::_delay(1000);
  Trackio::sendCommand((char *)"AT+CGNSCHK=3,1");
  Trackio::_delay(1000);
  Trackio::sendCommand((char *)"AT+CGNSPWR=1");
  Trackio::_delay(1000);
  Trackio::sendCommand((char *)"AT+CGNSAID=31,1,1");
  Trackio::_delay(5000);
  Trackio::sendCommand((char *)"AT+CGNSINF");
  Trackio::_delay(1000);
}

// #############################################################################

int Trackio::countLog() {
  int total = 0;
  for (int i = 0; i < RH_logSize; i++) {
    if (strlen(cfg.log[i]) > 0) total++;
  }
  return total;
}

int Trackio::getLogNextIndex() {
  for (int i = 0; i < RH_logSize; i++) {
    if (strlen(cfg.log[i]) == 0) {
      // este indice está vacío, lo devolvemos
      return i;
    }
  }
  return -1;
}

bool Trackio::transmitLogIfFull() {
  _title(F("Transmit log if full"));
  if (Trackio::countLog() >= RH_logSize) {
    __(F(" -> IS FULL"));
    return Trackio::transmitLog();
  }
  __(F("  -> NOT FULL"));
  return false;
}

bool Trackio::transmitLog() {
  _title(F("Transmit Log"));

  if (Trackio::tcpIsOpen()) {
    // cerramos el TCP, no sabemos en que condiciones se ha abierto y
    // necesitamos que haya una sesion del dispositivo en el servidor, para lo
    // cual es necesario realizar el registro con sayHello(). Cerramos y a
    // continuación volvemos a abrir con sayHello();
    Trackio::closeTcp(0);
  }

  if (!Trackio::gprsIsOpen()) {
    if (!Trackio::openGprs()) {
      __(F("  == Fallo al transmitir log, el GPRS no se inicia"));
      return false;
    }

    // -> OK GPRS abierto
  }

  if (!Trackio::openTcp()) {
    __(F("  == Fallo al transmitir log, no se puede abrir el TCP"));
    return false;
  }

  // -> OK TCP abierto

  // nos registramos en el servidor. Esto dejará el TCP abierto y podremos ir
  // enviando los mensajes del log, al finalizar cerraremos el socket. Cada
  // mensaje que se envía se borra del log, si un mensaje falla no se eliminará,
  // quedará ahí para la próxima vez.
  if (!Trackio::sayHello()) {
    __(
        F("  == Fallo al transmitir log. TCP abierto, pero sayHello() ha "
          "fallado"));
    return false;
  }

  // recorremos el log
  for (int i = 0; i < RH_logSize; i++) {
    if (strlen(cfg.log[i])) {
      if (Trackio::transmit(cfg.log[i])) {
        // el mensaje se ha transmitido, lo eliminamos del log
        strcpy(cfg.log[i], "");
        ___(F("  -> log "), i);
      }
      // else: la transmision del mensaje ha fallado, lo mantenemos en el para
      // próximo intento
      delay(100);  // damos un tiempo entre cada escritura
    }
  }

  Trackio::saveConf();
  __(F("  == TRANSMISION DEL LOG FINALIZADA"));

  if (cfg.keepTCPOpen) {
    Trackio::closeTcp(0);
  }

  return true;
}

void Trackio::showLog() {
  _title("Show Log");
  uint8_t total = 0;
  for (int i = 0; i < RH_logSize; i++) {
    if (strlen(cfg.log[i])) {
      ___(i, cfg.log[i]);
      total++;
    }
  }

  if (!total) __(F("  == El log está vacío"));
}

// #############################################################################

void Trackio::setupSensors() {
  _title("Setup sensores");

  if (cfg.useAccel) {
    Wire.begin();
    Trackio::setupAccel();
  }

#if RH_useBME680 == true
  if (!bme.begin()) {
    __(F("Could not find a valid BME680 sensor, check wiring!"));
    __(F("  == ERROR BME680"));
  } else {
    __(F("  -> BME680 OK"));
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);  // 320*C for 150 ms
#endif

#if RH_useBMP280 == true
  if (bmp.begin()) {
    __(F("  -> BMP280 OK"));
  } else {
    __(F("  == ERROR BMP280"));
  }
#endif

#if RH_useBME280 == true
  if (bme.begin()) {
    __(F("  -> BME280 OK"));
  } else {
    __(F("  == ERROR BMP280"));
  }
#endif

#if RH_useCCS811 == true
  if (ccs.begin()) {
    while (!ccs.available())
      ;
    float temp = ccs.calculateTemperature();
    ccs.setTempOffset(temp - 25.0);
    __(F("  -> CCS811 OK"));
  } else {
    __(F("  == ERROR CCS811"));
  }
#endif

#if RH_useCAN == true
  Trackio::initCan();
#endif

#if RH_useDHT == true
  dht.setup(RH_useDTHPin);
#endif

#if RH_useSHT20 == true
  sht20.initSHT20();
#endif

#if RH_useDustSensor == true
  pinMode(RH_useDustSensorILed, OUTPUT);
#endif

#if RH_useSDI12 == true
  Trackio::setupSDI();
#endif
}

void Trackio::getTemperature() {
  strcpy(Trackio::sensors.temperature, "0");
#if RH_useBME680 == true
  sprintf(Trackio::sensors.temperature, "%3.1f", bme.temperature);
#elif RH_useBMP280 == true
  float temp = bmp.readTemperature();
  sprintf(Trackio::sensors.temperature, "%3.1f", temp);
#elif RH_useDHT == true
  float temp = dht.getTemperature();
  sprintf(Trackio::sensors.temperature, "%3.1f", temp);
#elif RH_useBME280 == true
  float temp = bme.readTemperature();
  sprintf(Trackio::sensors.temperature, "%3.1f", temp);
#elif RH_useSHT20 == true
  float x = sht20.readTemperature();
  sprintf(Trackio::sensors.temperature, "%3.1f", x);
#endif
}

void Trackio::getHumidity() {
  strcpy(Trackio::sensors.humidity, "0");
#if RH_useBME680 == true
  sprintf(Trackio::sensors.humidity, "%3.1f", bme.humidity);
#elif RH_useDHT == true
  float humidity = dht.getHumidity();
  sprintf(Trackio::sensors.humidity, "%3.1f", humidity);
#elif RH_useBME280 == true
  float humd = bme.readHumidity();
  sprintf(Trackio::sensors.humidity, "%3.1f", humd);
#elif RH_useSHT20 == true
  float humd = sht20.readHumidity();
  sprintf(Trackio::sensors.humidity, "%3.1f", humd);
#endif
}

void Trackio::getPressure() {
  strcpy(Trackio::sensors.pressure, "0");
#if RH_useBME680 == true
  unsigned int a = bme.pressure / 100;
  sprintf(Trackio::sensors.pressure, "%u", a);
#elif RH_useBMP280 == true
  float x = bmp.readPressure();
  sprintf(Trackio::sensors.pressure, "%3.1f", x);
#elif RH_useBME280 == true
  float x = bme.readPressure() / 100;
  sprintf(Trackio::sensors.pressure, "%3.1f", x);
#endif
}

void Trackio::getVoc() {
  strcpy(Trackio::sensors.voc, "0");
#if RH_useBME680 == true
  sprintf(Trackio::sensors.voc, "%4.1f", (bme.gas_resistance / 1000.0));
#elif RH_useCCS811
  int TVOC = ccs.getTVOC();
  sprintf(Trackio::sensors.voc, "%d", TVOC);
#endif
}

void Trackio::getRaindrops() {
  sprintf(Trackio::sensors.raindrops, "%lu", analogRead(RH_useRaining));
}

void Trackio::getCO() {
#if RH_useMQ7 != 0
  sprintf(Trackio::sensors.co, "%lu", analogRead(RH_useMQ7Pin));
#endif
}

void Trackio::getCO2() {
  strcpy(Trackio::sensors.co2, "0");
#if RH_useCCS811 == true
  if (!ccs.readData()) {
    uint16_t co2 = ccs.geteCO2();
    ;
    sprintf(Trackio::sensors.co2, "%d", co2);
  }
#endif
}

void Trackio::getDustSensor() {
#if RH_useDustSensor == true
  int adcvalue;
  digitalWrite(RH_useDustSensorILed, HIGH);
  delayMicroseconds(280);
  adcvalue = analogRead(RH_useDustSensorAnalogPin);
  digitalWrite(RH_useDustSensorILed, LOW);

  adcvalue = ds.Filter(adcvalue);

  float density = ds.Conversion(adcvalue);

  // density será un valor de ug/m^3 (microgramos metro cubico)
  sprintf(Trackio::sensors.airParticles, "%4f", density);
#endif
}

void Trackio::getSonarDist() {
#if RH_useSonar == true
  // Send ping, get distance in cm and print result (0 = outside set distance
  // range)
  sprintf(Trackio::sensors.distance, "%3lu", sonar.ping_cm());
#endif
}

// #############################################################################

void Trackio::readSDIBuffer() {
#if RH_useSDI12
  int i = 0;
  memset(sdiBuffer, 0, sizeof sdiBuffer);

  sdi12.read();  // consume address
  while (sdi12.available() && i < 50) {
    char c = sdi12.read();
    if ((c != '\n') && (c != '\r')) sdiBuffer[i] = c;
    Trackio::_delay(10);
    i++;
  }

  sdiBuffer[i] = '\0';
#endif
}

void Trackio::sdiReadBuffer() {
#if RH_useSDI12
  sdi12.sendCommand(SDI_CMD_READ);
  Trackio::_delay(100);
  readSDIBuffer();
  sdi12.clearBuffer();
#endif
}

void Trackio::readSDIProbeInfo() {
#if RH_useSDI12
  memset(buffer, 0, sizeof buffer);

  sdi12.sendCommand("?!");
  Trackio::_delay(400);  // WTF! No puedo quitarme este 400 del medio... deja de
                         // dar respuestas
  sdiReadBuffer();
  _(F("ADDR: "));
  __(sdiBuffer);
  sdi12.clearBuffer();

  sdi12.sendCommand(SDI_CMD_ID);
  Trackio::_delay(400);
  sdiReadBuffer();
  _(F("ID: "));
  __(sdiBuffer);
  sdi12.clearBuffer();
#endif
}

bool Trackio::readSDISensor(const char *sensorCmd) {
#if RH_useSDI12
  int count = 5;
  while (count) {
    // readSDISensor(cmd);

    sdi12.sendCommand(sensorCmd);
    Trackio::_delay(300);
    sdi12.clearBuffer();
    sdiReadBuffer();

    if (strlen(sdiBuffer) > 10) {
      return true;
    }

    count--;
    Trackio::_delay(10);
  }
#endif

  return false;
}

void Trackio::addSDIToMessage() {
#if RH_useSDI12
  memset(buffer, 0, sizeof buffer);

  // humedad
  memset(buffer, 0, sizeof buffer);
  if (Trackio::readSDISensor(SDI_CMD_HUMI)) {
    strcat(buffer, SDI_CMD_HUMI);
    strcat(buffer, ":");
    strcat(buffer, sdiBuffer);
    Trackio::addToMessage(buffer);
  }

  // temperatura
  memset(buffer, 0, sizeof buffer);
  if (Trackio::readSDISensor(SDI_CMD_TEMP)) {
    strcat(buffer, SDI_CMD_TEMP);
    strcat(buffer, ":");
    strcat(buffer, sdiBuffer);
    Trackio::addToMessage(buffer);
  }

  // temperatura
  memset(buffer, 0, sizeof buffer);
  if (Trackio::readSDISensor(SDI_CMD_SALI)) {
    strcat(buffer, SDI_CMD_SALI);
    strcat(buffer, ":");
    strcat(buffer, sdiBuffer);
    Trackio::addToMessage(buffer);
  }
#endif
}

// gets identification information from a sensor, and prints it to the serial
// port expects a character between '0'-'9', 'a'-'z', or 'A'-'Z'.
void Trackio::getSDIID() {
#if RH_useSDI12
  sdi12.sendCommand(SDI_CMD_ID);
  Trackio::_delay(30);
  readSDIBuffer();
  strcpy(sdiId, sdiBuffer);
#endif
}

void Trackio::setupSDI() {
#if RH_useSDI12
  __(F("  -- Opening SDI-12 bus..."));
  sdi12.begin();
  Trackio::_delay(200);
  Trackio::getSDIID();
  if (strlen(sdiId) > 1) {
    ___(F("  == sdiId: "), sdiId);
    cfg.useSDI12 = true;
  } else {
    __(F("  == sdiId ERROR: Lectura SDI12 deshabilitada"));
    cfg.useSDI12 = false;
  }
#endif
}

// #############################################################################

void Trackio::_delay(int time) {
  Watchdog.reset();
  delay(time);
  Watchdog.reset();
}

// #############################################################################

void Trackio::ledStatus(uint8_t status) { digitalWrite(RH_ledPin, status); }

void Trackio::blink(uint8_t times, int ms) {
  // see main.cpp > led()
  Trackio::blinkTimes = times;
  Trackio::blinkTime = ms;
}

void Trackio::blink() { Trackio::blink(3, 200); }

void Trackio::buzz(uint8_t times, int ms) {
#if RH_buzzPin != 0
  while (times--) {
    digitalWrite(RH_buzzPin,
                 LOW);  // carefull with the pin, it may be inverted (LOW = ON)
    Trackio::_delay(ms);
    Trackio::_delay(ms);
  }
  digitalWrite(RH_buzzPin, HIGH);
#endif
}

void Trackio::buzz() { Trackio::buzz(3, 200); }

void Trackio::hardReset() {
  __(F(""));
  __(F("#############################################"));
  __(F("################ HARD RESET #################"));
  __(F("#############################################"));
  __(F(""));
  Trackio::_delay(500);
  NVIC_SystemReset();  // software reset
  // ------------- aquí muere la aplicación - RESET NOW

  // y esto... ?
  Trackio::_delay(1000);

  // ...en teoría no deberíamos llegar aquí ya y que NVIC_SystemReset() debería
  // reiniciar el micro. Lo que sigue es un "por si acaso". Habilitamos watchdog
  // y ejecutamos while infinito

  Watchdog.enable(8000);
  Trackio::powerOff();
  while (1) {
  }
}

// #############################################################################

void Trackio::enableGSMClock() {
  if (Trackio::sendAt((char *)"AT+CLTS?", 1, (char *)"+CLTS: 1")) {
    __(F("  == reloj interno del modem YA habilitado"));
    return;
  }

  if (Trackio::sendAt((char *)"AT+CLTS=1;&W", 1, OK)) {
    __(F("  == reloj interno del modem habilitado"));
    return;
  }

  __(F("  == ERROR al habilitar reloj interno del modem"));
}

bool Trackio::getGSMTime() {
  if (!Trackio::sendAt((char *)"AT+CCLK?", 1, (char *)"+CCLK: \"")) {
    __(F("  == ERROR al leer la hora GSM"));
    return false;
  }

  char *splitTime;
  splitTime = strtok(buffer, "\"");  // splitTime = '+CCLK: '
  splitTime = strtok(NULL, "\"");    // splitTime = '19/01/26,23:43:29+04'
  if (splitTime == NULL) {
    __(F("  == ERROR al parsear hora GSM"));
    return false;
  }

  Trackio::parseGSMTime(splitTime);
  Trackio::setTheTime();

  return true;
}

void Trackio::parseGSMTime(char *dateTime) {
  char datetime[30];
  strcpy(datetime, dateTime);
  ___("datetime: ", datetime);

  ___(F("  == datetime "), datetime);
  char *split;

  split = strtok(dateTime, "/");
  strcpy(Trackio::time.year, split);  // 19
  split = strtok(NULL, "/");
  strcpy(Trackio::time.month, split);  // 01
  split = strtok(NULL, ",");
  strcpy(Trackio::time.day, split);  // 27
  split = strtok(NULL, ":");
  strcpy(Trackio::time.hours, split);  // 23
  split = strtok(NULL, ":");
  strcpy(Trackio::time.minutes, split);  // 53
  split = strtok(NULL, "+");
  strcpy(Trackio::time.seconds, split);  // 16
  split = strtok(NULL, "+");
  strcpy(Trackio::time.zone, split);  // 04
}

bool Trackio::getGsmLocation() {
  gsmLoc.fix = false;

  if (!Trackio::sendAt((char *)"AT+CLBS=1,1", 1, (char *)"+CLBS: 0,")) {
    __(F("  == ERROR CellID"));
    Trackio::addCellIdError();
    return false;
  }

  __(F("  -> Tenemos CellID"));
  gsmLoc.fix = true;
  Trackio::resetCellIdError();

  char *split;
  // buffer = AT+CLBS=1,1 = +CLBS: 0,-0.639775,39.552156,550
  split = strtok(buffer, " ");  // +CLBS:
  split = strtok(NULL, ",");    // 0
  split = strtok(NULL, ",");    // -0.639775
  strcpy(Trackio::gsmLoc.lon, split);
  split = strtok(NULL, ",");  // 39.552156
  strcpy(Trackio::gsmLoc.lat, split);
  split = strtok(NULL, ",");  // 550
  strcpy(Trackio::gsmLoc.precission, split);

  _(F("  == "));
  _(Trackio::gsmLoc.lat);
  _(F(","));
  _(Trackio::gsmLoc.lon);
  _(F(","));
  __(Trackio::gsmLoc.precission);

  return true;
}

void Trackio::addCellIdError() {
  Trackio::cellIdFails++;
  if (Trackio::cellIdFails > Trackio::cellIdFailsLimit) {
    __(F("Sobrepasado el límite de fallos CellID"));
    cfg.opmode = OP_STARTUP;
  }
}

void Trackio::resetCellIdError() { Trackio::cellIdFails = 0; }

// #############################################################################

void Trackio::setTheTime() {
#if RH_useDS3231 == true
  rtc.adjust(DateTime(atoi(Trackio::time.year), atoi(Trackio::time.month),
                      atoi(Trackio::time.day), atoi(Trackio::time.hours),
                      atoi(Trackio::time.minutes),
                      atoi(Trackio::time.seconds)));

  ___(F("  == timestamp: "), Trackio::getTimeNow());
#endif

  if (cfg.useGsmTime) {
    // TimeLib.h
    // hr,min,sec,day,mnth,yr
    setTime(atoi(Trackio::time.hours), atoi(Trackio::time.minutes),
            atoi(Trackio::time.seconds), atoi(Trackio::time.day),
            atoi(Trackio::time.month), atoi(Trackio::time.year));
  }
}

long Trackio::getTimeNow() {
#if RH_useDS3231 == true
  DateTime now = rtc.now();
  return (long)now.unixtime();
#else
  // hr,min,sec,day,mnth,yr
  return (long)now();
#endif
}

void Trackio::setupRTC() {
  _title(F("setupRTC"));

#if RH_useDS3231 == true
  __(F("  -> Using DS3231"));
  if (!rtc.begin()) {
    __(F("  == Error loading RTC"));
  }

  if (rtc.lostPower()) {
    // El RTC ha perdido la hora, lo configuramos a fecha de ultima compilación
    __(F("  == RTC lost power, lets set the time!"));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
#else
  __(F("  -> NO RTC - Using TimeLib"));
#endif
}

// #############################################################################

bool Trackio::sleepNow(uint8_t times) {
  // rectificamos todos los timers para compensar el desfase que va a producir
  // el modo power down, donde el reloj de la CPU se para y la función millis
  // deja de contar el tiempo.

  if (times < 1) {
    times = 1;
  }

  __(F("------- SLEEP NOW -------"));

  while (times--) {
    ___(F("REMAINING: "), times);

    // sleep
    Watchdog.sleep(8000);

    // al entrar en modo sleep el reloj del micro deja de funcionar, por lo que
    // añadimos un desfase de 8 segundos. Añadimos el mismo tiempo como offset.
    Trackio::timers.offset += 8000L;

    if (RH_useNMIForWakeup && !digitalRead(NMI)) {
      // se ha pulsado el botón, salimos de dormir
      Trackio::blink(2, 100);
      times = 0;
      return false;
    }
  }

  __(F("------- WAKEUP -------"));
  return true;
}

void Trackio::enterDeepsleep() {
  if (Trackio::tcpIsOpen()) {
    Trackio::closeTcp(0);
  }

  Trackio::powerOff();
}

void Trackio::exitDeepsleep() {
  if (!Trackio::powerOn()) {
    // si falla realizamos la rutina de arranque desde el principio
    cfg.opmode = OP_STARTUP;
  } else {
    Trackio::checkModem();
    if (cfg.useGps) Trackio::powerOnGps();
    Trackio::checkCreg();
    Trackio::openGprs();
  }
}

// #############################################################################

void Trackio::setupAccel() {
#if RH_useLIS2HH12 == true
  lis.powerOn();
#elif RH_useMPU6050 == true
  Wire.begin();
  Wire.beginTransmission(RH_MPU6050_ADDRESS);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
#endif
}

void Trackio::readAccel() {
#if RH_useLIS2HH12 == true
  lis.readAccel(&AcX, &AcY, &AcZ);
  Trackio::scaleAccel();
#elif RH_useMPU6050 == true
  Wire.beginTransmission(RH_MPU6050_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(RH_MPU6050_ADDRESS, 14, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Trackio::scaleAccel();
#endif
}

void Trackio::scaleAccel() {
#if RH_useAccel
  if (Trackio::gravity == 2) {
    Trackio::x = (float)AcX * 0.000244;
    Trackio::y = (float)AcY * 0.000244;
    Trackio::z = (float)AcZ * 0.000244;
  } else if (Trackio::gravity == 1) {
    Trackio::x = (float)AcX * 0.000122;
    Trackio::y = (float)AcY * 0.000122;
    Trackio::z = (float)AcZ * 0.000122;
  } else {
    Trackio::x = (float)AcX * 0.000061;
    Trackio::y = (float)AcY * 0.000061;
    Trackio::z = (float)AcZ * 0.000061;
  }
#endif
}

// #############################################################################

bool Trackio::enableBl() {
  if (Trackio::sendAt((char *)"AT+BTPOWER=1", 1, OK)) {
    __(F("  --> Bluethooth activado"));
    Trackio::sendAt((char *)"AT+BTHOST?", 1);
    Trackio::sendAt((char *)"AT+BTPAIRCFG=2", 1);
    Trackio::setBlVisibility(true);
    Trackio::sendCommand((char *)"AT+BTSTATUS?");

    return true;
  }

  __(F("  --> ERROR Bluethooth no activado"));
  return false;
}

bool Trackio::disableBl() {
  if (Trackio::sendAt((char *)"AT+BTPOWER=0", 1, OK)) {
    __(F("  --> Bluethooth disabled"));
    return true;
  }

  __(F("  --> ERROR can't disable Bluethooth"));
  return false;
}

bool Trackio::setBlVisibility(bool isVisible) {
  if (isVisible) {
    if (Trackio::sendAt((char *)"AT+BTVIS=1", 1, OK)) {
      return true;
    }

    return false;
  }

  // ocultar
  if (Trackio::sendAt((char *)"AT+BTVIS=0", 1, OK)) {
    return true;
  }

  return false;
}

bool Trackio::setBlName(char *blName) {
  if (strlen(blName) > 20) {
    __(F("  -> Bluetooth device name can't be more than 20 chars"));
    return false;
  }

  char atCmd[35];
  sprintf(atCmd, "AT+BTHOST=%s", blName);
  if (Trackio::sendAt(atCmd, 1, OK)) {
    return true;
  }

  return false;
}

bool Trackio::acceptBlPair() {
  if (Trackio::sendAt((char *)"AT+BTPAIR=1,1", 1, OK)) {
    __(F("  -> Bluetooth pairing aceptado"));
    return true;
  }

  __(F("  -> ERROR Bluetooth pairing"));
  return false;
}

void Trackio::processBlData(char *data) {
  // data = +BTSPPDATA: 1,4,TEXTO RECIBIDO
  char *splitData;
  splitData = strtok(data, ",");
  splitData = strtok(NULL, ",");
  splitData = strtok(NULL, ",");

  ___(F("bldata: "), splitData);
  strcpy(buffer, splitData);
  Trackio::processBuffer();
}

// #############################################################################

#if RH_useCAN == true
void Trackio::initCan() {
  _title("Init CAN");
  Trackio::CANEnabled = false;

  Watchdog.disable();
  byte obdBegin = obd.begin();
  ___(F("BOD BEGIN?"), obdBegin);

  if (obdBegin != 0) {
    if (obd.init()) {
      Trackio::CANEnabled = true;
    }
  }

  Watchdog.enable(8000);
}
#endif

// #############################################################################
bool Trackio::sendCommand(char *cmd, unsigned int timeout) {
  __(F(""));
  __(F(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"));
  _(F("sendCommand > "));
  __(cmd);

  // vaciamos el buffer previo
  while (SerialSim.available()) SerialSim.read();
  Trackio::emptyBuffer();

  // enviamos el comando al modem
  SerialSim.println(cmd);
  SerialSim.flush();
  Trackio::_delay(100);

  uint32_t start = millis();
  while ((millis() - start) < timeout) {
    Trackio::_delay(1);  // feed the dog
    if (!SerialSim.available()) continue;

    int counter = 0;
    while (SerialSim.available() > 0 && counter < UART_BUFFER_SIZE) {
      buffer[counter] = SerialSim.read();
      counter++;
      Trackio::_delay(1);  // feed the dog
    }

    buffer[counter] = '\0';
    __(buffer);
    return true;
  }

  __(F("  == FAILED"));
  return false;
}

// #############################################################################

bool Trackio::sendAt(char *cmd) { return sendAt(cmd, 2, NULL, 0); }

bool Trackio::sendAt(char *cmd, char *validate) {
  return sendAt(cmd, 2, NULL, 0);
}

bool Trackio::sendAt(char *cmd, int returnLine) {
  return sendAt(cmd, returnLine, NULL, 0);
}

bool Trackio::sendAt(char *cmd, int returnLine, char *validate) {
  return sendAt(cmd, returnLine, validate, 0);
}

bool Trackio::sendAt(char *cmd, int returnLine, int timeout) {
  return sendAt(cmd, returnLine, NULL, timeout);
}

bool Trackio::sendAt(char *cmd, int returnLine, char *validate, int timeout) {
  int loopCounter = 0;
  int lineCount = 0;
  bool hasLine = false;

  // empty buffer
  while (SerialSim.available()) SerialSim.read();
  Trackio::emptyBuffer();

  // debug
  __(F(""));
  __(F(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"));
  _(F("sendAt > "));
  __(cmd);

  // send command to the modem
  SerialSim.println(cmd);
  SerialSim.flush();  // ensure command is sent

  // algunos comandos requieren cierto tiempo antes de ser ejecutados
  if (timeout) Trackio::_delay(timeout);

  while (loopCounter < 150) {
    char *serialBuffer = readLine.feed(&SerialSim);
    if (serialBuffer != NULL) {
      lineCount++;

      if (lineCount == returnLine) {
        strcpy(buffer, serialBuffer);
        hasLine = true;
        ___(F("  -> "), serialBuffer);  // -> es línea
        break;
      }

      ___(F("  -- "), serialBuffer);  // -- no es linea
    }

    loopCounter++;
    Trackio::_delay(100);
  }

  if (!hasLine) {
    __(F("Serial Failed"));
    modemSerialFails++;
    if (modemSerialFails >= 4) {
      Trackio::hardReset();
    }

    return false;
  } else {
    modemSerialFails = 0;
    if (validate != NULL) {
      if (!strstr(buffer, validate)) {
        return false;
      }
    }
  }

  return true;
}

bool Trackio::sendSerialExt(char *string) {
  if (!cfg.useSerialExt) {
    return false;
  }
  SerialExt.println(string);
  return true;
}
