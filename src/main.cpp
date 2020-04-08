
/** @file */

/*
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
 */

/**
 * @brief Trackio, The IoT device for tracking and sensing.
 *
 * The library itself is in the lib/Trackio directory
 *
 * @see Source Code Repository https://github.com/Rhomb-io/Trackio
 * @see Online User Documentation https://trackio.m2msystemsource.com
 *
 * @file main.cpp
 * @author Jordi Enguídanos <jenguidanos@rhomb.io>
 * @copyright (C) 2018 Tecnofingers S.L
 */

#include <Arduino.h>
#include "Scheduler.h"
#include <avr/dtostrf.h>
#include <Adafruit_SleepyDog.h>
#include "Rhio-RGB.h"

#include "Trackio.h"
Trackio trackio;

void op_startup();
void op_auto();
void op_low();
void blinkLed();     // task with scheduler.h
void blinkLedRGB();  // task with scheduler.h
void tick();
bool alive();
void sleep();
bool transmitAlive();
void checkMovement();  // acelerometer
void processMovementAlert(unsigned long vibrationTime);
void addFailTransmission();

// Defines if the first message has been sent, this allows not having to wait
// for tickTimer after a reset
bool firstMessageSent = false;
// flag to start the transmission
bool transmitNow = false;
uint8_t transmissionFailsCount = 0;
const uint8_t transmissionFailsLimit = 5;

// #############################################################################

/**
 * @brief We set Watchdog and float data types in printf.
 * Nothing else ... the rest is done in the loop (OP_STARTUP).
 */
void setup() {
  // https://github.com/arduino/ArduinoCore-samd/issues/217
  asm(".global _printf_float");
  Watchdog.enable(8000);

  if (RH_useLED) Scheduler.startLoop(blinkLed);
  if (RH_useRGBLed) Scheduler.startLoop(blinkLedRGB);
  if (RH_useAccel) Scheduler.startLoop(checkMovement);
}

void loop() {
  trackio.timers.base = millis() + trackio.timers.offset;

  if (cfg.opmode == OP_AUTO)
    op_auto();
  else if (cfg.opmode == OP_STARTUP)
    op_startup();
  else if (cfg.opmode == OP_LOW)
    op_low();
  else if (cfg.opmode == OP_RST) {
    SerialMon.println("Reset Micro - Firing watchdog");
    trackio.hardReset();
  }

  trackio._delay(1);
}

void blinkLed() {
  if (trackio.blinkTimes == 0) {
    yield();
    return;
  }

  uint32_t previousLedState = digitalRead(LED);

  while (trackio.blinkTimes--) {
    trackio.ledStatus(1);
    delay(trackio.blinkTime);
    trackio.ledStatus(0);
    delay(trackio.blinkTime);
  }

  // reset
  trackio.blinkTimes = 0;
  trackio.ledStatus(previousLedState);

  yield();
}

void blinkLedRGB() {
  if (!trackio.ready) {
    yield();
    return;
  }

  if (atoi(trackio.gsmSignal) > 5) {
    // Green led if there is GSM coverage
    rh_greenLedOn();
  } else {
    // Red if no coverage
    rh_redLedOn();
  }

  trackio._delay(100);
  rh_ledOff();

  if (cfg.useGps) {
    trackio._delay(100);
    if (trackio.gps.fix) {
      // Blue led if there is GPS fix
      rh_blueLedOn();
    } else {
      // red if not gps fix
      rh_redLedOn();
    }
    trackio._delay(100);
    rh_ledOff();
  }

  // Run this method every 5 seconds
  trackio._delay(5000);
  yield();
}

// #############################################################################

/**
 * @brief Ejecución del modo OP_STARTUP
 *
 * OP_STARTUP is set by default when we start the application.
 * Configure trackio and the other components, for example commands are sent
 * AT to the modem to verify its operation or if modules have been selected
 * third parties will enable them and check their status.
 *
 * If an errors occur during the course of the application (the device
 * loses GPRS /GPS coverage, eg) this mode will be set again
 * to re-initialize the application

 * @see Trackio::begin()
 */
void op_startup() {
  // reset data...
  firstMessageSent = false;

  if (!trackio.begin()) {
    SerialMon.println(F("Trackio Critical FAIL. Firing watchdog..."));
    while (1) {
    }  // fire watchdog
  }

  // trackio.begin() does battery check and can enable low power mode
  if (cfg.opmode == OP_LOW) {
    return;  // We enter low battery mode, exit and return to the loop ()
  }

  cfg.opmode = cfg.primaryOpMode;
}

// #############################################################################

/**
 * @brief Normal operating mode
 *
 * Trackio will create and transmit a message with data in a time interval
 * defined in cfg.tickTimer
 */
void op_auto() {
  if (cfg.transmitAlways && trackio.countLog() > 0) {
    trackio.transmitLog();
  }

  // TICKTIMER - Normal transmission data
  tick();

  // ALIVE TIMER - keepalive for the server
  if (cfg.aliveTimer) {
    alive();
  }

  trackio.readModemBuffer();  // read messages from TCP server
  if (RH_readDebugRX) trackio.readDebugBuffer();  // read buffer for commands
}

// #############################################################################

/**
 * @brief Enable sleep mode on the microcontroller
 *
 * The sleep mode is enabled according to the configuration of the application,
 * usually when the battery is running out or when we program an application to
 * work in low power mode and extend the battery life.
 *
 * @see Trackio::checKBattery()
 */
void op_low() {
  unsigned long secs = trackio.sleepSecs ? trackio.sleepSecs : cfg.tickTimer;
  unsigned long times = (int)secs / 8;
  trackio.sleepNow(times);
  trackio.sleepSecs = 0;      // reset sleep now after wakeup
  trackio.checkLowBattery();  // change OP_MODE if possible
}

// #############################################################################

/**
 * @brief Crate a message and transmit to the server or save to log
 *
 * This is the function that runs with the main time interval,
 * cfg.tickTimer. This interval is configured in static-conf.h (RH_tickTimer)
 *
 * A message will be created and transmitted or stored
 * in the log according to the configuration of static-conf.h
 */
void tick() {
  unsigned long timeDiff;
  bool transmitted = false;
  timeDiff = trackio.timers.base - trackio.timers.tickTimer;
  if (cfg.tickTimer > 0 && timeDiff > (cfg.tickTimer * 1000L)) {
    transmitNow = true;
  }

  if (!transmitNow && firstMessageSent) {
    // nothing to do
    return;
  }

  // reset tickTimer y aliveTimer for next time
  trackio.timers.tickTimer = trackio.timers.base;
  trackio.timers.aliveTimer = trackio.timers.base;

  // Create the messge. it will be stored in Trackio::message
  trackio.createMessage();

  if (cfg.transmitAlways) {
    if (cfg.transmitWithSMS) {  // transmit with SMS
      if (trackio.sendSMS(trackio.message)) {
        transmitted = true;
        firstMessageSent = true;
        transmitNow = false;
      }

    } else {  // Transmitir with TCP
      if (!trackio.tcpIsOpen()) {
        trackio.prepareForTransmission();
        trackio.sayHello();
      }

      if (trackio.transmit(trackio.message)) {
        transmitted = true;
        firstMessageSent = true;
        transmitNow = false;
      }
    }
  } else {
    firstMessageSent = true;
    transmitNow = false;
  }

  if (RH_logSize > 0) {
    if (!transmitted) {
      trackio.saveMessage();
    }

    if (cfg.transmitLogIfFull) {
      trackio.transmitLogIfFull();
    }
  }

  if (!cfg.keepTCPOpen && trackio.tcpOk) {
    trackio.closeTcp(0);
  }

  sleep();
}

bool alive() {
  unsigned int timeDiff;

  if (cfg.aliveTimer > 0 && cfg.keepTCPOpen) {
    timeDiff = (trackio.timers.base - trackio.timers.aliveTimer);
    if (timeDiff >= (cfg.aliveTimer * 1000L)) {
      trackio.timers.aliveTimer = trackio.timers.base;
      return transmitAlive();
    }
  }

  return false;
}

void sleep() {
  if (!cfg.sleep) return;

  if (cfg.deepSleep) trackio.enterDeepsleep();

  // the sleep method goes to sleep in 8 second cycles.
  // we must deduce how many 8-second cycles we have in cfg.tickTimer.
  int sleepTimes = (int)cfg.tickTimer / 8;
  trackio.sleepNow(sleepTimes);

  if (cfg.deepSleep) trackio.exitDeepsleep();
}

/**
 * @brief Hello server! I'm here, safe and well
 *
 * On a low interval time (10 secs) communicate with the server to transmit
 * the status of the master GPIO (only 1 byte). This GPIO may inform if an
 * external device is ON or OFF (eg an electric scooter on a sharing
 * application).
 *
 * Using this alive also allow the device to receive data from the server
 * constantly and enable a bidirectional communication.
 *
 * This option can increase GPRS 2G communication costs by transmitting one byte
 * every 10 seconds. In a simple calculation this is equivalent to:
 * 10 / 3600 = 360 bytes per hour
 * 360 * 24 = 8640 bytes per day
 * 8640 * 365 = 3153600 / 1024 = 3079kb per year
 *
 * @return true
 * @return false
 */
bool transmitAlive() {
  char msg[5];
  sprintf(msg, "%d", digitalRead(RH_masterGPIO));
  if (!trackio.transmit(msg)) {
    __(F(" == FAIL ALIVE"));
    if (trackio.prepareForTransmission()) {
      __(F(" == OK prepareForTransmission"));
      return trackio.sayHello();
    }
    return false;
  }
  return true;
}

/**
 * @brief Data transmission fault counter
 *
 * Keeps a count of the failures produced when transmitting data. It will
 * restart the application on passing the threshold `transmissionFailsLimit`
 */
void addFailTransmission() {
  transmissionFailsCount++;
  if (transmissionFailsCount > transmissionFailsLimit) {
    // too much fails - RESTART
    transmissionFailsCount = 0;
    cfg.opmode = OP_STARTUP;
  }
}
/**
 * @brief Check if the device has been moved.
 *
 * Se inicializa en main.cpp:setup(), utilizando un scheduler de Arduino Zero.
 * Esto quiere decir que el acelerómetro funcionará en paralelo con el loop
 * principal y nunca dejará de realizar mediciones, aunque se ejecuten
 * métodos bloqueantes como delays
 *
 * Actualmente se utiliza un algoritmo que, aunque funciona, es bastante
 * rudimentario. Básicamente se toma una medición cada 200ms, se suma el total
 * del valor de los 3 ejes (x + y + z), se resta el valor anterior con el nuevo
 * valor y se almacena en la variable diffMov. Si el resultado es > o < que
 * el Indice de Diferencia de Movimiento (IDM) se da por hecho que ha habido
 * un movimiento.
 *
 * Se debe utilizar un umbral de tiempo para determinar si un movimiento debe
 * generar una alarma y avisar al servidor. Por ejemplo una pequeña vibración
 * puede ser aceptada como algo normal, pero un movimiento prolongado durante
 * unos segundo debe hacer saltar la alarma. Este umbral se establece en el
 * `static-conf.h` con RH_accelTime
 *
 * Para determinar que el movimiento ha finalizado y el dispositivo vuelve a
 * estar quieto se utiliza la variable Tiempo Mínimo Parado (TMP) que contiene
 * un tiempo mínimo por el cual, si no hay movimiento, se entiende que nos hemos
 * parado.
 *
 * Cuando salta una alarma pueden suceder dos cosas:
 * 1. Siempre se marcará la variable `Trackio::moving == true` lo cual añadirá
 *    la notificación de alarma al próximo mensaje que se envíe al servidor
 * 2. Si `RH_accelGPIO != 0` se activará/desactivará dicha GPIO
 *
 * Por ultimo, al producirse una alarma, si `RH_transmitAccelNow = true` se
 * transmitirá un nuevo mensaje con la alarma al momento, si no la alarma
 * quedará marcada en la variable `Trackio::moving` y se transmitirá cuando se
 * cumpla el tiempo de intervalo de envío de mensajes marcado en `RH_tickTimer`
 *
 * @todo Parametrizar IDM y TMP en static-conf.h y añadir como comando remoto
 * @note IDM y TMP son conceptos inventados aquí para la ocasión
 * @note Para testear, podemos flashear el dispositivo y ponerlo en marcha.
 *       Abrimos consola de debug y esperamos a ver la conexión con el server.
 *       A partir de aquí podemos agitar el dispositivo y en consola veremos
 *       los mensajes de debug generados en el método `processMovementAlert()`,
 *       en este archivo, más abajo. Durante desarrollo añadimos los mensajes
 *       de debug necesarios para entender el funcionamiento
 */
void checkMovement() {
  // Esperamos a que trackio.begin() finalice
  if (!trackio.ready) {
    yield();
    return;
  }

  // valor total de los 3 ejes. Se guarda una medida para
  // realizar comparación, con el fin de determninar si ha
  // habido movimiento
  static float prevAccel;
  // ultima hora en la que ha habido movimiento
  static unsigned int accelOn = 0;
  // ultima hora en la que NO ha habido movimiento
  static unsigned int accelOff = 0;

  uint16_t TMP = 1000;
  float IDM = 0.2;
  float total = 0;  // suma de los 3 ejes
  float diffMov;    // diferencia con el total de la medición anterior

  // Obtenemos lectura de acelerómetro
  // Los datos se almacenarán en las variables trackio.x, trackio.y y trackio.z
  trackio.readAccel();
  // punto clave aquí. ¿Tiene sentido comparar la suma de los 3 ejes para
  // detectar el movimiento? ¿Sería mejor una implemnetación individual por eje?
  // Otro modo?
  total = trackio.x + trackio.y + trackio.z;

  diffMov = abs(total - prevAccel);
  prevAccel = total;  // registramos nueva referencia anterior

  // para debug:

  if (diffMov > IDM || diffMov < -IDM) {
    accelOff = 0;    // si hay movimiento no puede estar off
    if (!accelOn) {  // registramos los millis solo la primera vez
      accelOn = millis();
    }
  } else {
    // estamos por debajo del IDM

    // tomamos marca de tiempo solo la primera vez
    accelOff = accelOff != 0 ? accelOff : millis();
    unsigned int diffOff = millis() - accelOff;

    // si estamos más de 1 segundo sin movernos finalizamos el movimiento
    if (diffOff > TMP) {
      accelOff = 0;  // reset fecha fin movimiento
      if (accelOn != 0) {
        unsigned int diffOn = millis() - accelOn;
        // processMovementAlert comprobará si el tiempo en movimiento
        // que se acaba de registrar es superior al tiempo minimo establecido
        // en la configuración, y en tal caso marcará la alerta.
        processMovementAlert(diffOn);
        accelOn = 0;
      }
    }
  }

  // Tomamos una medición cada 200ms
  // @think Es este tiempo correcto? necesitamos más o menos resolución? que
  //        implicaciones de batería podría tener aumentar la resolución?
  trackio._delay(200);

  // Soltamos el scheduler
  yield();
}

void processMovementAlert(unsigned long vibrationTime) {
  if (vibrationTime < cfg.accelTime) {
    // el tiempo en movimiento es inferior al tiempo mínimo configurado
    __(F(" -------------------------- FALSE ALARM"));
    if (cfg.accelGPIO > 0) {
      digitalWrite(cfg.accelGPIO, LOW);
    }
    return;
  }

  // Flag que indica que hay movimiento
  // En la próxima transmisión se informará de la alarma
  trackio.moving = true;
  __(F(" -------------------------- MOVING TRUE"));

  if (cfg.transmitAccelNow) {
    transmitNow = true;
  }

  if (cfg.accelGPIO > 0) {
    digitalWrite(cfg.accelGPIO, HIGH);
  }
}
