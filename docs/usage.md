# Introduction

Trackio can be configured to behave in many different ways, depending on factors such as battery consumption or cost of GSM data. The main function is to create a message with sensor data and send it to a remote server in a fixed time interval.

All configuration parameters are in the `./lib/Trackio/static-conf.h` file. Both the data collected and the transmission time interval are configurable on that file. You can create from low-power applications that transmit daily data to real-time applications that transmit at intervals of a few seconds.

The configuration of the device can be done by connecting a USB cable to the computer or by remote configuration with the M2M System Source web platform. This tutorial will teach you how to configure the device using the configuration file. Please read the Getting Started if you don't know how to connect your rhomb.io device to the computer.

## Real-time device

Whether we want a device that sends GPS or sensor data, we can set up a fast data send with `RH_tickTime` and `RH_aliveTime`, the two main timers.

The first, `RH_tickTime`, is used to create messages at time intervals, for example 60 seconds. The second, `RH_aliveTime`, is the keep-alive that is usually configured with short intervals of 5/10 seconds. In the keep alive _Trackio_ transmits a byte to tell the server that it is alive, the server could use that period of time to respond with a command. You can send remote commands with the M2M System Source web platform.

A tipical configuration for this timers may be:

```c++
#define RH_tickTime 60 // create message
#define RH_aliveTime 10 // keep alive to the server
// keep tcp always open, otherwise RH_keepAlive wouldn't make sense.
#define RH_keepTCPOpen true
// Saving messages in the log doesn't make sense either
#define RH_transmitAlways true
```

From here we will choose the data we want to send to the server every 60 seconds: GPS, Sensors or CAN.

!!! note
    `RH_aliveTime` in addition to being useful for telling the server _hey you, I'm still here!_ is also used to send the status of the main GPIO (`RH_masterGPIO`), which tells you if an external device is on or off. This is very useful to know at all times the status of an external device, for example a electrical scooter or bicycle in a sharing application.

!!! warning
    If `RH_aliveTime = 0` the keep alive will not be sent (and it doesn't have to be a problem...). If `RH_tickTime = 0` data (or messages) will never be sent to the server, never (and this can be a problem).

## Low Power Mode

The following is an example of sending one minute interval messages, where the energy consumption is better controlled in relation to the previous [real time example](/usage/#real-time-device):

```c++
#define RH_sleep true // sleep on each interval
#define RH_tickTime 60 // create message
#define RH_aliveTime 0 // no alive notification
#define RH_keepTCPOpen false // Close TCP after transmit data
#define RH_transmitAlways true
```

In this way the microcontroller will go into low power mode (thanks to [Adafruit_SleepyDog](https://github.com/adafruit/Adafruit_SleepyDog)), however the modem will remain on with the GPRS connection active (TCP closed).

If we add `#define RH_deepSleep true` to this previous example we will also turn off the modem, but in this case with such a short time interval (60 secs) it would not make much sense because every minute we have to wake up the modem, and the process of registering in the GSM network and enabling the GPRS could cost about 30 seconds.

!!! warn
    There is a bug when using the SerialUSB as serial monitor (`#define SerialMon SerialUSB`). The USB module on the SAMD21 goes off when intering in sleep mode. When the serial port wakes up, it does not work again and console messages are no longer printed. This does not happen when using `Serial` as monitor.

!!! note
    We will get the lowest possible consumption when `RH_sleep=true` and `RH_deepSleep=true`.

!!! note
    The time spent sleeping will be an interval, whatever is defined in `RH_tickTime`.

`RH_tickTime` will always be in seconds. If we want a message every hour we will use `#define RH_tickTime 3600` (or `#define RH_tickTime (60 * 60)`).

## Transmit Multiple Messages at Once

Transmit multiple messages at once may help to reduce battery conssumption. The next example will create a message every minute and transmit all after creating 10 message

```c
#define RH_logSize 10
#define RH_transmitLogIfFull true

#define RH_tickTime 60
#define RH_aliveTime 0
#define RH_transmitAlways false // <-- important
#define RH_sleep true
#define RH_deepSleep false
```

The option `RH_transmitAlways = false` will cause messages to be logged on EEPROM instead of being transmitted to the server. `RH_logSize = 10` sets a maximum value of messages to be saved and `RH_transmitLogIfFull = true` indicates that when that number of messages is reached, the transmission of all of them will start.

## Transmit With SMS

Alternatively Trackio can transmit messages by sending SMS instead of transmitting data over TCP.

```c
#define RH_transmitWithSMS true
#define RH_SMSNumber "555123456"
```

You can type in any mobile phone number, but it is recommended to use an Orange SMS server to integrate the data with the M2M System Source platform. [Contact us](http://m2msystemsource.com/) for more details.

!!! warning
    When working with SMS the GPRS modem is disabled, this prevents GSM geolocation from working.

## Controlling and external device

With the `RH_masterGPIO` property you can control one of the GPIOS of the microcontroller to control the switching on or off of an external device, for example a relay.

!!! note
    The web interface of M2M System Source offers an ON/OFF button which sends a command to change the status of this line and thus be able to control the external device remotely.

This GPIO has to be one of those available in the microcontroller. If you take into account that the modem is using the GPIOs from IO0 to IO5 and it is working with a Duino Zero, it will have available IO6, IO7, SDA, SCL, MOSI, MISO, SCK, CS0, CS1 and AD0.

The most common are IO6 and IO7. It also depends on the PCB to use. For example in a Halleybox the pin IO7 is not available, while in Hydra it only have IO7 and those of the I2C port available.

## Using SerialExt

!!! warning
    This is an experimental function.

SerialExt can be used as a bridge between an external device with a UART communication port and a web platform. This function enables a third USB communication port that can be used to give orders to an external device, for example an LCD screen or any device that has an UART communication port, regardless of the protocol that can be used.

The configuration can then be similar to this:

```c
#define RH_useSerialExt true // Enable the SerialExt function
#define RH_serialExtRX INT0 // Assign the RX port
#define RH_serialExtTX NMI // and TX
#define RH_serialExtBauds 9600 // Port speed
```

Actualmente los puertos RX y TX no se pueden modificar, deben ser INT0 y NMI respectivamente. Estos puertos deberán ser conectados al dispositivo externo.

## Using Sensors

Comming soon...

## Using Bluetooth

Coming soon...

## Using OBDII

Coming soon...
