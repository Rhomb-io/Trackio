# General Configuration

The general configuration is the one that we define in the `static-conf.h` file. This is the general configuration of our application and there are a lot of options to change.

## List of configuration properties

**Serial Communication**

| Property            | Default           | At runtime | Details                                                                                                                                                                      |
| ------------------- | ----------------- | :--------: | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `SerialSim`         | Arduino Serial1   |     no     | Serial object from SerialHardware library of Arduino. On Arduino Zero it should be `Serial1` if the SIM868 is in the Socket1 of the PCB or `Serial` if is placed on socket 2 |
| `SerialSimBauds`    | 38400             |     no     | Bauds for the modem serial                                                                                                                                                   |
| `SerialMon`         | Arduino SerialUSB |     no     | Serial port for monitor debug/log. Use Arduino `SerialUSB` or `Serial`                                                                                                       |
| `SerialMonBauds`    | 38400             |     no     | Bauds for the monitor serial                                                                                                                                                 |
| `RH_useSerialExt`   | true              |    yes     | Enable send commands to an external device connected with UART. Please see [Using SerialExt](/usage/#using-serialext) for more info.                                         |
| `RH_serialExtRX`    | INT0              |     no     | Do not should change this :)                                                                                                                                                 |
| `RH_serialExtTX`    | NMI               |     no     | This neither :)                                                                                                                                                              |
| `RH_serialExtBauds` | 9600              |     no     |                                                                                                                                                                              |
| `RH_readDebugRX`    | false             |     no     | Enable listen on debug port to catch operational commands                                                                                                                    |

**Network & SIM**

| Property                  | Default       | At runtime | Details                                                                                                                                                                                                                                        |
| ------------------------- | ------------- | :--------: | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `RH_tickTimer`            | 60            |    yes     | Time interval in seconds for the creation of messages. It also determines how long the device will sleep in low-power mode.                                                                                                                    |
| `RH_aliveTimer`           | 0             |    yes     | Time interval in seconds to ping the server (keep-alive). The ping will send the state of the master GPIO, usually IO6, represented as 0 (low) or 1 (high), 1 simple string, 1 byte sent through TCP.                                          |
| `RH_keepTCPOpen`          | false         |    yes     | It is convenient to keep the TCP open when we are going to send data in very short time intervals, less than 60 seconds.                                                                                                                       |
| `RH_transmitAlways`       | true          |    yes     | If TRUE each message will be sent by TCP/SMS automatically once it is created. IF FALSE the messages will not be sent by TCP, but will accumulate in the LOG and will be sent when the log is full according to the value of `RH_transmitLog`. |
| `RH_transmissionProtocol` | "TCP"         |    yes     | Transmission protocol, TCP or UDP                                                                                                                                                                                                              |
| `RH_remoteServer`         | ""            |    yes     | Remote TCP server IP or URL                                                                                                                                                                                                                    |
| `RH_remotePort`           | ""            |    yes     | Remote TCP port                                                                                                                                                                                                                                |
| `RH_useBl`                | false         |     no     | Enables bluetooth for remote communication and programming                                                                                                                                                                                     |
| `RH_apn`                  | ""            |    yes     | APN server                                                                                                                                                                                                                                     |
| `RH_apnUser`              | ""            |    yes     | APN user                                                                                                                                                                                                                                       |
| `RH_apnPass`              | ""            |    yes     | APN password                                                                                                                                                                                                                                   |
| `RH_transmitWithSMS`      | false         |    yes     | Send messages using SMSs instead of TCP. IMPORTANT! This will disable the GPRS connection and `RH_useGSMLocation` may be negatively affected.                                                                                                  |
| `RH_SMSNumber`            | String Number |    yes     | Telephone number to receive SMSs                                                                                                                                                                                                               |

**Battery Management**

| Property            | Default | At runtime | Details                                                                                                   |
| ------------------- | ------- | :--------: | --------------------------------------------------------------------------------------------------------- |
| `RH_battMode`       | 3       |     no     | Battery reading mode. 1=sim868 (at command), 2=deimos (required shield), 3=halleybox (using dcdc TLA2024) |
| `RH_requiredVbat`   | 3650    |    yes     | Minimum value for which the low power mode OP_LOW will be enabled when reading VBAT battery.              |
| `RH_requiredVin`    | 0       |    yes     | Minimum value for which the low power mode OP_LOW will be enabled when reading VIN battery.               |
| `RH_requiredVsys5v` | 0       |    yes     | Minimum value for which the low power mode OP_LOW will be enabled when reading VSYS battery.              |
| `RH_useBatt`        | true    |    yes     | Add battery status to message. `RH_useVBAT`, `RH_useVSYS` and/or `RH_useVIN` must be enabled              |
| `RH_useVBAT`        | true    |    yes     | Add VBAT to message                                                                                       |
| `RH_useVSYS`        | false   |    yes     | Add VSYS to message                                                                                       |
| `RH_useVIN`         | false   |    yes     | Add VIN to message                                                                                        |

**Sleep Modes**

| Property       | Default | At runtime | Details                                                                                                                                                                                                                                                                                  |
| -------------- | ------- | :--------: | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `RH_sleep`     | true    |    yes     | Sets the low power mode on the microcontroller after creating/sending a new message. The sleeping time will be `RH_tickTimer`                                                                                                                                                            |
| `RH_deepSleep` | false   |    yes     | Before sleep the microcontroller will cut the power to the modem (SIM868) and will be completely shut down. The modem will have a consumption of 0ma. When the modem wakes up, the entire process of connecting to the GSM network must be carried out. This will take about 20 seconds. |

**GPS Modem**

| Property         | Default | At runtime | Details                                                                                                                                                                                         |
| ---------------- | ------- | :--------: | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `RH_useGps`      | false   |    yes     | Habilita la lectura de datos GPS                                                                                                                                                                |
| `RH_useGpsTime`  | false   |    yes     | Add GPS time to message                                                                                                                                                                         |
| `RH_useLocation` | false   |    yes     | Add GPS location to message                                                                                                                                                                     |
| `RH_useAltitude` | false   |    yes     | Add altitude to message (determined by GPS)                                                                                                                                                     |
| `RH_useSpeed`    | false   |    yes     | Add speed to message (determined by GPS)                                                                                                                                                        |
| `RH_useCog`      | false   |    yes     | Add course to message (determined by GPS)                                                                                                                                                       |
| `RH_useSats`     | false   |    yes     | Add nº of satellites to message                                                                                                                                                                 |
| `RH_useHDOP`     | false   |    yes     | Add HDOP to message                                                                                                                                                                             |
| `RH_useGpsL86`   | false   |     no     | Use the Slave Module Quectel L86 from rhomb.io as GPS modem. It should be connected to the Socket 2 in a rhomb.io boarda like halley or deimos. In case of `false` will use the GPS from SIM868 |
| `SerialL86`      | false   |     no     | Uart for the L86 (fixed to 9600 bauds). It should be placed on the socket II of a Deimos or Halley board. The `SerialMon` should be `SerialUSB`  while using Quectel L86.                       |

**GSM Modem**

| Property            | Default | At runtime | Details                                                                                                 |
| ------------------- | ------- | :--------: | ------------------------------------------------------------------------------------------------------- |
| `RH_useGSMSignal`   | true    |    yes     | Add GSM signal strength to message                                                                      |
| `RH_useGsmTime`     | true    |    yes     | Add GSM time to message                                                                                 |
| `RH_useGSMLocation` | true    |    yes     | Add GSM location to message. It will be disable if `RH_useGPSLocation` is enabled and there is GPS Fix. |

**Message Log**

| Property               | Default | At runtime | Details                                                                                                                                                                                                                                                                                                                               |
| ---------------------- | ------- | :--------: | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `RH_logSize`           | 10      |     no     | Number of messages to be stored in memory. Messages are stored when `RH_transmitAlways`=false or when a data transmission fails (backup). This is saved on the emulated EEPROM of the micro. Caution because this uses the RAM while store/save on EEPROM. As a rule, (`RH_logSize` * `RH_logBytes`) should be less than 2000 (bytes) |
| `RH_logBytes`          | 150     |     no     | Size of each message stored in memory                                                                                                                                                                                                                                                                                                 |
| `RH_transmitLogIfFull` | true    |    yes     | Number of messages to be stored in the log. When this number is reached, all messages will be transmitted. The max value is `RH_logSize`                                                                                                                                                                                              |
| `RH_persistLog`        | true    |    yes     | Save the log in flash memory when new message is added                                                                                                                                                                                                                                                                                |

**Sensors**

| Property             | Default | At runtime | Details                                                                                                                                                                      |
| -------------------- | ------- | :--------: | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `RH_useBME280`       | false   |     no     | Enables the BME280 module for reading sensors (temperature, humidity and pressure)                                                                                           |
| `RH_useBME680`       | false   |     no     | Enables the BME680 module for reading sensors (temperature, humidity, pressure and VOC)                                                                                      |
| `RH_useTemp`         | false   |    yes     | Add temperature to message. Requires module BME280 or BME680                                                                                                                 |
| `RH_useHumidity`     | false   |    yes     | Add humidity to message. Requires module BME280 or BME680                                                                                                                    |
| `RH_usePressure`     | false   |    yes     | Add pressure to message. Requires module BME280 or BME680                                                                                                                    |
| `RH_useVOC`          | false   |    yes     | Add air quality to message. Requires module BME680.                                                                                                                          |
| `RH_useMQ7`          | 0       |    yes     | Add CO to message. The value of the Analog PIN where the sensor is connected, for example _A5_. The arduino function `analogRead(RH_useMQ7)` will be used. Set 0 to disable. |
| `RH_useRaining`      | 0       |    yes     | The value of the Analog PIN where the sensor is connected, for example _A2_. The arduino function `analogRead(RH_useRaining)` will be used. Set 0 to disable.                |
| `RH_useSonar`        | false   |    yes     | Enables de sonar HS-SR04                                                                                                                                                     |
| `RH_SonarEchoPin`    | MOSI    |     no     | Mosi pin                                                                                                                                                                     |
| `RH_SonarTriggerPin` | MISO    |     no     | MISO pin                                                                                                                                                                     |
| `RH_SonarMAX_DIST`   | 400     |     no     | Maximum distance we want to ping for (in cm). Maximum sensor distance is rated at 400-500cm.                                                                                 |
| `RH_useBuzzer`       | false   |     no     | Enables a buzzer. The buzzer should be connected to a GPIO and enabled with LOW signal.                                                                                      |
| `RH_buzzPin`         | false   |     no     | Pin to write buzzer status. LOW = enabled                                                                                                                                    |
| `RH_useCCS811`       | false   |     no     | Use CCS811 (CJMCU-811V1) for VOC and CO2                                                                                                                                     |
| `RH_useDHT`          | false   |     no     | Use DGT11 for temp and humidity                                                                                                                                              |
| `RH_useDTHPin`       | MISO    |     no     | Configure DHT11 PIN                                                                                                                                                          |
| `RH_useSHT20`        | false   |     no     | Use SHT20 for temperature and humidity                                                                                                                                       |
| `RH_useMQ7`          | false   |     no     | Carbon Monoxide                                                                                                                                                              |
| `RH_useMQ7Pin`       | CS1     |     no     | Pin where MQ7 is connected                                                                                                                                                   |
| `RH_useMQ135`        | false   |     no     | Air Quality (CO, Ammonia, Benzene, Alcohol, smoke)                                                                                                                           |
| `RH_useMQ135Pin`     | CS1     |     no     | Pin where MQ135 is connected                                                                                                                                                 |

**Interruption PIN**

| Property             | Default | At runtime | Details                                                                                                                                                                                          |
| -------------------- | ------- | :--------: | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `RH_useNMIForWakeup` | true    |     no     | Use NMI to wakeup from sleep. NMI does not have interruption support on the SAMD21 but is the button mounted on many rhomb.io PCBs. On each sleep cycle can check if this pin is HIGH and wakeup |

**GPIOS & LEDS**

| Property        | Default | At runtime | Details                                                                                                                            |
| --------------- | ------- | :--------: | ---------------------------------------------------------------------------------------------------------------------------------- |
| `RH_useLED`     | true    |     no     | Enables the use of the main LED of the microcontroller                                                                             |
| `RH_ledPin`     | LED     |     no     | Default microcontroller LED. Change GPIO to use an external LED                                                                    |
| `RH_useRGBLed`  | false   |     no     | Enables the RGB led mounted on boards like Halley or Hydra. Is used to inform about the status of GSM and GPS networks             |
| `RH_masterGPIO` | IO6     |    yes     | MCU pin to control an external device like a relay. See [Controlling and external device](/usage/#controlling-and-external-device) |

**Accelerometer**

| Property              | Default | At runtime | Details                                                                                  |
| --------------------- | ------- | :--------: | ---------------------------------------------------------------------------------------- |
| `RH_accel`            | false   |    yes     | Add accelerometer data to message                                                        |
| `RH_useMPU6050`       | false   |     no     | Uses MPU6050 module for accelerometer reading.                                           |
| `RH_MPU6050_ADDRESS`  | 0x68    |     no     | I2C Addres for MPU6050 module                                                            |
| `RH_useLIS2HH12`      | false   |    yes     | Enable the accelerometer LIS2HH12, mounted on the Halley PCB                             |
| `RH_accelTime`        | 3000    |    yes     | Threshold time to enable alarm movement                                                  |
| `RH_accelGPIO`        | 0       |    yes     | GPIO to enable or disable when movement alarm  is enabled                                |
| `RH_transmitAccelNow` | false   |    yes     | If true a new message will be created and send to the server after enable movement alarm |

**OBDII / CAN**

| Property           | Default | At runtime | Details                                                            |
| ------------------ | ------- | :--------: | ------------------------------------------------------------------ |
| `RH_useCAN`        | false   |     no     | Enables reading of the OBDII port using the Freematics I2C library |
| `RH_useCANBattery` | false   |     no     | Add car battery to message                                         |
| `RH_useCANSpeed`   | false   |     no     | Add car speed to message                                           |
| `RH_useCANRPM`     | false   |     no     | Add car RPM to message                                             |
| `RH_useCANFuel`    | false   |     no     | Add car fuel to message                                            |

**SDI12**

| Property      | Default | At runtime | Details                                   |
| ------------- | ------- | :--------: | ----------------------------------------- |
| `RH_useSDI12` | false   |     no     | Use enviropro sensors with SDI12 protocol |
| `RH_SDI12Pin` | IO6     |     no     | GPIO for SDI12 sensor                     |

**External RTC**

| Property       | Default | At runtime | Details                                   |
| -------------- | ------- | :--------: | ----------------------------------------- |
| `RH_useRTC`    | false   |     no     | Enables and external RTC to get the time. |
| `RH_useDS3231` | false   |     no     | Use the breakboard DS3231 as external RTC |

**System**

| Property           | Default    | At runtime | Details                                                                                                          |
| ------------------ | ---------- | :--------: | ---------------------------------------------------------------------------------------------------------------- |
| `RH_DEBUG`         | 1          |     no     | Show debug messages on UART port B (Serial)                                                                      |
| `RH_opmode`        | OP_STARTUP |    yes     | Default [operational mode](#operational-modes). Always should be OP_STARTUP                                      |
| `RH_primaryOpMode` | OP_AUTO    |    yes     | Primary OP mode. Currently only OP_AUTO is supported. This mode will be set at the end of OP_STARTUP operations. |



*At runtime: The configuration may be changed at runtime, using TCP/Bluetooth/Serial commands

## Remote Server

Remote server is configured with `RH_remoteServer` to set the domain or IP of the server and `RH_remotePort` for the port number.

For the APN of the SIM set `RH_apn`, `RH_apnUser` and `RH_apnPass`.

## UART Ports

Trackio needs two UART ports, one for communication with the modem and another one for the monitor debug. The Duino Zero module (SAMD21) includes three UART ports. The first one, `Serial`, corresponds to the UART-B port in the rhomb.io standard and is used for debugging. The second, `Serial1`, corresponds to the rhomb.io UART-A and is used to communicate with the modem connected to socket 1 of the PCB.

```c
// #####################################################
// Serial Configuration
// Modem at socket 1 (rhomb.io UART-A)
#define SerialSim Serial1
#define SerialSimBauds 38400
// Debug (rhomb.io UART-B)
#define SerialMon SerialUSB
#define SerialMonBauds 38400
```

To see the debug messages on the serial port connect your PCB to the computer using a USB cable and open a serial monitor on the TTY/COM port assigned. You maycan use software like Putty on Windows or Minicom on Linux.

Platformio IDE, running on Visual Studio Code or Atom Editor, has an incorporated serial monitor very useful to work while coding.

#### The fourth member, SerialExt

There is a fourth configurable serial port called SerialExt. With this UART you can send commands to an external device using the web interface of M2M System Source. More info at [SerialExt](/usage/#using-serialext)

## Battery

Trackio can handle up to 3 different battery types:

VBAT
:   This is the internal lithium battery connected to the PCB. They are usually batteries that work in a range between 3.5 and 4.2v. It is very likely that the device cannot work properly when this battery is below 3.9v unless there is a 5v source in VSYS. Use `RH_requiredVBAT` to set a minimun value before entering low power mode. In fact, the battery interval you can work with will depend to a great extent on the PCB you use (_Deimos_, _Halleybox_, _Hydra_...).

VSYS
:   When connected the USB VSYS will go up to 5V, if there is no USB VSYS = VBAT. When connecting a USB we will be loading VBAT and when receiving this data in the server we can interpret if the device is chargin or not. Use `RH_requiredVSYS` to set a minimun value before entering low power mode.

VIN
:   It is an external power supply that is generally under the control of the end user. The end user can use VIN to know the battery status of an external component, such as a bicycle or electric scooter. In boards such as _Halleybox_ or _Hydra_ this power supply has connected a DCDC from 6v to 60v and like VSYS can be used to charge VBAT. Use `RH_requiredVIN` to set a minimun value before entering low power mode.

Battery control may vary depending on the used PCB (_Deimos_, Phobos, _Halleybox_...). Please check the documentation of these boards for more details. For example some versions of _Deimos_ don't have support for VIN (or requires some hacking on the board)

### Transmit battery status to the server

You can add the 3 battery types to messages sent to the server using the `RH_useVBAT`, `RH_useVSYS` and `RH_useVIN` properties. `RH_useBatt` Should be `true`.

On a _Halley_ board and others, like _Hydra_, VIN could be very useful to control the battery of an external device connected to the PCB. They incorporate a DCDC that supports between 6V and 60V of input, ideal for industrial environments.

### Enter Sleep Mode

Use `RH_requiredVBAT`, `RH_requiredVSYS` and `RH_requiredVIN` to establish minimum operating values

requiredVBAT
:   The _Halleybox_ can reach minimum values of 3650mv. With _Deimos_ we will ensure a higher range above 3800mv.

requiredVSYS and requiredVIN
:   These voltages come from external sources and each user may have specific needs. Contact _rhomb.io_ technical support to discuss your case.

These values can be edited at runtime using remote configuration from the device manager of M2M System Source.

See usage for more details about [low power modes](/usage/#low-power-mode)

## Configuring Trackio at runtime

Trackio can be configured at runtime once it has been programmed. The  M2M System Source platform is used for this, which can send commands through TCP. It can also be configured via a Bluetooth with a mobile phone.

A command is a text string with a predetermined format which is interpreted by Trackio. For example:

```text
$8ksdj3234|tick:60;temp:1;humi:0;save:1|28#
```

This command is telling Trackio to change the message interval to 60 seconds, add  temperature and remove humidity, and finally save the new setting in EEPROM memory (persistence after reset). A unique command identifier is included (8ksdj3234), at the end of it the size of the command (without ID and symbols **$**, **|** and **#**).

The server can send commands in two different ways: direct transmission and delayed transmission.

Direct transmission implies that the server will attempt to connect to the device at the time the command is sent. If for some reason the device is not connected or the message cannot be sent, the command transmission will fail and the web browser will notify the web client (end user). The command will not be saved for forwarding in the future, it will simply be lost.

Delayed transmission will save the commands created by users and transmit them to the device the next time it makes a connection or send data. This mode is much more secure than direct transmission but can also take longer to communicate.

Direct transmission only will work if Trackio uses keep TCP always open with the `RH_keepTCPOpen=true` property.

!!! warning
    To learn more about how Trackio receives information read the notes from [receiving commands](/notes/#receiving-commands)

### List of remote parameters and the equivalent property on Trackio config

| Id  | Property          | data type          | Since Version |
| --- | ----------------- | ------------------ | ------------- |
| 1   | deepSleep         | boolean            | 0.3.0         |
| 4   | sleep             | boolean            | 0.3.0         |
| 2   | opmode            | number: 1\|2\|3\|4 | 0.3.0         |
| 3   | primaryOpMode     | number: 1\|2\|3\|4 | 0.3.0         |
| 5   | requiredVbat      | number 3600-3900   | 0.3.2         |
| 6   | requiredVin       | number > 6000      | 0.3.2         |
| 7   | requiredVsys5v    | number 4200-5000   | 0.3.2         |
| 8   | tickTimer         | number             | 0.4.0         |
| 9   | aliveTimer        | number             | 0.4.0         |
| 10  | keepTCPOpen       | boolean            | 0.4.0         |
| 11  | useTemp           | boolean            | 0.4.0         |
| 12  | usePressure       | boolean            | 0.4.0         |
| 13  | useCO             | boolean            | 0.4.0         |
| 14  | useHumidity       | boolean            | 0.4.0         |
| 15  | useVOC            | boolean            | 0.4.0         |
| 16  | useGpsTime        | boolean            | 0.4.0         |
| 17  | useGps            | boolean            | 0.4.0         |
| 18  | useLocation       | boolean            | 0.4.0         |
| 19  | useAltitude       | boolean            | 0.4.0         |
| 20  | useSpeed          | boolean            | 0.4.0         |
| 21  | useCog            | boolean            | 0.4.0         |
| 22  | useSats           | boolean            | 0.4.0         |
| 23  | useHDOP           | boolean            | 0.4.0         |
| 24  | useGSMSignal      | boolean            | 0.4.0         |
| 25  | useGsmTime        | boolean            | 0.4.0         |
| 26  | useGSMLocation    | boolean            | 0.4.0         |
| 27  | transmitWithSMS   | boolean            | 0.4.0         |
| 28  | SMSNumber         | string             | 0.4.0         |
| 29  | transmitLogIfFull | boolean            | 0.4.0         |
| 30  | persistLog        | boolean            | 0.4.0         |
| 31  | useBatt           | boolean            | 0.4.0         |
| 32  | useVBAT           | boolean            | 0.4.0         |
| 33  | useVSYS           | boolean            | 0.4.0         |
| 34  | useVIN            | boolean            | 0.4.0         |
| 35  | apn               | string             | 0.4.0         |
| 36  | apnUser           | string             | 0.4.0         |
| 37  | apnPass           | string             | 0.4.0         |
| 38  | remoteServer      | string (domain     | ip            | 0.4.0 |
| 39  | remotePort        | string (number)    | 0.4.0         |
| 40  | saveConf          | boolean            | 0.4.0         |
| 41  | hard reset        | boolean            | 0.4.0         |
| 42  | RH_masterGPIO     | boolean            | 0.4.0         |
| 43  | GPIO HIGH         | number             | 0.4.0         |
| 44  | GPIO LOW          | number             | 0.4.0         |
| 45  | GPIO INPUT        | number             | 0.4.0         |
| 46  | GPIO OUTPUT       | number             | 0.4.0         |
| 47  | useGas            | boolean            | 0.4.0         |
| 48  | useDustSensor     | boolean            | 0.4.0         |
| 49  | useSonar          | boolean            | 0.4.0         |
| 50  | useSerialExt      | boolean            | 0.4.0         |
| 51  | Send Serial Ext   | string             | 0.4.0         |
| 52  | useAccel          | boolean            | 0.4.0         |
| 53  | accelTime         | number             | 0.4.0         |
| 54  | transmitAccelNow  | boolean            | 0.4.0         |
| 55  | accelGPIO         | number             | 0.4.0         |
| 0   | reserved          |                    | 0.4.0         |

* boolean = 1|0 -> true,high | false,low
* Number values for batteries are stimations or for reference only
* Numbers for GPIOS should be valid GPIOS on the MCU. See rhip-pinmap.h for a complete list of available GPIOS on the Duino Zero

## Operational Modes

Operating modes are functions within the code that determine how the device should behave. There are currently two main modes, OP_STARTUP and OP_AUTO. In the future we want to develop other specific modes

!!! note
    The operational mode names, OP_STARTUP, OP_AUTO... etc, are macros defined in `Trackio.h`

OP_STARTUP (0)
:    The OP_STARTUP mode verifies that the configuration of the application is correct, the modem can be turned on and a connection to the server can be made. If all steps are correct, switching to OP_AUTO mode will be allowed. If any of the steps are invalid the OP_STARTUP mode will continue to run until it is resolved. Check `void op_startup()` in `main.c`

OP_AUTO (1)
:   Normal operating mode. The device will create messages according to the time set in     RH_tickTime. The message data will depend on the configuration selected with the options of type RH_useX. All the behaviour of the OP_AUTO mode will depend on the configuration, for example a message will be transmitted to the server if RH_transmitAlways=true, else will be saved on EEPROM... etc.

OP_LOW (2)
:   The device will sleep (deep sleep) in 30-second increments. Each time it wakes up it will check the battery status and if it has the appropriate value again it will exit from OP_LOW mode, returning to OP_STARTUP to start the whole system again. The Trackio::checkLowBattery() method will activate the OP_LOW mode if the battery readings are below the values indicated in RH_requiredVBAT, RH_requiredVIN and RH_requiredVSYS.

OP_RST (3)
:   Executes a system reset. For this purpose an infinite while will be launched which will trigger the mcu watchdog and the application will be started again in OP_STARTUP mode.

!!! warning

    The default operational mode must always be OP_STARTUP, in the configuration of `RH_primaryOpmode`, you set the mode to be changed once the startup ends, this must be OP_AUTO. It would not make sense to use OP_LOW and OP_RST as primary modes.

## 2G Modem

Trackio is compatible with [SIM868](https://rhomb.io/products/slave-modules/gps-gprs-sim868-esim/) and [SIM800](https://rhomb.io/products/slave-modules/gprs-sim800h-esim/) modules from rhomb.io. Both modules must be connected to socket 1 either _Deimos_ or _Halleybox_. The default configuration in static-conf.h uses the SIM868 with the following PINS:

```c
#define GSM_PWREN  IO0
#define GSM_PWRKEY IO1
#define GPS_EN     IO2
#define GSM_STATUS IO3
```

To use the SIM800 we can make the following change:

```c
#define GSM_PWREN  IO2
#define GSM_PWRKEY IO1
#define GPS_EN     0
#define GSM_STATUS IO0
```

> The deep sleep mode (`RH_deepSleep`) will not work with the Sim800 module of rhomb.io.