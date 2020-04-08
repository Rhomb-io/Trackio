# Welcome to Trackio

Trackio is a firmware written specially for the rhomb.io platform using the SAMD21 microcontroller (Duino Zero) and the SIM868/SIM800 GPRS/GPS modem from Simcom.

## Introduction

Although this documentation deals with Trackio as a firmware, it is actually a part of an ecosystem that includes [rhomb.io][1] hardware and the architecture of servers and web services created by [M2M System Source](https://m2msystemsource.com).

Initially Trackio was conceived as a GPS device. Over time new features were added, such as support for third-party sensor modules, BME280/680 or GAS sensors from the MQ family.

The aim is to create a general-purpose IoT device that covers a large number of configurations and modes of use thanks to the power of Microchip's SAMD21 microcontroller, with Cortex M0+ architecture.

## Features

* SAMD21 Microcontroller with SIM868 for GPRS/GPS or SIM800 for GPRS and Quectel L86 for GPS
* GPS Tracker with support for GSM Cell location when no GPS coverage is present
* Real Time bidirectional communication with custom TCP protocol
* Full remotely configurable through TCP, Bluetooth (SIM868) or UART
* Transmit data to remote servers through TCP or SMS
* Motion alarm with accelerometer
* Backup messages when transmissión fails (no GSM/GPRS signal)
* Power ON/OFF external device (like relays) through a GPIO
* Monitor the status of an external device (power on/off and battery status)
* OBDII Support through Freematics OBDII I2C Adapter
* Real Time configuration, transmit 1 message each 3 seconds
* Low power configuration, transmit 1 message per hour/day/week, mcu deep sleep.
* Integrated support for third-party sensors: BME280, BME680, CS811, DHT11... and more
* Support for protocol SDI12 (agricultural probes from EnviroPro)
* Easy configuration before flash the mcu, with a single configuration file
* Based on Arduino

# Hardware

We recommend the use of [rhomb.io][1] hardware, especially the [SAMD21DM2G1K - M2M System Source][2] kit that can be purchased in our online store and which is composed of a Deimos carrier board, the mcu Duino Zero (SAMD21), and the SIM868 modem.

## What is Trackio?

Trackio has one main function: to collect data from sensors and send it to a cloud server on the internet at set intervals. It is also possible to control it remotely from a web application or via bluetooth with a mobile phone.

Initially it was a GPS tracker, thanks to the SIM868 modem (rhomb.io slave module) that incorporates GPRS modem functionality to transmit over the internet, GPS to receive geolocation data and Bluetooth. Later, support was added for different types of sensors, such as temperature/humidity modules (dht11, sht20), air quality (bme680/bme280/ccs811, MQx), actuators (raindrop, GPIO control, sonar, buzzer, led rgb), OBDII port reading, agricultural probes (SDI12), bi-directional communication via TCP for remote control, bluetooth and serial control, accelerometer vibration alarm, data sending via TCP/UDP/SMS, message log, low power modes... and more.

There is a way to control all this and configure how we want our Trackio to behave. The file `lib/Trackio/static-conf.h` contains all the definitions/macros to configure the sending times, the sensors we want to use, the low power modes and a long etc of options.

## What cloud is used?
M2M System Source develops the gateway where Trackio devices are connected. We use a custom TCP communication protocol based on the exchange of text strings.

The operation of the communication protocol is described in the [Protocol](/protocol) section.

M2Mss exposes a public API of type RestFul with which any user can make requests and obtain the data from the sensors associated with his account. Details in [M2M System Source Api Docs][3].

M2Mss also has a web control panel for the display of real-time and historical data. Visit [M2M System Source Dashboard][4].

To access both the Api and the web dashboard you will need an M2Mss user account. Please contact us for more information.

[1]: https://rhomb.io
[2]: https://rhomb.io/products/rhombio-kits/rhombio_samd21dm2g1k-m2m-system-source-white/
[3]: https://apidocs.m2msystemsource.com
[4]: https://dash.m2msystemsource.com/