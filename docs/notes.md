# Notes

## Receiving Commands

Trackio can receive information to change its configuration. We call them *commands* and there are currently three ways to receive them, by TCP, Bluetooth or SerialMonitor.

There are certain limitations when receiving data because it may be performing other tasks, such as transmitting data. This could cause the device to not be able to carry out all operations as it is expected to.

The web server, even though it has a TCP socket constantly open, it will be by default waiting for Trackio to send a data to take advantage of that period of time and respond with a command. As soon as Trackio transmits something the server can be sure that, when finished, the device is free of work and can receive information.

When Trackio accepts a Bluetooth connection, it changes its operational mode and stops transmitting information to the server. At this point Trackio waits for the Bluetooth device to transmit a command. There is a limited time for which Trackio will be listening and if it does not receive anything it will close the Bluetooth connection and return to its normal operating mode.

# Equivalence codes for properties in message's parameters

!!! warning
    These codes have not yet been included in Trackio's code but it is planned to do so.

!!! warning
    This is a work in progress, the list may be incomplete

When a message is transmitted (tcp or sms) to the server it includes a set of "property:value", where "property" represents a simple data, for example "temp (temperature)", "lat (latitude)"... etc. This list shows the number equivalences for each parameter. By using numbers instead of names we get smaller text strings that consume less GPRS data, on the other hand the strings lose readability for the human eye.

| Property            | Number | Desc.                                                                                      |
| ------------------- | :----: | ------------------------------------------------------------------------------------------ |
| Gsm Time            |   1    | Date from the GSM network. YYMMDDHHMMSSZZ -> 19012813080104                                |
| GPS Time            |   2    | Date from the GPS network. 20190128121443.000                                              |
| Localización        |   3    | Geolocation: latitude,longitude                                                            |
| Loc. GSM            |   4    | Geo localización with GSM: latitude,longitude,precission                                   |
| Altitude            |   5    | Number in meters                                                                           |
| Speed               |   6    | Number in Km/h                                                                             |
| Cog                 |   7    | Number from 1 to 359                                                                       |
| HDOP                |   8    | Number, GPS Signal Quality                                                                 |
| Sats.               |   9    | Número de satellites GPS                                                                   |
| Red GSM             |   10   | Number from 1 to 30                                                                        |
| Vbat                |   11   | Number Milvoltios                                                                          |
| Vin                 |   12   | Number mv                                                                                  |
| Vsys                |   13   | Number mv                                                                                  |
| Vmodem              |   14   | Number mv                                                                                  |
| Temperature         |   15   | Number centigrade degrees                                                                  |
| Humedad             |   16   | Number percentage                                                                          |
| Acelerómetro        |   17   | 3 comma separated values (x,y,z)                                                           |
| Giroscopio          |   18   | 3 comma separated values (x,y,z)                                                           |
| Presión Atmosférica |   19   | número                                                                                     |
| MQ-2                |   20   | Sensitive for Methane, Butane, LPG, smoke.                                                 |
| MQ-3                |   21   | numero                                                                                     |
| MQ-4                |   22   | Sensitive for Methane, CNG Gas                                                             |
| MQ-5                |   23   | Sensitive for Natural gas, LPG                                                             |
| MQ-6                |   24   | Sensitive for LPG, butane gas                                                              |
| MQ-7                |   25   | Sensitive for Carbon Monoxide                                                              |
| MQ-8                |   26   | Sensitive for Hydrogen Gas                                                                 |
| MQ-9                |   27   | Sensitive for Carbon Monoxide, flammable gasses.                                           |
| MQ-131              |   28   | Sensitive for Ozone                                                                        |
| MQ-135              |   29   | For Air Quality                                                                            |
| MQ-136              |   30   | Sensitive for Hydrogen Sulfide gas                                                         |
| MQ-137              |   31   | Sensitive for Ammonia                                                                      |
| MQ-138              |   32   | Sensitive for Benzene, Toluene, Alcohol, Acetone, Propane, Formaldehyde gas, Hydrogen gas. |
| MQ-214              |   33   | Sensitive for Methane, Natural gas.                                                        |
| MQ-216              |   34   | Sensitive for Natural gas, Coal gas                                                        |
| MQ303A              |   35   | Sensitive for Alcohol, Ethanol, smoke (just like the MQ-3)                                 |
| MQ306A              |   36   | Sensitive for LPG, butane gas                                                              |
| MQ307A              |   37   | Sensitive for Carbon Monoxide                                                              |
| MQ309A              |   38   | Sensitive for Carbon Monoxide, flammable gasses.                                           |
| MG811               |   39   | Sensitive for Carbon Dioxide (CO2).                                                        |
| Particles           |   40   | Particulas Ambientales                                                                     |
| AQ-104              |   40   | For air quality                                                                            |
| AQ-2                |   41   | Sensitive for Flamable gasses, smoke                                                       |
| AQ-3                |   42   | Sensitive for Alcohol, Benzine                                                             |
| AQ-7                |   43   | Sensitive for Carbon Monoxide                                                              |
| Iluminación         |   44   | número                                                                                     |
| Color               |   45   | numero                                                                                     |
| VBAT                |   46   | Batería interna                                                                            |
| VIN                 |   47   | Alimentación externa                                                                       |
| VSYS                |   48   | Alimentación USB                                                                           |
| VSIM                |   49   | Alimentación del Simcom                                                                    |
| IO0                 |   50   | IO status: 0=low,1=high                                                                    |
| IO1                 |   51   | IO status: 0=low,1=high                                                                    |
| IO2                 |   52   | IO status: 0=low,1=high                                                                    |
| IO3                 |   53   | IO status: 0=low,1=high                                                                    |
| IO4                 |   54   | IO status: 0=low,1=high                                                                    |
| IO5                 |   55   | IO status: 0=low,1=high                                                                    |
| IO6                 |   56   | IO status: 0=low,1=high                                                                    |
| IO7                 |   57   | IO status: 0=low,1=high                                                                    |
| IO8                 |   58   | IO status: 0=low,1=high                                                                    |
| IO9                 |   59   | IO status: 0=low,1=high                                                                    |
| IO10                |   60   | IO status: 0=low,1=high                                                                    |
| IO11                |   61   | IO status: 0=low,1=high                                                                    |
| IO12                |   62   | IO status: 0=low,1=high                                                                    |
| IO13                |   63   | IO status: 0=low,1=high                                                                    |
| IO14                |   64   | IO status: 0=low,1=high                                                                    |
| IO15                |   65   | IO status: 0=low,1=high                                                                    |
| IO16                |   66   | IO status: 0=low,1=high                                                                    |
| IO17                |   67   | IO status: 0=low,1=high                                                                    |
| IO18                |   68   | IO status: 0=low,1=high                                                                    |
| IO19                |   69   | IO status: 0=low,1=high                                                                    |
| IO20                |   70   | IO status: 0=low,1=high                                                                    |
| IO21                |   71   | IO status: 0=low,1=high                                                                    |
| IO22                |   72   | IO status: 0=low,1=high                                                                    |
| IO23                |   73   | IO status: 0=low,1=high                                                                    |
| IO24                |   74   | IO status: 0=low,1=high                                                                    |
| IO25                |   75   | IO status: 0=low,1=high                                                                    |
| IO26                |   76   | IO status: 0=low,1=high                                                                    |
| IO27                |   77   | IO status: 0=low,1=high                                                                    |
| IO28                |   78   | IO status: 0=low,1=high                                                                    |
| IO29                |   79   | IO status: 0=low,1=high                                                                    |
| IO30                |   80   | IO status: 0=low,1=high                                                                    |
| IO31                |   81   | IO status: 0=low,1=high                                                                    |
| IO32                |   82   | IO status: 0=low,1=high                                                                    |
| IO33                |   83   | IO status: 0=low,1=high                                                                    |
| IO34                |   84   | IO status: 0=low,1=high                                                                    |
| IO35                |   85   | IO status: 0=low,1=high                                                                    |
| IO36                |   86   | IO status: 0=low,1=high                                                                    |
| IO37                |   87   | IO status: 0=low,1=high                                                                    |
| IO38                |   88   | IO status: 0=low,1=high                                                                    |
| IO39                |   89   | IO status: 0=low,1=high                                                                    |
| IO40                |   90   | IO status: 0=low,1=high                                                                    |
| user                |   u    | Username                                                                                   |
| pass                |   p    | Password                                                                                   |
| litres              |   l    | Litres                                                                                     |
| tag epc             |   t    | Text string in a tag (NFC)                                                                 |

## Changelog

### 0.4.0 - 2019-04-23

* Added unique ID and size on remote commands
* Added more than 20 remote configurable parameters
* Added support for third party sensing modules (BME280/680, CCS811, MQ gas sensors...)
* Added support for OBDII using Freematics I2C Adapter
* Added geo location with GSM CELL ID
* Added support for transmission through SMS
* Added backup log messages when transmit fails
* Added Bluetooth support for remote configuration
* Added new docs
* Changed any GPIO can be used to control an external device
* Change sayHello() to send more data (iccid, firmware version, imei...)
* Many bug fixes, code refactoring and improvements

### 0.3.2 - 2019-01-21

* static-conf.h: ADD UART_BUFFER_SIZE para asignar tamaño de la variable buffer en Trackio.cpp
* Trackio::sendCommand(): FIX bug overflow del buffer
* Trackio::gprsIsOpen(): FIX, no se procesaba correctamente la respuesta de AT+CGATT?
* main.cpp->transmitAlive(): FIX bug al reconectar el TCP, realizar registro en el gateway con Trackio::sayHello()
* Trackio::applyConf(): ADD transmissionClock, requiredVbat/Vin/Vsys configurables de forma remota
* Trackio::processCommand(): ADD comando "reset" realiza un hardReset -> #RST|1$
* Trackio::processCommand(): ADD comando "save" guarda la configuración -> #SAVE|1$
* Trackio::cmd_setConf(): eliminado el guardado automatica al recibir una nueva configuración remota
* Trackio::hardReset(): ADD reset del Sim868 con "Trackio::powerOff()" previo al hardreset
* Trackio::openTcp(): ADD reset de Trackio::transmissionClockCounter
* Trackio::hardReset(): FIX uso correcto de hardReset()
* Trackio.cpp/h: ADD soporte para lectura de baterías con HalleyBox
* Trackio::enableGprs() renombrado como Trackio::openGprs()
* Trackio::begin(): Cambio en secuencia de arranque para ejecutar ATE0
* Minor Fixes

### 0.3.1 - 2018-11-15

* FIX bug al procesar comando CIPSTART, se evaluaba la linea 1 en lugar de la 2
* Añadido parametro timeout a sendAt
* Reconectar TCP si hay fallo al enviar alive
* Trackio::sayHello(): Enviar el número de versión del firmware

### 0.3.0 - 2018-12-15

* Refactoring para Arduino Zero (veníamos de Atmega328p): Añadidas librerías FlashStorage y Adafruit Sleepydog y ReadLine
* Nuevo método sendAt reemplaza sendCommand. Mejorada la comunicación con el modem
* Gestión de acelerometro y alarma de movimiento
* Otros arreglos y limpieza

### 0.2.5 - 2018-10-16

* Todos los valores de configuración se han transferido a `static-conf.h` para evitar modificaciones "innecesarias" en archivos CORE.
* Añadidas 3 nuevos GPIOS configurables de forma remota, IO7, MOSI y MISO. Eliminada LED01 como configurable de forma remota
* FIX BUG. Se detecta un error al entrar en modo sleep (Trackio::sleepNow()) por el cual el dispositivo nunca llegaba a entrar en modo bajo consumo (OP_LOW) y esto podía impedir que la batería se cargara correctamente cuando el potenciomtro (P1) tuviera un nivel de carga bajo
* FIX. Las variables requriedVbat/Vsys5v/Vin no estaban incluídas por defecto en la configuración inicial (Trackio::loadConf())
* FIX. Trackio::openTcp() existía una comprobación redundante al verificar si el puerto estaba realmente abierto
* Eliminadas algunas variables en desuso y ajustados los tipos de datos para reducir consumo de ram
* Cambios menores.

### 0.2.4 - 2018-10-15

* En modo OP_TCP se utiliza el método `serialEvent()` (main.cpp) de Arduino para evitar el pool constante al `Serial.available()` en el loop principal.
* En modo OP_TCP se añade un nuevo pool al servidor (`transmitAlive()`) y su variable de configuración `cfg.transmissionClock`. Este pool junto al anterior serialEvent() permiten que la transmisión de comandos servidor->cliente se realice de forma más eficiente.
* Valores _required_ de vbat, vin y vsys incluídos en la configuración general
* Actualizado README.md con información detallada
* Los ACK al servidor siempre devuelven el estado de la IO6
* Nuevo archivo de configuración static-conf.h
* Se crean archivos con extensión .sample para static-conf.h y platformio.ini con el fin de evitar subir los archivos originales a los repositorios públicos.


### 0.2.3 - 2018-10-05

* Fix watchdog reset repentino en el startup, se añade wdt_disable() en el setup
* Enviar en trama VSYS y VIN junto con VBAT
* Enviar GSM junto con batería cuando no hay GPS
* Eliminado cfg.battInterval, se mide batería siempre que se envía trama


### 0.2.2 - 2018-10-03

* Añadido powerOff en la secuencia de arranque
* Añadido watchdog reset cuando openTcp falla


### 0.2.1 UNO - 2018-07-27

* Eliminado todo el código de Duino Zero, incluídas las librerías externas
    de RTC y SleepyDog para dar compatibilidad exclusiva a Duino Uno
* Eliminadas variables de configuración y otras funciones (como bluetoth) para
    reducir consumo de memoria de ram.
* Se utilizan las funciones de eeprom propias de AVR para guardar la configuración
    general. El objeto de configuración se saca de la clase trackio y se convierte
    en un struct global.
* Añadida lectura analógica de baterías para VBAT, VIN y VSYS.


### 0.2.0 - 2018-07-12:

* OP_TCP: timer para verificar estado de la conexión tcp y reconectar en caso necesario
* Duino Zero: Implementado watchdog+sleep con la librería SleepyDog
* RTC: Implementado un reloj simple obteniendo la fecha desde servidor web. Se utiliza
    la librería timeLib.h para parsear el unix timestamp en fechas para humanos
* Se vacía el setup() del main, OP_STARTUP lo reemplaza. Implementado mecanismo de
    protección ante fallos para reestablecer modo OP_STARTUP si necesario
* OP_AUTO abre/cierra el TCP cuando necesario
* Tramas de datos: Datos de GPS, Sensing y CAN unificados en una unica trama
    y agregado el método transmitData() para realizar el envío
* Método sendCommand, agregada limpieza de buffer y aumentado de 15 a 25 el número
    de intentos para esperar respuesta

## Credits

Develop by Jordi E. with the support of Guillermo A. and Pedro P. Docs written by Jordi E,

Third party
