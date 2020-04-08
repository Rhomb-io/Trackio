# Communication Protocol

Trackio implements the communication protocol over TCP from *M2M System Source* (M2Mss). A text string is built with a certain format. This string includes the device identifier and the sensor or geolocation data.

There are two ways to make a connection and send data to the M2Mss server:

* Standar mode: Once the TCP is opened you can transmit strings directly by writing to the socket.
* Login mode: Alternatively, you can log in to the device first and then send the data

When we transmit without login the M2Mss server will close the TCP connection after receiving a data. On the other hand, if we login first, the server will keep the connection open. Closing or keeping the connection open will depend on the purpose of our application, but generally the connection is kept open when we have to send data in very short intervals of time, in the order of 5/10 seconds. On the contrary, if we want to send data in longer intervals, minutes/hours/days, we will close the socket after each sending.

# Standard Communication

The following example shows a sensing type data string without login (standard)

`883828383|s|temp:12.5$hum:23`

| Value            | Description        |
| ---------------- | ------------------ |
| 883828383        | DeviceID           |
| s                | data type: Sensing |
| temp:12.5$hum:23 | Body, sensor data  |

The body contains a group of values ordered by property/value and separated by the dollar ($) symbol. In this case we see a measurement of temperature at 12.5º and humidity at 23%.

We can say that our string is separated by 3 data groups: device id, data type and values.

# Communication with Login

The login string will be sent each time the TCP port is opened. Once logged in, the server will keep the TCP open and it will not be necessary to make the connection again.

The procedure is simplified in 4 steps:

1. Send login: `@id:868183033440186,v:0.4.0b3,iccid:8934013131824005391f,io:24,iostatus:0$`
2. Server responds `OK|1583153465430` (the number is a unix timestamp)
3. Send data: `s|gsm:20$gloc:39.511182,-0.453197,550$vb:4012$vs:4982$`
4. We repeat step 3 indefinitely. We return to point 1 if the TCP connection is closed.

Details to be taken into account:

* The login string has a special format, starting with "@" and ending with "$". It contains data sorted by property/value and separated with a comma.
* It is mandatory that the login contains the property id, the rest of the data is optional (explained below)
* When sending the login, the answer returns an unix timestamp value that we can use to keep a clock in our microcontroller.
* The data string does not include the device ID, it is not necessary because we have sent it in the login (the server holds the session)


## Login Format
Next table show parameters available for login

| Property | Description                                   | Required |
| -------- | --------------------------------------------- | -------- |
| id       | Device Id                                     | yes      |
| v        | Firmware Version                              | no       |
| iccid    | SIM card ICCID number                         | no       |
| io       | Nº of master GPIO on the microcontroller      | no       |
| iostatus | The status of the master GPIO (1=high, 0=low) | no       |

A good practice is to send as much data as possible at the first login after a microcontroller reset. The following logins send only the device ID and the other data may be added if there are changes (not usual)

## Transmit after login

Podemos empezar a enviar datos de sensores cuando la conexión TCP está abierta y hemos hecho login. La cadena a enviar no necesita incluir el id del dispositivo ya y que el servidor lo obtiene del login. Un ejemplo de trama:

`s|gsm:20$gloc:39.511182,-0.453197,550$vb:4012$vs:4982$`

We see two parts differentiated by the `|` symbol. The `s` tells the server that the string contains sensor information, the second part contains the data that will be sent to the server, with the same format mentioned in the Standard Communication

In the future it is expected that the platform will accept other types of messages, for example alarms, where the `s` will be changed to `a`.

## Accepted data in sensor chains

The following table shows the parameters recognized by the server:

| Property | Data type            | Description                                             |
| -------- | -------------------- | ------------------------------------------------------- |
| vin      | Number               | Battery value in mv for VIN                             |
| vbat     | Number               | Battery value in mv for VBAT                            |
| vsys     | Number               | Battery value in mv for VSYS                            |
| loc      | Number,Number        | GPS Location: [latitude,longitude]                      |
| sats     | Number               | Number of GPS satellites used to get location           |
| hdop     | Number               | GPS HDOP value                                          |
| gsm      | Number[1-30]         | GSM signal quality                                      |
| gloc     | Number,Number,Number | GSM Location: [latitude,longitude,precission] in meters |
| sog      | Number               | Speed                                                   |
| temp     | Number               | Temperature Cº                                          |
| humi     | Number               | Temperature Humidity %                                  |
| press    | Number               | Atmospheric Pressure mbars                              |
| voc      | Number               | VOC Air Quality from CCS811                             |
| iaq      | Number               | IAQ Air Quality (BSEC BMP680 driver)                    |
| co       | Number               | Gas value from MQ7 sensor                               |
| eco2     | Number               | Gas value from CCS811                                   |
| fuel     | Number               | Remaining liters                                        |
| rpm      | Number               | Car RPM                                                 |
| tmpr     | Number[0-1]          | Tamper protection. 1 to inform for a tamper alarm       |
| vbra     | Number[0-1]          | Vibration protection. 1 to inform for a vibration alarm |
