# Download the Code

The source is available in a private repository at Gitlab. The most courageous can download the master branch, which contains the latest changes and may not be stable. If you prefer to use a stable version download a zip in the releases section.

[Trackio private repository at Gitlab](https://gitlab.com/m2m-system-source/Trackio)

## Setup a development environment

Trackio has been written with PlatformIO and VS Code. Platformio is a CLI application, you can use the editor you're most comfortable with (more details in the Platformio documentation), but VS Code and Atom Editor have specific plugins that integrate very well.

Watch this tutorial if you have never used VS Code and PlatformIO: [PlatformIO IDE for VSCode](https://docs.platformio.org/en/latest/ide/vscode.html).

PlatformIO has a large user community and you'll find more information on a multitude of websites, as well as a fantastic official documentation.

## First Steps

After downloading Trackio it will be necessary to create the configuration files _platformio.ini_ and _static-conf.h_. Both files are offered with _.sample_ extension and will have to be duplicated (NOT removed) to remove the extension.

Once duplicated without the _.sample_ extension, the different parameters can be configured in both files.

### Configuring platformio.ini file

At the root of the project we should have created the file _platformio.ini_ from _platformio.ini.sample_. On this file we tell PlatformIO which compiler and upload method to use.

~~~ini
[env:zeroUSB]
platform = atmelsam
board = zeroUSB
framework = arduino
build_flags = -D SERIAL_BUFFER_SIZE=128
upload_port = /dev/ttyACM0
~~~

In `upload_port` you must indicate the port to which you have connected the board (through USB cable). In windows it will be something like `COM3`, `COM7`... etc. In Linux/Mac `/dev/ttyXXX`. If you only have one serial device connected you can comment this variable (`;upload_port = /dev/ttyUSB0`) and let PlatformIO recognize it by itself. This is useful when connecting and disconnecting the board several times from USB, the OS constantly changes the name of the port and you have to reconfigure it.

!!! Warning
    It is necessary to define the size of the Serial buffer because the GPS output from the modem often exceeds the 64 byte limit that the Arduino framework sets by default. 128 bytes are sufficient.

    On Arduino Zero look for the file _{$HOME}/.platformio/packages/framework-arduinosam/cores/samd/RingBuffer.h_. In this library the constant `SERIAL_BUFFER_SIZE` is defined, but it does not have a `#ifndef SERIAL_BUFFER_SIZE` to avoid duplicating the declaration when we add it to the `platformio.ini`. The solution is to directly edit this file and indicate a value of 128.

    Caution! this is a fix on an Arduino core file, so if you update _PlatformIO or the _Arduino_ libraries the file will be overwritten and the change will have to be made again.

### Configuring static-conf.h

In _lib/Trackio/static-conf.h_ we define all the configuration parameters that establish how our application will work, the main configuration. We can enable GPS, sensors reading, data transmit intervals, low consumption mode... etc.

See the complete list of [configuration options](/configuration/#static-configuration)

This configuration is saved on the SAMD21's emulated EEPROM and can be modified at runtime via TCP commands.

## Flash the Micro

It's time to upload the Code!

If you have configured the `platformio.ini` and `static-config.h` files correctly, all you have to do is connect your rhomb.io tracker to your computer using a USB cable and upload the code.

In the hardware configuration you will find more details about how to assemble your device with rhomb.io.
