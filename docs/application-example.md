# Application example

### Before starting

* It is compulsory to have a Connected Battery above 3.8v up to 4.2v

### Software

* Use PlatformIO trough [VsCode](https://code.visualstudio.com/download)

### Steps

1. Clone the git project from here:
    * `https://gitlab.com/m2m-system-source/Trackio`
2. Rename `platformio.ini.sample` to `platformio.ini`
3. Rename `lib/Trackio/static-conf.h.sample` to `lib/Trackio/static-conf.h`
4. Fill out the info for the APN and the server in the `static-conf.h` file.

```
// APN
 #define RH_apn "example.isp.server"
 #define RH_apnUser ""
 #define RH_apnPass ""
 #define RH_remoteServer ""
 #define RH_remotePort ""
```

5. Set the `RH_useGps` to true

```
// GPS
#define RH_useGps true
```

6. Upload the firmware.

At this point the device is ready to work and should upload GPS data to the server specified in `RH_remoteServer`.