/** @file */

/*
 * @brief Trackio, The IoT device for tracking and sensing.
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
 * @file Trackio.h
 * @author Jordi Enguídanos <jenguidanos@rhomb.io>
 */

#ifndef TRACKIO
#define TRACKIO
#define VERSION "0.4.0RC1"

#include <Arduino.h>
#include "Wire.h"

#define S200_DUINO_ZERO_v1_0
#include "rhio-pinmap.h"
#include "static-conf.h"

#define _(x) \
  if (RH_DEBUG) SerialMon.print(x)
#define __(x) \
  if (RH_DEBUG) SerialMon.println(x)
#define ___(x1, x2) \
  if (RH_DEBUG) {   \
    _(x1);          \
    __(x2);         \
  }
#define _title(x)                                                   \
  if (RH_DEBUG) {                                                   \
    __(F(""));                                                      \
    __(F(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")); \
    ___(F("> "), x);                                                \
  }

/**
 * @brief Default operational mode when starting the device
 */
#define OP_STARTUP 0

/**
 * @brief Automatic operational mode, create messages on time intervals
 */
#define OP_AUTO 1

/**
 * @brief Force a RESET of the system using the watchdog through a infinity loop
 */
#define OP_RST 2

/**
 * @brief Operational mode low power.
 *
 * It will turn off the MCU and the modem, the device will stop working until
 * the battery values ​​determined in cfg.required (Vbat / Vsys / Vin) are
 * normalized.
 */
#define OP_LOW 3


/**
 * @brief Store for configuration parameter received over TCP or bluetooth
 */
struct Command {
  char id[20];                  /// id of parameter, a string of 8 chars
  char props[CMD_BUFFER_SIZE];  /// body of the command (prop:value csv),
                                /// something like "42:1;40:1"
  char propsProcessed[100];     /// after processing the commandm the numbers of
                                /// props applied something like "42,40"
  size_t size;                  /// size of the body, from 42:1;40:1 -> 9
};

/**
 * @brief Trackio general configuration
 *
 * This struct is stored in flash memory and read at each reset. When the device
 * is reset, given the nature of the emulated memory of the SAMD21, it is
 * computed if it already exists in flash, otherwise a new conf will be created
 * by reading the default values defined in the static-conf.h frames.
 */
struct Conf {
  int eeprom;  /// Flag to know if there is already a configuration saved in
               /// EEPROM
  char log[RH_logSize][RH_logBytes];
  char battMode;  // 1 simcom, 2 deimos, 3 halleybox

  bool deepSleep;
  bool sleep;

  uint8_t opmode = RH_opmode;
  uint8_t primaryOpMode;

  unsigned int requiredVbat;
  unsigned int requiredVin;
  unsigned int requiredVsys5v;

  unsigned long aliveTimer;
  unsigned long tickTimer;

  bool keepTCPOpen;
  bool transmitAlways;
  uint8_t transmitLogIfFull;
  bool persistLog;
  uint32_t masterGPIO;

  char apn[20];
  char apnUser[20];
  char apnPass[20];
  char remoteServer[50];
  char remotePort[6];

  bool useTemp;
  bool useHumidity;
  bool usePressure;
  bool useCO;
  bool useCO2;
  bool useVOC;
  bool useGas;
  uint32_t useRaindrops;
  bool useDustSensor;
  bool useSonar;
  bool useGsmTime;
  bool useGSMLocation;
  bool useGSMSignal;
  bool useGpsTime;
  bool useGps;
  bool useLocation;
  bool useAltitude;
  bool useSpeed;
  bool useCog;
  bool useSats;
  bool useHDOP;
  bool useBatt;
  bool useVBAT;
  bool useVSYS;
  bool useVIN;
  bool useAccel;
  uint32_t accelTime;
  bool transmitAccelNow;
  uint32_t accelGPIO;
  bool useBl;
  bool useCAN;
  bool useCANSpeed;
  bool useCANBattery;
  bool useCANFuel;
  bool useCANRPM;
  bool transmitWithSMS;
  char SMSNumber[14];
  bool useRTC;
  bool useSDI12;
  bool useSerialExt;
};

extern struct Conf cfg;

class Trackio {
 public:
  Trackio();
  /**
   * @brief Almacena una trama GPS.
   */
  struct GPSData {
    int fix = 0;
    char time[20];
    char lat[15];
    char lon[15];
    char alt[10];
    char sog[7];
    char cog[7];
    char hdop[5];
    char sats[5];
  } gps;

  /**
   * @brief Localización via GSM
   * Los datos se guardan como chars para luego ser añadidos al mensaje con
   * mayor facilidad usando strcat
   */
  struct GSMLocation {
    bool fix = false;
    char lat[15];
    char lon[15];
    char precission[10];
  } gsmLoc;

  /**
   * @brief Fecha y hora extraída por GSM o GPS
   */
  struct Time {
    char year[4];
    char month[4];
    char day[4];
    char hours[4];
    char minutes[4];
    char seconds[4];
    char zone[4];
    time_t t;
  } time;

  /**
   * @brief Mediciones de sensores
   */
  struct Sensors {
    char temperature[20];
    char humidity[20];
    char pressure[20];
    char altitude[20];
    char co[20];         // Gas CO
    char co2[20];        // Gas CO
    char raindrops[20];  // Hoja de Agua
    char voc[20];        // Volatile organic compound (Air Quality)
    char airParticles[20];
    char distance[20];
  } sensors;

  /**
   * @brief Timers.
   *
   * Los timers permiten configurar cada cuanto tiempo se realiza una acción,
   * por ejemplo enviar una trama GPS. El timer base sirve como comparador
   * para el resto de timers (realiza el cálculo) y se inicializa en el método
   * Trackio::sayHello(), tras recibir el tiemstamp del servidor.
   */
  struct Timers {
    /**
     * @brief Timer principal. Al entrar al loop se actualiza con millis()
     *
     * El resto de timers utilizan este para comparar el tiempo pasado
     * desde la última medición. Trackio::Timers.base solo se inicializa cuando
     * se sincroniza el relog con el servidor, en el método Trackio::sayHello()
     *
     * Hasta que Trackio::Timers.base no se inicialice el resto de relojes
     * (timer1, timer,2 ...) tampoco lo hará
     */
    unsigned long base;

    /**
     * @brief Timer para enviar los keep alive al servidor
     */
    unsigned long aliveTimer;

    /**
     * @brief Timer para la creación de mensajes
     */
    unsigned long tickTimer;

    /**
     * @brief Añade tiempo (8 segundos) en cada ciclo sleep para rectificar
     * el reloj interno del microcontrolador, el cual se detiene cada vez que
     * se habilita el modo sleep (Trackio::sleepNow())
     */
    unsigned long offset;

  } timers{0, 0, 0, 0};

  /**
   * @brief Almacena el voltaje de la batería interna (VBAT)
   * Se obtiene mediante el método Trackio::getBattery()
   */
  unsigned int vbat;

  /**
   * @brief Almacena el voltaje de la batería externa (VIN)
   * Se obtiene mediante el método Trackio::getBattery()
   */
  unsigned int vin;

  /**
   * @brief Valor de VSYS_5V (según esquemáticos en PCBs... deimos, halley)
   */
  unsigned int vsys_5v;

  /**
   * @brief Número de IMEI del simcom
   *
   * Se utiliza como id del dispositivo para conectar con el servidor
   */
  char imei[16];

  /**
   * @brief Número de ICCID.
   *
   * Se envía al servidor la primera vez que se  ejecuta Trackio::sayHello()
   * tras un reset.
   */
  char iccid[22];

  /**
   * @brief Indica si el registro de la red GSM se ha llevado a cabo.
   */
  bool cregOk;

  /**
   * @brief Indica si el comando AT ha devuelto OK
   *
   * Sirve como comprobación básica para determinar que el simcom está conectado
   * a la PCB y funcionando.
   */
  bool atOk;

  /**
   * @brief Indica si se ha abierto la conexión TCP con el servidor
   */
  bool tcpOk;

  /**
   * @brief Indica si hay conexión GPRS.
   */
  bool gprsOk;

  /**
   * @brief Almacena el ultimo dato de calidad de señal GSM.
   * @see Trackio::getSignalStrength()
   */
  char gsmSignal[4];

  /**
   * @brief Almacena el mensaje creado en Trackio::createMessage()
   */
  char message[MAX_MESSAGE_SIZE];

  /**
   * @brief Almacena la ultima respuesta devuelta por transmit()
   */
  char lastTcpResponse[120];

  /**
   * @brief Comando para aplicar configuración.
   *
   * Detalles en Trackio::isCommand/parseCommand/processCommand
   */
  Command command;

  /**
   * @brief Determina si el can está accesible. Se pasa a true si obd.init()
   * se ejecuta con éxito.
   */
  bool CANEnabled = false;

  /**
   * @brief Solo enviaremos el número de bastidor una vez
   * @todo Añadir al login en lugar de tramas convencionales
   */
  bool CANVINSent = false;

  /**
   * @brief Indica si se ha realizado un primer registro en el servidor
   *
   * En el primer registro se envian multiples datos, como IMEI, ICCID,
   * version del firmware. En los siguientes registros únicamente enviaremos
   * el IMEI o identificador de dispositivo
   */
  bool firstGreetingsSent = false;

  /**
   * @brief Almacena la IP local, asignada por el operador después de
   * habilitar GPRS. Enviaremos la IP en el primer greetings, tras un reset.
   */
  char localIp[16];

  /**
   * @brief Indica si el dispositivo está en movimiento (true) o no (false).
   *
   * Se utiliza el acelerómetro para determinar el movimiento. En el main.cpp
   * se encuentra el método encargado de comprobar si existe movimiento,
   * funcionando en un loop en paralelo de forma ininterrumpida.
   *
   * Se pasa a false en el método Trackio::afterTransmit(), el cual comprueba
   * si el estado de movimiento se ha enviado al servidor.
   *
   * @see main.cpp > chechMovement()
   */
  bool moving = false;

  /**
   * @brief Modo de cálculo de aceleración (fuerzas G)
   *
   * Los datos en bruto leídos en el acelerómetro se convierten a valores
   * tipo fuerzas G. El LIS2HH12 (acelerómetro halley) soporta 2, 4 y 8 G:
   *
   * 0 = +/-2g
   * 1 = +/-4g
   * 2 = +/-8g
   */
  int gravity = 0;

  /** @brief Eje X acelerómetro */
  float x = 0;
  /** @brief Eje Y acelerómetro */
  float y = 0;
  /** @brief Eje Z acelerómetro */
  float z = 0;

  /**
   * @brief cuenta el número de veces que se produce un fallo a la hora de
   * obtener posición GPS con CELL ID
   *
   * Se ha visto que en determinados momentos el comando `AT+CLBS=1,1`
   * (utilizado para obtener localización por GSM) deja de funcionar.
   */
  uint8_t cellIdFails = 0;

  /**
   * @brief Número de fallos permitidos al obtener CELL ID antes de reiniciar
   *
   * Depende de `Trackio::cellIdFails` (contador) y Trackio::getGsmLocation()
   */
  uint8_t cellIdFailsLimit = 4;

  /**
   * @brief Force sleep during this number of seconds.
   *
   * This is used when activating the opmode OP_LOW. The device can be remotely
   * configured to sleep during a number of seconds. When trackio receive a commands
   * for sleep it saves the number on this variable and enables OP_LOW.
   *
   * The op_low() method on main.cpp will reset this after wakeup.
   */
  static unsigned long sleepSecs;

  /**
   * @brief Duración de cada blink
   *
   */
  uint16_t blinkTime;

  /**
   * @brief Número de veces que se ahrá blink. Se resetea tras cada ejecución
   * @see main.cpp/led()
   */
  uint16_t blinkTimes;

  // #########################################################################

  /**
   * @brief Inicializa varios componentes como los puertos serial, Sim868...etc
   *
   * - Habilita los puertos UART
   * - Configura los pines del Sim868 (Trackio::Configure())
   * - Enciende el Sim868 (Trackio::powerOn())
   * - Carga la configuración guardad en EEPROM (Trackio::loadConf())
   *
   * @return true  Si powerOn() se ejecuta con éxito.
   * @return false De lo contrario
   */
  bool begin();

  /**
   * @brief Establece el modo de las distintas GPIOS
   */
  void configureIOs();

  /**
   * @brief Enciende el modem
   *
   * Encender el modem supone activar a HIGH la línea PWR_EN y activar
   * PWR_KEY dando un pulso de 1.5s a HIGH.
   *
   * PWR_EN, PWR_KEY y GSM_STATUS están definidos en static-conf.h
   *
   * @return true  Si la línea GSM_STATUS obtiene el valor alto.
   * @return false Si GSM_STATUS recibe valor LOW (el momdem no se enciende)
   */
  bool powerOn();

  /**
   * @brief Apaga el SIMCOM por completo
   */
  void powerOff();

  /**
   * @brief Llama a getIccid() y getImei()
   *
   * No realiza nada con la info devuelta, simplemente la escribe en SerialMon
   * para verla en consola
   *
   * OJO: este método es el único encargado de llamar a getImei(). Si no
   * ejecutamos printInfo() en nuestro main.cpp deberemos ejecutar manualmente
   * el método printImei() ya y que este (el imei) se almacena en una variable
   * para ser usado a posteriori en las comunicaciones con el servidor y es
   * IMPRESCINDIBLE
   */
  void printInfo();

  /**
   * @brief Muestra el ICCID en consola
   *
   * El ICCID es un valor dado por la eSim, en caso de que falle esta lectura
   * probablemente sea debido a que el chip eSim tiene algún problema de
   * conexión (teniendo en cuenta que el resto de operaciones funcionan con
   * normalidad)
   */
  void getIccid();

  /**
   * @brief Lee el estado de la batería
   *
   * Si cfg.battMode=1 se leerá batería del simcom (AT+CBC), solo VBAT
   * Si cfg.battMode=2 se leerá batería de los pines analógicos.
   *  Las analógicas de VBAT como VIN están declaradas en `static-conf.h`
   * Si cfg.battMode=3 estamos usando Halley, leemos baterías del ADC con
   *  la librería TLA2024
   *
   * El resultado se añadirá al mensaje en función del modo operacional
   * seleccionado y la configuración
   */
  void getBattery();

  /**
   * @brief Lee la batería interna (VBAT) utilizando el comando `AT+CBC` del
   * simcom.
   *
   * Este método es menos preciso que `getAnalogBattery()` o
   * `getTLA2024Battery()` y solo se utiliza cuando ninguno de los dos
   * anteriores está disponible. No conoceremos el valor de VSYS y VIN y
   * el valor devuelto para VBAT está rectificado por el propio simcom, por
   * lo que nos dará un marge de error de 200/300mv en comparación a medir
   * la batería directamente con multimetro. No se ha hecho un estudio
   * preciso sobre cuanta caída hay realmente entre el resultado devuelto
   * por `AT+CBC` y medir directamente con multímetro.
   *
   * Para activar este método de lectura se utiliza el macro
   * `readBatteryMode = 1`  en `static-conf.h`
   *
   * @see Trackio::getBattery()
   */
  void getSimcomBattery();

  /**
   * @brief Lee la batería de los pines analógicos definidos en static-conf.h
   * para VBAT, VIN y VSYS
   *
   * Se utiliza este método para la lectura de baterías cuando `RH_battMode=2`
   *
   * Suele usarse en PCBs como Deimos o Phobos, las cuales no
   * tienen el DCDC TLA2024 que sí usan Halley o Hydra. En estas últimas PCBs
   * es mejor usar el método Trackio::getTLA2024Battery();
   */
  void getAnalogBattery();

  /**
   * @brief Lee el estado de un pin analógico
   *
   * @param adc_pin
   * @return uint16_t
   */
  uint16_t readAnalogBatt(byte adc_pin);

  /**
   * @brief Configuración inicial para el DCDC TLA2024 usado en la lectura
   * de baterías.
   */
  void configureTLA2024();

  /**
   * @brief Lee los tres voltajes para VBAT, VIN y VSYS desde el DCDC TLA2024
   *
   * El TLA2024 es un DCDC que acepta entradas de voltaje de hasta 70v
   */
  void getTLA2024Battery();

  /**
   * @brief Lee un canal concreto del TLA2024
   *
   * @param channel
   * @param aux
   * @return float
   */
  float readTLA2024Battery(uint8_t channel, float aux);

  /**
   * @brief Se ejecuta en Trackio::getBattery() y comprueba que el estado
   * de las distintas baterías sea correcto según la configuración de
   * cfg.requiredVbat/Vsys/Vin.
   *
   * Si alguna batería es baja se establecerá el modo operacional OP_LOW
   * y el main.cpp/loop entrará en bajo consumo.
   *
   * @return true
   * @return false
   */
  bool checkLowBattery();

  /**
   * @brief Muestra y guarda la calidad de la señal GSM como un valor númerico
   * entre 1 y 30
   *
   * Posible valores:
   * 2-9   -> Marginal
   * 10-14 -> OK
   * 15-19 -> Good
   * 20-30 -> Excellent
   */
  void getSignalStrength();

  /**
   * @brief Ejecuta comando `AT+CPIN?` para ver el estado del pin de la sim
   */
  void printPin();

  /**
   * @brief Muestra el IMEI en consola y lo almacena en la variable
   * Trackio::imei
   */
  void getImei();

  // #########################################################################
  /**
   * @brief Llama a checkModem() y checkCreg(), devuelve TRUE si ambos métodos
   * también lo hacen.
   *
   * @return true Si el estado es válido para continuar
   * @return false Si hay algún problema. Por favor revisa checkModem() y
   * checkCreg()
   */
  bool checkStatus();

  /**
   * @brief Envia comando AT y ATE0 y verifica OK en respuesta.
   *
   * @return true  Indicaría que el Simcom está listo para funcionar
   * @return false No se puede continuar, hard reset podría ser necesario
   */
  bool checkModem();

  /**
   * @brief Verifica el registro de red
   *
   * Envía el comando `AT+CREG?` y espera que la respuesta contenga las cadenas
   * ",5" o ",1". `AT+CREG` devuelve dos valores, estado del registro y tipo
   * de tecnología. Se representa con dos números separados por coma: "0,5",
   * "0,1", "0,3", "3,1"...El primero indica el tipo de tecnología, el segundo
   * el estadoo del registro:
   *
   * Possible values of registration status are,
   *  - 0 not registered, MT is not currently searching a new operator to
   register to
   *  - 1 registered, home network
   *  - 2 not registered, but MT is currently searching a new operator to
   register to
   *  - 3 registration denied
   *  - 4 unknown (e.g. out of GERAN/UTRAN/E-UTRAN coverage)
   *  - 5 registered, roaming
   *  - 6 registered for "SMS only", home network (applicable only when
   indicates E-UTRAN)
   *  - 7 registered for "SMS only", roaming (applicable only when indicates
   E-UTRAN)
   *  - 8 attached for emergency bearer services only (see NOTE 2) (not
   applicable)
   *  - 9 registered for "CSFB not preferred", home network (applicable only
   when indicates E-UTRAN)
   *  - 10 registered for "CSFB not preferred", roaming (applicable only when
   indicates E-UTRAN)*

   * Possible values for access technology are,
   *  - 0 GSM
   *  - 1 GSM Compact
   *  - 2 UTRAN
   *  - 3 GSM w/EGPRS
   *  - 4 UTRAN w/HSDPA
   *  - 5 UTRAN w/HSUPA
   *  - 6 UTRAN w/HSDPA and HSUPA
   *  - 7 E-UTRAN
   *
   * @return true Si encontramos ",5" o ",1"
   * @return false
   */
  bool checkCreg();

  /**
   * @brief Indica si Trackio se ha inicializado. Se cambia en Trackio::begin()
   */
  bool ready = false;

  // #########################################################################

  /**
   * @brief comprueba si está conectado a GPRS
   *
   * Ejecuta comando AT+CGATT? sobre el modem y espera recibir un OK. También
   * se comprueba que haya una dirección IP
   *
   * @return true  Si el GPRS está habilitado
   * @return false
   */
  bool gprsIsOpen();

  /**
   * @brief Habilita el GPRS. De no hacerlo el dispositivos no podrá iniciar
   * el tcp para enviar datos.
   *
   * No se habilitará el GPRS si se utiliza RH_transmitWithSMS=true
   *
   * @return true
   * @return false
   */
  bool openGprs();
  bool connectGprs();

  /**
   * @brief Habilita GPS
   *
   * Ejecuta los comandos `AT+CGNSPWR=1` y `AT+CGPSRST=0` y pone a HIGH la
   * línea `GPS_EN`
   *
   * @return true Si AT+CGNSPWR=1 devuelve OK
   * @return false
   */
  bool powerOnGps();

  /**
   * @brief Obtiene una trama GPS con el comando `AT+CGNSINF`
   *
   * Al recibir la trama la enviará al método Trackio::parseGps()
   */
  void getGps();

  /**
   * @brief Resetea los valores del struct Trackio::GPSData. Basta con
   * establecer la propiedad FIX a 0.
   */
  void resetGpsData();

  /**
   * @brief Recibe una trama GPS de tipo CGNSINF y utiliza strtok() para
   * extraer los datos y almacenarlos en Trackio::gps
   *
   * @param gps
   */
  void parseGps(char* gps);

  /**
   * @brief Limpia algunas cadenas para reducir de los datos GPS
   *
   * Basicamente se eliminan los decimales de algunos valores como altitud o
   * velocidad.
   */
  void clearGps();

  /**
   * @brief Trata de descargar un EPO file
   *
   * En recientes comprobaciones se ha visto que este método probáblemente
   * no pueda funcionar correctamente debido a que se necesita utiliza el
   * módulo FS (File System) del SIMCOM, para el cual es necesario conectar
   * una memoría MicroSD a los pines hardware dedicados a ello en el SIMCOM.
   *
   * @see
   * https://simcom.ee/documents/SIM868E/SIM868_GNSS_AGPS_Application%20Note_V1.01.pdf
   */
  void downloadEpoFile();

  /**
   * @brief SimFastSim es una tecnología de SIMCOM para habilitar el AGPS
   *
   * El proceso, similar a Trackio::downloadEpoFile(), es relativamente
   * delicado ya y que implica conetar con un servidor FTP, descargar un
   * fichero, guardarlo en file system del simcom...
   *
   * Se ha seguido la documentación del SIM868, en el pdf `GNSS AGPS
   * Application Note`
   *
   * nota: esto nunca jamás en la vida ha funcionado. probablemente porque
   *       no es posible conectar una interfaz microsd con el simcom
   *
   * @see
   * https://simcom.ee/documents/SIM868E/SIM868_GNSS_AGPS_Application%20Note_V1.01.pdf
   */
  void enableSimFasFix();

  // #########################################################################

  bool transmitLogIfFull();
  bool transmitLog();
  int getLogNextIndex();
  int countLog();
  void showLog();

  // #########################################################################

  void setupSensors();
  void getTemperature();
  void getHumidity();
  void getPressure();
  void getRaindrops();
  void getVoc();
  void getCO();
  void getCO2();
  void getDustSensor();
  void getSonarDist();
  void getSSI12(char* data);

  // #########################################################################

  void readSDIBuffer();
  void sdiReadBuffer();
  bool readSDISensor(const char* sensorCmd);
  void readSDIProbeInfo();
  void addSDIToMessage();
  void getSDIID();
  void setupSDI();

  // #########################################################################

  /**
   * @brief Abre puerto TCP
   *
   * @return true Si el puerto se abre correctamente
   * @return false
   */
  bool openTcp();

  /**
   * @brief Envía comando al simcom para determinar si la conexión TCP está
   * abierta
   *
   * Se utiliza el comando `AT+CIPSTATUS`. Simcom dispone de otro modo para
   * verificar si el TCP está abierto: el pin DCD. A día de hoy este pin no
   * está ruteado a nivel de Hardware por lo que no es accesible (sim868 v1.1).
   *
   * @return true Si la conexión está abierta
   * @return false
   */
  bool tcpIsOpen();

  /**
   * @brief Cerrar conexión TCP
   *
   * Se utiliza el comando AT+CIPCLOSE que admite dos valores, 0=slow-close
   * (RECOMENDADO) y 1=quick-close. Según la documentación:
   *
   * User can use the command AT+CIPCLOSE=<mode> to close the TCP or UDP
   * connection. If <mode> is 0, it is slow closing, if <mode> is 1, it is
   * quick closing. In slow closing, the module will interactive with the
   * server when it closes the TCP connection. Thus, the time of returning
   * “CLOSE OK” will be a bit long. This method is suitable for steady network.
   * In quick closing, the module will disconnect the connection compulsorily
   * and return “CLOSE OK” immediately, without interaction with the server.
   *
   */
  void closeTcp(int mode);

  /**
   * @brief Registro en el servidor
   *
   * Básicamente se escribe el imei, el servidor lo recibe y lo valida. Si
   * es OK mantiene el TCP abierto, si el servidor desconoce el IMEI cerrará
   * la conexión TCP y no permitirá envío de datos.
   *
   * Tras un reset del micro el primer registro también envía otros datos
   * como el ICCID, version del firmware... etc.
   *
   * @return true
   * @return false
   */
  bool sayHello();

  // #########################################################################

  /**
   * @brief Lee el buffer del UART debug (SerialMon) y parsea los datos
   * recibidos con Trackio::processBuffer().
   *
   * Se ejecuta desde op_auto() en main.cpp. Requiere RH_readDebugRX=true
   * en static-conf.h
   *
   * Habilitando este método permitimos que Trackio pueda ser configurado
   * desde el puerto serial de debug. Para ello se podrán enviar através de
   * este puerto mensajes como se hace desde el servidor remoto o bluetooth.
   */
  void readDebugBuffer();

  /**
   * @brief Comprueba si hay algún dato en el buffer UART-RX del modem
   *
   * En caso afirmativo lo almacenará en la variable buffer y ejecutará
   * Trackio::processBuffer() para evaluar el contenido.
   */
  void readModemBuffer();

  /**
   * @brief Procesa cualquier cosa que haya en la variable buffer
   *
   * La variable buffer (definida al inicio de Trackio.cpp) contiene cualquier
   * cosa que el modem devuela por el puerto UART (RX). Desde un comando TCP,
   * hasta un mensaje recibido por bluetooth.
   *
   * processBuffer() hará un split por líneas de la variable buffer, por cada
   * línea verificará su contenido y ejecutará el comando que sea necesario.
   */
  void processBuffer();

  /**
   * @brief Vacía el buffer de Trackio.cpp
   */
  void emptyBuffer();

  // #########################################################################

  /**
   * @brief Comprueba que el formato de command sea un comando
   *
   * Se espera que empiece con $ y acabe con #. No se comprueba que el comando
   * exista y sea realmente válido. Utilizamos este método en
   * Trackio::processBuffer() para comprobar si el buffer contiene un comando
   * operacional.
   */
  bool isCommand(char* command);

  /**
   * @brief Extrae un comando en partes que se añaden a Trackio::command
   */
  void parseCommand(char* cmd);

  /**
   * @brief Ejecuta un comando recibido por TCP o cualquier otro canal
   *
   * Un comando puede tener multilpes parámetros que a su vez están compuestos
   * por propiedades y valores. Por ejemplo podemos recibir el comando:
   *
   * 3:0;1:1;4:gwtcp2.m2mss.io
   *
   * Primero se separará en parámetros:
   * 3:0
   * 1:1
   * 4:gwtcp2.m2mss.io
   *
   * Luego cada parámetro se separa en propiedad:valor y se envía a
   * Trackio::applyConf(propiedad, valor) -> Trackio::applyConf("3", "1")
   *
   * La cantidad de parámetros que puede contener un comando y el tamaño
   * de cada uno se define en static-conf.h con los macros
   * MAX_PARAMS_PER_CMD y CMD_PARAM_SIZE
   *
   * Se valida que la propiedad sea un número. Se utiliza strtok y
   * un par de bucles, por lo que es un método con carga de trabajo pesada.
   *
   * Las propiedaes son arrays de tipo char[3] y los valores char[30]
   * @return true
   * @return false
   */
  bool processCommand();

  /**
   * @brief Recibe un parámetro de configuración y ejecuta el método "cmd_X"
   * que corresponda (según el valor de la propiedad)
   *
   * prop ha sido previamente validado con charIsValidNumber() en el método
   * Trackio::processCommand()
   *
   * Si prop no fuera un valor númerico se podría producir un segmentation
   * fault en tiempo de ejecución.
   *
   * @param prop Un número, se convertirá a INT y se buscará como indice
   *               del array availableCommands() (definido al inicio de
   * Trackio.cpp)
   * @return true
   * @return false
   */
  bool applyConf(char* prop, char* value);

  // #########################################################################

  /**
   * @brief Creación de un mensaje.
   *
   * Los parámetros que se añaden a cada mensaje dependen de la configuración.
   * Cuando se flashea el dispositivo se aplica la configuración de
   * static-conf.h pero esta puede ser modifica en tiempo de ejecución vía
   * comandos TCP o Bluetooth
   */
  void createMessage();

  /**
   * @brief Añade datos al mensaje
   *
   * Se tiene encuenta que no se supera el tamaño máximo de mensaje según
   * MAX_MESSAGE_SIZE definido en static-conf.h
   *
   * @param part
   */
  void addToMessage(char* part);

  /**
   * @brief Añade al mensaje datos obtenidos por red GSM: calidad señal GSM,
   * hora y localicazación por CELL ID
   */
  void addGSMToMessage();

  /**
   * @brief Añade datos GPS al mensaje
   */
  void addGPSToMessage();

  /**
   * @brief Añade datos de lectura CAN al mensaje
   */
  void addCANToMessage();

  /**
   * @brief Añade datos de batería al mensaje
   */
  void addBattToMessage();

  /**
   * @brief Añade información de sensores al mensaje
   */
  void addSensingToMessage();

  /**
   * @brief Añade la hora al mensaje
   */
  void addTimeToMessage();

  /**
   * @brief Guarda el mensaje que haya en Trackio::message
   * @see Trackio::saveMessage(char * message)
   */
  void saveMessage();

  /**
   * @brief Guarda un mensaje en el array cfg.log (memoría RAM).
   *
   * En función de cfg.persistLog también se guardará en memoría EEPROM.
   */
  void saveMessage(char* message);

  // #########################################################################

  /**
   * @brief Envía un SMS al número que haya en cfg.SMSNumber
   *
   * @param message
   * @return true
   * @return false
   */
  bool sendSMS(char* message);

  // #########################################################################

  /**
   * @brief Envía un mensaje al servidor, puede ser cualquier cosa, desde un
   * comando, a una trama GPS o la lactura de un sensor.
   *
   * Se da por válida la transmisión si el SIMCOM devuelve el texto "SEND OK"
   * (lo encontramos en la respuesta del comando AT que envía el texto)
   *
   * Con processResponse ejecutamos el método Trackio::processBuffer(). Aunque
   * processResponse sea false siempre se comprobará que los datos se han
   * transmitido
   *
   * @param msg Texto que se enviará al servidor
   * @param processResponse Si debe procesarse la respuesta
   * @return true  Si la transmisión se realiza correctamente
   * @return false
   */
  bool transmit(char* msg, bool processResponse);

  /**
   * @brief Trackio::transmit() con procesResponse == true
   *
   * @param msg
   * @return true
   * @return false
   */
  bool transmit(char* msg);

  /**
   * @brief Cajón de los desastres donde ejecutar acciones tras la transmisión
   * de datos.
   *
   * @param msg
   */
  void afterTransmit(char* msg, bool processResponse);

  /**
   * @brief Abre el TCP, uso conjunto con Trackio::transmit()
   *
   * El método Trackio::transmit() da por hecho que una conexión TCP está
   * activa, esto se hizo así cuando trabajamos exclusivamente con TCP abierto
   * (modo bidireccional de comunicación).
   *
   * Si queremos hacer una transmission pero no conocemos el estado del TCP
   * debemos usar primero este método. No solo comprobará el estado del TCP,
   * si no tambien del GPRS y estado de la red en general.
   *
   * Si despues de todo consigue abrir el TCP devolverá TRUE, sino FALSE
   *
   * @return true Si consigue abrir el TCP
   * @return false En caso de error
   */
  bool prepareForTransmission();

  // #########################################################################

  /**
   * @brief Wrapper para el método delay() que incluye reset del watchdog
   *
   * Máximo 7.5 segundos (recomendable menos). Se realiza un reset antes y
   * después de ejecutar el delay. Si se requiere un delay más largo, p.e 1
   * minuto, se debe ejecutar el método tantas veces como sea necesario con
   * mediciones inferiores a 8000:
   *
   * ```c
   * // delay de 20 segundos
   * Trackio::delay(5000);
   * Trackio::delay(5000);
   * Trackio::delay(5000);
   * Trackio::delay(5000);
   * ```
   *
   * Un delay superior a 8 segundos hará que el watchdog salte y se reinicie
   * el dispositivo.
   *
   * @param time Tiempo en milisegundos. Máximo 8 segundos (8000) pero se
   * recomienda un valor inferior, p.e. 7000
   */
  void _delay(int time);

  // #########################################################################

  /**
   * @brief Lee la configuración almacenada en EEPROM y la aplica a la
   * configuración de la aplicación (Trackio::Conf)
   */
  void loadConf();

  /**
   * @brief Guarda la configuración de Trackio::Conf en memoria EEPROM
   */
  static void saveConf();

  // #########################################################################

  /**
   * @brief Enable or disable the led
   *
   * @param status
   */
  void ledStatus(uint8_t status);

  /**
   * @brief Realiza un blink de N ms durante N times
   *
   * Se ejecuta enun loop secundario en el main.cpp, método led()
   *
   * @param times Número de veces
   * @param ms  Duración entre blink en milis
   */
  void blink(uint8_t times, int ms);

  /**
   * @brief Realiza un 3 blinks con una duración de 200ms cada uno.
   */
  void blink();

  /**
   * @brief Realiza un pitido (beep) de N ms durante N times
   *
   * @param times Number of beeps
   * @param ms time between beeps in miliseconds
   */
  void buzz(uint8_t times, int ms);

  /**
   * @brief Buzzes 3 times.
   */
  void buzz();

  /**
   * @brief Forzar reset del micro con un loop infinito que hará saltar
   * el watchdog
   */
  void hardReset();

  // #########################################################################

  /**
   * @brief Habilita el reloj del simcom para poder obtener la hora mediante
   * comando `AT+CCLK?`
   *
   * Por defecto está deshabilitado. Se comprobará primero si ya se encuentra
   * habilitado ya y que se utiliza el comando `&W` del modem para guardar los
   * parámetros de configuración
   *
   * @param times
   */
  void enableGSMClock();

  /**
   * @brief Lee la hora del modem y la guarda en trackio.gsmTime
   *
   * @return true Si es posible obtener la hora
   * @return false Cualquier otro caso
   */
  bool getGSMTime();

  /**
   * @brief Parsea resultado devuelto por getGSMTime() y lo añade al struct
   * Trackio::Time
   */
  void parseGSMTime(char* dateTime);

  /**
   * @brief Lee la localización GPS desde la red GSM
   *
   * Se utiliza cuando no es posible hacer FIX con el modem GPS.
   *
   * La precisión es variable y se indica en el campo `precission`. Se utiliza
   * el comando `AT+CLBS=1,1` y depende de que el "bearer context" se haya
   * activado en el método `Trackio::openGprs()`.
   *
   * Se obtiene longitude, latitude (en este orden) y precissión:
   * ```
   * +CLBS: 0,121.359505,31.217055,637
   * ```
   *
   * Se parsea la cadena con strtok y se guarda en Trackio::GSMLocation.
   * Se espera que el primer parámetro sea `0`, como en el ejemplo anterior,
   * si no habrá ocurrido un fallo.
   *
   * @see Trackio::getGps()
   * @see https://simcom.ee/documents/SIM800x/SIM800 Series_GSM
   * Location_Application Note_V1.01.pdf
   *
   * @return true Si consigue obtener la localización
   * @return false
   */
  bool getGsmLocation();

  /**
   * @brief Añade un erorr a Trackio::cellIdFails y comprueba si se ha
   * sobrepadado Trackio::cellIdFailsLimit
   *
   * En caso de que Trackio::cellIdFailsLimit se ha ya sobrepasado se
   * cambiará el OP_MODE a OP_STARTUP
   *
   * @see Trackio::cellIdFails
   */
  void addCellIdError();

  /**
   * @brief Resetea Trackio::cellIdFails al obtener posición válida de CellID
   */
  void resetCellIdError();

  // #########################################################################

  void setTheTime();
  long getTimeNow();
  void setupRTC();

  // #########################################################################

  /**
   * @brief Envía el micro a dormir en ciclos de 8 segundos
   *
   * Devuelve true si consigue dormir todos los ciclos (times), o false si hay
   * una interrupción y se despierta antes de tiempo.
   *
   * Actualmente la única interrupción posible es el botón NMI. Este pin (NMI)
   * no es una interrupción de hardware tal cual, lo unico que hacemos es
   * comprobar su estado HIGH/LOW cuando salimos de un ciclo, antes de entrar
   * en el siguiente. Por lo tanto es necesario mantener el botón NMI al menos
   * 8 segundos para salir del ciclo.
   *
   * @param times Número de ciclos que estará durmiendo
   * @return true Si duerme todos los ciclos
   * @return false Si se despierta sin dormir todos los ciclos (interrupción)
   */
  bool sleepNow(uint8_t times);

  /**
   * @brief Cierra puerto TCP y ejecuta Trackio::powerOff()
   */
  void enterDeepsleep();

  /**
   * @brief Ejecuta el método powerOn()
   *
   * Si falla pasará a ejecutar toda la rutina OP_STARTUP defininda en main.cpp
   * Si powerOn() se ejecuta OK pasará a comprobar el modem y habilitará el GPS
   * y GPRS
   *
   */
  void exitDeepsleep();

  // #########################################################################

  /**
   * @brief
   *
   */
  void setupAccel();
  void readAccel();
  void checkAccel();

  /**
   * @brief Los datos en bruto del acelerómetro se convierten a fuerza G
   *
   *
   */
  void scaleAccel();

  // #########################################################################

  void initCan();

  // #########################################################################

  void enableMovementAlert();
  void disableMovementAlert();

  // #########################################################################

  /**
   * @brief Habilita bluetooth (power on) y hace visible el dispositivo
   *
   * Se utiliza AT+BTPOWER=1. Se mostrará el nombre del dispositivo en consola
   *
   * @return true Si consigue habilitarlo
   * @return false si se produce algún error
   */
  bool enableBl();

  /**
   * @brief Deshabilita bluetooth (power off)
   *
   * Se utiliza AT+BTPOWER=0
   *
   * @return true Si consigue deshabilitarlo
   * @return false si se produce algún error
   */
  bool disableBl();

  /**
   * @brief Set the Bluetooth Visibility
   *
   * @param isVisible Si se hará visible o no
   * @return true Si se ejecuta correctamente, bien sea mostrar/ocultar,
   *              en función de isVisible
   * @return false Si se produce algún error
   */
  bool setBlVisibility(bool isVisible);

  /**
   * @brief Se Bluetooth device name
   *
   * El nombre del dispositivo lo devuelve el servidor al realizar el greetings
   * en el método Trackio::sayHello()
   *
   * @return true
   * @return false
   */
  bool setBlName(char* blName);

  /**
   * @brief Acepta una petición de pairing
   *
   * Por defecto siemrpe se aceptan todas las peticiones, se valdia la
   * autorización cuando se procede a conectar
   *
   * @return true
   * @return false
   */
  bool acceptBlPair();

  /**
   * @brief Se recibe un texto por bluetooth para ser procesado
   *
   * Se recibe un dato con este formato: "+BTSPPDATA: 1,4,TEXTO RECIBIDO"
   * Extraemos los datos en sí "TEXTO RECIBIDO", los copiamos al buffer
   * y ejecutamos Trackio::pricessBuffer()
   */
  void processBlData(char* data);

  // #########################################################################

  /**
   * Envía un comando por serial al modem. Se espera a recibir respuesta
   * y almacena el resultado en `buffer` (definida en Trackio.cpp).
   *
   * Utiliza el método `println` de la clase serial, no existe alternativa
   * para el método `print`, pero se puede acceder directamente al objeto
   * HardwareSerial con `SerialSim` en el main.cpp.
   *
   * @param cmd Comando a ejecutar, se utiliza println
   * @return true Si obtuvo respuesta
   * @return false
   */
  bool sendCommand(char* cmd, unsigned int timeout = 50000);

  /**
   * Version de sendCommand en la que se comprueba que la respuesta recibida
   * del comando AT contenga la cadena pasada por segundo parámetro. Es útil
   * para comprobar si tenemos OK o ERROR.
   *
   * Devuelve true si hay respuesta y contiene el segundo parametro, false
   * si no hay respuesta o no contiene el segundo parámetro
   *
   * @param cmd Comando a ejecutar, se utiliza println
   * @param returnLine Número de linea que se desea almacenar en buffer
   * @param validate Valor que se espera que contenga la línea devuelta
   * @param timeout Tiempo que se esperar antes de comprobar si el modem ha
   *                devuelto algo (no confundir con el tiempo que se esperará
   *                a que el modem devuelva algo)
   * @return true Si obtuvo respuesta
   * @return false
   */

  // #########################################################################
  bool sendAt(char* cmd);
  bool sendAt(char* cmd, int returnline);
  bool sendAt(char* cmd, char* validate);
  bool sendAt(char* cmd, int returnLine, char* validate);
  bool sendAt(char* cmd, int returnLine, int timeout);
  bool sendAt(char* cmd, int returnLine, char* validate, int timeout);

  /**
   * @brief Envía una cadena de texto al UART SerialExt
   *
   * Se utiliza este método para implmentar un tuneling entre un tercer
   * dispositivo conectado a Trackio (por UART) y el servidor Web. De este modo
   * el servidor puede enviar comandos directamente a este tercer dispositivo.
   *
   * @param string
   * @return true
   * @return false
   */
  static bool sendSerialExt(char* string);
};

#endif
