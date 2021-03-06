NodeManager is intended to take care on your behalf of all those common tasks a MySensors node has to accomplish, speeding up the development cycle of your projects.

NodeManager includes the following main components:

* Sleep manager: allows managing automatically the complexity behind battery-powered sensors spending most of their time sleeping
* Power manager: allows powering on your sensors only while the node is awake
* Battery manager: provides common functionalities to read and report the battery level
* Remote configuration: allows configuring remotely the node without the need to have physical access to it
* Built-in sensors: for the most common sensors, provide embedded code so to allow their configuration with a single line

## Features

* Manage all the aspects of a sleeping cycle by leveraging smart sleep
* Allow configuring the node and any attached sensors remotely
* Allow waking up a sleeping node remotely at the end of a sleeping cycle
* Allow powering on each connected sensor only while the node is awake to save battery
* Report battery level periodically and automatically or on demand
* Calculate battery level without requiring an additional pin and the resistors
* Report signal level periodically and automatically or on demand
* Allow rebooting the board remotely
* Provide out-of-the-box sensors personalities and automatically execute their main task at each cycle
* Allow collecting and averaging multiple samples, tracking the last value and forcing periodic updates for any sensor
* Provide buil-in capabilities to handle interrupt-based sensors 

### Built-in sensors

NodeManager provides built-in implementation of a number of sensors through dedicated classes:

Sensor's class  | Description
 ------------- | -------------
SensorAnalogInput | Generic analog sensor, return a pin's analog value or its percentage
SensorLDR | LDR sensor, return the light level of an attached light resistor in percentage
SensorRain | Rain sensor, return the percentage of rain from an attached analog sensor
SensorSoilMoisture | Soil moisture sensor, return the percentage of moisture from an attached analog sensor
SensorThermistor | Thermistor sensor, return the temperature based on the attached thermistor
SensorML8511 | ML8511 sensor, return UV intensity
SensorACS712 | ACS712 sensor, measure the current going through the attached module
SensorDigitalInput |  Generic digital sensor, return a pin's digital value
SensorDigitalOutput | Generic digital output sensor, allows setting the digital output of a pin to the requested value
SensorRelay | Relay sensor, allows activating the relay
SensorLatchingRelay| Latching Relay sensor, allows activating the relay with a pulse
SensorDHT11 | DHT11 sensor, return temperature/humidity based on the attached DHT sensor
SensorDHT22 | DHT22 sensor, return temperature/humidity based on the attached DHT sensor
SensorSHT21 | SHT21 sensor, return temperature/humidity based on the attached SHT21 sensor
SensorHTU21D | HTU21D sensor, return temperature/humidity based on the attached HTU21D sensor
SensorSwitch | Generic switch, wake up the board when a pin changes status
SensorDoor | Door sensor, wake up the board and report when an attached magnetic sensor has been opened/closed
SensorMotion | Motion sensor, wake up the board and report when an attached PIR has triggered
SensorDs18b20 | DS18B20 sensor, return the temperature based on the attached sensor
SensorBH1750 | BH1750 sensor, return light level in lux
SensorMLX90614 | MLX90614 contactless temperature sensor, return ambient and object temperature
SensorBME280 | BME280 sensor, return temperature/humidity/pressure based on the attached BME280 sensor
SensorBMP085 | BMP085/BMP180 sensor, return temperature and pressure
SensorBMP280 | BMP280 sensor, return temperature/pressure based on the attached BMP280 sensor
SensorSonoff | Sonoff wireless smart switch
SensorHCSR04 | HC-SR04 sensor, return the distance between the sensor and an object
SensorMCP9808 | MCP9808 sensor, measure the temperature through the attached module
SensorMQ | MQ sensor, return ppm of the target gas
SensorMHZ19 | MH-Z19 CO2 sensor via UART (SoftwareSerial, default on pins 6(Rx) and 7(Tx)
SensorAM2320 | AM2320 sensors, return temperature/humidity based on the attached AM2320 sensor
SensorTSL2561 | TSL2561 sensor, return light in lux
SensorPT100 | High temperature sensor associated with DFRobot Driver, return the temperature in C?? from the attached PT100 sensor
SensorDimmer | Generic dimmer sensor used to drive a pwm output
SensorRainGauge | Rain gauge sensor
SensorPowerMeter | Power meter pulse sensor
SensorWaterMeter | Water meter pulse sensor

## Installation

* Download the package or clone the git repository from https://github.com/mysensors/NodeManager
* Open the NodeManager.ino sketch and save it under a different name
* Configure you sensors and upload the sketch to your arduino board

Please note NodeManager cannot be used as an arduino library since requires access to your MySensors configuration directives, hence its files have to be placed into the same directory of your sketch.

### Installing the dependencies

Some of the sensors rely on third party libraries. Those libraries are not included within NodeManager and have to be installed from the Arduino IDE Library Manager (Sketch -> Include Library -> Manager Libraries) or manually. You need to install the library ONLY if you are planning to enable to use the sensor:

Sensor  | Required Library
 ------------- | -------------
SensorSHT21, SensorHTU21D | https://github.com/SodaqMoja/Sodaq_SHT2x
SensorDHT11, SensorDHT22 | https://github.com/mysensors/MySensorsArduinoExamples/tree/master/libraries/DHT
SensorDs18b20 | https://github.com/milesburton/Arduino-Temperature-Control-Library
SensorBH1750 | https://github.com/claws/BH1750
SensorMLX90614 | https://github.com/adafruit/Adafruit-MLX90614-Library
SensorBME280 | https://github.com/adafruit/Adafruit_BME280_Library
SensorSonoff | https://github.com/thomasfredericks/Bounce2
SensorBMP085 | https://github.com/adafruit/Adafruit-BMP085-Library
SensorHCSR04 | https://github.com/mysensors/MySensorsArduinoExamples/tree/master/libraries/NewPing
SensorMCP9808 | https://github.com/adafruit/Adafruit_MCP9808_Library
SensorAM2320 | https://github.com/thakshak/AM2320
SensorTSL2561 | https://github.com/adafruit/TSL2561-Arduino-Library
SensorBMP280 | https://github.com/adafruit/Adafruit_BMP280_Library

### Upgrade

* Download the latest version of NodeManager
* Replace the NodeManagerLibrary.ino and NodeManagerLibrary.h of your project with those just downloaded
* Review the release notes in case there is any manual change required to the main sketch

## Configuration

Configuring a sketch with is using NodeManager requires a few steps. All the configuration directives are located within the main sketch.

### MySensors configuration

Since NodeManager has to communicate with the MySensors network on your behalf, it has to know how to do it. On top of the main sketch you will find the typical MySensors directives you are used to which can be customized to configure the board to act as a MySensors node or a MySensors gateway. 
Please note you don't necessarily need a NodeManager gateway to interact with a NodeManager node. A NodeManager node is fully compatible with any existing gateway you are currently operating with.

### NodeManager configuration

The next step is to enable NodeManager's modules required for your sensors. When a module is enabled, the required library will be loaded and the corresponding sensor will be made available. To enable it, set it to 1. Enabled only what you need to ensure enough storage is left for your custom code.

### Add your sensors

Find in the main sketch `Add your sensors below` and add your sensors to NodeManager. To add a sensor, just create an instance of the class, passing it `node` as an argument and an optional pin. 

~~~c
SensorThermistor thermistor(node,A0);
SensorSHT21 sht21(node);
~~~

The sensor will be then registered automatically with NodeManager which will take care of it all along its lifecycle. Please ensure the corresponding module has been previously enabled for a successful compilation of the code.
NodeManager will assign a child id automatically, present each sensor for you to the controller, query each sensor and report the measure back to the gateway/controller. For actuators (e.g. relays) those can be triggered by sending a `REQ` message with the expected type to their assigned child id.

For your convenience, NodeManager makes available additional special sensors which can be added in the same way as other built-in sensors:

Special Sensor  | Description
 ------------- | -------------
SensorBattery | Add it to enable automatic battery reporting
SensorSignal | Add it to enable automatic signal level reporting
SensorConfiguration | Add it to enable OTA remote configuration of any registered sensor

### Configuring your sensors

NodeManager and all the sensors can be configured from within `before()` in the main sketch. Find `Configure your sensors below` to customize the behavior of any sensor by calling one of the functions available.

~~~c
// report measures of every attached sensors every 10 minutes
node.setReportIntervalMinutes(10);
// set the node to sleep in 5 minutes cycles
node.setSleepMinutes(5);
// report battery level every 10 minutes
battery.setReportIntervalMinutes(10);
// set an offset to -1 to a thermistor sensor
thermistor.setOffset(-1);
// Change the id of a the first child of a sht21 sensor
sht21.children.get(1)->child_id = 5;
// power all the nodes through dedicated pins
node.setPowerManager(power);
~~~

If not instructed differently, the node will stay awake and all the sensors will report every 10 minutes, battery level and signal level will be automatically reported every 60 minutes (if the corresponding sensors have been added). 

Please note, if you configure a sleep cycle, this may have an impact on the reporting interval since the sensor will be able to report its measures ONLY when awake. For example if you set a report interval of 5 minutes and a sleep cycle of 10 minutes, the sensors will report every 10 minutes.

## Running the node

Once finished configuring your node, upload your sketch to your arduino board as you are used to.

Check your gateway's logs to ensure the node is working as expected. You should see the node presenting itself, presenting all the registered sensors and reporting new measures at the configured reporting interval.
When `DEBUG` is enabled, detailed information will be available through the serial port. Remember to disable debug once the tests have been completed to save additional storage.

## Communicate with the sensors

You can interact with each registered sensor by sending to the child id a `REQ` command (or a `SET` for output sensors like relays). For example to request the temperature to node_id 254 and child_id 1:

`254;1;2;0;0;`

To activate a relay connected to the same node, child_id 100 we need to send a `SET` command with payload set to 1:

`254;100;1;0;2;1`

No need to implement anything on your side since for built-in sensors this is handled automatically. 

## API

You can interact with each class provided by NodeManager through a set of API functions. 

### NodeManager API

~~~c
    // [10] send the same message multiple times (default: 1)
    void setRetries(int value);
    int getRetries();
    // [3] set the duration (in seconds) of a sleep cycle
    void setSleepSeconds(int value);
    long getSleepSeconds();
    // [4] set the duration (in minutes) of a sleep cycle
    void setSleepMinutes(int value);
    // [5] set the duration (in hours) of a sleep cycle
    void setSleepHours(int value);
    // [29] set the duration (in days) of a sleep cycle
    void setSleepDays(int value);
    // [19] if enabled, when waking up from the interrupt, the board stops sleeping. Disable it when attaching e.g. a motion sensor (default: true)
    void setSleepInterruptPin(int value);
    // configure the interrupt pin and mode. Mode can be CHANGE, RISING, FALLING (default: MODE_NOT_DEFINED)
    void setInterrupt(int pin, int mode, int initial = -1);
    // [28] ignore two consecutive interrupts if happening within this timeframe in milliseconds (default: 100)
    void setInterruptMinDelta(long value);
    // [20] optionally sleep interval in milliseconds before sending each message to the radio network (default: 0)
    void setSleepBetweenSend(int value);
    int getSleepBetweenSend();
    // register a built-in sensor
    int registerSensor(int sensor_type, int pin = -1, int child_id = -1);
    // register a custom sensor
    int registerSensor(Sensor* sensor);
    // [26] un-register a sensor
    void unRegisterSensor(int sensor_index);
    // return a sensor by its index
    Sensor* get(int sensor_index);
    Sensor* getSensor(int sensor_index);
    // assign a different child id to a sensor
    bool renameSensor(int old_child_id, int new_child_id);
    #if POWER_MANAGER == 1
      // to save battery the sensor can be optionally connected to two pins which will act as vcc and ground and activated on demand
      void setPowerPins(int ground_pin, int vcc_pin, int wait_time = 50);
      // [23] if enabled the pins will be automatically powered on while awake and off during sleeping (default: true)
      void setAutoPowerPins(bool value);
      // [24] manually turn the power on
      void powerOn();
      // [25] manually turn the power off
      void powerOff();
    #endif
    // [21] set this to true if you want destination node to send ack back to this node (default: false)
    void setAck(bool value);
    bool getAck();
    // request and return the current timestamp from the controller
    long getTimestamp();
    // Request the controller's configuration on startup (default: true)
    void setGetControllerConfig(bool value);
    // [22] Manually set isMetric setting
    void setIsMetric(bool value);
    bool getIsMetric();
    // Convert a temperature from celsius to fahrenheit depending on how isMetric is set
    float celsiusToFahrenheit(float temperature);
    // return true if sleep or wait is configured and hence this is a sleeping node
    bool isSleepingNode();
    // [1] Send a hello message back to the controller
    void hello();
    // [6] reboot the board
    void reboot();
    // [8] send NodeManager's the version back to the controller
    void version();
    // [7] clear the EEPROM
    void clearEeprom();
    // [9] wake up the board
    void wakeup();
    // process a remote request
    void process(Request & request);
    // return the value stored at the requested index from the EEPROM
    int loadFromMemory(int index);
    // [27] save the given index of the EEPROM the provided value
    void saveToMemory(int index, int value);
    // return vcc in V
    float getVcc();
    // setup the configured interrupt pins
    void setupInterrupts();
    // return the pin from which the last interrupt came
    int getLastInterruptPin();
    // [36] set the default interval in minutes all the sensors will report their measures. If the same function is called on a specific sensor, this will not change the previously set value. or sleeping sensors, the elapsed time can be evaluated only upon wake up (default: 10 minutes)
    void setReportIntervalSeconds(int value);
    // [37] set the default interval in minutes all the sensors will report their measures. If the same function is called on a specific sensor, this will not change the previously set value. or sleeping sensors, the elapsed time can be evaluated only upon wake up (default: 10 minutes)
    void setReportIntervalMinutes(int value);
    // [38] set the default interval in minutes all the sensors will report their measures. If the same function is called on a specific sensor, this will not change the previously set value. or sleeping sensors, the elapsed time can be evaluated only upon wake up (default: 10 minutes)
    void setReportIntervalHours(int value);
    // [39] set the default interval in minutes all the sensors will report their measures. If the same function is called on a specific sensor, this will not change the previously set value. or sleeping sensors, the elapsed time can be evaluated only upon wake up (default: 10 minutes)
    void setReportIntervalDays(int value);
    // [30] if set and when the board is battery powered, sleep() is always called instead of wait() (default: true)
    void setSleepOrWait(bool value);
    // sleep if the node is a battery powered or wait if it is not for the given number of milliseconds 
    void sleepOrWait(long value);
    // [31] set which pin is connected to RST of the board to reboot the board when requested. If not set the software reboot is used instead (default: -1)
    void setRebootPin(int value);
    // [32] turn the ADC off so to save 0.2 mA
    void setADCOff();
    #if SIGNAL_SENSOR == 1 && defined(MY_SIGNAL_REPORT_ENABLED)
      // [33] How frequenly to send a signal report to the controller (default: 60 minutes)
      void setSignalReportMinutes(int value);
      // [43] How frequenly to send a signal report to the controller (default: 60 minutes)
      void setSignalReportSeconds(int value);
      // [44] How frequenly to send a signal report to the controller (default: 60 minutes)
      void setSignalReportHours(int value);
      // [45] How frequenly to send a signal report to the controller (default: 60 minutes)
      void setSignalReportDays(int value);
      // [34] define which signal report to send. Possible values are SR_UPLINK_QUALITY, SR_TX_POWER_LEVEL, SR_TX_POWER_PERCENT, SR_TX_RSSI, SR_RX_RSSI, SR_TX_SNR, SR_RX_SNR (default: SR_RX_RSSI)
      void setSignalCommand(int value);
      // [35] report the signal level to the controller
      void signalReport();
    #endif
~~~

### Set reporting intervals and sleeping cycles

If not instructed differently, the node will stay awake and all the sensors will report every 10 minutes, battery level and signal level will be automatically reported every 60 minutes. To change those settings, you can call the following functions on the nodeManager object:

Function  | Description
------------ | -------------
setSleepSeconds(), setSleepMinutes(), setSleepHours(), setSleepDays() | the time interval the node will spend in a (smart) sleep cycle
setReportIntervalSeconds(), setReportIntervalMinutes(), setReportIntervalHours(), setReportIntervalDays() | the time interval the node will report the measures of all the attached sensors
setBatteryReportSeconds(), setBatteryReportMinutes(), setBatteryReportHours(), setBatteryReportDays() | the time interval the node will report the battery level
setSignalReportSeconds(), setSignalReportMinutes(), setSignalReportHours(), setSignalReportDays() | the time interval the node will report the radio signal level

For example, to put the node to sleep in cycles of 10 minutes:

~~~c
	nodeManager.setSleepMinutes(10);
~~~

If you need every sensor to report at a different time interval, you can call `setBatteryReportSeconds(), setBatteryReportMinutes(), setBatteryReportHours(), setBatteryReportDays()` on the sensor's object. For example to have a DHT sensor reporting every 60 seconds while all the other sensors every 20 minutes:
~~~c
int id = nodeManager.registerSensor(SENSOR_DHT22,6);
SensorDHT* dht = (SensorDHT*)nodeManager.get(id);
dht->setReportIntervalSeconds(60);
nodeManager.setReportIntervalMinutes(20);
~~~

Please note, if you configure a sleep cycle, this may have an impact on the reporting interval since the sensor will be able to report its measures ONLY when awake. For example if you set a report interval of 5 minutes and a sleep cycle of 10 minutes, the sensors will report every 10 minutes.

### Register your sensors
Once configured the node, it is time to tell NodeManager which sensors are attached to the board and where. In your sketch, inside the `before()` function and just before calling `nodeManager.before()`, you can register your sensors against NodeManager. The following built-in sensor types are available. Remember the corresponding module should be enabled in `config.h` for a successful compilation: 

Sensor type  | Description
 ------------- | -------------
SENSOR_ANALOG_INPUT | Generic analog sensor, return a pin's analog value or its percentage
SENSOR_LDR | LDR sensor, return the light level of an attached light resistor in percentage
SENSOR_THERMISTOR | Thermistor sensor, return the temperature based on the attached thermistor
SENSOR_DIGITAL_INPUT |  Generic digital sensor, return a pin's digital value
SENSOR_DIGITAL_OUTPUT | Generic digital output sensor, allows setting the digital output of a pin to the requested value
SENSOR_RELAY | Relay sensor, allows activating the relay
SENSOR_LATCHING_RELAY| Latching Relay sensor, allows activating the relay with a pulse
SENSOR_DHT11 | DHT11 sensor, return temperature/humidity based on the attached DHT sensor
SENSOR_DHT22 | DHT22 sensor, return temperature/humidity based on the attached DHT sensor
SENSOR_SHT21 | SHT21 sensor, return temperature/humidity based on the attached SHT21 sensor
SENSOR_SWITCH | Generic switch, wake up the board when a pin changes status
SENSOR_DOOR | Door sensor, wake up the board and report when an attached magnetic sensor has been opened/closed
SENSOR_MOTION | Motion sensor, wake up the board and report when an attached PIR has triggered
SENSOR_DS18B20 | DS18B20 sensor, return the temperature based on the attached sensor
SENSOR_HTU21D | HTU21D sensor, return temperature/humidity based on the attached HTU21D sensor
SENSOR_BH1750 | BH1750 sensor, return light level in lux
SENSOR_MLX90614 | MLX90614 contactless temperature sensor, return ambient and object temperature
SENSOR_BME280 | BME280 sensor, return temperature/humidity/pressure based on the attached BME280 sensor
SENSOR_MQ | MQ sensor, return ppm of the target gas
SENSOR_ML8511 | ML8511 sensor, return UV intensity
SENSOR_SONOFF | Sonoff wireless smart switch
SENSOR_BMP085 | BMP085/BMP180 sensor, return temperature and pressure
SENSOR_HCSR04 | HC-SR04 sensor, return the distance between the sensor and an object
SENSOR_ACS712 | ACS712 sensor, measure the current going through the attached module
SENSOR_MCP9808 | MCP9808 sensor, measure the temperature through the attached module
SENSOR_RAIN_GAUGE | Rain gauge sensor
SENSOR_RAIN | Rain sensor, return the percentage of rain from an attached analog sensor
SENSOR_SOIL_MOISTURE | Soil moisture sensor, return the percentage of moisture from an attached analog sensor
SENSOR_MHZ19 | MH-Z19 CO2 sensor via UART (SoftwareSerial, default on pins 6(Rx) and 7(Tx)
SENSOR_TSL2561 | TSL2561 sensor, return light in lux
SENSOR_AM2320 | AM2320 sensors, return temperature/humidity based on the attached AM2320 sensor
SENSOR_PT100 | High temperature sensor associated with DFRobot Driver, return the temperature in C?? from the attached PT100 sensor
SENSOR_BMP280 | BMP280 sensor, return temperature/pressure based on the attached BMP280 sensor
SENSOR_DIMMER | Generic dimmer sensor used to drive a pwm output
SENSOR_POWER_METER | Power meter pulse sensor
SENSOR_WATER_METER | Water meter pulse sensor

To register a sensor simply call the NodeManager instance with the sensory type and the pin the sensor is conncted to and optionally a child id. For example:
~~~c
	nodeManager.registerSensor(SENSOR_THERMISTOR,A2);
	nodeManager.registerSensor(SENSOR_DOOR,3,1);
~~~

Once registered, your job is done. NodeManager will assign a child id automatically if not instructed differently, present each sensor for you to the controller, query each sensor and report the measure back to the gateway/controller. For actuators (e.g. relays) those can be triggered by sending a `REQ` message with the expected type to their assigned child id.

When called, registerSensor returns the child_id of the sensor so you will be able to retrieve it later if needed. Please note for sensors creating multiple child IDs (like a DHT sensor which creates a temperature and humidity sensor with different IDs), the last id is returned.

#### Creating a custom sensor

If you want to create a custom sensor and register it with NodeManager so it can take care of all the common tasks, you can create an inline class inheriting from `Sensor` or other subclasses and implement the following methods:
~~~c
    // define what to do during before() to setup the sensor
    void onBefore();
	// define what to do during setup() by executing the sensor's main task
    void onSetup();
    // define what to do during loop() by executing the sensor's main task
    void onLoop();
    // define what to do during receive() when the sensor receives a message
    void onReceive(const MyMessage & message);
	// define what to do when receiving a remote configuration message
    void onProcess(Request & request);
    // define what to do when receiving an interrupt
    void onInterrupt();
~~~

You can then instantiate your newly created class and register it with NodeManager:
~~~c
    nodeManager.registerSensor(new SensorCustom(&nodeManager,child_id, pin));
~~~

### Configuring the sensors
Each built-in sensor class comes with reasonable default settings. In case you want/need to customize any of those settings, after having registered the sensor, you can retrieve it back and call set functions common to all the sensors or specific for a given class.

To do so, use `nodeManager.getSensor(child_id)` which will return a pointer to the sensor. Remeber to cast it to the right class before calling their functions. For example:

~~~c
	SensorLatchingRelay* relay = (SensorLatchingRelay*) nodeManager.getSensor(2);
	relay->setPulseWidth(50);
~~~


#### Sensor's general configuration

The following methods are available for all the sensors and can be called on the object reference as per the example above:
~~~c
    // [1] where the sensor is attached to (default: not set)
    void setPin(int value);
    int getPin();
    // [2] child_id of this sensor (default: not set)
    void setChildId(int value);
    int getChildId();
    // presentation of this sensor (default: S_CUSTOM)
    void setPresentation(int value);
    int getPresentation();
    // [3] type of this sensor (default: V_CUSTOM)
    void setType(int value);
    int getType();
    // [4] description of the sensor (default: '')
    void setDescription(char *value);
    // [5] For some sensors, the measurement can be queried multiple times and an average is returned (default: 1)
    void setSamples(int value);
    // [6] If more then one sample has to be taken, set the interval in milliseconds between measurements (default: 0)
    void setSamplesInterval(int value);
    // [7] if true will report the measure only if different than the previous one (default: false)
    void setTrackLastValue(bool value);
    // [9] if track last value is enabled, force to send an update after the configured number of minutes
    void setForceUpdateMinutes(int value);
    // [19] if track last value is enabled, force to send an update after the configured number of hours
    void setForceUpdateHours(int value);
    // [10] the value type of this sensor (default: TYPE_INTEGER)
    void setValueType(int value);
    int getValueType();
    // [11] for float values, set the float precision (default: 2)
    void  setFloatPrecision(int value);
    // [21] for double values, set the double precision (default: 4)
    void  setDoublePrecision(int value);
    #if POWER_MANAGER == 1
      // to save battery the sensor can be optionally connected to two pins which will act as vcc and ground and activated on demand
      void setPowerPins(int ground_pin, int vcc_pin, int wait_time = 50);
      // [12] if enabled the pins will be automatically powered on while awake and off during sleeping (default: true)
      void setAutoPowerPins(bool value);
      // [13] manually turn the power on
      void powerOn();
      // [14] manually turn the power off
      void powerOff();
    #endif
    // get the latest recorded value from the sensor
    int getValueInt();
    float getValueFloat();
    char* getValueString();
    // [17] After how many minutes the sensor will report back its measure (default: 10 minutes)
    void setReportIntervalSeconds(int value);
    // [16] After how many minutes the sensor will report back its measure (default: 10 minutes)
    void setReportIntervalMinutes(int value);
    // [19] After how many hours the sensor will report back its measure (default: 10 minutes)
    void setReportIntervalHours(int value);
    // [20] After how many days the sensor will report back its measure (default: 10 minutes)
    void setReportIntervalDays(int value);
    // return true if the report interval has been already configured
    bool isReportIntervalConfigured();
    // process a remote request
    void process(Request & request);
    // return the pin the interrupt is attached to
    int getInterruptPin();
    // listen for interrupts on the given pin so interrupt() will be called when occurring
    void setInterrupt(int pin, int mode, int initial);
~~~

#### Sensor's specific configuration

Each sensor class can expose additional methods.

* SensorAnalogInput / SensorLDR / SensorRain / SensorSoilMoisture
~~~c
    // [101] the analog reference to use (default: not set, can be either INTERNAL or DEFAULT)
    void setReference(int value);
    // [102] reverse the value or the percentage (e.g. 70% -> 30%) (default: false)
    void setReverse(bool value);
    // [103] when true returns the value as a percentage (default: true)
    void setOutputPercentage(bool value);
    // [104] minimum value for calculating the percentage (default: 0)
    void setRangeMin(int value);
    // [105] maximum value for calculating the percentage (default: 1024)
    void setRangeMax(int value);
~~~

* SensorThermistor
~~~c
    // [101] resistance at 25 degrees C (default: 10000)
    void setNominalResistor(long value);
    // [102] temperature for nominal resistance (default: 25)
    void setNominalTemperature(int value);
    // [103] The beta coefficient of the thermistor (default: 3950)
    void setBCoefficient(int value);
    // [104] the value of the resistor in series with the thermistor (default: 10000)
    void setSeriesResistor(long value);
    // [105] set a temperature offset
    void setOffset(float value);
~~~

* SensorMQ
~~~c
    // [101] define the target gas whose ppm has to be returned. 0: LPG, 1: CO, 2: Smoke (default: 1);
    void setTargetGas(int value);
    // [102] define the load resistance on the board, in kilo ohms (default: 1);
    void setRlValue(float value);
    // [103] define the Ro resistance on the board (default: 10000);
    void setRoValue(float value);
    // [104] Sensor resistance in clean air (default: 9.83);
    void setCleanAirFactor(float value);
    // [105] define how many samples you are going to take in the calibration phase (default: 50);
    void setCalibrationSampleTimes(int value);
    // [106] define the time interal(in milisecond) between each samples in the cablibration phase (default: 500);
    void setCalibrationSampleInterval(int value);
    // [107] define how many samples you are going to take in normal operation (default: 50);
    void setReadSampleTimes(int value);
    // [108] define the time interal(in milisecond) between each samples in the normal operations (default: 5);
    void setReadSampleInterval(int value);
    // set the LPGCurve array (default: {2.3,0.21,-0.47})
    void setLPGCurve(float *value);
    // set the COCurve array (default: {2.3,0.72,-0.34})
    void setCOCurve(float *value);
    // set the SmokeCurve array (default: {2.3,0.53,-0.44})
    void setSmokeCurve(float *value);
~~~

* SensorACS712
~~~c
    // [101] set how many mV are equivalent to 1 Amp. The value depends on the module (100 for 20A Module, 66 for 30A Module) (default: 185);
    void setmVPerAmp(int value);
    // [102] set ACS offset (default: 2500);
    void setOffset(int value);
~~~

* SensorRainGauge / SensorPowerMeter / SensorWaterMeter
~~~c
    // [102] set how many pulses for each unit (e.g. 1000 pulses for 1 kwh of power, 9 pulses for 1 mm of rain, etc.)
    void setPulseFactor(float value);
    // set initial value - internal pull up (default: HIGH)
    void setInitialValue(int value);
    // set the interrupt mode to attach to (default: FALLING)
    void setInterruptMode(int value);
~~~

* SensorDigitalOutput / SensorRelay
~~~c
    // [103] define which value to set to the output when set to on (default: HIGH)
    void setOnValue(int value);
    // [104] when legacy mode is enabled expect a REQ message to trigger, otherwise the default SET (default: false)
    void setLegacyMode(bool value);
    // [105] automatically turn the output off after the given number of minutes
    void setSafeguard(int value);
    // [106] if true the input value becomes a duration in minutes after which the output will be automatically turned off (default: false)
    void setInputIsElapsed(bool value);
    // [107] optionally wait for the given number of milliseconds after changing the status (default: 0)
    void setWaitAfterSet(int value);
    // manually switch the output to the provided value
    void setStatus(int value);
    // get the current state
    int getStatus();
~~~

* SensorLatchingRelay (in addition to those available for SensorDigitalOutput / SensorRelay)
~~~c
    // [201] set the duration of the pulse to send in ms to activate the relay (default: 50)
    void setPulseWidth(int value);
    // [202] set the pin which turns the relay off (default: the pin provided while registering the sensor)
    void setPinOff(int value);
    // [203] set the pin which turns the relay on (default: the pin provided while registering the sensor + 1)
    void setPinOn(int value);
~~~

*  SensorSwitch / SensorDoor / SensorMotion
~~~c
    // [101] set the interrupt mode. Can be CHANGE, RISING, FALLING (default: CHANGE)
    void setMode(int value);
    // [102] milliseconds to wait before reading the input (default: 0)
    void setDebounce(int value);
    // [103] time to wait in milliseconds after a change is detected to allow the signal to be restored to its normal value (default: 0)
    void setTriggerTime(int value);
    // [104] Set initial value on the interrupt pin (default: HIGH)
    void setInitial(int value);
~~~

*  SensorDs18b20
~~~c
    // returns the sensor's resolution in bits
    int getResolution();
    // [101] set the sensor's resolution in bits
    void setResolution(int value);
    // [102] sleep while DS18B20 calculates temperature (default: false)
    void setSleepDuringConversion(bool value);
    // return the sensors' device address
    DeviceAddress* getDeviceAddress();
~~~

*  SensorBH1750
~~~c
    // [101] set sensor reading mode, e.g. BH1750_ONE_TIME_HIGH_RES_MODE
    void setMode(uint8_t mode);
~~~

*  SensorBME280 / SensorBMP085 / SensorBMP280
~~~c
    // [101] define how many pressure samples to keep track of for calculating the forecast (default: 5)
    void setForecastSamplesCount(int value);
~~~

* SensorHCSR04
~~~c
    // [101] Arduino pin tied to trigger pin on the ultrasonic sensor (default: the pin set while registering the sensor)
    void setTriggerPin(int value);
    // [102] Arduino pin tied to echo pin on the ultrasonic sensor (default: the pin set while registering the sensor)
    void setEchoPin(int value);
    // [103] Maximum distance we want to ping for (in centimeters) (default: 300)
    void setMaxDistance(int value);
~~~

*  SensorSonoff
~~~c
    // [101] set the button's pin (default: 0)
    void setButtonPin(int value);
    // [102] set the relay's pin (default: 12)
    void setRelayPin(int value);
    // [103] set the led's pin (default: 13)
    void setLedPin(int value);
~~~

* SensorMHZ19
~~~c
    // set the RX and TX pins for the software serial port to talk to the sensor
    void setRxTx(int rxpin, int txpin);
~~~

* SensorTSL2561
~~~c
    // [101] set the gain, possible values are SensorTSL2561::GAIN_0X (0), SensorTSL2561::GAIN_16X (1) (default 16x)
    void setGain(int value);
    // [102] set the timing, possible values are SensorTSL2561::INTEGRATIONTIME_13MS (0), SensorTSL2561::INTEGRATIONTIME_101MS (1), SensorTSL2561::INTEGRATIONTIME_402MS (2) (default: 13ms)
    void setTiming(int value);
    // [103] set the spectrum, possible values are SensorTSL2561::VISIBLE (0), SensorTSL2561::FULLSPECTRUM (1), SensorTSL2561::INFRARED (2), SensorTSL2561::FULL (3) (default: visible)
    void setSpectrum(int value);
    // [104] set the i2c address values are SensorTSL2561::ADDR_FLOAT, SensorTSL2561::ADDR_LOW, SensorTSL2561::ADDR_HIGH
    void setAddress(int value);
~~~

* SensorDimmer
~~~c
    // [101] set the effect to use for a smooth transition, can be one of SensorDimmer::EASE_LINEAR, SensorDimmer::EASE_INSINE, SensorDimmer::EASE_OUTSINE, SensorDimmer::EASE_INOUTSINE (default: EASE_LINEAR)
    void setEasing(int value);
    // [102] the duration of entire the transition in seconds (default: 1)
    void setDuration(int value);
    // [103] the duration of a single step of the transition in milliseconds (default: 100)
    void setStepDuration(int value);
    // fade the output from the current value to the target provided in the range 0-100
    void fadeTo(int value);
~~~

### Creating a gateway

NodeManager can be also used to create a MySensors gateway. Open your config.h file and look for the gateway-specific defines under "MySensors gateway configuration". The most common settings are reported there, just uncomment those you need to use based on the network you are creating.

Please note you don't necessarily need a NodeManager gateway to interact with a NodeManager node. The NodeManager node is fully compatible with any existing gateway you are currently operating with.

### Upload your sketch

Upload your sketch to your arduino board as you are used to.

Check your gateway's logs to ensure the node is working as expected. You should see the node presenting itself, reporting battery level, presenting all the registered sensors and the configuration child id service.
When `DEBUG` is enabled, detailed information is available through the serial port. Remember to disable debug once the tests have been completed.

### Communicate with NodeManager and its sensors

You can interact with each registered sensor by sending to the child id a `REQ` command (or a `SET` for output sensors like relays). For example to request the temperature to node_id 254 and child_id 1:

`254;1;2;0;0;`

To activate a relay connected to the same node, child_id 100 we need to send a `SET` command with payload set to 1:

`254;100;1;0;2;1`

No need to implement anything on your side since for built-in sensors this is handled automatically. 

NodeManager exposes also a configuration service which is by default on child_id 200 so you can interact with it by sending `V_CUSTOM` type of messages and commands within the payload. For each `REQ` message, the node will respond with a `SET` message if successful. 

Almost all the functions made available through the API can be called remotely. To do so, the payload must be in the format `<function_id>[,<value_to_set>]` where `function_id` is the number between square brackets you can find in the description above and, if the function takes and argument, this can be passed along in `value_to_set`. 
For example, to request a battery report, find the function you need to call remotely within the documentation:
~~~c
    // [2] Send a battery level report to the controller
    void batteryReport();
~~~
In this case `function_id` will be 2. To request a battery report to the node_id 100, send the following message:
`<node_id>;<configuration_child_id>;<req>;0;<V_CUSTOM>;<function_id>`
`100;200;2;0;48;2`

The change the sleep time to e.g. 10 minutes:
~~~c
    // [4] set the duration (in minutes) of a sleep cycle
    void setSleepMinutes(int value);
~~~
`<node_id>;<configuration_child_id>;<req>;0;<V_CUSTOM>;<function_id>,<value>`
`100;200;2;0;48;4,10`

To wake up a node previously configured as sleeping, send the following as the node wakes up next:
~~~c
    // [9] wake up the board
    void wakeup();
~~~
`100;200;2;0;48;9`

The same protocol can be used to execute remotely also sensor-specific functions. In this case the message has to be sent to the sensor's child_id, with a `V_CUSTOM` type of message. For example if you want to collect and average 10 samples for child_id 1:
~~~c
    // [5] For some sensors, the measurement can be queried multiple times and an average is returned (default: 1)
    void setSamples(int value);
~~~
`100;1;2;0;48;5,10`

If you want to decrease the temperature offset of a thermistor sensor to -2:
~~~c
    // [105] set a temperature offset
    void setOffset(float value);
~~~
`100;1;2;0;48;105,-2`

Please note that anything set remotely will NOT persist a reboot apart from the sleep interval which is saved to the EEPROM (provided `PERSIST` is enabled).

## Understanding NodeManager: how it works

A NodeManager object is created for you at the beginning of your sketch and its main functions must be called from within `before()`, `presentation()`, `loop()` and `receive()` to work properly. NodeManager will do the following during each phase:

NodeManager::before():
* Setup the interrupt pins to wake up the board based on the configured interrupts
* If persistance is enabled, restore from the EEPROM the latest sleeping settings
* Call `before()` of each registered sensor

Sensor::before():
* Call sensor-specific implementation of before by invoking `onBefore()` to initialize the sensor

NodeManager::setup():
* Send a custom message with a STARTED payload to the controller
* Call `setup()` of each registered sensor

Sensor::setup():
* Call sensor-specific implementation of setup by invoking `onSetup()` to initialize the sensor

NodeManager::loop():
* If all the sensors are powered by an arduino pin, this is turned on
* Call `loop()` of each registered sensor
* If all the sensors are powered by an arduino pin, this is turned off

Sensor::loop():
* If the sensor is powered by an arduino pin, this is set to on
* For each registered sensor, the sensor-specific `onLoop()` is called. If multiple samples are requested, this is run multiple times. `onLoop()` is not intended to send out any message but just sets a new value to the requested child
* A message is sent to the gateway with the value. Depending on the configuration, this is not sent if it is the same as the previous value or sent anyway after a given number of cycles. These functionalies are not sensor-specific and common to all the sensors inheriting from the `Sensor` class.
* If the sensor is powered by an arduino pin, this is turned off

NodeManager::receive():
* Receive a message from the radio network 
* Dispatch the message to the recipient sensor

Sensor::receive(): 
* Invoke `Sensor::loop()` which will execute the sensor main taks and eventually call `Sensor::onReceive()`

Sensor::interrupt():
* Calls the sensor's implementation of `onInterrupt()` to handle the interrupt

### Custom sensors

If you want to create a new sensor, you can create a new class inheriting from Sensor or other subclasses. The constructor is supposed to assign to assign the sensor a name through the `_name` variable. The following methods have to be implemented:
~~~c
    // define what to do during before(). Usually creates all the Child(ren) which belong to the sensor
    void onBefore();
	// define what to do during setup(). Usually initialize the required libraries
    void onSetup();
    // define what to do during loop() by executing the sensor's main task. Usually does a calculation and store the value to send back to the given Child class.
    void onLoop(Child* child);
    // define what to do during receive() when the sensor receives a message
    void onReceive(MyMessage* message);
    // define what to do when receiving an interrupt
    void onInterrupt();
~~~

If the sensor implements a remote API, this has to be made available in SensorConfiguration::onReceive.

## Examples

Examples and sample code is provided from within the main sketch.

## Contributing

Contributes to NodeManager are of course more than welcome. 

### Reporting an issue or request an enhancement

For reporting an issue, requesting support for a new sensor or any other kind of enhancement, please drop a message either on the project's main page (<https://www.mysensors.org/download/node-manager>), on the MySensors Forum (<https://forum.mysensors.org/category/43/nodemanager>) or open an issue directly on Github (<https://github.com/mysensors/NodeManager/issues>).


### Contributing to the code

If you want to contribute to the code, a pull request on Github is the way to go. First of all setup your development environment:

* Create a copy of the project in your Github account by clicking on the "Fork" button on `https://github.com/mysensors/NodeManager` 
* Check the copy actually exists on `https://github.com/<username>/NodeManager`
* Clone your repository on your computer: `git clone https://github.com/<username>/NodeManager.git`
* Configure the main project's repository as an upstream: `git remote add upstream https://github.com/mysensors/NodeManager.git`
* Create and switch to a local development branch: `git checkout -b development origin/development`

Before applying any change, ensure you have the latest development version available:
* Switch to your local development branch: `git checkout development`
* Fetch the latest version from the main project's repository: `git fetch upstream`
* Merge into your development copy all the changes from the main repository: `git merge development upstream/development`
* Update the development branch of your repository: `git push origin development`

Create a branch for the fix/feature you want to work on and apply changes to the code:
* Create and switch to a new branch (give it a significant name, e.g. fix/enum-sensors): `git checkout -b <yourbranch>`
* Do any required change to the code
* Include all the files changed for your commit: `git add .`
* Ensure both the main sketch and the config.h file do not present any change
* Commit the changes: `git  commit -m"Use enum instead of define for defining each sensor #121"`
* Push the branch with the changes to your repository: `git push origin <yourbranch>`
* Visit `https://github.com/<username>/NodeManager/branches` and click the "New pull request" button just aside your newly created branch
* Fill in the request with a significant title and description and select the "development" branch from the main repository to be compared against your branch. Ensure there is one or more issues the pull request will fix and make it explicit within the description
* Submit the request and start the discussion
* Any additional commits to your branch which will be presented within the same pull request
* When the pull request is merged, delete your working branch: `git branch -D <yourbranch>`
* Update your local and remote development branch as per the instructions above

If there are changes introduced to the development branch that conflicts with an open pull request, you will have to resolve the conflicts and update the PR:
* Fetch and merge into development any change from upstream/development as detailed above
* Switch to your branch: `git checkout <yourbranch>`
* Rebase the branch you filed the PR from against your updated development branch: `git rebase development`
* Resolve the conflicts and commit again
* Force push your updated branch so the PR gets updated: `git push HEAD:<yourbranch> -f`

## Release Notes

v1.0:

* Initial release

v1.1:
* Added ability to sleep between send() so to save additional battery
* Bug fixes

v1.2:

* Added out-of-the-box support for BH1750 light sensor
* Added out-of-the-box support for HTU21D temperature and humidity sensor
* Added out-of-the-box support for MLX90614 contactless temperature sensor
* Added a few examples to the documentation
* Fixed a few bugs

v1.3:

* Added support for BME280 temperature/humudity/pressure sensor
* Added option to measure battery level via a pin in addition to internal Vcc
* Added example sketches to the documentation
* Fixed a few bugs

v1.4:

* Added support for ML8511 UV intensity sensor
* Added support for MQ air quality sensor
* Added ability to manually assign a child id to a sensor
* Ensured compatibility for non-sleeping nodes
* Ability to control if waking up from an interrupt counts for a battery level report
* When power pins are set the sensor is powered on just after
* Service messages are disabled by default
* Bug fixes

v1.5:

* Added support for ACS712 current sensor
* Added support for HC-SR04 distance sensor
* Added support for BMP085/BMP180 temperature and pressure sensor
* Added support for Sonoff smart switch
* Added support for Rain Gauge sensor
* Added support for MCP9808 temperature sensor
* Added forecast output to all Bosch sensors
* Added I2C address auto-discovery for all Bosch sensors
* Added support for running as a gateway
* Added option to retrieve the latest value of a sensor from outside NodeManager
* Remote reboot now does not need a reboot pin configured
* A heartbeat is now sent also when waking up from a wait cycle
* When waking up for an interrupt, only the code of the sensor expecting that interrupt is executed
* Added capability to retrieve the time from the controller
* Optimized battery life for DS18B20 sensors
* SLEEP_MANAGER has been deprecated (now always enabled) and setMode() replaces setSleepMode()
* New mode ALWAYS_ON to let the node staying awake and executing each sensors' loop
* ESP8266WiFi.h has to be included in the main sketch if MY_GATEWAY_ESP8266 is defined
* Added receiveTime() wrapper in the main sketch
* Fixed the logic for output sensors
* Added common gateway settings in config.h

v1.6:
* Introduced new remote API to allow calling almost ALL NodeManager's and its sensors' functions remotely
* Reporting interval configuration is now indipendent from the sleep cycle
* Reporting interval can be customized per-sensor
* All intervals (measure/battery reports) are now time-based
* Added support for BMP280 temperature and pressure sensor
* Added support for RS485 serial transport 
* Added support for TSL2561 light sensor
* Added support for DHT21 temperature/humidity sensor
* Added support for AM2320 temperature/humidity sensor
* Added support for PT100 high temperature sensor
* Added support for MH-Z19 CO2 sensor
* Added support for analog rain and soil moisture sensors
* Added support for generic dimmer sensor (PWM output)
* Added support for power and water meter pulse sensors
* Radio signal level (RSSI) is now reported automatically like the battery level
* SensorRainGauge now supports sleep mode
* SensorSwitch now supports awake mode
* SensorLatchingRealy now handles automatically both on and off commands
* SensorMQ now depends on its own module
* Added safeguard (automatic off) to SensorDigitalOutput
* Any sensor can now access all NodeManager's functions
* DHT sensor now using MySensors' DHT library
