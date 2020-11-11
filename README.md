# Radio module for leak monitoring

The module is a board based on STM32F030F4P6 microcontroller and NRF24L01 radio module.
The module is designed to detect water leaks and notify the head unit about them.
This version of the module is powered by the CR2032 battery which should be enough for 6 months of operation.

The default operation logic of the module is as follows:
1. Deep sleep for one minute;
2. humidity sensor interrogation;
3. In case the threshold value is exceeded, send a message via NRF24L01 to the head unit which controls the devices;

## Sensor general circuit
The radio sensor design itself is on EasyEDA https://easyeda.com/leech001/leakcontrol.

Scheme

![shema](https://raw.githubusercontent.com/leech001/LeakControl/master/RF_module/img/sheme.png)

Board layout

![pcb](https://raw.githubusercontent.com/leech001/LeakControl/master/RF_module/img/pcb.png)


Gerber file in gerber directory.

## Software description

A project source code based on the HAL library is available.
The project itself is generated in STM32CubeMX.

Setting up the pipe identifier
```
const uint64_t pipe1 = 0xF0F0F0F0A1LL;
```
Sleep Flag Reset
```
__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
```
Adjusting the sensor threshold
```
if (adc > 200) {
```
Initialization of the radio module, speed setting, installing the channel and opening the pipe to the transmission.
```
NRF_Init();
setDataRate(RF24_250KBPS);
setChannel(76);
openWritingPipe(pipe1);
```

For those who are not strong, there is a ready-made binary that just needs to be write in microcontroler. Binary directory ``bin``.

Translated with https://www.deepl.com/ru/translator (free version)