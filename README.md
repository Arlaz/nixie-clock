# A cool nixie clock project I work on with many features!

Here is a non exhaustive list of planned features :
- esp32 and HV5530 communication (using spi library)
- Get date using NTP protocol
- Control date/hour format using HomeKit + check temperature
- temperature sensors (DS18B20 + SHT85 + BME680)
- Relative Humidity sensors (SHT85 + BME680)
- Pressure sensors (BME680)
- microSD Card logging
- Use of DS3231SN specific RTC

## Environmental Data
[ESP-IDF Components library](https://github.com/UncleRus/esp-idf-lib) is used for the 3 sensors and the RTC clock

## The Board
The schematics will be published
The board is designed to drive 8 IN-18 tubes to display up to 1/100s or several data (temperature : humidity : pressure)
