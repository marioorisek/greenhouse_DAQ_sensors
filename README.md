# greenhouse DAQ unit
DAQ unit for greenhouse.

DAQ unit for greenhouse weather station. Following data to be gathered:
Temperature inside
Humidity inside
Temperature outside
Atmospheric pressure
Light instensity 
Battery voltage
Battery current
Battery power

Measured data are transmitted via SigFox modem to ThingSpeak cloud for storage and analysis.
Measured data are also sent to second Arduino (greenhouse_DAQ_display) via I2C bus to be accessible locally.

## Arduino Pro Mini 5 V 16 MHz
Tweaks:
- removed PWR LED
- removed voltage regulator

## Sensors:
### BME 280
- temperature inside
- humidity inside
- atmospheric pressure

### DS 1820
- temperature outside

### BH 1750
- light intensity

### INA 3221
- battery voltage
- battery current
