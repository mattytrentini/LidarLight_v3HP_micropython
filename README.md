# MicroPython library for the Garmin Lidar Lite v3HP

## This is a basic MicroPython library for the Garmin Lidar Lite v3HP. It is based on the official [Arduino library ](https://github.com/garmin/LIDARLite_Arduino_Library/blob/master/src/LIDARLite_v3HP.cpp) and the official [Datasheet](https://static.garmin.com/pumac/LIDAR-Lite_v3HP_Instructions_EN.pdf).


Basic usage:

```python
from machine import I2C, Pin
from lidarLitev3hp import V3HP

i2c = I2C(1, scl=Pin(7), sda=Pin(6), freq=400000)
            # mode, i2c
lidar = V3HP( 0 ,   i2c)

# Single measurement, most accurate method
distance = lidar.range_single()

# Single measurement, fast method, less accurate
distance = lidar.distance_fast()
```

## Modes

    0: Default mode, balanced performance.

    1: Short range, high speed.

    2: Default range, higher speed short range. Turns on quick termination detection for faster measurements at short range (with decreasedaccuracy)

    3: Maximum range.

    4: High sensitivity detection. Overrides default valid measurement detection algorithm, and uses a threshold value for high sensitivity and noise.

    5: Low sensitivity detection. Overrides default valid measurement detection algorithm, and uses a threshold value for low sensitivity and noise.

    6: Short range, high speed, higher error. Overrides default valid measurement

### On the pi pico, I was able to get a reliable 330 measurements per second using mode 1 with fast distance method, and 280 measurements per second using mode 1 with range_single method.