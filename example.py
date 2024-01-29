from machine import Pin, I2C
from lidarLitev3hp import V3HP
import time

i2c = I2C(1, scl=Pin(7), sda=Pin(6), freq=400000)

lidar = V3HP(1, i2c)

start = time.ticks_ms()

# range_single is reliable at around 280 measurements per second on pi pico
for i in range(280):
    dist = lidar.range()
    #print(dist)

print("Single Time: ", time.ticks_ms() - start)

start = time.ticks_ms()

# distance_fast is reliable around 330 measurements per second on pi pico
for i in range(330):
    dist = lidar.range_fast()
    #print(dist)

print("Fast Time: ", time.ticks_ms() - start)

