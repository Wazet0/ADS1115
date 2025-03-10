import ads1115
from machine import I2C

i2c = I2C(0, freq=1_000_000)
ads = ads1115.ADS1115(i2c, pga=ads1115.PGA_6_144)

#Continuous mode
print("Continuous mode:")
ads.start_conversion()
for i in range(10):
    print(ads.read_v())
ads.end_conversion()

#Single shot mode
print("Single shot mode:")
ads.one_conversion()
print(ads.read_v())