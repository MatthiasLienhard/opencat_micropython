from cat import Cat  
from machine import Timer, I2C, Pin
import limb
from time import sleep
import network

timer= Timer(0)
try:
    i2c=I2C(-1,Pin(22), Pin(21), freq=100000)
except TypeError:
    i2c=I2C(scl=Pin(22), sda=Pin(21), freq=100000) #old version of micropython
limbs=limb.get_cat_limbs(i2c )
cat=Cat(limbs, timer=timer) 


cat.stand(height=7,t=.5)
cat.wag(6, freq=2)
cat.yes(3)

print('{}: telnet is {} on {}'.format(*network.telnet.status()))
