from cat import Cat  
from machine import Timer, I2C, Pin, ADC, DAC
import limb
from time import sleep
import network
from mpu6050 import MPU6050
from hcsr04 import HCSR04
import wave

sensor=HCSR04(14,12)

dac=DAC(25)
meow = wave.open('meow.wav')
dac.write_timed(meow.readframes(meow.getnframes()), meow.getframerate())
timer= Timer(0)
try:
    i2c=I2C(-1,Pin(22), Pin(21), freq=100000)
except TypeError:
    i2c=I2C(scl=Pin(22), sda=Pin(21), freq=100000) #old version of micropython
limbs=limb.get_cat_limbs(i2c )
cat=Cat(limbs, timer=timer,mpu=MPU6050(i2c)) 

cat.mpu.get_values()

cat.stand(height=7,t=2)
cat.wag(6, freq=2)
meow = wave.open('meow.wav')
dac.write_timed(meow.readframes(meow.getnframes()), meow.getframerate())

cat.yes(3)

print('{}: telnet is {} on {}'.format(*network.telnet.status()))
vbat= ADC(35)
vbat.atten(ADC.ATTN_11DB)
vbat.read()/4095*3.9*2