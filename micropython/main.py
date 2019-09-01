from cat import Cat  
from machine import Timer, I2C, Pin, ADC, DAC, TouchPad
import limb
from time import sleep
import network
from mpu6050 import MPU6050
from hcsr04 import HCSR04
import wave

print('{}: telnet is {} on {}'.format(*network.telnet.status()))

sensor=HCSR04(14,12)

dac=DAC(26)
dac.write(0)
#dac=None

t=TouchPad(13)
t.config(500)

feedback= ADC(32)
feedback.atten(ADC.ATTN_11DB)
feedback.read()/4095

vbat= ADC(35)
vbat.atten(ADC.ATTN_11DB)
print("battery is at {} Volts".format(vbat.read()/4095*3.9*2))

timer= Timer(0)

try:
    i2c=I2C(-1,Pin(22), Pin(21), freq=100000)
except TypeError:
    i2c=I2C(scl=Pin(22), sda=Pin(21), freq=100000) #old version of micropython
limbs=limb.get_cat_limbs(i2c )
cat=Cat(limbs, timer=timer,mpu=MPU6050(i2c), dac=dac, touch_pin=t, v_bat=vbat, dist_sensor=sensor) 
cat.meow()
cat.mpu.get_values()

cat.stand(height=7,t=2)
cat.wag(6, freq=2)
cat.yes(3)
cat.meow()
for _ in range(10):
    sleep(1)
    if t.read()<500:
        cat.sleep()
        break
else:
    cat.autonomous()