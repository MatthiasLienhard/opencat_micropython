from cat import Cat  
from machine import Timer, I2C, Pin
import limb
from time import sleep

timer= Timer(0)
i2c=I2C(-1,Pin(22), Pin(21), freq=100000)
limbs=limb.get_cat_limbs(i2c )
cat=Cat(limbs, timer=timer) 


for i in range(4):
    cat.stand(height=5,leg_idx=[i], t=1)
    sleep(1)

cat.stand(height=7,t=.5)
cat.wag(6, freq=2)
cat.yes(3)

