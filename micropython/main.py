from cat import Cat  
import time


cat=Cat() 

for i in range(4):
    cat.stand(height=5,leg_idx=[i], t=1)
    time.sleep(1)

cat.stand(height=7,t=.5)
cat.wag(6, freq=2)
cat.yes(3)

