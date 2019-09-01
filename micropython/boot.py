# This file is executed on every boot (including wake-boot from deepsleep)
#import esp
#esp.osdebug(None)
import network
import machine
import time

sta = network.WLAN(network.STA_IF)
sta.active(True)
wlan=[]
timeout=5000
with open( 'credentials.txt','r') as f:
    # this file should contain ssid<space>pw, one per line
    line=f.readline().strip().split()
    while line:
        wlan.append(line)
        line=f.readline().strip().split()

#if machine.wake_reason()[0] not in [4,6]:#Soft reset
    # configuration below MUST match your home router settings!!
#    print('configure wlan')
sta.ifconfig(('192.168.178.64', '255.255.255.0', '192.168.178.1', '8.8.8.8'))
    


if not sta.isconnected():
    # try all wlans from the file
    # todo: scan first and check which can be seen
    for pw in wlan:    
        print('connecting to {}'.format(pw[0]))
        start = time.ticks_ms()
        while not sta.isconnected():
            sta.connect(*pw)
            
            for i in range(5):
                print('attempt {}, ip is {}'.format(i, sta.ifconfig()[0]))
                if sta.isconnected():
                    break
                time.sleep(1)

        else: break

if not sta.isconnected():
    #todo: start station mode
    pass


print('starting telnet...')
network.telnet.start()

