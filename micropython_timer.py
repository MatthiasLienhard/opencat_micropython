#!/usr/bin/python3
import threading
from datetime import timedelta

'''
see:
https://docs.python.org/3/library/threading.html#timer-objects
'''

#simple python3 implementation of the micropython timer    
class Timer(threading.Thread):
    PERIODIC=0
    ONE_SHOT=1
    #CHRONO,EXTBASE
    def __init__(self):
        threading.Thread.__init__(self)
        self.daemon = True
        self._stopped = threading.Event()
        self._interval=None
        self._callback=None    
        
    def _run_periodically(self):
        while not self._stopped.wait(self._interval.total_seconds()):
            self._callback(self._stopped)

    def _run_once(self):
        self._stopped.wait(self._interval.total_seconds())
        self._callback(self._stopped)

    def init(self, period=1000, mode=PERIODIC, callback=None):
        if callback is not None:
            self._callback=callback
        elif self._callback is None:
            raise NotImplementedError('callback not initialized')
        self._interval=timedelta(milliseconds=period)
        
        if mode is self.PERIODIC:
            self._target=self._run_periodically
        elif mode is self.ONE_SHOT:
            self._target=self._run_once
        self.start()

    def deinit(self):
        self._stopped.set()
        self.join()
    
    def pause(self):
        raise NotImplementedError('pause not implemented')

    def resume(self):
        raise NotImplementedError('resume not implemented')

if __name__=='__main__':
    import time
    #from micropython_timer import Timer
    t=Timer()
    def cb(event):
        print('hi')

    t.init(period=2000, callback=cb)

    for s in 'do other things at the same time'.split():
        print(s)
        time.sleep(2)
    t.deinit()


    from micropython.cat import Cat
    import simulation
    
    cat=Cat(simulation.get_simulation_limbs(), timer=Timer())




