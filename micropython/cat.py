import wave
from time import sleep
try: 
    from machine import Timer
except ImportError:
    class Timer:
        PERIODIC=0
try:
    from .limb import LimbMotionPlan, ticks_ms
except ValueError: #"cannot perform relative import" raises a value error
    from limb import LimbMotionPlan, ticks_ms

from collections import deque

class Cat:
    def __init__(self, limbs, freq=25, timer=None, mpu=None, dac=None, touch_pin=None, v_bat=None, dist_sensor=None):
        self.limbs=limbs        
        #get the names to define a order (based on servo_nr)
        self._limb_names=sorted([(l.get_servo_nr(), l.name) for l in self.limbs.values()])
        self.freq=25
        self.mpu=mpu # this is not used so far
        self.dac=dac
        if dac is not None:
            with wave.open('meow.wav') as meow:
                volume=.1
                meow._convert=lambda data : wave.volume(data, volume, append=[0]*2) #performance is low ->precompute
                self.meow_data=meow.readframes(meow.getnframes())
                self.meow_framerate=meow.getframerate()

        self.touch_pin=touch_pin
        self.dist_sensor=dist_sensor
        self.v_bat=v_bat
        self.autonomous=False
        self.distances=[None, None, None]
        try:
            self.motion_queue=deque(maxlen=20)
        except TypeError:
            self.motion_queue=deque((),20) # strange micropython syntax, no kw allowed
        self.timer= timer
        self.next_motion=None
        self.stand()
        if timer is not None:
            period=1000//freq
            try:
                timer.init(period=period, mode=Timer.PERIODIC, callback=self._update)
            except OSError:
                timer.deinit()
                sleep(1)
                timer.init(period=period,mode=Timer.PERIODIC, callback=self._update)

    def __del__(self):
        if self.timer is not None:
            self.timer.deinit()

    def meow(self, volume=.1):
        print('meow')
        if self.dac is None:
            print('no dac specified')
        else:
            with wave.open('meow.wav') as meow:
                #meow._convert=lambda data : wave.volume(data, volume) #performance is low
                self.dac.write_timed(meow.readframes(meow.getnframes()), meow.getframerate(), wait=True)
                #dac.wavplay
            self.dac.write(0) # otherwise there is noise
            self.dac.stopwave() #performance?
            #self.dac.write_timed(self.meow_data,self.meow_framerate)

    def keep_level(self, t=.3, p=1/5000, x=0,y=0, height=6):
        position={'roll':0,'pitch':0, 'height':height}
        while True:
            sense=self.mpu.get_values()
            print("X="+str((sense['AcX']-x)*p)+"\nY="+str((sense['AcY']-y)/5000))
            position['pitch']+=(sense['AcX']-x)/5000
            position['roll']-=(sense['AcY']-y)/5000
            print(position)
            if abs(position['pitch']) + abs(position['roll'])>5:
                self.stand()
                break
            self.stand(**position, t=t)
            while self.is_active():
                pass
    
    def get_vbat(self):
        if self.v_bat:
            return self.v_bat.read()/4095*3.9*2 # more or less


    def set_frequency(self, freq):
        self.freq=freq
        if self.timer is not None:
            period=1000//freq
            self.timer.deinit()
            try:
                self.timer.init(period=period, mode=Timer.PERIODIC, callback=self._update)
            except OSError:
                self.timer.deinit()
                sleep(1)
                self.timer.init(period=period,mode=Timer.PERIODIC, callback=self._update)

    @property
    def leg_names(self,which='all'):
        if which=='all':
            which=''
        return [n for _,n in self._limb_names if 'leg' in n and which in n]

    def _update(self,timer=None ):
        active=self.active_limbs() 
        if self.motion_queue and self.next_motion is None:
            self.next_motion=self.motion_queue.popleft()
        if self.next_motion is not None:
            if not any (limb in active for limb in self.next_motion.wait_for): # apply next motion
                if self.autonomous:
                    current_dir=self.get_head_direction()   
                    n_dir=sum([d is not None for d in self.distances])
                    if n_dir<3:
                        if self.distances[current_dir+1] is not None:
                            self.next_motion.iterations = None
                        self.distances[current_dir+1]=self.dist()
                    elif any([d>300 for d in self.distances]): #ensure we are not walking backwards
                        self.distances[current_dir+1]=self.dist()                        
                        if self.distances[current_dir+1]<700: #obstical in 70cm -> check directions (if not walking backwards)
                            #self.next_motion=None
                            self.next_motion.iterations=1
                            self.distances=[self.distances[i] if i ==current_dir+1 else None for i in range(3)]
                    print(self.distances)
                
                for limb,motion in self.next_motion.limb_motions.items():
                    self.limbs[limb].set_motion(motion)
                if self.next_motion.iterations is not None:
                    self.next_motion.iterations-=1
                if self.next_motion.iterations is None or self.next_motion.iterations<1:
                    self.next_motion=None
        elif self.autonomous: #next_motion is None
            self.distances[self.get_head_direction()+1]=self.dist()
            if any ([d is None for d in self.distances]):
                print('check distances')
                self.check_direction()
            elif all([d<300 for d in self.distances]):
                print('backwards')
                self.walk(step_size=-4, ground_pos=2, n_steps=4)
            else:
                max_dist=self.distances.index(max(self.distances))-1
                if max_dist==0:
                    self.set_head([0,0])
                else:
                    self.set_head([-20, 50*max_dist])
                print(['left', 'straight', 'right'][max_dist+1])
                self.walk(direction=max_dist*3, step_size=5, n_steps=20, speed=8)
                    
        
        
        for limb in self.limbs.values():
            limb.update_position()
        
        if self.touch_pin and self.touch_pin.read()<500:
            #toggle autonomous mode
            self.meow()
            
            if self.autonomous:
                self.autonomous=False
                self.next_motion=None
                self.sleep()
            else:
                self.autonomous=True
                self.distances=[None, None, None]
                self.next_motion=None
                self.stand()


    def get_head_direction(self):
        th=self.limbs['head'].get_theta()[1]
        if th<0:
            return -1
        elif th>0:
            return 1
        return 0
        
        

    def check_direction(self):
        if self.distances[0] is None:
            self.set_head([-20,-50])
        if self.distances[2] is None:
            self.set_head([-20,50])
        self.set_head([0,0])
        #self.set_head([0,0], t=0) #just to have one more motion in the queue

    def get_limb_thetas(self,ref_time=None):
        
        new_thetas=dict()
        #compute leg positions (it might take time due to kinematics)
        for limb in self.limbs.values():            
            new_thetas[limb.name]=limb.get_theta(ref_time)
        return(new_thetas)


    def move_limbs(self, thetas):
        #print('move to {}'.format(thetas))
        for limbname, th in thetas.items():
            self.limbs[limbname].move_to(theta=th)

    def is_active(self):        
        return any([l.active for l in self.limbs.values()])
    
    def active_limbs(self):
        return [n for n,l in self.limbs.items() if l.active ]

    
    def leg_position(self, **kwargs):
        #make all arguments to lists of same length
        arg_names=['height','ground_pos', 'spread', 'pitch','roll']
        if any([a not in arg_names for a in kwargs]):
            raise TypeError('Unexpected keyword argument')
        for k in arg_names[1:]:
            kwargs.setdefault(k,0)
        kwargs.setdefault('height',7)
        for n,v in kwargs.items():
            if not isinstance(v,list):
                kwargs[n]=[v]
        maxlen=max([len(v) for v in kwargs.values()])    
        for n,v in kwargs.items():
            if len(v)<maxlen:
                kwargs[n]=v+[v[-1]]*(maxlen-len(v))#repeat the last value

        leg_names=self.leg_names
        pos={l:[] for l in leg_names}
        for height,ground_pos, spread, pitch,roll in zip(*[kwargs[n] for n in arg_names]):
            for l in leg_names:
                pos_i=[ground_pos,height]      
                if spread != 0:                
                    pos_i[0]+= spread/2 if 'front' in l else -spread/2
                if roll != 0:                
                    pos_i[1]+= roll/2 if 'left' in l else -roll/2
                if pitch != 0:                
                    pos_i[1]+= pitch/2 if 'front' in l else -pitch/2
                pos[l].append(pos_i)
        #print(pos)
        return(pos)
        
    def stand(self,t=1, iterations=0, **kwargs):  
        leg_position=self.leg_position(**kwargs)
        
        if not isinstance(t,list):
            t=[t]
        n_pos=len(next(iter(leg_position.values())))
        if n_pos==1:
            iterations=0
            leg_position={k:[[None]]+v for k,v in leg_position.items()}
        if iterations ==0:
            n_pos-=1
        if len(t)<n_pos:
            t=t+[t[-1]]*(n_pos-len(t))
        t=[t_s *1000 for t_s in t]
        
        #now=ticks_ms()  
        legs=list(leg_position)
        self.motion_queue.append(MotionPlan(
                limbs=[self.limbs[l] for l in legs],
                steps=[leg_position[l] for l in legs],
                steps_duration=t,
                iterations=iterations,
                position_mode=True))

    def walk(self, n_steps=10, gait=0,lift=1, step_size=6, direction=0, t=1, speed=6,**kwargs):  
        #two opposit legs are on the ground
        #speed in cm/sec
        kwargs.setdefault('ground_pos',0)
        leg_position=self.leg_position(**kwargs) #center position
        reverse= (step_size<0 or speed <0)
        step_size=abs(step_size)
        speed=abs(speed)        
        if gait ==0:
            t_ground=step_size/speed/2*1000 
            t_air=step_size/(2*lift+step_size)
            steps_duration=[t_ground*f for f in [1, (1-t_air)/2 ,t_air, (1-t_air)/2]]
            phase=[0,.5,.5,0] 
        elif gait==1:
            t_ground=step_size/speed*1000/3
            t_air=step_size/(2*lift+step_size)
            steps_duration=[t_ground*f for f in [3, (1-t_air)/2 ,t_air, (1-t_air)/2]]
            phase=[p/4 for p in [0,2,1,3]]
        step_size={l:step_size for l in leg_position}
        for l in step_size:
            if direction > 0 and 'left' in l:
                step_size[l]+=direction #shorten left steps
            elif direction < 0 and 'right' in l:
                step_size[l]-=direction #shorten right steps
            if reverse:
                step_size[l]= -step_size[l]
        steps=    [[[ pos[0][0]+hss, pos[0][1] ], 
                    [ pos[0][0]-hss, pos[0][1] ],
                    [ pos[0][0]-hss, pos[0][1]-lift ],
                    [ pos[0][0]+hss, pos[0][1]-lift ]] for pos, hss in ((leg_position[l],step_size[l]/2) for l in self.leg_names)]
        self.motion_queue.append(MotionPlan( 
            limbs=[self.limbs[l] for l in self.leg_names], 
            steps=steps,
            phase=phase,
            steps_duration=steps_duration,
            position_mode=True,
            iterations=n_steps))
        
    def dist(self):
        dist=self.dist_sensor.distance_mm()
        if dist == -1:
            dist=None
        return dist


    def set_head(self, pos=[0, 0], t=1):
        self.motion_queue.append(MotionPlan(
                    limbs=[self.limbs['head']],
                    steps=[[None], pos],
                    steps_duration=[t*1000], iterations=0))

    def set_tail(self, pos=0, t=1):
        self.motion_queue.append(MotionPlan(
                    limbs=[self.limbs['tail']],
                    steps=[[None],[pos]],
                    steps_duration=[t*1000], iterations=0))

    def wag(self, n=3, amplitude=30, freq=1):
        self.motion_queue.append(MotionPlan(
                    limbs=[self.limbs['tail']],
                    steps=[[amplitude],[-amplitude]],
                    steps_duration=[1/freq/2*1000]*2 ,
                    phase=.25, #start in the middle
                    iterations=n))

    def eat(self,t=1):
        self.stand(height=6, pitch=-2, ground_pos=-1)
        self.set_head([-60,0])
        self.wag(8,freq=2)

    def sit(self,t=1):
        self.stand(height=6, pitch=4, ground_pos=-1)
        self.set_head([-20,0])
        self.set_tail(-60)
        
    def sleep(self):
        self.stand(height=2.5,spread=4.5)
        self.set_head([-80,-40])

    def yes(self, n=3, amplitude=30, freq=1):
        self.motion_queue.append(MotionPlan(
                    limbs=[self.limbs['head']], 
                    steps=[[amplitude,0],[-amplitude,0]],
                    steps_duration=[1/freq/2*1000]*2 ,
                    phase=.25, #start in the middle
                    iterations=n))

    def no(self,n=3, amplitude=30, freq=1):
        self.motion_queue.append(MotionPlan(
                    limbs=[self.limbs['head']], 
                    steps=[[0,0],[-amplitude/4,-amplitude],[0,0],[-amplitude/4,amplitude]],
                    steps_duration=[1/freq/4*1000]*4 ,
                    iterations=n))                   


    def sit2(self):
        self.set_tail() 
        self.set_head()
        self.stand(1,ground_pos=1, height=7 ,pitch=-2, spread=-3)    
        self.stand(2,ground_pos=1, height=7 ,pitch=-2, spread=-3) #wait
        self.stand(.1,ground_pos=2.5, height=7.7 ,pitch=2.5, spread=-4) 
        self.set_tail(-80,t=.1)                                                                                        
        self.set_head([-50,0], t=.1) 
    
    def pushups(self, iterations=5, t=1):
        self.stand(pitch=[-3,3], height=[5,7],iterations=iterations, t=t)



class MotionPlan:
    def __init__(self, limbs, steps, steps_duration, phase=0,iterations=1,  position_mode=False,wait_for=None, callback=None):
        self.callback=callback
        if wait_for is None:
            wait_for=[l.name for l in limbs]
        self.wait_for=wait_for
        self.iterations=iterations
        if not isinstance(phase, list):
            phase=[phase for _ in limbs]
        if get_levels(steps_duration)==1:
            steps_duration=[steps_duration for _ in limbs]
        if get_levels(steps)==2:
            steps=[steps for _ in limbs]
        #print('new motion plan {} {} {}'.format(limbs, steps, steps_duration))
        self.limb_motions={l.name:LimbMotionPlan(l,steps=s, steps_duration=d, phase=p, position_mode=position_mode) 
                for l,s,d,p in zip(limbs, steps, steps_duration, phase)}


        
def get_levels(val):
    levels=0
    while True:
        if isinstance(val, list):
            levels+=1
            if val:
                val=val[0]
            else:
                return levels
        else:
            return levels

