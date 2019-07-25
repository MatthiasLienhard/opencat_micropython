
from time import sleep
try: 
    from machine import Timer
except ModuleNotFoundError:
    class Timer:
        PERIODIC=0
try:
    from .limb import LimbMotionPlan, ticks_ms
except ModuleNotFoundError:
    from limb import LimbMotionPlan, ticks_ms

from collections import deque

class Cat:
    def __init__(self, limbs, freq=25, timer=None):
        self.limbs=limbs        
        #get the names to define a order (based on servo_nr)
        self._limb_names=sorted([(l.get_servo_nr(), l.name) for l in self.limbs.values()])
        self.freq=25
        self.motion_queue=deque(maxlen=20)
        self.timer= timer
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
        if self.motion_queue:
            next_motion=self.motion_queue[0]
            if not any (limb in active for limb in next_motion.wait_for):
                if next_motion.iterations is None:
                    _=self.motion_queue.popleft()
                else:
                    next_motion.iterations-=1
                    if next_motion.iterations<1:
                        _=self.motion_queue.popleft()
                for limb,motion in next_motion.limb_motions.items():
                    self.limbs[limb].set_motion(motion)

        for limb in self.limbs.values():
            limb.update_position()
        
        

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
        
    def stand(self,t=1, iterations=None, **kwargs):  
        leg_position=self.leg_position(**kwargs)
        if not isinstance(t,list):
            t=[t]
        n_pos=len(next(iter(leg_position.values())))
        if n_pos==1:
            iterations=None
        if iterations is None:
            n_pos-=1
        if len(t)<n_pos:
            t=t+[t[-1]]*(n_pos-len(t))
        t=[t_s *1000 for t_s in t]
        
        now=ticks_ms()  
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
        kwargs.setdefault('ground_pos',2.5)
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
            if direction < 0 and 'left' in l:
                step_size[l]+=direction #shorten left steps
            elif direction > 0 and 'right' in l:
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
        


    def set_head(self, pos=[0, 0], t=1):
        self.motion_queue.append(MotionPlan(
                    limbs=[self.limbs['head']],
                    steps=[None, pos],
                    steps_duration=[t*1000]))

    def set_tail(self, pos=0, t=1):
        self.motion_queue.append(MotionPlan(
                    limbs=[self.limbs['tail']],
                    steps=[None,[pos]],
                    steps_duration=[t*1000]))

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
        self.stand(height=4,ground_pos=2, pitch=1, spread=3)
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
        sleep(1)
        self.stand(.1,ground_pos=2.5, height=7.7 ,pitch=2.5, spread=-4) 
        self.set_tail(-80,t=.1)                                                                                        
        self.set_head([-50,0], t=.1) 
    
    def pushups(self, iterations=5, t=1):
        self.stand(pitch=[-3,3], height=[5,7],iterations=iterations, t=t)

class MotionPlan:
    def __init__(self, limbs, steps, steps_duration, phase=0,iterations=1,  position_mode=False,wait_for=None):
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

