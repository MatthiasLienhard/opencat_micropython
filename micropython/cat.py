
from time import sleep
try: 
    from machine import Timer
except ModuleNotFoundError:
    pass
try:
    from .limb import MotionPlan, ticks_ms
except:
    from limb import MotionPlan, ticks_ms

class Cat:
    def __init__(self, limbs, freq=25, timer=None):
        self.limbs=limbs        
        self._limb_names=[(l.get_servo_nr(), l.name) for l in self.limbs.values()]
        self._limb_names.sort()#sort by servo number to get defined order
        self.freq=25
        for nr, name in self._limb_names:
            print('{}: {}'.format(name, nr))

        if timer is not None:
            self.timer= timer
            try:
                timer.init(freq=self.freq, mode=Timer.PERIODIC, callback=self._update)
            except OSError:
                timer.deinit()
                sleep(1)
                timer.init(freq=self.freq, mode=Timer.PERIODIC, callback=self._update)

    def leg_names(self,which='all'):
        if which=='all':
            which=''
        return [n for _,n in self._limb_names if 'leg' in n and which in n]

    def _update(self,timer=None ):
        new_thetas=self.get_limb_thetas(ticks_ms())  #ticks_ms()+1000//self.timer_freq #compute one step in the future
        self.move_limbs(new_thetas)

    def get_limb_thetas(self,ref_time=None):
        if ref_time is None:
            ref_time=ticks_ms()
        new_thetas=dict()
        #compute leg positions (it might take time due to kinematics)
        for limb in self.limbs.values():            
            if limb.motion is None:
                new_thetas[limb.name]=[None] *len(limb.joints)
            else:
                new_thetas[limb.name]=limb.motion.get_theta(ref_time)
        return(new_thetas)


    def move_limbs(self, thetas):
        #print('move to {}'.format(thetas))
        for limbname, th in thetas.items():
            self.limbs[limbname].move_to(theta=th)

    def is_active(self):
        terminated=[l.motion.terminated for l in self.limbs.values()]
        return not all(terminated)
    
    
    def leg_position(self, height=7,ground_pos=0, spread=0, pitch=0,roll=0):
        leg_names=self.leg_names()
        pos={l:[ground_pos, height] for l in leg_names}
        if spread != 0:
            for l in leg_names:
                pos[l][0]+= spread/2 if 'front' in l else -spread/2
        if roll != 0:
            for l in leg_names:
                pos[l][1]+= roll/2 if 'left' in l else -roll/2
        if pitch != 0:
            for l in leg_names:
                pos[l][1]+= pitch/2 if 'front' in l else -pitch/2
        return(pos)
        
    def stand(self,t=1, **kwargs):  
        leg_position=self.leg_position(**kwargs)
        now=ticks_ms()      
        for leg,pos in leg_position.items():
            self.limbs[leg].motion=MotionPlan(
                    limb=self.limbs[leg], 
                    steps=[pos],
                    position_mode=True,
                    transition_time=t*1000, 
                    start_time=now)
    
    def tripod_gait(self, n_steps=10, lift=1, step_size=8, direction=0, t=1, speed=6,**kwargs):  
        #tripod gait, three legs are on the ground
        #speed in cm/sec
        kwargs.setdefault('spread',2)
        leg_position=self.leg_position(**kwargs) #center position
        t_ground=step_size/speed*1000/3
        t_air=step_size/(2*lift+step_size)
        steps_duration=[t_ground*f for f in [3, (1-t_air)/2 ,t_air, (1-t_air)/2]]
        step_size={l:step_size for l in leg_position}
        for l in step_size:
            if direction < 0 and 'left' in l:
                step_size[l]+=direction #shorten left steps
            elif direction > 0 and 'right' in l:
                step_size[l]-=direction #shorten right steps
        phase={n:p/4 for n,p in zip(self.leg_names(),[0,3,2,1])}
        now=ticks_ms()      
        for leg,pos in leg_position.items():
            hss=step_size[leg]/2#half step size
            steps=[ [ pos[0]+hss, pos[1] ], 
                    [ pos[0]-hss, pos[1] ],
                    [ pos[0]-hss, pos[1]-lift ],
                    [ pos[0]+hss, pos[1]-lift ]]
            self.limbs[leg].motion=MotionPlan(
                limb=self.limbs[leg], 
                steps=steps,
                position_mode=True,
                phase=phase[leg],
                steps_duration=steps_duration,
                iterations=n_steps,
                transition_time=t*1000, 
                start_time=now)

    def walk(self, n_steps=10, lift=1, step_size=8, direction=0, t=1, speed=6,**kwargs):  
        #two opposit legs are on the ground
        #speed in cm/sec
        leg_position=self.leg_position(**kwargs) #center position
        t_ground=step_size/speed/2*1000 
        t_air=step_size/(2*lift+step_size)
        step_size={l:step_size for l in leg_position}
        steps_duration=[t_ground*f for f in [1, (1-t_air)/2 ,t_air, (1-t_air)/2]]
        for l in step_size:
            if direction < 0 and 'left' in l:
                step_size[l]+=direction #shorten left steps
            elif direction > 0 and 'right' in l:
                step_size[l]-=direction #shorten right steps
        phase={n:p for n,p in zip(self.leg_names(),[0,.5,.5,0] ) }#the order is important here!
        now=ticks_ms()         
        for leg,pos in leg_position.items():
            hss=step_size[leg]/2#half step size
            steps=[ [ pos[0]+hss, pos[1] ], 
                    [ pos[0]-hss, pos[1] ],
                    [ pos[0]-hss, pos[1]-lift ],
                    [ pos[0]+hss, pos[1]-lift ]]   
            self.limbs[leg].motion=MotionPlan(
                limb=self.limbs[leg], 
                steps=steps,
                position_mode=True,
                phase=phase[leg],
                steps_duration=steps_duration,
                iterations=n_steps,
                transition_time=t*1000, 
                start_time=now) 


    def set_head(self, pos=[0, 0], t=1):
        self.limbs['head'].motion=MotionPlan(
                    limb=self.limbs['head'],
                    steps=[pos],
                    transition_time=t*1000)

    def set_tail(self, pos=0, t=1):
        self.limbs['tail'].motion=MotionPlan(
                    limb=self.limbs['tail'],
                    steps=[[pos]],
                    transition_time=t*1000)

    def wag(self, n=3, amplitude=30, freq=1):
        self.limbs['tail'].motion=MotionPlan(
                    limb=self.limbs['tail'],
                    steps=[[amplitude],[-amplitude]],
                    steps_duration=[1/freq/2*1000]*2 ,
                    phase=.25, #start in the middle
                    iterations=n,
                    transition_time=100) #time to reach center

    def eat(self,t=1):
        self.stand(height=6, pitch=-2, ground_pos=-1)
        self.set_head([-60,0])
        self.wag(8,freq=2)

    def sit(self,t=1):
        self.stand(height=6, pitch=4, ground_pos=-1)
        self.set_head([-20,0])
        
    def sleep(self):
        self.stand(height=4,ground_pos=2, pitch=1, spread=3)
        self.set_head([-80,-40])

    def yes(self, n=3, amplitude=30, freq=1):
        self.limbs['head'].motion=MotionPlan(
                    limb=self.limbs['head'], 
                    steps=[[amplitude,0],[-amplitude,0]],
                    steps_duration=[1/freq/2*1000]*2 ,
                    phase=.25, #start in the middle
                    iterations=n,
                    transition_time=100) #time to reach center

    def no(self,n=3, amplitude=30, freq=1):
        self.limbs['head'].motion=MotionPlan(
                    limb=self.limbs['head'], 
                    steps=[[0,0],[-amplitude/4,-amplitude],[0,0],[-amplitude/4,amplitude]],
                    steps_duration=[1/freq/4*1000]*4 ,
                    iterations=n,
                    transition_time=100) #time to reach center


 