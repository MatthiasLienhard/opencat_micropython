
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
            period=1000//freq
            try:
                timer.init(period=period, mode=Timer.PERIODIC, callback=self._update)
            except OSError:
                timer.deinit()
                sleep(1)
                timer.init(period=period,mode=Timer.PERIODIC, callback=self._update)
    
    @property
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
        if iterations is not None:
            n_pos+=1
        if len(t)<n_pos:
            t=t+[t[-1]]*(n_pos-len(t))
        t=[t_s *1000 for t_s in t]
        
        now=ticks_ms()      
        for leg,pos in leg_position.items():
            self.limbs[leg].motion=MotionPlan(
                    limb=self.limbs[leg], 
                    steps=pos,
                    position_mode=True,
                    transition_time=t[0], 
                    steps_duration=t[1:],
                    start_time=now, 
                    iterations=iterations)
    
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
        phase={n:p/4 for n,p in zip(self.leg_names,[0,2,1,3])}
        now=ticks_ms()      
        for leg,pos in leg_position.items():
            hss=step_size[leg]/2#half step size
            steps=[ [ pos[0][0]+hss, pos[0][1] ], 
                    [ pos[0][0]-hss, pos[0][1] ],
                    [ pos[0][0]-hss, pos[0][1]-lift ],
                    [ pos[0][0]+hss, pos[0][1]-lift ]]
            self.limbs[leg].motion=MotionPlan(
                limb=self.limbs[leg], 
                steps=steps,
                position_mode=True,
                phase=phase[leg],
                steps_duration=steps_duration,
                iterations=n_steps,
                transition_time=t*1000, 
                start_time=now)

    def walk(self, n_steps=10, lift=1, step_size=6, direction=0, t=1, speed=6,**kwargs):  
        #two opposit legs are on the ground
        #speed in cm/sec
        kwargs.setdefault('ground_pos',2.5)
        leg_position=self.leg_position(**kwargs) #center position
        reverse= (step_size<0 or speed <0)
        step_size=abs(step_size)
        speed=abs(speed)
        t_ground=step_size/speed/2*1000 
        t_air=step_size/(2*lift+step_size)
        step_size={l:step_size for l in leg_position}
        steps_duration=[t_ground*f for f in [1, (1-t_air)/2 ,t_air, (1-t_air)/2]]
        for l in step_size:
            if direction < 0 and 'left' in l:
                step_size[l]+=direction #shorten left steps
            elif direction > 0 and 'right' in l:
                step_size[l]-=direction #shorten right steps
        phase={n:p for n,p in zip(self.leg_names,[0,.5,.5,0] ) }#the order is important here!
        now=ticks_ms()         
        for leg,pos in leg_position.items():
            hss=step_size[leg]/2#half step size
            if reverse: hss*=-1
            steps=[ [ pos[0][0]+hss, pos[0][1] ], 
                    [ pos[0][0]-hss, pos[0][1] ],
                    [ pos[0][0]-hss, pos[0][1]-lift ],
                    [ pos[0][0]+hss, pos[0][1]-lift ]]   
            self.limbs[leg].motion=MotionPlan(
                limb=self.limbs[leg], 
                steps=steps,
                position_mode=True,
                phase=phase[leg],
                steps_duration=steps_duration,
                iterations=n_steps,
                transition_time=t*1000, 
                start_time=now) 

        self.limbs['head'].motion=MotionPlan(
            limb=self.limbs['head'], 
            steps=[[0,0],[10,40],[0,0],[10,-40]],
            phase=0,
            steps_duration= [t_ground//2]*4,
            iterations=n_steps//2,
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
        self.set_tail(-60)
        
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