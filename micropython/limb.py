try:
    from pca9685 import Servos
except ImportError:
    pass
import math
try:
    from .kinematics import TwoLinkArmKinematics as K 
except:
    from kinematics import TwoLinkArmKinematics as K 
try:
    from utime import ticks_ms, ticks_diff
except ImportError: #make it compatible to python3
    from time import time_ns
    def ticks_ms():
        return time_ns() // 1000000 
    def ticks_diff(new,old):
        return(new-old)           
          

def get_cat_limbs(i2c , init_theta=None, offset=None, invert=None, kinematics=None):
    limb_names=['leg_front_left', 'leg_back_left', 'leg_front_right', 'leg_back_right', 'tail', 'head']
    if init_theta is None:
        init_theta=[[170,-140],[170,-140],[170,-140],[170,-140],[10],[-40,0]] #sleeping_cat
    if invert is None:
        invert=[[0,1],[0,1],[1,0],[1,0],[0],[0,1]]
    if offset is None:
        offset=[[15,130],[10,35],[15,150],[15,30],[80],[100,95]] #EMPIRICAL    
    if kinematics is None:
        kinematics=[K(invert_th2=i) for i in [False, True, False, True]]+[None, None]
    servos=Servos(i2c) 
    s_nr=[[0,1],[2,3],[4,5],[6,7],[8],[9,10]]
    limbs={n:Limb(servos, n, nr, th, o, i,kin) for n, nr, th, o, i, kin in zip(limb_names,s_nr,init_theta, offset, invert, kinematics )}
    return(limbs)
        
class Limb:
    def __init__(self, servos,name, servo_nr, init_theta, offset, invert,kinematics=None):
        self.name=name
        self.joints=[Joint(servos, nr, it, os,iv ) for nr,it,os,iv in zip(servo_nr,  init_theta, offset, invert)]
        self.kinematics=kinematics
        self.active=False
        self.motion_start=None
        self.set_motion(LimbMotionPlan(self))
        
    
    def set_motion(self, motion, t=None):
        self.motion=motion
        if t is None:
            t=ticks_ms()
        self.motion_start=t
        self.active=True

    def update_position(self, t=None):
        if not self.active:
            return
        if t is None:
            t=ticks_ms()
        delta_t=ticks_diff(t,self.motion_start)
        if delta_t>self.motion.iter_duration:
            #todo: call callback????
            self.active=False
            theta=self.motion.end_theta
        else:
            theta=self.motion.get_theta(delta_t)
        self.move_to(theta)


    def __repr__(self):
        return 'Limb: {} with servos {}'.format(self.name, self.get_servo_nr())

    def get_servo_nr(self):
        return [j.servo_nr for j in self.joints]

    def get_theta(self, pos=None):
        if pos is not None:
            return self.kinematics.inverse(pos, degrees=True)
        return [j.theta for j in self.joints]


    def get_position(self, theta=None):
        if self.kinematics is None:
            return [None,None]
        if theta is None:
            theta=self.get_theta()
        return self.kinematics.forward(theta, degrees=True)
    
    def move_to(self, theta, pos=None):
        if pos is not None:
            theta=self.get_theta(pos)
        for j,th in zip (self.joints, theta):
            j.move_to(th)
    
    
    def get_distance(self, pos):
        #return the distance from the current position
        current=self.get_position()[1]
        delta=[c-p for c,p in zip(current, pos)]
        dist=math.sqrt(sum([d**2 for d in delta]))
        return(dist)

    

class Joint:
    def __init__(self, servos, servo_nr, theta=None, offset=0, invert=False):
        self.servos=servos
        self.servo_nr=servo_nr
        servos.release(servo_nr)
        self.theta=theta
        self.offset=offset
        self.invert=invert    
   
    def move(self, delta):
        self.move_to(self.theta+delta)

    def move_to(self,theta=None):
        if theta is None:
            self.servos.release(self.servo_nr)
        else:
            self.theta=theta
            theta+=self.offset 
            if self.invert:
                theta=180-theta
            #todo: warn if outside range
            self.servos.position(self.servo_nr, theta)

    def define_as(self, theta=0): #define current angle to be theta
        self.offset+=(self.theta-theta)
        self.theta=theta

    def get_range(self):
        if self.invert():
            return 0+self.offset, 180+self.offset #sure???
        else:
            return 0-self.offset, 180-self.offset
        #assuming [0-180]
 

class LimbMotionPlan: 
    def __init__(self, limb, steps=None, steps_duration=None, phase=0,position_mode=False, callback=None):
        self.limb=limb
        self.callback=callback
        if steps is None:
            steps=[[None]]
        self.steps=steps
        if steps_duration is None:
            steps_duration=[]
        self.steps_duration=steps_duration
        if len(self.steps)==len(self.steps_duration):
            self.steps.append(self.steps[0]) #make it circular
            
        if len(self.steps)-1!=len(self.steps_duration):
            raise ValueError('number of steps ({}) does not match number of durations ({})'.format(len(self.steps),len(self.steps_duration)))
        self.position_mode=position_mode
        self.n_steps=len(steps_duration)
        self.iter_duration=sum(steps_duration)
        self.phase=self.iter_duration*phase
        self.initialized=False
    
    def initialize(self):
        if self.steps[0][0] is None:
            self.steps[0]=self.limb.get_position()[1] if self.position_mode else self.limb.get_theta()
        
        self.start_pos=self._get_pos_within_iteration(self.phase)
        #compute final theta
        self.end_theta=self._get_pos_within_iteration(self.phase+self.iter_duration)
        if self.position_mode:
            self.end_theta=self.limb.get_theta(self.end_theta)
        #print("steps {}, duration {}, start {}, end {}".format(self.steps, self.steps_duration, self.start_pos, self.end_theta))
        self.initialized=True
        
        

    @staticmethod    
    def mean( start, end, w):#weighted mean
        return [s*(1-w)+e*w for s,e in zip(start, end)]

    def _get_pos_within_iteration(self, t):
        if t==0 or self.iter_duration==0:
            return(self.steps[0])
        if t == self.iter_duration:
            return(self.steps[-1])
        t%= self.iter_duration
        #find the current step
        prev=0
        step=0
        for i,step in enumerate(self.steps_duration):
            if prev+step>t:
                break
            prev+=step
        #get weighted mean
        progress=(t-prev)/step
        return self.mean(self.steps[i],self.steps[(i+1)%len(self.steps)],progress)

    def get_theta(self, t=None):
        if not self.initialized:
            self.initialize()
        current_pos=self._get_pos_within_iteration(t+self.phase)
        if self.position_mode:
            return self.limb.get_theta(current_pos)
        return current_pos

