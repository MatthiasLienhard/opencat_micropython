try:
    from pca9685 import Servos
    import machine
except ModuleNotFoundError:
    pass
import math
try:
    from .kinematics import TwoLinkArmKinematics as K 
except:
    from kinematics import TwoLinkArmKinematics as K 

try:
    from utime import ticks_ms, ticks_diff
except ModuleNotFoundError: #make it compatible to python3
    from time import time_ns
    def ticks_ms():
        return time_ns() // 1000000 
    def ticks_diff(new,old):
        return(new-old)           

def get_cat_limbs(i2c , init_theta=None, offset=None, invert=None, kinematics=None):
    limb_names=['leg_front_left', 'leg_back_left', 'leg_front_right', 'leg_back_right', 'tail', 'head']
    if init_theta is None:
        init_theta=[[160,-130],[20,130],[160,-130],[20,130],[10],[-40,0]] #sleeping_cat
    if invert is None:
        invert=[[0,1],[0,1],[1,0],[1,0],[0],[0,0]]
    if offset is None:
        offset=[[5,130],[5,50],[5,150],[5,40],[80],[110,80]] #EMPIRICAL    
    if kinematics is None:
        kinematics=[K(invert_th2=i) for i in [False, True, False, True]]+[None, None]
    servos=Servos(i2c) 
    s_nr=[[0,1],[2,3],[4,5],[6,7],[8],[9,10]]
    limbs={n:Limb( n,servos,  nr, th, o, i,kin) for n, nr, th, o, i, kin in zip(limb_names,s_nr,init_theta, offset, invert, kinematics )}
    return(limbs)
        
class Limb:
    def __init__(self, name,servos, servo_nr, init_theta, offset, invert,kinematics=None):
        self.name=name
        self.joints=[Joint(servos, nr, it, os,iv ) for nr,it,os,iv in zip(servo_nr,  init_theta, offset, invert)]
        self.kinematics=kinematics
        self.motion=MotionPlan(self, transition_time=0)  # set the initial position
    
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
        if self.invert:
            return 0+self.offset, 180+self.offset #sure???
        else:
            return 0-self.offset, 180-self.offset
        #assuming [0-180]
 
       
class MotionPlan: 
    def __init__(self, limb, transition_time,steps=None, position_mode=False, steps_duration=None,start_time=None, iterations=0, phase=0):
        self.limb=limb
        if position_mode:  
            self.init=limb.get_position()[1]
        else:
            self.init=limb.get_theta()
        if steps is None:
            steps=[self.init]
        if start_time is None:
            start_time= ticks_ms()
        
        self.steps=steps
        if steps_duration is None:
            steps_duration=[]
        self.steps_duration=steps_duration
        self.position_mode=position_mode
        self.transition_time=transition_time
        self.terminated=False
        self.n_steps=len(steps_duration)
        self.start_time=start_time
        self.iterations=iterations
        self.iter_duration=sum(steps_duration)
        self.phase=self.iter_duration*phase
        self.start_pos=self._get_pos_within_iteration(self.phase)
        #compute final theta
        if self.iterations==0:
            self.end_theta=self.steps[-1]
        else:#in iterative moves, start == end
            self.end_theta=self.start_pos
        if position_mode:
            self.end_theta=self.limb.get_theta(self.end_theta)
        
        

    @staticmethod    
    def mean( start, end, w):#weighted mean
        return [s*(1-w)+e*w for s,e in zip(start, end)]

    def _get_pos_within_iteration(self, t):
        if t==0 or self.iter_duration==0:
            return(self.steps[0])
        t%=self.iter_duration
        #find the current step
        prev=0
        for i,step in enumerate(self.steps_duration):
            if prev+step>t:
                break
            prev+=step
        t-=prev
        #get weighted mean
        progress=t/step
        return self.mean(self.steps[i],self.steps[(i+1)%self.n_steps],progress)

    def get_theta(self, t=None):
        if self.terminated:
            return(self.end_theta)
        if t is None:
            t= ticks_ms()
        t=ticks_diff(t,self.start_time)
        if t < self.transition_time:
            #move towards first position
            if t>0:
                progress=t/self.transition_time
                current_pos=self.mean(self.init,self.start_pos,progress)            
            else:
                current_pos=self.init
        elif self.n_steps>0: #not just one position
            i=t//self.iter_duration
            if i<self.iterations:
                current_pos=self._get_pos_within_iteration(t+self.phase)
            else:
                self.terminated=True
                return(self.end_theta)
        else:
            self.terminated=True
            return(self.end_theta)
        if self.position_mode:
            return self.limb.get_theta(current_pos)
        return current_pos

