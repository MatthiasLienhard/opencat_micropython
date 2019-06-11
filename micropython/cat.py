from pca9685 import Servos
import machine
import time
import kinematics
import micropython 
micropython.alloc_emergency_exception_buf(100) 


class Cat:
    def __init__(self):
        self.i2c = machine.I2C(-1,machine.Pin(22), machine.Pin(21), freq=100000)
        self.servos=Servos(self.i2c)
        for i in range(11):
            self.servos.release(i)
        init_theta=[170,-140,170,-140,170,-140,170,-140,10,-40,0] #sleeping_cat
        invert=[1,0,1,0,0,1,0,1,0,0,0]
        #offsets=[0]*11 #todo: read from file
        #offset=[0,150,0,150,0,150,0,150,90,110,90] #GENERIC
        offset=[-15,150,-15,140,-15,150,-15,150,80,110,80] #EMPIRICAL
    
        self.joints=[Joint(self.servos, i, theta=init_theta[i], offset=offset[i], invert=invert[i]) for i in range(11)]
        self.leg_kinematics=kinematics.TwoLinkArmKinematics(link_len=[4,5.5]) #todo: constrains
        self.current_head_motion=None
        self.current_tail_motion=None8
        self.current_leg_motion=[None, None, None, None]
        self.legs=[[self.joints[i*2], self.joints[i*2+1]] for i in range(4)]
        self.tail=[self.joints[8]]
        self.head=[self.joints[9], self.joints[10]]
        self.timer= machine.Timer(0)
        #time.sleep(1)# wait until initial position is reached
        self.set_timer(25)

    def set_timer(self,freq):
        if freq>50:
            freq=50
        self.timer_freq=freq
        try:
            self.timer.init(period=1000//freq, mode=machine.Timer.PERIODIC, callback=self._update)
        except OSError:
            self.timer.deinit()
            self.timer.init(period=1000//freq, mode=machine.Timer.PERIODIC, callback=self._update)

        
    def _update(self,timer=None ):
        new_thetas=self.get_thetas(time.ticks_ms()+1000//self.timer_freq) #compute one step in the future
        self.move_joints(new_thetas)

    def get_thetas(self,ref_time):
        new_thetas=[]
        #compute leg positions (it might take time due to kinematics)
        for leg_motion in self.current_leg_motion:
            if leg_motion is None:
                new_thetas+=[None,None]
            else:
                new_thetas+=leg_motion.get_theta(ref_time)
        #compute tail positions
        if self.current_tail_motion is None:
            new_thetas+=[None]
        else:
            new_thetas+=self.current_tail_motion.get_theta(ref_time)
        #compute head positions
        if self.current_head_motion is None:
            new_thetas+=[None, None]
        else:
            new_thetas+=self.current_head_motion.get_theta(ref_time)    
        return(new_thetas)



    def move_joints(self, thetas, delay=0):
        for i, th in enumerate(thetas):
            self.joints[i].move_to(theta=th)
            time.sleep(delay)

    def stand(self, height=5,ground_pos=None, t=2, leg_idx=range(4), now=None):  
        if ground_pos is None:
            ground_pos=[0]*4
        if not isinstance(ground_pos, list):
            ground_pos=[ground_pos]*4
        if not isinstance(height, list):
            height=[height]*4
        if now is None:
            now=time.ticks_ms()      
        for i in leg_idx:
            self.current_leg_motion[i]=MotionPlan(
                    init_theta=[self.legs[i][j].theta for j in range(2)], 
                    pos_steps=[[ground_pos[i],height[i]]],
                    kinematics=self.leg_kinematics, 
                    transition_time=t*1000, start_time=now
                    )
    
    def tripod_gait(self, n_steps=3,height=[7,6], ground_pos=[-2,2], leg_idx=range(4),transition=1, speed=1):  
        #tripod gait, three legs are on the ground
        #speed in cm/sec

        step_size=abs(ground_pos[0]-ground_pos[1])
        lift=height[0]-height[1]
        t_ground=step_size/speed*1000 
        t_air=step_size/(2*lift+step_size)
        steps_duration=[t_ground*f for f in [3, (1-t_air)/2 ,t_air, (1-t_air)/2]]
        pos_steps=[ [ground_pos[0],height[0]], 
        [ground_pos[1],height[0]],
        [ground_pos[1],height[1]],    
        [ground_pos[0],height[1]]]
        now=time.ticks_ms()      
        for i in leg_idx:            
            self.current_leg_motion[i]=MotionPlan(
                init_theta=[self.legs[i][j].theta for j in range(2)], 
                pos_steps=pos_steps,
                kinematics=self.leg_kinematics, 
                phase=i/4,
                steps_duration=steps_duration,
                iterations=n_steps,
                transition_time=1000)

    def walk(self, n_steps=10,height=[7,6], ground_pos=[-4.5,3],transition=.5, speed=6, leg_idx=range(4)):  
        #two legs are on the ground
        #speed in cm/sec

        step_size=abs(ground_pos[0]-ground_pos[1])
        lift=height[0]-height[1]
        t_ground=step_size/speed/2*1000 
        t_air=step_size/(2*lift+step_size)
        steps_duration=[t_ground*f for f in [1, (1-t_air)/2 ,t_air, (1-t_air)/2]]
        pos_steps=[ [ground_pos[0],height[0]], 
            [ground_pos[1],height[0]],
            [ground_pos[1],height[1]],    
            [ground_pos[0],height[1]]]
        now=time.ticks_ms() 
        phase=[0,.5,.5,0]     
        for i in leg_idx:            
            self.current_leg_motion[i]=MotionPlan(
                init_theta=[self.legs[i][j].theta for j in range(2)], 
                pos_steps=pos_steps,
                kinematics=self.leg_kinematics, 
                phase=phase[i],
                steps_duration=steps_duration,
                iterations=n_steps,
                transition_time=1000)

    def set_head(self, pos=[0, 0], t=1):
        self.current_head_motion=MotionPlan(
                    init_theta=[self.head[j].theta for j in range(2)], 
                    pos_steps=[pos],
                    transition_time=t*1000)
    def set_tail(self, pos=0, t=1):
        self.current_tail_motion=MotionPlan(
                    init_theta=[self.tail[0].theta], 
                    pos_steps=[[pos]],
                    transition_time=t*1000)

    def wag(self, n=3, amplitude=30, freq=1):
        self.current_tail_motion=MotionPlan(
                    init_theta=[self.tail[0].theta], 
                    pos_steps=[[amplitude],[-amplitude]],
                    steps_duration=[1/freq/2*1000]*2 ,
                    phase=.25, #start in the middle
                    iterations=n,
                    transition_time=100) #time to reach center

    def eat(self,t=1):
        now=time.ticks_ms()+10      
        self.stand(height=8,leg_idx=[1,3], ground_pos=-3, t=t, now=now)
        self.stand(height=4,leg_idx=[0,2], ground_pos=4, t=t, now=now)
        self.set_head([-60,0])
        self.wag(8,freq=2)

    def sleep(self):
        self.stand(height=2,ground_pos=2)
        self.set_head([-80,0])

    def yes(self, n=3, amplitude=30, freq=1):
        self.current_head_motion=MotionPlan(
                    init_theta=[self.head[j].theta for j in range(2)], 
                    pos_steps=[[amplitude,0],[-amplitude,0]],
                    steps_duration=[1/freq/2*1000]*2 ,
                    phase=.25, #start in the middle
                    iterations=n,
                    transition_time=100) #time to reach center

    def no(self,n=3, amplitude=30, freq=1):
        self.current_head_motion=MotionPlan(
                    init_theta=[self.head[j].theta for j in range(2)], 
                    pos_steps=[[0,0],[-amplitude/4,-amplitude],[0,0],[-amplitude/4,amplitude]],
                    steps_duration=[1/freq/4*1000]*4 ,
                    iterations=n,
                    transition_time=100) #time to reach center

class Joint:
    def __init__(self, servos, servo_nr, theta=None, offset=0, invert=False):
        self.servos=servos
        self.servo_nr=servo_nr
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
 
class MotionPlan: 
    def __init__(self, init_theta, transition_time,pos_steps, kinematics=None, steps_duration=None,start_time=None, iterations=0, phase=0):
        if kinematics is None:        
            self.init_pos=init_theta
        if kinematics is not None:
            self.init_pos=kinematics.forward(init_theta, degrees=True)[-1]
        if start_time is None:
            start_time= time.ticks_ms()
        if steps_duration is None:
            steps_duration=[]
        self.steps_duration=steps_duration
        self.kinematics=kinematics
        self.transition_time=transition_time
        self.terminated=False
        self.n_steps=len(steps_duration)
        self.start_time=start_time
        self.pos_steps=pos_steps
        self.iterations=iterations
        self.iter_duration=sum(steps_duration)
        self.phase=self.iter_duration*phase
        self.start_pos=self._get_pos_within_iteration(self.phase)
        #compute final theta
        if self.iterations==0:
            self.end_theta=self.pos_steps[-1]
        else:#in iterative moves, start == end
            self.end_theta=self.start_pos
        if self.kinematics is not None:
            self.end_theta=self.kinematics.inverse(self.end_theta, degrees=True)

    @staticmethod    
    def mean( start, end, w):#weighted mean
        return [s*(1-w)+e*w for s,e in zip(start, end)]

    def _get_pos_within_iteration(self, t):
        if t==0 or self.iter_duration==0:
            return(self.pos_steps[0])
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
        return self.mean(self.pos_steps[i],self.pos_steps[(i+1)%self.n_steps],progress)

    def get_theta(self, t=None):
        if self.terminated:
            return(self.end_theta)
        if t is None:
            t= time.ticks_ms()
        t-=self.start_time
        if t < self.transition_time:
            #move towards first position
            progress=t/self.transition_time
            current_pos=self.mean(self.init_pos,self.start_pos,progress)            
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
        if self.kinematics is None:
            return current_pos
        else:
            return self.kinematics.inverse(current_pos, degrees=True)

