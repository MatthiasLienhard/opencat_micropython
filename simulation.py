#!/usr/bin/python3

from micropython.cat import Cat,MotionPlan
from micropython.kinematics import TwoLinkArmKinematics as K
from tkinter import Tk, Canvas, Entry, Frame,StringVar,Label, N, W, E, S
import tkinter as tk
from tkinter import ttk
import math
from micropython.limb import Limb
from collections import deque
try:
    from time import ticks_ms, ticks_diff
except ImportError:
    from time import time_ns
    def ticks_ms():
        return time_ns() // 1000000 
    def ticks_diff(new,old):
        return(new-old)

DEBUG=True

if DEBUG:
    def print_attributes(o):
        for n in dir(o):
            v=getattr(o,n)
            if not callable(v) and n[0] != '_':
                print('{}: {}'.format(n,v))
      
  
        
class SimulationLimb(Limb):
    def __init__(self, name,servo_nr,init_theta, offset, invert, kinematics=None):
        self.name=name
        self.joints=[SimulationJoint( it, os,iv) for it,os,iv in zip( init_theta, offset, invert)]
        self.kinematics=kinematics        
        self.servo_nr=servo_nr
        self.trace=deque(maxlen=20)
        self.motion=MotionPlan(self, transition_time=0)  # set the initial position
        
    def get_servo_nr(self):
        return self.servo_nr
    
    def move_to(self, theta, pos=None):
        if pos is not None:
            self.trace.append(pos)
            super().move_to(theta=None, pos=pos)
        else:
            pos=self.get_position(theta)[1]
            self.trace.append(pos)
            super().move_to(theta)
        


class SimulationJoint(): 
    #simple class as in cat but without servos
    def __init__(self, theta=None, offset=0, invert=False):
        self.theta=theta
        self.offset=offset
        self.actual_theta=theta+offset #relative to the imaginatory servo (not used)
        self.invert=invert
        self.trace=deque(maxlen=10)

       
    def move_to(self,theta=None):
            self.theta=theta
            self.actual_theta=theta+self.offset 
            if self.invert:
                self.actual_theta=180-theta

   
 
  
def get_simulation_limbs(init_theta=None, offset=None, invert=None, kinematics=None):
    limb_names=['leg_front_left', 'leg_back_left', 'leg_front_right', 'leg_back_right', 'tail', 'head']
    s_nr=[[0,1],[2,3],[4,5],[6,7],[8],[9,10]]
    if init_theta is None:
        init_theta=[[170,-140],[170,-140],[170,-140],[170,-140],[10],[-40,0]] #sleeping_cat
    if invert is None:
        invert=[[1,0],[1,0],[0,1],[0,1],[0],[0,0]]
    if offset is None:
        offset=[[0,150]]*4+[[90],[110,90]] #EMPIRICAL    
    if kinematics is None:
        kinematics=[K(invert_th2=i) for i in [False, True, False, True]]+[None, None]
    limbs={n:SimulationLimb( n,nr, th, o, i,kin) for n, nr,th, o, i, kin in zip(limb_names,s_nr, init_theta, offset, invert, kinematics )}
    return(limbs)


class SimulationGUI:        
    def __init__(self, limbs):
        self.scale=20
        #[math.radians(i) for i in [45,90]]
        #self.arm=TwoLinkArm()
        self.cat=Cat(limbs)
        leg_idx=[n for _,n in self.cat._limb_names if 'leg' in n]
        offset=[[15,2],[5,2],[14,2],[4,2]]
        self.limb_offset={l:o for l,o in zip(leg_idx, offset)}
        self.nsteps=0
        self.stepcount=0
        self.step=[0,0]
        #self.steptime=100 #ms
        self.pt_per_step=10
        self.root = Tk()
        self.root.title("robot cat kinematic simulation")
        self.canvas = Canvas(self.root)
        self.canvas.grid(column=0, row=0, sticky=(N, W, E, S))
        self.canvas.bind("<Button-1>", self.xy)
        #self.canvas.bind("<B1-Motion>", self.xy)
        self.canvas.pack(side='top')
        self.repl_frame=Frame(self.root)
        self.repl_frame.pack(side='bottom')
        Label(self.repl_frame, text='>>> ').pack(side='left')       
        self.repl = Entry(self.repl_frame, width=30)
        self.repl.pack(side='left')
        self.repl_history=list()
        self.current=0
        
        self.active_leg=self.cat._limb_names[0][1]
        self.cat.walk()
        self._update()
        self.repl.bind('<Return>', self.execute)
        self.repl.bind('<Up>', self.prev_hist)
        self.repl.bind('<Down>', self.next_hist)
        self.root.mainloop()
    
    def prev_hist(self, event):
        self.current+=1
        if self.current>len(self.repl_history):
            self.current=1
        self.repl.delete(0,tk.END)
        self.repl.insert(0,self.repl_history[-self.current])


    def next_hist(self, event):
        self.current-=1
        if self.current>1:
            self.repl.delete(0,tk.END)
            self.repl.insert(0,self.repl_history[-self.current])
        else:
            self.current=0

    def execute(self, event):
        cmd=self.repl.get()
        print(cmd)
        try:
            eval ('self.'+cmd)
            self.repl_history.append(cmd)
        except Exception as e:
            print(e)
        if cmd[:4] =='cat.':
            self._update()    
        self.current=0        
        self.repl.delete(0, tk.END)
        
        return True


    def xy(self, event):
        x,y=event.x/self.scale, event.y/self.scale
        print('click at {}, {}'.format(x,y))
        active=self.get_leg([x,y])
        if active is not None:
            self.active_leg=active
            print('selected ' +active)
            self.draw_legs()
        else:
            print(self.cat.limbs[self.active_leg])
            print_attributes(self.cat.limbs[self.active_leg])
            aim=[x-self.limb_offset[self.active_leg][0], y-self.limb_offset[self.active_leg][1]]    
            #print('aim:[{},{}]'.format(*aim))
            dist=self.cat.limbs[self.active_leg].get_distance(aim)
            nsteps=(dist*self.scale)//self.pt_per_step
            th=self.cat.limbs[self.active_leg].get_theta(aim)
            print('aim:[{},{}] dist={}, nsteps={}, theta=[{},{}]'.format(*aim, dist, nsteps, *th))
            self.cat.limbs[self.active_leg].motion=MotionPlan(
                    limb=self.cat.limbs[self.active_leg], 
                    steps=[aim],
                    position_mode=True,
                    transition_time=nsteps*1000//self.cat.freq )
            print_attributes(self.cat.limbs[self.active_leg].motion)
            self.root.after(1000//self.cat.freq, self._update)

    def _update(self, event=None):
        
        if self.cat.is_active():
            #for l in self.cat.limbs.values():
            #    print_attributes(l.motion)
            self.cat._update(event) 
            self.draw_legs()
            self.root.after(1000//self.cat.freq, self._update)
            return True

    def get_leg(self, pos):
        for name, offset in self.limb_offset.items():
            if abs(pos[0]-offset[0])<.2 and abs(pos[1]-offset[1])<.2:
                return(name)
        return (None)

    
   
    def draw_legs(self):
        self.canvas.delete("all")
        for pos_rel in self.cat.limbs[self.active_leg].trace:
            x,y=[(p+o)*self.scale for p,o in zip(pos_rel, self.limb_offset[self.active_leg])]
            
            self.canvas.create_oval(x-1, y-1, x+1, y+1, width = 0, fill = 'darkgrey')
        for i in [n for _,n in self.cat._limb_names if 'leg' in n]:
            color='red' if i == self.active_leg else 'black'
            shoulder=[s*self.scale for s in self.limb_offset[i]]
            knee, foot=self.cat.limbs[i].get_position() #relative to shoulder
            knee=[k*self.scale+s for k,s in zip(knee, shoulder)] #get absolute position           
            foot=[f*self.scale+s for f,s in zip(foot, shoulder)] #get absolute position         
            self.canvas.create_line(shoulder[0],shoulder[1],knee[0],knee[1], fill=color)
            self.canvas.create_line(knee[0],knee[1],foot[0],foot[1], fill=color)
            



if __name__ == '__main__':
    SimulationGUI(get_simulation_limbs())
