#!/usr/bin/python3

from kinematics import TwoLinkArmKinematics
from tkinter import *
from tkinter import ttk
import math

#animated two link arm
class TwoLinkArm:
    def __init__(self,pos=None,theta=None, link_len=[100, 100],constrains=[None, None]):
        self.kinematics=TwoLinkArmKinematics(link_len,constrains)
        if theta is not None:
            self.theta=theta 
            self.pos=self.kinematics.forward()
        else:
            if pos is None:
                pos=[0,link_len[0]]
            self.set_pos(pos)
        self.step=[0,0]
        self.nsteps=0
        self.stepcount=0
        

    def move(self, aim, nsteps):
        delta=[aim[i]-self.pos[1][i] for i in [0,1]]
        self.step=[delta[0]/nsteps, delta[1]/nsteps]
        self.nsteps=nsteps
        self.stepcount=0

    def next_step(self):        
        self.stepcount+=1
        if self.stepcount>self.nsteps:
            return False
        newpos=[self.pos[1][0]+self.step[0], self.pos[1][1]+self.step[1]]
        #print('new position: ({}, {})'.format(*newpos))
        try:
            self.set_pos(newpos)
        except TypeError:
            print('not in reach')
            self.nsteps=0
            return False
        return True     
            
    def set_pos(self, pos):        
        self.theta=self.kinematics.inverse(pos, degrees=False)
        self.pos=self.kinematics.forward(self.theta)
    

    def get_distance(self, pos):
        #return the distance from the current position
        delta=[pos[i]-self.pos[1][i] for i in [0,1]]
        dist=math.sqrt(sum([delta[i]**2 for i in [0,1]]))
        return(dist)


class Simulation:        
    def __init__(self):
        self.offset=[200,50]
        #[math.radians(i) for i in [45,90]]
        self.arm=TwoLinkArm()
        self.nsteps=0
        self.stepcount=0
        self.step=[0,0]
        self.steptime=100 #ms
        self.pt_per_step=10
        self.root = Tk()
        self.root.title("robot arm kinematic")
        self.canvas = Canvas(self.root)
        self.canvas.grid(column=0, row=0, sticky=(N, W, E, S))
        self.canvas.bind("<Button-1>", self.xy)
        self.canvas.bind("<B1-Motion>", self.xy)
        self.draw_arm()
        self.root.mainloop()

    def xy(self, event):
        aim=[event.x-self.offset[0], event.y-self.offset[1]]    
        dist=self.arm.get_distance(aim)
        try:
            th=self.arm.kinematics.inverse(aim, degrees=False)
            th=[round(math.degrees(t),2) for t in th]
        except TypeError:
            th=['NA']*2
        print('aim:[{},{}] dist={}, theta=[{},{}]'.format(*aim, dist, *th))
        self.nsteps=int(dist/self.pt_per_step)+1
        self.arm.move(aim, self.nsteps)
        self.root.after(self.steptime, self.make_step)

    def make_step(self):    
        #print('call makestep')
        if self.arm.next_step():
            self.draw_arm()
            self.root.after(self.steptime, self.make_step)
            return True

    def draw_arm(self):
        self.canvas.delete("all")
        self.canvas.create_line(
                ( self.offset[0], 
                  self.offset[1], 
                  self.offset[0]+self.arm.pos[0][0], 
                  self.offset[1]+self.arm.pos[0][1]))
        self.canvas.create_line(
                ( self.offset[0]+self.arm.pos[0][0], 
                  self.offset[1]+self.arm.pos[0][1], 
                  self.offset[0]+self.arm.pos[1][0],
                  self.offset[1]+self.arm.pos[1][1]))



if __name__ == '__main__':
    Simulation()
