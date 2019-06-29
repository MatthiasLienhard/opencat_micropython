import math
import sys
try:
    from time import ticks_ms, ticks_diff
except ImportError:
    from time import time_ns
    def ticks_ms():
        return time_ns() // 1000000 
    def ticks_diff(new,old):
        return(new-old)


#kinematics for a two link arm
class TwoLinkArmKinematics:
    def __init__(self,link_len=[4.5, 5], constrains=[None, None], invert_th2=False): 
        self.link_len=link_len
        self.constrains=constrains
        self.invert_th2= invert_th2

    def forward(self,theta, degrees=True):
        if degrees:
            theta=[math.radians(t) for t in theta]            
        pos1=[self.link_len[0]*math.cos(theta[0]), 
                self.link_len[0]*math.sin(theta[0])]
        pos2=[pos1[0]+self.link_len[1]*math.cos(sum(theta)), 
                pos1[1]+self.link_len[1]*math.sin(sum(theta))]
        return pos1,pos2

    def inverse(self, pos, degrees=True):
        x,y=pos
        l1, l2= self.link_len
        r_sq = x**2 +y**2
        l_sq = l1**2 + l2**2
        term2 = (r_sq - l_sq)/(2*l1*l2)
        term1 = ((1 - term2**2)**0.5)*-1
        th2 = math.atan2(term1, term2)
        if self.invert_th2:        
            th2 = -1*th2
        k1 = l1 + l2*math.cos(th2)
        k2 = l2*math.sin(th2)
        #r  = (k1**2 + k2**2)**0.5
        gamma = math.atan2(k2,k1)
        #calculate th1
        th1 = math.atan2(y,x) - gamma
        #TODO: check constrains
        #if violated, try invert theta2
        if degrees:
            return math.degrees(th1), math.degrees(th2)
        return th1, th2
 