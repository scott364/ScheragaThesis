
# https://github.com/yconst/balance-bot
import os
import math
import numpy as np
import gym
from gym import spaces
from gym.utils import seeding
import pybullet as p
import pybullet_data
import time
import datetime
import sys

import matplotlib.pyplot as plt
from IPython.display import display, clear_output
import random

#class BalancebotEnv(gym.Env):
class UR5Env0(gym.Env):
    #metadata = {
    #   'render.modes': ['human', 'rgb_array'],
    #    'video.frames_per_second' : 50}
    
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second' : 50}
    

    def __init__(self,totalepisodes=100):
        self._observation = []
        #self.action_space = spaces.Discrete(9)#Generates number between 0 and 9
        self.action_space = spaces.Discrete(4)#Generates number between 0 and 9

      
        #self.action_space = spaces.Box(np.array([-0.01, -0.01, -0.01]), """ x, y, z min """
        #                                    np.array([0.01, 0.01, 0.01])) # x, y, z max
        

        self.observation_space = spaces.Box(np.array([-1000,-1000,-1000,-1000,-1000, -1, -1]), 
                                            np.array([1000,1000,1000,1000,1000,1, 1])) # pitch, gyro, com.sp.

        
        self.totalepisodes=totalepisodes


        self.fig, (self.ax1) = plt.subplots(1,figsize=(7,7)) # was 8,8
        #p.setTimeStep(0.01) # sec
        self.sim.set_timestep(0.01)

        self.curr_action_config = None #when the sim starts, there is no current target. needs to call motion selector to get a target
        self.episodecounter=0
        self.rewardlist=[]


        self.resetEnvironment()

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    #def _step(self, action):
    def step(self, action):
        self.motionselector(action)
       
            
                
        # we can still check if we violate a torque limit etc and if that happens, we can collect an observation, reward and make sure done is now true so we move onto a new action or terminate the current policy rollout.
        
        self._observation = self._compute_observation()  
        reward = self._compute_reward()
        done = self._compute_done()
        #if (wrist_forcetorque[2]<-20   or wrist_forcetorque[2]>20):
        #    done=True
        
            
        #print("step:", self._envStepCounter, "action:",action, "observation",[self.xcurrent,self.ycurrent,self.zcurrent]," reward",self.currentreward )
        self._envStepCounter += 1
        return np.array(self._observation), reward, done, {}
    
    def motionselector(self,action):
    
  
        movedist=0.001    #0.001
       
        
        if action==0:  
            self.xcurrent=round(self.xcurrent+ movedist,3)
            self.zcurrent=round(self.zcurrent-movedist,3) 
        if action==1:  
            self.xcurrent=round(self.xcurrent- movedist,3)
            self.zcurrent=round(self.zcurrent-movedist,3) 
        if action==2:  
            self.ycurrent=round(self.ycurrent+movedist,3)
            self.zcurrent=round(self.zcurrent-movedist,3) 
        if action==3:  
            self.ycurrent=round(self.ycurrent-movedist,3)   
            self.zcurrent=round(self.zcurrent-movedist,3) 
         
            
        self.actionlist.append(action)
      
       
        
                
    def reset(self):    
        self._envStepCounter=0
        p.resetSimulation(0)
        self.resetEnvironment()


        self._observation = self._compute_observation() #you *have* to compute and return the observation from reset()
        return np.array(self._observation)
    
    def resetEnvironment(self):
 
        self._envStepCounter = 0

    
        
        self._seed()
        random.seed()
    
        self.xstart=random.uniform(self.targetx-(.5*0.0127) , self.targetx+(0.5*0.0127) ) # 0.0127 m=0.5in
        self.ystart=random.uniform(self.targety-(.5*0.0127) , self.targety+(0.5*0.0127) )   #1*.001
        self.zstart=0.82#0.87#0.95
        
        self.xcurrent=self.xstart
        self.ycurrent=self.ystart
        self.zcurrent=self.zstart
        self.xyz = [self.xcurrent, self.ycurrent, self.zcurrent]

        self.orientation= [0,1,0,0]#[-0.06,1,0,0] #[0,1,0,0] #rpy2quat  in cairo_planning/geometric/transformation
       
    
        self.actionlist=[]
    

    def _compute_observation(self):
        
   
        currentjointstate=self.sawyer_robot.get_current_joint_states()
        current_pose=self.sawyer_robot.solve_forward_kinematics(currentjointstate[:-2])

        x_pose=current_pose[0][0][0] +self.xoffset#+0.114#+.114 #.1
        y_pose=current_pose[0][0][1]+self.yoffset#-0.02#-0.02
        z_pose=current_pose[0][0][2]+self.zoffset#+0.13#+0.13 #.1

    
        wrist_jointinfo=p.getJointState(self.sawyerID,16) 
        wrist_forcetorque = [round(num, 3) for num in wrist_jointinfo[2]]
     
        return[wrist_forcetorque[0],wrist_forcetorque[1],wrist_forcetorque[2],wrist_forcetorque[3],wrist_forcetorque[4],x_pose,y_pose]
    
     
    
    def _compute_reward(self):

        self.currentreward=0 
        currentjointstate=self.sawyer_robot.get_current_joint_states()
        current_pose=self.sawyer_robot.solve_forward_kinematics(currentjointstate[:-2])

        z_pose=current_pose[0][0][2]+self.zoffset

        
        if self.dist==math.inf:
                self.currentreward=-1
        else:
            self.currentreward=-1*(z_pose-0.794)*39.3701  #-1* inches off of table

            
        return self.currentreward  #negative magnitude. hopefully system tries to reduce this. 
    

    def _compute_done(self):

        stepsPerEpisode=80 #did 20 before
        if self._envStepCounter >= stepsPerEpisode:    
            #print("RESET!-env counter at max", "final reward value:",self.currentreward)
   
            self.rewardlist.append(self.currentreward)
            
            self.episodecounter=self.episodecounter+1
            if self.episodecounter%50==0 or self.episodecounter==self.totalepisodes-1:
                clear_output(wait = True)   #uncomment to clear output at each reset 
                print("Ep.",self.episodecounter)
                self.fig, (self.ax1) = plt.subplots(1,figsize=(8,8))
                self.ax1.cla() #clear axes 
                self.ax1.plot(self.rewardlist)

                plt.setp(self.ax1, xlim=(0, self.totalepisodes), ylim=(-0.5,2))

                display(self.fig)
            
            if self.episodecounter%10==0:
                ts = time.time()
                st = datetime.datetime.fromtimestamp(ts).strftime('%H:%M:%S')
                print("Ep:",self.episodecounter,"Dist (m)",self.dist, " Reward:",self.currentreward," Final Position: ", self.xyz, "Time:",st)
                #"Target:",(self.targetx,self.targety,self.targetz)
                
            #print("action list",self.actionlist)

            if len(self.rewardlist)>self.totalepisodes:
                self.rewardlist.pop(0)

        return  self._envStepCounter >= stepsPerEpisode #self.currentreward > -0.001 or
    
