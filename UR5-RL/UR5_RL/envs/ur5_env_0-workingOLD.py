
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

import socket
import numpy as np
import struct
from scipy.spatial.transform import Rotation as R
from remote_FT_client import RemoteFTclient
from time import sleep
import selectors

HOST2 = '192.168.0.103'
PORT2= 65481

# Create a TCP/IP socket


sock_DC = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


server_address_DC = (HOST2, PORT2)# Connect the socket to the port where the server is listening
print('DC socket connecting to {} port {}'.format(*server_address_DC))
sock_DC.connect(server_address_DC)


FTclient = RemoteFTclient( '192.168.0.103', 10000 )
print( FTclient.prxy.system.listMethods() )

print( "Biasing wrist force" )
FTclient.bias_wrist_force()


def rot2rpy(Rt, degrees=False):
    """ Converts a rotation matrix into rpy / intrinsic xyz euler angle form.
    Args:    Rt (ndarray): The rotation matrix.
    Returns: ndarray: Angles in rpy / intrinsic xyz euler angle form.
    """
    return R.from_matrix(Rt).as_euler('xyz', degrees=degrees)


def rpy2rot(rpy, degrees=False):
    """Converts rpy / intrinsic xyz euler angles into a rotation matrix.
    Args:    rpy (array-like): rpy / intrinsic xyz euler angles.
             degrees (bool) : Whether or not euler_angles are in degrees (True), or radians (False).
    Returns: ndarray: The rotation matrix.
    """
    return R.from_euler('xyz', rpy, degrees=degrees).as_matrix()

#class BalancebotEnv(gym.Env):
class UR5Env0(gym.Env):


    def __init__(self,TotalEpisodes=100,StepsPerEpisode=10):
        print("TotalEpisodes:",TotalEpisodes, "   StepsPerEpisode:",StepsPerEpisode)
        self._observation = []
        #self.action_space = spaces.Discrete(9)#Generates number between 0 and 9
        self.action_space = spaces.Discrete(4)#Generates number between 0 and 9

        #self.action_space = spaces.Box(np.array([-0.01, -0.01, -0.01]), """ x, y, z min """
        #                                    np.array([0.01, 0.01, 0.01])) # x, y, z max
        """
        self.observation_space = spaces.Box(np.array([-1000,-1000,-1000,-1000,-1000, -40, -40,-40]), 
                                            np.array([1000,1000,1000,1000,1000,40, 40,40])) # 6 forcetorque, xyz pose
        """
        self.observation_space = spaces.Box(np.array([-30, -30]), 
                                            np.array([30, 30])) # 6 forcetorque, xy pose                                           
        """
        target is at 
        x -9.334348196083308 y -23.681130581510068 z 2.5003196929477154
        roll 179.26909783765905 pitch 2.003501129340695 yaw -179.58695200183655
        Forces and Torques:
        [-0.06000000000000001, -0.02, 0.36000000000000004, 0.023599999999999996, 0.0034000000000000002, 0]

        """
        self.TotalEpisodes=TotalEpisodes
        self.StepsPerEpisode=StepsPerEpisode

        self.fig, (self.ax1) = plt.subplots(1,figsize=(7,7)) # was 8,8

        self.curr_action_config = None #when the sim starts, there is no current target. needs to call motion selector to get a target
        self.episodecounter=1
        self.rewardlist=[]
        self.totalstepstaken=0

        self.resetEnvironment()
        time.sleep(0.1) 

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    #def _step(self, action):
    def step(self, action):

        # we can still check if we violate a torque limit etc and if that happens, we can collect an observation, reward and make sure done is now true so we move onto a new action or terminate the current policy rollout.
        self.motionselector(action)
        self._observation = self._compute_observation()  
        reward = self._compute_reward()
        
        #if (wrist_forcetorque[2]<-20   or wrist_forcetorque[2]>20):
        #    done=True
        
            
        #print("step:", self._envStepCounter, "action:",action, "observation",[self.xcurrent,self.ycurrent,self.zcurrent]," reward",self.currentreward )
   
        
        self._envStepCounter += 1  #was uncommented. commenting this may break the system
        self.totalstepstaken+=1
        done = self._compute_done()
        return np.array(self._observation), reward, done, {}
    
    def motionselector(self,action):
    
        if action==0:  
            inputstring='h'
            if self.totalstepstaken>=410:
                print("action taken:", inputstring) 
        if action==1:  
            inputstring='k'
            if self.totalstepstaken>=410:
                print("action taken:", inputstring) 
        if action==2:  
            inputstring='u'
            if self.totalstepstaken>=410:
                print("action taken:", inputstring) 
        if action==3:  
            inputstring='j'
            if self.totalstepstaken>=410:
                print("action taken:", inputstring) 
        data1=inputstring.encode('ascii')    
        sock_DC.sendall(data1) 
        self.actionlist.append(action)
        time.sleep(0.05)  
                
    def reset(self):    
        self._envStepCounter=0
        if self.totalstepstaken>=410:
            print("reset")
        self.resetEnvironment()


        self._observation = self._compute_observation() #you *have* to compute and return the observation from reset()
        return np.array(self._observation)
    
    def resetEnvironment(self):
         #inputstring=='home',inputstring=='end'
        #self._envStepCounter = 0

        inputstring='home'
        data1=inputstring.encode('ascii')    
        sock_DC.sendall(data1) 
        
        #self._seed()
        #random.seed()
    
        #self.xstart=random.uniform(self.targetx-(.5*0.0127) , self.targetx+(0.5*0.0127) ) # 0.0127 m=0.5in
        #self.ystart=random.uniform(self.targety-(.5*0.0127) , self.targety+(0.5*0.0127) )   #1*.001
        #self.zstart=0.82#0.87#0.95
    
        self.actionlist=[]
    

    def _compute_observation(self):
        if self.totalstepstaken>=410:
            print("obs")
        inputstring='obs'
        data1=inputstring.encode('ascii')    
        sock_DC.sendall(data1)
        #print("sent",inputstring)
        data2 = sock_DC.recv(64) 
        while data2== b'':
            data2 = sock_DC.recv(64)  #48 bytes
            #print(data1)
        unpacked = struct.unpack('ffffffffffffffff', data2)
        TransRotatmatrix=np.zeros([4,4])
        for i in range(4):
            TransRotatmatrix[i][:]=unpacked[0+(4*i):4+(4*i)]
        rpy=rot2rpy(TransRotatmatrix[0:3,0:3],True)
        x_pose=TransRotatmatrix[0][3]*39.3701  #convert to inches 
        y_pose=TransRotatmatrix[1][3]*39.3701
        z_pose=TransRotatmatrix[2][3]*39.3701
        roll=rpy[0]  #In degrees!
        pitch=rpy[1]
        yaw=rpy[2]
       
        forcesamples=5
        FT_list=[]
        AVG_FT_list=[0]*6  #3 forces 3 torques
        for i in range(5):  #get 5 force samples
            FT_list.append(FTclient.get_wrist_force())
            sleep( 0.020 )            
        for i in range(len(FT_list)): 
            #print("i:",i)
            AVG_FT_list[i]=0
            for j in range(len(FT_list)):  
                AVG_FT_list[i]+=FT_list[j][i]    mysocket.setblocking(0)

ready = select.select([mysocket], [], [], timeout_in_seconds)
if ready[0]:
    data = mysocket.recv(4096)
            AVG_FT_list[i]=AVG_FT_list[i]/(len(FT_list))
        #print("x",x_pose,"y",y_pose,"z",z_pose)
        #print("roll",roll,"pitch",pitch,"yaw",yaw)
        #print("Forces and Torques:", AVG_FT_list)

        return[x_pose,y_pose]

     
    
    def _compute_reward(self):
        if self.totalstepstaken>=410:
                print("compute reward called, about to call 'obs'")
        [currentX,currentY]=self._compute_observation()
        if self.totalstepstaken>=410:
                print("pose=",[currentX,currentY])
        """
        target is at 
        x -9.334348196083308 y -23.681130581510068 z 2.5003196929477154
        roll 179.26909783765905 pitch 2.003501129340695 yaw -179.58695200183655
        Forces and Torques:
        [-0.06000000000000001, -0.02, 0.36000000000000004, 0.023599999999999996, 0.0034000000000000002, 0]

        """
        targetX= -9.334
        targetY= -23.681
        self.currentreward=0 
        self.currentreward=-1* math.sqrt(pow(currentX-targetX,2)+pow(currentY-targetY,2))  #2D distance formula
        if self.totalstepstaken>=410:
                print("self.currentreward=",self.currentreward)
        return self.currentreward  #negative magnitude. hopefully system tries to reduce this. 
    

    def _compute_done(self):
        print(f"COMPUTE DONE? Episode: {self.episodecounter} step: {self._envStepCounter} Total Steps taken: {self.totalstepstaken} " )
        
        
        # - done, a boolean, value that is TRUE if the environment reached an endpoint, and should be reset, or FALSE otherwise;
        
         #system whent 'home" 39 times
        if self._envStepCounter >= self.StepsPerEpisode:    
            #print("Episode", self.episodecounter, "over. envStepCounter:", self._envStepCounter," StepsPerEpisode:" , self.StepsPerEpisode)
            #print("RESET!-env counter at max", "final reward value:",self.currentreward)
   
            self.rewardlist.append(self.currentreward)
            
            self.episodecounter=self.episodecounter+1
            if self.episodecounter%4==0 or self.episodecounter==self.TotalEpisodes+1:
                clear_output(wait = True)   #uncomment to clear output at each reset 
                print("Episode",self.episodecounter, "  Total Steps taken:", self.totalstepstaken)
                self.fig, (self.ax1) = plt.subplots(1,figsize=(8,8))
                self.ax1.cla() #clear axes 
                self.ax1.plot(self.rewardlist)

                plt.setp(self.ax1, xlim=(0, self.TotalEpisodes), ylim=(-.75,0))

                display(self.fig)
            
            #if self.episodecounter%10==0:
            #    ts = time.time()
            #    st = datetime.datetime.fromtimestamp(ts).strftime('%H:%M:%S')
            #    print("Ep:",self.episodecounter, " Reward:",self.currentreward, "Time:",st)
                #"Target:",(self.targetx,self.targety,self.targetz)
                
            #print("action list",self.actionlist)

            #if len(self.rewardlist)>self.TotalEpisodes:
            #   self.rewardlist.pop(0)
        """      
        if self.episodecounter>= self.TotalEpisodes+1:
            inputstring='end'
            data1=inputstring.encode('ascii')    
            sock_DC.sendall(data1)
            print("Training End")
            print('closing DC socket in .py policy file')
            print(self.rewardlist)
            sock_D.close()
        """    
        if self._envStepCounter >= self.StepsPerEpisode:
            print ("Episode done at step", self._envStepCounter )
        
        return  self._envStepCounter >= self.StepsPerEpisode #self.currentreward > -0.001 or
    
