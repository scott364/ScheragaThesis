
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
from time import sleep
import selectors

bot='red'  #'blue'
print(bot, "robot is being used. Please change the bot variable if this is incorrect")
if bot=='red':
    from remote_FT_client import RemoteFTclient
    FTclient = RemoteFTclient( '192.168.0.103', 10000 )
    #print( FTclient.prxy.system.listMethods() )
    FTclient.bias_wrist_force() #Biasing wrist force

HOST_DC = '192.168.0.103'
PORT_DC= 65485


sock_DC = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#sock_DC.setblocking(False)
server_address_DC = (HOST_DC, PORT_DC)# Connect the socket to the port where the server is listening
print('DC socket connecting to {} port {}'.format(*server_address_DC))
sock_DC.connect(server_address_DC)

initialrunflag=1   #changed to zero after the first run


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
        #self.action_space = spaces.Discrete(5)#Generates number between 0 and 9
        
        #9-8-2021:  Normalize the box- style action space!!  min = -1, max -1
        self.action_space = spaces.Box(np.array([-1,-1]),np.array([1,1]))
        
    
        #self.action_space = spaces.Box(np.array([-0.01, -0.01, -0.01]), """ x, y, z min """
        #                                    np.array([0.01, 0.01, 0.01])) # x, y, z max
        #self.observation_space = spaces.Box(np.array([-100,-100,-100,-100,-100, -30, -30]), 
         #                                   np.array([100,100,100,100,100,30, 30])) # 5 forcetorque, xyz pose
        
        self.observation_space = spaces.Box(np.array([-5,-5,-25,-1,-1,-8.5, -20.5]), 
                                            np.array([5,  5,  0, 1, 1,-6.5,-18.5])) # 5 forcetorque, xyz pose
        
        """
        target is at 
        x -9.334348196083308 y -23.681130581510068 z 2.5003196929477154
        roll 179.26909783765905 pitch 2.003501129340695 yaw -179.58695200183655
        Forces and Torques:
        [-0.06000000000000001, -0.02, 0.36000000000000004, 0.023599999999999996, 0.0034000000000000002, 0]


        cylinder above target pos:
        [[-0.99934544 -0.00730877  0.0354299  -0.19068683]
         [-0.00689006  0.99990515  0.01192549 -0.49560643]
         [-0.0355137   0.01167357 -0.99930101  0.10501393]
         [ 0.          0.          0.          1.        ]]
         
         cylinder half-inserted pose:
         [[-0.99934435 -0.0073168   0.0354588  -0.19065135]
         [-0.00690059  0.99990602  0.01184609 -0.49437242]
         [-0.03554215  0.01159364 -0.99930093  0.07050386]
         [ 0.          0.          0.          1.        ]]



        """
        self.TotalEpisodes=TotalEpisodes
        self.StepsPerEpisode=StepsPerEpisode
        self.xforcemin=-5
        self.xforcemax=5
        self.yforcemin=-5
        self.yforcemax=5
        
        self.zforcemin=-25
        self.zforcemax=0
        self.rolltorquemin=-1
        self.rolltorquemax=1
        self.pitchtorquemin=-1
        self.pitchtorquemax=1
        self.yawtorquemin=-.1
        self.yawtorquemax=.1
        
        
        self.fig, (self.ax1) = plt.subplots(1,figsize=(7,7)) # was 8,8

        self.curr_action_config = None #when the sim starts, there is no current target. needs to call motion selector to get a target
        self.episodecounter=1
        self.episodeinitialobsflag=1
        self.rewardlist=[]
        self.totalstepstaken=0
        self.totalsuccesscounter=0
        
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
    """
    def motionselector(self,action):
    
        if action==0:  
            inputstring='h'
            #if self.totalstepstaken>=410:
            #    print("action taken:", inputstring) 
        if action==1: 
            inputstring='k'
            #if self.totalstepstaken>=410:
            #    print("action taken:", inputstring) 
        if action==2:  
            inputstring='u'
            #if self.totalstepstaken>=410:
            #    print("action taken:", inputstring) 
        if action==3:  
            inputstring='j'
            #if self.totalstepstaken>=410:
            #    print("action taken:", inputstring) 
        if action==4:  
            inputstring='l'  #moves straight down'    
            
        command_msg=inputstring.encode('ascii')    
        sock_DC.send(command_msg) 
        self.actionlist.append(action)
        time.sleep(0.05)  
    """    
    
    def motionselector(self,action):
        
        inputstring='action'
        msgaction=inputstring.encode('ascii')    
        #print("action",action)  #action [ 1.         -0.93657553]
        #print("action message",msgaction) #action message b'action'

        sock_DC.send(msgaction)       
        actionbyte=struct.pack('dd',action[0],action[1])
        sock_DC.send(actionbyte) 
        #self.actionlist.append(action)
        time.sleep(0.05)  
        
    def reset(self):    
        self._envStepCounter=1
        self.doneflag=0
        #if self.totalstepstaken>=410:
        #    print("reset")
        self.resetEnvironment()
        self._observation = self._compute_observation() #you *have* to compute and return the observation from reset()
        self.episodeinitialpose=self.currentpose #contains just initial xyz poses IN INCHES
        return np.array(self._observation)
    
    def resetEnvironment(self):
         #inputstring=='home',inputstring=='end'
        
        global initialrunflag
        if initialrunflag==1:  # for first episode of training/running policy
                    
            #From Safepose, get the peg from holder, bring above target
            inputstring='fetch'
            msg=inputstring.encode('ascii')    
            sock_DC.send(msg)    
            #sleep(5)#may need to increase!!! #0.020 ) 
            initialrunflag=0
        else:
            #from pose near target, bring peg to above-target-pose, 
            #then to safepose, then to pose above holder, drop peg, regrab, bring to safepose then to target, 
            inputstring='reset'
            msg=inputstring.encode('ascii')    
            sock_DC.send(msg) 
            #sleep(5)#may need to increase!!! #0.020 ) 
            
            
        data2 = sock_DC.recv(64) #receive the "done" command
        while data2== b'':
            data2 = sock_DC.recv(64)  #48 bytes
            
            
        #print(data2, "env reset complete")   
            
        #self._seed()
        #random.seed()
    
        #self.xstart=random.uniform(self.targetx-(.5*0.0127) , self.targetx+(0.5*0.0127) ) # 0.0127 m=0.5in
        #self.ystart=random.uniform(self.targety-(.5*0.0127) , self.targety+(0.5*0.0127) )   #1*.001
        #self.zstart=0.82#0.87#0.95
    
        self.actionlist=[]
    

    def _compute_observation(self):
        
        
        #if self.totalstepstaken>=410:
        #    print("obs")
        inputstring='obs'
        msg2=inputstring.encode('ascii')    
        sock_DC.send(msg2)
        
        #print("sent",inputstring)
        if bot=='red':
            data2 = sock_DC.recv(64)#64 
            while data2== b'':
                data2 = sock_DC.recv(64)#64  #48 bytes
                #print(data1)
            #unpacked = struct.unpack('ffffffffffffffff', data2)
            unpacked = struct.unpack('ffffffffffffffff', data2)
            #if self.totalstepstaken>=410:
            #    print("unpacked data: ",unpacked)
            TransRotatmatrix=np.zeros([4,4])
            for i in range(4):
                TransRotatmatrix[i][:]=unpacked[0+(4*i):4+(4*i)]

            x_pose=TransRotatmatrix[0][3]*39.3701  #convert to inches 
            y_pose=TransRotatmatrix[1][3]*39.3701
            z_pose=TransRotatmatrix[2][3]*39.3701
            rpy=rot2rpy(TransRotatmatrix[0:3,0:3],True)
            roll=rpy[0]  #In degrees!
            pitch=rpy[1]
            yaw=rpy[2]
            self.currentpose=[x_pose,y_pose,z_pose]#In inches

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
                    AVG_FT_list[i]+=FT_list[j][i]    

                AVG_FT_list[i]=AVG_FT_list[i]/(len(FT_list))
                
        elif bot=='blue':
            
            data2 = sock_DC.recv(88)#64 
            while data2== b'':
                data2 = sock_DC.recv(88)#64  #48 bytes
                #print(data1)
            #unpacked = struct.unpack('ffffffffffffffff', data2)
            unpacked = struct.unpack('ffffffffffffffffffffff', data2)
            #if self.totalstepstaken>=410:
            #    print("unpacked data: ",unpacked)
            TransRotatmatrix=np.zeros([4,4])
            for i in range(4):
                TransRotatmatrix[i][:]=unpacked[0+(4*i):4+(4*i)]

            x_pose=TransRotatmatrix[0][3]*39.3701  #convert to inches 
            y_pose=TransRotatmatrix[1][3]*39.3701
            z_pose=TransRotatmatrix[2][3]*39.3701
            rpy=rot2rpy(TransRotatmatrix[0:3,0:3],True)
            roll=rpy[0]  #In degrees!
            pitch=rpy[1]
            yaw=rpy[2]
            self.currentpose=[x_pose,y_pose,z_pose]#In inches
            
            #For the blue robot, observation messages contain force torque data. 
            AVG_FT_list=[]
            for j in range(6):
                    AVG_FT_list.append(unpacked[16+j])
                #print("forcetorque:",forcetorque)

        #print("x",x_pose,"y",y_pose,"z",z_pose)
        #print("roll",roll,"pitch",pitch,"yaw",yaw)
        #print("Forces and Torques:", AVG_FT_list)
        
        """  NEED TO RE-SAMPLE THE VALUES AND FIND A WAY TO ZERO THE FORCES AND TORQUES!!
        self.xforcemin=-5
        self.xforcemax=5
        self.yforcemin=-5
        self.yforcemax=5
        
        self.zforcemin=-25
        self.zforcemax=0
        self.rolltorquemin=-1
        self.rolltorquemax=1
        self.pitchtorquemin=-1
        self.pitchtorquemax=1
        self.yawtorquemin=-.1
        self.yawtorquemax=.1
        """ 
        
        xforce_normalized=((AVG_FT_list[0]-self.xforcemin)/(self.xforcemax-self.xforcemin)*2)-1
        yforce_normalized=((AVG_FT_list[1]-self.yforcemin)/(self.yforcemax-self.yforcemin)*2)-1
        zforce_normalized=((AVG_FT_list[2]-self.zforcemin)/(self.zforcemax-self.zforcemin)*2)-1
        rolltorque_normalized=((AVG_FT_list[3]-self.rolltorquemin)/(self.rolltorquemax-self.rolltorquemin)*2)-1
        pitchtorque_normalized=((AVG_FT_list[4]-self.pitchtorquemin)/(self.pitchtorquemax-self.pitchtorquemin)*2)-1
        #yawtorque_normalized=((AVG_FT_list[0]-self.xforcemin)/(self.xforcemax-self.xforcemin)*2)-1
        
        #return[AVG_FT_list[0],AVG_FT_list[1],AVG_FT_list[2],
        #           AVG_FT_list[3],AVG_FT_list[4],x_pose,y_pose]
    
        return[xforce_normalized,yforce_normalized,zforce_normalized,
                   rolltorque_normalized,pitchtorque_normalized,x_pose,y_pose]
    
     
    
    def _compute_reward(self):
        #if self.totalstepstaken>=410:
        #        print("compute reward called, about to call 'obs'")
        a=self._compute_observation()
        [currentX,currentY,currentZ]=self.currentpose  #in inches 
     
        
        #Default initial Z is 0.06915259
        #half inserted peg Z is 0.044 ish
        
        [initialX,initialY,initialZ]=self.episodeinitialpose #in inches 
        bonusreward=0
        self.currentreward=0 
        

        XYdist=math.sqrt(pow(currentX-initialX,2)+pow(currentY-initialY,2))  #2D distance formula
    
 
        if XYdist>0.2 and (1.96-currentZ)<0.1 :
            self.currentreward =-1
        else: 
            #2.620986220472 initial z pose
            #2.16  z at contacting the table w/o peg/gripper sliding 
            
            self.currentreward =1.96-currentZ  #2.16 seems to be z position of just on the plastic though
            
        successZthreshold=0.79
        if initialZ-currentZ>successZthreshold: #1 inch  #DOUBLE CHECK THAT  THIS IS IN INCHES NOT METERS OR MM!!!!
            bonusreward=(1-(self._envStepCounter/self.StepsPerEpisode))+0.3
            self.doneflag=1
            
            
        print("Ep:",self.episodecounter, " tStep:", self._envStepCounter, "Z difference",(initialZ-currentZ), " Reward:",self.currentreward )
        #print("   Rewards--Base:",XYdist, " Bonus:",bonusreward," Total:", (XYdist+bonusreward) )
        if initialZ-currentZ>successZthreshold:
            self.totalsuccesscounter+=1
            print("*****Success condition achieved at tStep",self._envStepCounter,"Total Successes:",self.totalsuccesscounter, "*****")
            
        #self.currentreward = XYdist + bonusreward         
        
        #print("Ep:",self.episodecounter, " tStep:", self._envStepCounter, "InitialZ:",initialZ, "currentZ:",currentZ, "Difference",(initialZ-currentZ))
        #check for success condition, and if success, add bonus reward :)
        
        
        
        return self.currentreward 
    

    def _compute_done(self):
        #print(f"COMPUTE DONE? Episode: {self.episodecounter} step: {self._envStepCounter} Total Steps taken: {self.totalstepstaken} " )
        # - done, a boolean, value that is TRUE if the environment reached an endpoint, and should be reset, or FALSE otherwise;
        
         #system whent 'home" 39 times
        if self._envStepCounter >= self.StepsPerEpisode+1 or self.doneflag==1:    
            #print("Episode", self.episodecounter, "over. envStepCounter:", self._envStepCounter," StepsPerEpisode:" , self.StepsPerEpisode)
            #print("RESET!-env counter at max", "final reward value:",self.currentreward)

            self.rewardlist.append(self.currentreward)
            
            self.episodecounter=self.episodecounter+1
            print("   Actionlist:",self.actionlist)
            if self.episodecounter%20==0 or self.episodecounter==self.TotalEpisodes+1:
                clear_output(wait = True)   #uncomment to clear output at each reset 
                print("Ep:",self.episodecounter, "  Total Steps taken:", self.totalstepstaken)
                self.fig, (self.ax1) = plt.subplots(1,figsize=(8,8))
                self.ax1.cla() #clear axes 
                self.ax1.plot(self.rewardlist)

                plt.setp(self.ax1, xlim=(0, self.TotalEpisodes), ylim=(-0.5,0.5))

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
        #if self._envStepCounter >= self.StepsPerEpisode:
        #    print ("Episode done at step", self._envStepCounter )
        
        return  self._envStepCounter >= self.StepsPerEpisode+1 or self.doneflag==1#self.currentreward > -0.001 or
    
