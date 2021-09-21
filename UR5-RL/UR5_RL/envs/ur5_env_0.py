
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
import csv
import time
from datetime import date


#Arduino Button
import serial
import serial.tools.list_ports
ports = serial.tools.list_ports.comports()
#sudo chmod a+rw /dev/ttyACM0
#print(ports)
[port.manufacturer for port in ports]
def find_arduino(port=None):
    """Get the name of the port that is connected to Arduino."""
    if port is None:
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if p.manufacturer is not None and "SparkFun" in p.manufacturer:
                port = p.device
    return port
arduinoport = find_arduino()
print("port=",arduinoport)
arduinoserial = serial.Serial(arduinoport,9600)


bot='red'  #'blue'
print('the', bot, ' robot is being used. Please change the bot identity variable if this is incorrect')
if bot=='red':
    from remote_FT_client import RemoteFTclient
    FTclient = RemoteFTclient( '192.168.0.103', 10000 )
    #print( FTclient.prxy.system.listMethods() )
    FTclient.bias_wrist_force() #Biasing wrist force
    

HOST_DC = '192.168.0.103'
PORT_DC= 65482

#standard messaging method
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


class UR5Env0(gym.Env):


    def __init__(self,TotalEpisodes=100,StepsPerEpisode=10,continuousactionspace=False):
        print("TotalEpisodes:",TotalEpisodes, "   StepsPerEpisode:",StepsPerEpisode)
        self._observation = []
        
        self.continuousactionspace=continuousactionspace
        
        if self.continuousactionspace==False:
            self.action_space = spaces.Discrete(5)#Generates number between 0 and 9
        
        #9-8-2021:  Normalize the box- style action space!!  min = -1, max -1
        if self.continuousactionspace==True:
            self.action_space = spaces.Box(np.array([-1,-1]),np.array([1,1]))
        
    
        #self.action_space = spaces.Box(np.array([-0.01, -0.01, -0.01]), """ x, y, z min """
        #                                    np.array([0.01, 0.01, 0.01])) # x, y, z max
        #self.observation_space = spaces.Box(np.array([-100,-100,-100,-100,-100, -30, -30]), 
         #                                   np.array([100,100,100,100,100,30, 30])) # 5 forcetorque, xyz pose
        
        self.observation_space = spaces.Box(np.array([-5,-5,-25,-1,-1,-8.5, -20.5]), 
                                            np.array([5,  5,  0, 1, 1,-6.5,-18.5])) # 5 forcetorque, xyz pose
        
        
        self.TotalEpisodes=TotalEpisodes
        self.StepsPerEpisode=StepsPerEpisode
        
        self.xforcemin=-10
        self.xforcemax=10
        self.yforcemin=-10
        self.yforcemax=10
        
        self.zforcemin=-15
        self.zforcemax=10
        self.rolltorquemin=-2
        self.rolltorquemax=2
        self.pitchtorquemin=-2
        self.pitchtorquemax=2
        self.yawtorquemin=-.1
        self.yawtorquemax=.1
        
        self.fig, (self.ax1) = plt.subplots(1,figsize=(7,7)) # was 8,8

        self.curr_action_config = None 

        self.episodecounter=1
        self.episodeinitialobsflag=1
        self.rewardlist=[]
        self.totalstepstaken=0
        self.totalsuccesscounter=0
        
        self.headers=[]
        for i in range(self.StepsPerEpisode):
            label=str(i)
            self.headers.append("header"+label)
        today = date.today()    
        todaydate = today.strftime("%m_%d_%Y")
        self.forcetorquebuttonresultsfilename="forcetorquebuttonresults_"+todaydate+'.csv'    
        self.rewardlistfilename="rewardlist_"+todaydate+'.csv'  
        
        with open(self.forcetorquebuttonresultsfilename, mode='w') as outputfile:
                writer = csv.writer(outputfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                writer.writerow(self.headers)
                
        with open(self.rewardlistfilename, mode='w') as outputfile:
                writer = csv.writer(outputfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                writer.writerow(["Rewardlist"])
        
        
        self.resetEnvironment()
        time.sleep(0.1) 

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    
    def step(self, action):

        # we can still check if we violate a torque limit etc and if that happens, we can collect an observation, reward and make sure done is now true so we move onto a new action or terminate the current policy rollout.
        self.motionselector(action)
        self._observation = self._compute_observation()  
        reward = self._compute_reward()
        
            
        #print("step:", self._envStepCounter, "action:",action, "observation",[self.xcurrent,self.ycurrent,self.zcurrent]," reward",self.currentreward )
   
        
        self._envStepCounter += 1  #was uncommented. commenting this may break the system
        self.totalstepstaken+=1
        done = self._compute_done()
        return np.array(self._observation), reward, done, {}
    
    
    def motionselector(self,action):
        if self.continuousactionspace==False:
            #print("action",action)
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
      
        if self.continuousactionspace==True:
            inputstring='action'
            msgaction=inputstring.encode('ascii')    
            print("action",action)  #action [ 1.         -0.93657553]
            #print("action message",msgaction) #action message b'action'

            sock_DC.send(msgaction)       
            actionbyte=struct.pack('ff',action[0],action[1])
            sock_DC.send(actionbyte) 
            #self.actionlist.append(action)
            time.sleep(0.05)  

    
    def reset(self):    
        self._envStepCounter=1
        self.initialaction=1
        self.doneflag=0
        #if self.totalstepstaken>=410:
        #    print("reset")
        self.resetEnvironment()
        self._observation = self._compute_observation() #you *have* to compute and return the observation from reset()
        self.episodeinitialpose=self.currentpose #contains just initial xyz poses IN INCHES
        return np.array(self._observation)
    
    def resetEnvironment(self):
         #inputstring=='home',inputstring=='end'
        """
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
        """
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
        self.xforcelist=[]
        self.yforcelist=[]
        self.zforcelist=[]
        self.rolltorquelist=[]
        self.pitchtorquelist=[]
        self.yawtorquelist=[]
        self.buttonoutputlist=[]

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
        
        
        if self.initialaction==1:
            self.initialaction=0 
            return[xforce_normalized,yforce_normalized,zforce_normalized,
                       rolltorque_normalized,pitchtorque_normalized,0,0] 
            #Initial difference in pose is 0, as it is before first action
            
        elif self.initialaction==0:
            [initialX,initialY,initialZ]=self.episodeinitialpose #in inches 
            return[xforce_normalized,yforce_normalized,zforce_normalized,
                       rolltorque_normalized,pitchtorque_normalized,x_pose-initialX,y_pose-initialY]
    
    def _compute_reward(self):
        #if self.totalstepstaken>=410:
        #        print("compute reward called, about to call 'obs'")
        rewardobs=self._compute_observation()
        
        
        self.xforcelist.append(rewardobs[0])
        self.yforcelist.append(rewardobs[1])
        self.zforcelist.append(rewardobs[2])
        self.rolltorquelist.append(rewardobs[3])
        self.pitchtorquelist.append(rewardobs[4])
        #self.yawtorquelist.append(rewardobs[5])
        
        [currentX,currentY,currentZ]=self.currentpose  #in inches 
     
        
        #Default initial Z is 0.06915259
        #half inserted peg Z is 0.044 ish
        
        [initialX,initialY,initialZ]=self.episodeinitialpose #in inches 
        bonusreward=0
        self.currentreward=0 
        

        XYdist=math.sqrt(pow(currentX-initialX,2)+pow(currentY-initialY,2))  #2D distance formula from initial point
    
        #if peg is too far away from initial, set reward to -1
        if XYdist>0.25:  
            self.currentreward =-2
        else: 
            self.currentreward = -2
            self.currentreward+=  (initialZ-currentZ)

        #request a 0 or 1 from the arduino button   
        arduinoserial.write(b'q\n')  
        arduinobuttonstatus = arduinoserial.readline()

        if arduinobuttonstatus== b'1\r\n':
            print("BUTTON PRESSED! Episode over!")
            self.buttonoutputlist.append(1)
            
            self.currentreward=2
            #self.currentreward+(1-(self._envStepCounter/self.StepsPerEpisode))+0.3  #bonus reward for success, increases             #the earlier in the episode it happens. 
            self.doneflag=1
            self.totalsuccesscounter+=1
            print("*****Success condition achieved at tStep",self._envStepCounter,"Total Successes:",self.totalsuccesscounter, "*****")
            
        elif arduinobuttonstatus== b'0\r\n':  #If not
                self.buttonoutputlist.append(0) 
                
        print("Ep:",self.episodecounter, " tStep:", self._envStepCounter, "Z difference",(initialZ-currentZ), " Reward:",self.currentreward )
        

        return self.currentreward 
    

    def _compute_done(self):
        # - done, a boolean, value that is TRUE if the environment reached an endpoint, and should be reset, or FALSE otherwise;
        
        if self._envStepCounter >= self.StepsPerEpisode+1 or self.doneflag==1:    
            #print("Episode", self.episodecounter, "over. envStepCounter:", self._envStepCounter," StepsPerEpisode:" , self.StepsPerEpisode)
            #print("RESET!-env counter at max", "final reward value:",self.currentreward)
            
            with open(self.forcetorquebuttonresultsfilename, mode='a') as outputfile:
                writer = csv.writer(outputfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

                writer.writerow(self.xforcelist)
                writer.writerow(self.yforcelist)
                writer.writerow(self.zforcelist)
                writer.writerow(self.rolltorquelist)
                writer.writerow(self.pitchtorquelist)
                writer.writerow(self.buttonoutputlist)
                
            with open(self.rewardlistfilename, mode='a') as outputfile:
                    writer = csv.writer(outputfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    writer.writerow([self.currentreward])
                    
            self.rewardlist.append(self.currentreward)
            self.episodecounter=self.episodecounter+1
            print("   Actionlist:",self.actionlist)
            if self.episodecounter%20==0 or self.episodecounter==self.TotalEpisodes+1:
                clear_output(wait = True)   #uncomment to clear output at each reset 
                print("Ep:",self.episodecounter, "  Total Steps taken:", self.totalstepstaken)
                self.fig, (self.ax1) = plt.subplots(1,figsize=(8,8))
                self.ax1.cla() #clear axes 
                self.ax1.plot(self.rewardlist)

                plt.setp(self.ax1, xlim=(0, self.TotalEpisodes), ylim=(-2.1,2.1))

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
        print("done status:",(self._envStepCounter >= self.StepsPerEpisode+1 or self.doneflag==1))
        return  self._envStepCounter >= self.StepsPerEpisode+1 or self.doneflag==1#self.currentreward > -0.001 or
    
