
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

import torch
import torch.nn as nn
from torch.utils.data import TensorDataset, DataLoader
import torch.nn.functional as F
import torch.optim as optim

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


bot='blue'  #'red'
print('the', bot, ' robot is being used. Please change the bot identity variable if this is incorrect')
if bot=='red':
    from remote_FT_client import RemoteFTclient
    FTclient = RemoteFTclient( '192.168.0.103', 10000 )
    #print( FTclient.prxy.system.listMethods() )
    FTclient.bias_wrist_force() #Biasing wrist force
    

#HOST_DC = '192.168.0.103'
HOST_DC = '128.138.224.89' 
PORT_DC= 65480

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



#GRU MODEL-----------------------------------



def evaluate_episode(model, data,  maxdifference=0.2, verbose=False):
    device = torch.device("cpu")
    model.eval()
    inp = torch.from_numpy(np.array(data)) # should be 5x1
    h = model.init_hidden(inp.shape[0])
    #print("inp",inp)
    #print("labs",labs)
    #print("h",h)
    out, h = model(inp.to(device).float(), h)
    #print("model output",out)
    return out



#GRU model above

class UR5Env0(gym.Env):


    def __init__(self,TotalEpisodes=100,StepsPerEpisode=10,continuousactionspace=False,actualbutton=True,GRUrewards=True):
        
        device = torch.device("cpu")
        #self.gru_model=torch.load('currentmodel_10_11_2021.pt', map_location=torch.device('cpu') )  #Getting error here : python AttributeError: Can't get attribute 'GRUNet' on <module '__main__'>
        #self.gru_model=torch.load('currentmodel_3Xcopiedsuccess10_19_2021.pt', map_location=torch.device('cpu') ) #trying this for 10-22 run. 
        
        #self.gru_model=torch.load('currentmodel_fromtraineddata_10_21_2021.pt', map_location=torch.device('cpu') ) 
        #self.gru_model=torch.load('currentmodel_from_training_data_10_21_2021.pt', map_location=torch.device('cpu') ) #used on 10_21_2021 run
        #self.gru_model=torch.load('currentmodel_9steplookhead10_23_2021.pt', map_location=torch.device('cpu') ) #used on 10-22-2021 run  Adjusted input data to be 5x9!
        self.gru_model=torch.load('currentmodel_V10A_retroactiveVals_3datasets10_25_2021.pt', map_location=torch.device('cpu') ) #used on 10-25-2021 run  
        
        
        
        
        
        self.gru_model.eval() #put into eval mode
        print("GRU model loaded")

        print("TotalEpisodes:",TotalEpisodes, "   StepsPerEpisode:",StepsPerEpisode)
        self._observation = []
        
        self.continuousactionspace=continuousactionspace
        self.actualbutton=actualbutton
        self.GRUrewards=GRUrewards
        
        if self.continuousactionspace==False:
            self.action_space = spaces.Discrete(5)#Generates number between 0 and 9
        
        #9-8-2021:  Normalize the box- style action space!!  min = -1, max -1
        if self.continuousactionspace==True:
            self.action_space = spaces.Box(np.array([-1,-1]),np.array([1,1]))
        
    
        #self.action_space = spaces.Box(np.array([-0.01, -0.01, -0.01]), """ x, y, z min """
        #                                    np.array([0.01, 0.01, 0.01])) # x, y, z max
        #self.observation_space = spaces.Box(np.array([-100,-100,-100,-100,-100, -30, -30]), 
         #                                   np.array([100,100,100,100,100,30, 30])) # 5 forcetorque, xyz pose
        
        #self.observation_space = spaces.Box(np.array([-5,-5,-25,-1,-1,-8.5, -20.5]), 
        #                                    np.array([5,  5,  0, 1, 1,-6.5,-18.5])) # 5 forcetorque, xyz pose
        
        self.observation_space = spaces.Box(np.array([-1,-1,-1,-1,-1]), 
                                            np.array([1,  1, 1, 1, 1])) # 5 forcetorque, xyz pose

       # xforce_normalized,yforce_normalized,zforce_normalized, rolltorque_normalized,pitchtorque_normalized,x_pose-initialX,y_pose-initialY]

    
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
        
        self.normalized5channel= np.array([[],[],[],[],[]])

        #largest and smallest forcetorque values from both the 10-4 and 10-6 datasets. 
        self.xforceminGRU= -23.11346244812012 
        self.xforcemaxGRU= 20.62649154663086
        self.yforceminGRU= -36.84435272216797 
        self.yforcemaxGRU= 48.10685729980469
        self.zforceminGRU= -136.04910278320312 
        self.zforcemaxGRU= 10.252020835876465
        self.rolltorqueminGRU= -8.972264289855957 
        self.rolltorquemaxGRU= 6.203413009643555
        self.pitchtorqueminGRU= -6.052636623382568 
        self.pitchtorquemaxGRU= 5.1588873863220215

        
        self.fig, (self.ax1) = plt.subplots(1,figsize=(7,7)) # was 8,8

        self.curr_action_config = None 

        self.episodecounter=1
        self.episodeinitialobsflag=1
        self.rewardlist=[]
        self.totalstepstaken=0
        self.totalsuccesscounter=0
        
        self.GRUsuccesscounter=0
        self.GRUfailcounter=0
        
        self.counter_truepositive=0
        self.counter_falsenegative=0    
        self.counter_truenegative=0    
        self.counter_falsepositive=0
                
        self.headers=[]
        
        self.GRUepisodeoutputlist=[]
        
        for i in range(self.StepsPerEpisode):
            label=str(i)
            self.headers.append("header"+label)
        today = date.today()    
        todaydate = today.strftime("%m_%d_%Y")
        #self.forcetorquebuttonresultsfilename="forcetorquebuttonresults_cylinder_withbutton_train_noposeobs_GRUrewards_10-4_13-2021GRU_lookahead_pos2rewardifbuttonpress"+todaydate+'.csv'    
        #self.GRUresultsfilename="GRUresults_cylinder_withbutton_train_noposeobs_GRUrewards_10-4_13-2021GRU_lookahead_pos2rewardifbuttonpress"+todaydate+'.csv'   
        #self.rewardlistfilename="rewardlist_cylinder_withbutton_train_noposeobs_GRUrewards_10-4_13-2021GRU_lookahead_pos2rewardifbuttonpress"+todaydate+'.csv'  
        
        namedetail="_cylinder_withbutton_retroactiveVals_with1rewardifbuttonpress_"
        self.forcetorquebuttonresultsfilename="forcetorquebuttonresults"+namedetail+todaydate+'.csv'    
        self.GRUresultsfilename="GRUresults"+namedetail+todaydate+'.csv'   
        self.rewardlistfilename="rewardlist"+namedetail+todaydate+'.csv'  
        self.AllGRUresultsfilename="AllGRUresults"+namedetail+todaydate+'.csv'   
        
        
        
        with open(self.forcetorquebuttonresultsfilename, mode='w') as outputfile:
                writer = csv.writer(outputfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                writer.writerow(self.headers)
                
        with open(self.GRUresultsfilename, mode='w') as outputfile:
                writer = csv.writer(outputfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                writer.writerow(["Episode","Timestep","ButtonData","GRUOutput","TruePositiveCount","TrueNegativeCount","FalsePositiveCount","FalseNegativeCount"])
                
        with open(self.rewardlistfilename, mode='w') as outputfile:
                writer = csv.writer(outputfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                writer.writerow(["Rewardlist","ButtonData","Zposition(Inches)","Zposition(mm)"])
        

        with open(self.AllGRUresultsfilename, mode='w') as outputfile:
                writer = csv.writer(outputfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                writer.writerow(self.headers)
                
        self.resetEnvironment()
        time.sleep(0.1) 

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    
    def step(self, action):
        #print("step")
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
        #print("motionselector")
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
        #print("reset")
        self._envStepCounter=1
        self.initialaction=1
        self.doneflag=0
        #if self.totalstepstaken>=410:
        #    print("reset")
        self.resetEnvironment()
        self._observation = self._compute_observation() #you *have* to compute and return the observation from reset()
        self.episodeinitialpose=self.currentpose #contains just initial xyz poses IN INCHES
        self.normalized5channel= np.array([[],[],[],[],[]])
        
        return np.array(self._observation)
    
    def resetEnvironment(self):
        #print("resetenv")
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
        self.GRUepisodeoutputlist=[]
    def _compute_observation(self):
        
        #print("compute obs")
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
            self.AVG_FT_list=[]
            for j in range(6):
                    self.AVG_FT_list.append(unpacked[16+j])
                #print("forcetorque:",forcetorque)

        #print("x",x_pose,"y",y_pose,"z",z_pose)
        #print("roll",roll,"pitch",pitch,"yaw",yaw)
        #print("Forces and Torques:", AVG_FT_list)
        
        """  NEED TO RE-SAMPLE THE VALUES AND FIND A WAY TO ZERO THE FORCES AND TORQUES!!
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
        """ 
        #normalized forces and torques for DQN. Normalized range is between -1 and 1.

        scaledmax=1
        scaledmin=-1
      
        xforce_normalized=(((self.AVG_FT_list[0]-self.xforceminGRU)/(self.xforcemaxGRU-self.xforceminGRU))*(scaledmax-scaledmin))+scaledmin
        yforce_normalized=(((self.AVG_FT_list[1]-self.yforceminGRU)/(self.yforcemaxGRU-self.yforceminGRU))*(scaledmax-scaledmin))+scaledmin
        zforce_normalized=(((self.AVG_FT_list[2]-self.zforceminGRU)/(self.zforcemaxGRU-self.zforceminGRU))*(scaledmax-scaledmin))+scaledmin
        rolltorque_normalized=(((self.AVG_FT_list[3]-self.rolltorqueminGRU)/(self.rolltorquemaxGRU-self.rolltorqueminGRU))*(scaledmax-scaledmin))+scaledmin
        pitchtorque_normalized=(((self.AVG_FT_list[4]-self.pitchtorqueminGRU)/(self.pitchtorquemaxGRU-self.pitchtorqueminGRU))*(scaledmax-scaledmin))+scaledmin
        
        if xforce_normalized>1:
            xforce_normalized=1
        elif xforce_normalized<-1:
            xforce_normalized=-1  
            
        if yforce_normalized>1:
            yforce_normalized=1
        elif yforce_normalized<-1:
            yforce_normalized=-1  
            
        if zforce_normalized>1:
            zforce_normalized=1
        elif zforce_normalized<-1:
            zforce_normalized=-1   
            
        if rolltorque_normalized>1:
            rolltorque_normalized=1
        elif rolltorque_normalized<-1:
            rolltorque_normalized=-1  
            
        if pitchtorque_normalized>1:
            pitchtorque_normalized=1
        elif pitchtorque_normalized<-1:
            pitchtorque_normalized=-1    
            
            
            
        #yawtorque_normalized=((AVG_FT_list[0]-self.xforcemin)/(self.xforcemax-self.xforcemin)*2)-1
        
        #return[AVG_FT_list[0],AVG_FT_list[1],AVG_FT_list[2],
        #           AVG_FT_list[3],AVG_FT_list[4],x_pose,y_pose]
        
        
        if self.initialaction==1:
            self.initialaction=0 
            #return[xforce_normalized,yforce_normalized,zforce_normalized,
            #           rolltorque_normalized,pitchtorque_normalized,0,0] 
            return[xforce_normalized,yforce_normalized,zforce_normalized,
                       rolltorque_normalized,pitchtorque_normalized] 
        
            #Initial difference in pose is 0, as it is before first action
            
        elif self.initialaction==0:
            [initialX,initialY,initialZ]=self.episodeinitialpose #in inches 
            #return[xforce_normalized,yforce_normalized,zforce_normalized,
            #           rolltorque_normalized,pitchtorque_normalized,x_pose-initialX,y_pose-initialY]
            return[xforce_normalized,yforce_normalized,zforce_normalized,
                       rolltorque_normalized,pitchtorque_normalized]
            
    def _compute_reward(self):
        #print("Compute reward")
        #if self.totalstepstaken>=410:
        #        print("compute reward called, about to call 'obs'")
        rewardobs=self._compute_observation()
        
        #Accumulate forces and torques to output to csv file at the end of episode
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
        
        if self.GRUrewards==False:
            XYdist=math.sqrt(pow(currentX-initialX,2)+pow(currentY-initialY,2))  #2D distance formula from initial point
            XYdist=XYdist*25.4 #convert dist to mm
            #if peg is too far away from initial, set reward to -1
            if XYdist>6:  #in mm. This is double the random positioning
                self.currentreward =0#-2
            else: 
                self.currentreward =0# -2
                self.currentreward+=  (initialZ-currentZ)
    
        
        if self.actualbutton==True:
        #request a 0 or 1 from the arduino button   
            arduinoserial.write(b'q\n')  
            arduinobuttonstatus = arduinoserial.readline()
            self.buttonvalue=0
            if arduinobuttonstatus== b'1\r\n':
                #print("BUTTON PRESSED! Episode over!")
                self.buttonvalue=1
                self.buttonoutputlist.append(self.buttonvalue)
                
                if self.GRUrewards==False:
                    self.currentreward=2
                    
                #self.currentreward+(1-(self._envStepCounter/self.StepsPerEpisode))+0.3  #bonus reward for success, increases             #the earlier in the episode it happens. 
                
                
                #even if GRU rewards are true, still end the episode if the button is pressed!
                self.doneflag=1
                self.totalsuccesscounter+=1
                #print("*****Success condition achieved at tStep",self._envStepCounter,"Total Successes:",self.totalsuccesscounter, "*****")

            elif arduinobuttonstatus== b'0\r\n':  #If not
                self.buttonvalue=0
                self.buttonoutputlist.append(self.buttonvalue) 
            
        elif self.actualbutton==False:  #Use "virtual button" based on Z force and Z position threshholds. 
             
            self.buttonvalue=0
            Zpositionthreshhold=0.281
            if self.AVG_FT_list[2]<-6 and (currentZ/39.3701)<Zpositionthreshhold:
                print("CONTACT!","Z pose:",(currentZ/39.3701),"Z force:",self.AVG_FT_list[2])
                self.buttonvalue=1
                self.buttonoutputlist.append(self.buttonvalue)
                if self.GRUrewards==False:
                    self.currentreward=2
                self.doneflag=1
                self.totalsuccesscounter+=1
            elif (currentZ/39.3701)<Zpositionthreshhold:
                print("NO CONTACT! Z pose:",(currentZ/39.3701),"Z force:",self.AVG_FT_list[2] ," Z force above threshold of -6")    
                self.buttonvalue=0
                self.buttonoutputlist.append(self.buttonvalue) 
            elif self.AVG_FT_list[2]<-6:
                print("NO CONTACT! Z pose:",(currentZ/39.3701),"Z force:",self.AVG_FT_list[2] ,"Z position above threshold of 0.281 meters")
                self.buttonvalue=0
                self.buttonoutputlist.append(self.buttonvalue) 
            else:
                print(" NO CONTACT! Z pose:",(currentZ/39.3701),"Z force:",self.AVG_FT_list[2] ,"Z position above threshold of 0.281 meters and Z force above threshold of -6")      
                self.buttonvalue=0
                self.buttonoutputlist.append(self.buttonvalue) 
            
        #Scaling for GRU. Output of normalized range is between 0 and 1. 
        scaledmax=1
        scaledmin=0
        timestep_datasetsize=10 #was 10 was 9 for lookahead
        xforce_normalizedGRU=(((self.AVG_FT_list[0]-self.xforceminGRU)/(self.xforcemaxGRU-self.xforceminGRU))*(scaledmax-scaledmin))+scaledmin
        yforce_normalizedGRU=(((self.AVG_FT_list[1]-self.yforceminGRU)/(self.yforcemaxGRU-self.yforceminGRU))*(scaledmax-scaledmin))+scaledmin
        zforce_normalizedGRU=(((self.AVG_FT_list[2]-self.zforceminGRU)/(self.zforcemaxGRU-self.zforceminGRU))*(scaledmax-scaledmin))+scaledmin
        rolltorque_normalizedGRU=(((self.AVG_FT_list[3]-self.rolltorqueminGRU)/(self.rolltorquemaxGRU-self.rolltorqueminGRU))*(scaledmax-scaledmin))+scaledmin
        pitchtorque_normalizedGRU=(((self.AVG_FT_list[4]-self.pitchtorqueminGRU)/(self.pitchtorquemaxGRU-self.pitchtorqueminGRU))*(scaledmax-scaledmin))+scaledmin
        
        newcol=np.array([[xforce_normalizedGRU],[yforce_normalizedGRU],[zforce_normalizedGRU],[rolltorque_normalizedGRU],[pitchtorque_normalizedGRU]])
        self.normalized5channel=np.concatenate((self.normalized5channel, newcol), 1)

        if self.normalized5channel.shape[1]>timestep_datasetsize:
            self.normalized5channel= np.delete(self.normalized5channel, 0, 1) #pop earliest collumn of data

        #print("normalized5channel:")
        #print(self.normalized5channel)
        #print("normalized5channel.shape: ",self.normalized5channel.shape)
        
        if self.normalized5channel.shape[1]<timestep_datasetsize and self.GRUrewards==True:  #for first 10 timesteps just use normal rewards.
            XYdist=math.sqrt(pow(currentX-initialX,2)+pow(currentY-initialY,2))  #2D distance formula from initial point
            XYdist=XYdist*25.4 #convert dist to mm
            self.GRUepisodeoutputlist.append(0)
            if XYdist>6:  #in mm. This is double the random positioning
                self.currentreward =0#-0.5    #if peg is too far away from initial, set reward to -0.5 switched from -2 because -.5 seems to be a common 
            else: 
                self.currentreward = 0#-0.5
                self.currentreward+=  (initialZ-currentZ)
            
            
        if self.normalized5channel.shape[1]==timestep_datasetsize:
            self.normalized5channel_expandeddims=np.expand_dims(self.normalized5channel, axis=0)
            outputfull=float(evaluate_episode(self.gru_model, self.normalized5channel_expandeddims))
            self.GRUepisodeoutputlist.append(outputfull)
            #print("GRU Output",outputfull)
            if self.GRUrewards==True:
                
                
                reward_gru_output=outputfull#*2
                if reward_gru_output>1:
                    reward_gru_output=1
                if reward_gru_output<-1:
                    reward_gru_output=-1    
                self.currentreward=reward_gru_output  #range is from -2 to 2
                #initial run had starting reward of -1, and added the positive or negative GRU output to it. 
                
                
                if arduinobuttonstatus== b'1\r\n':  #if button was pressed, overwrite existing reward and make it 1. This MAY be "cheating".  need to chat with Brad and Nicholaus 10/23/2021
                    self.currentreward=1
                
            cutoff=0.7
            if self.buttonvalue==1  and outputfull>= cutoff:
                self.counter_truepositive+=1
                resultstring="00000 GRU Output Correct! 00000"
            elif self.buttonvalue==1  and outputfull< cutoff:
                self.counter_falsenegative+=1
                resultstring="XXXXX GRU Output NOT Correct! XXXXX"
                #output incorrect GRU results to file, along with episode and timestep info
                with open(self.GRUresultsfilename, mode='a') as outputfile:
                    writer = csv.writer(outputfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    writer.writerow([self.episodecounter,self._envStepCounter,self.buttonvalue,round(outputfull, 2),
                                   self.counter_truepositive ,self.counter_truenegative,self.counter_falsepositive, self.counter_falsenegative])
                
            elif self.buttonvalue==0 and outputfull<= cutoff :
                self.counter_truenegative+=1
                resultstring="00000 GRU Output Correct! 00000" 
            elif self.buttonvalue==0 and outputfull> cutoff:
                self.counter_falsepositive+=1
                resultstring="XXXXX GRU Output NOT Correct! XXXXX"
                
                #output incorrect GRU results to file, along with episode and timestep info
                with open(self.GRUresultsfilename, mode='a') as outputfile:
                    writer = csv.writer(outputfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    writer.writerow([self.episodecounter,self._envStepCounter,self.buttonvalue,round(outputfull, 2),
                                   self.counter_truepositive ,self.counter_truenegative,self.counter_falsepositive, self.counter_falsenegative])
                    
                    
        #"Episode","Timestep","AttemptNum","ButtonData","GRUOutput","TruePositiveCount","TrueNegativeCount","FalsePositiveCount","FalseNegativeCount
        else: 
            resultstring=("unfilled GRU buffer at size ")
            resultstring=resultstring+str(self.normalized5channel.shape[1])
            outputfull=0
            #print(str(resultstring))
            
        
                                 
        #print("Ep:",self.episodecounter, " tStep:", self._envStepCounter, "Z difference",(initialZ-currentZ), " Reward:",self.currentreward )
        print("Ep:",self.episodecounter, " tStep:", self._envStepCounter, " Reward:",round(self.currentreward, 2)," Button Pressed?",self.buttonvalue,
              " GRU output:",  round(outputfull, 2), "GRU_TruePosQty", self.counter_truepositive,"GRU_TrueNegQty",self.counter_truenegative ,
              "GRU_FalsePosQty",self.counter_falsepositive,"GRU_FalseNegQty",self.counter_falsenegative,"TotalQty",
              self.counter_truepositive+self.counter_truenegative+self.counter_falsepositive+self.counter_falsenegative,resultstring ) 
        return self.currentreward 
    

    def _compute_done(self):
        #print("Compute done")
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
                
            [currentX,currentY,currentZ]=self.currentpose   
            #Write to rewardslist file the "Rewardlist","ButtonData","Zposition"
            with open(self.rewardlistfilename, mode='a') as outputfile:
                    writer = csv.writer(outputfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    writer.writerow([self.currentreward,self.buttonvalue,currentZ,currentZ*25.4])
                    #"Rewardlist","ButtonData","Zposition"
                    
            with open(self.AllGRUresultsfilename, mode='a') as outputfile:
                writer = csv.writer(outputfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

                writer.writerow(self.xforcelist)
                writer.writerow(self.yforcelist)
                writer.writerow(self.zforcelist)
                writer.writerow(self.rolltorquelist)
                writer.writerow(self.pitchtorquelist)
                writer.writerow(self.buttonoutputlist)
                writer.writerow(self.GRUepisodeoutputlist)
                
                
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
        #print("done status:",(self._envStepCounter >= self.StepsPerEpisode+1 or self.doneflag==1))
        return  self._envStepCounter >= self.StepsPerEpisode+1 or self.doneflag==1#self.currentreward > -0.001 or
    
