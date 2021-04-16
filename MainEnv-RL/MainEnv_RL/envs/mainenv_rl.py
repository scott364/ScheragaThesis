
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
from cairo_simulator.core.utils import ASSETS_PATH
from cairo_simulator.core.log import Logger
from cairo_simulator.core.link import *
from cairo_simulator.core.simulator import Simulator, SimObject
from cairo_simulator.devices.manipulators import Sawyer
from cairo_simulator.devices.sensors import LaserRangeFinder
from cairo_planning.geometric.transformation import rpy2quatV2

from .manipulatorsMOD import SawyerMOD
from .utils import load_configuration, save_config_to_configuration_file, manual_control, create_cuboid_obstacle
from pybullet_utils import bullet_client as bc
from pybullet_utils import urdfEditor as ed
import matplotlib.pyplot as plt
from IPython.display import display, clear_output
import random

#class BalancebotEnv(gym.Env):
class MainEnvRL(gym.Env):
    #metadata = {
    #   'render.modes': ['human', 'rgb_array'],
    #    'video.frames_per_second' : 50}
    
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second' : 50}
    

    def __init__(self, render=True,totalepisodes=100):
        self._observation = []
        #self.action_space = spaces.Discrete(9)#Generates number between 0 and 9
        self.action_space = spaces.Discrete(4)#Generates number between 0 and 9

      
        #self.action_space = spaces.Box(np.array([-0.01, -0.01, -0.01]), """ x, y, z min """
        #                                    np.array([0.01, 0.01, 0.01])) # x, y, z max
        

        self.observation_space = spaces.Box(np.array([-1000,-1000,-1000,-1000,-1000, -1, -1]), 
                                            np.array([1000,1000,1000,1000,1000,1, 1])) # pitch, gyro, com.sp.

        
        use_ros = False
        self.render=render
        self.totalepisodes=totalepisodes
        if render is False:
            use_real_time = False #True
        else:
            use_real_time = False #True
            
            
        logger = Logger()
        self.sim = Simulator(logger=logger, use_ros=use_ros, use_real_time=use_real_time) # Initialize the Simulator
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) #disable explorer and camera views 
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0) 
        p.setPhysicsEngineParameter(enableFileCaching=0)
        p.setPhysicsEngineParameter(solverResidualThreshold=1e-30)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
        p.setGravity(0,0,-9.81)
        self.fig, (self.ax1) = plt.subplots(1,figsize=(7,7)) # was 8,8
        #p.setTimeStep(0.01) # sec
        self.sim.set_timestep(0.01)
        
        
        #p.setPhysicsEngineParameter(numSolverIterations=100, numSubSteps=10) #numSolverIterations=100, numSubSteps=10) #  #make physics more accurate by iterating by smaller steps?
        #p.setPhysicsEngineParameter(solverResidualThreshold=1e-30)  # I am not sure what this does 
        
        p.resetDebugVisualizerCamera( cameraDistance=1.3, cameraYaw=92, cameraPitch=-37, cameraTargetPosition=[-0.001, 0.03, 0.03])  
        self.curr_action_config = None #when the sim starts, there is no current target. needs to call motion selector to get a target
        self.episodecounter=0
        self.rewardlist=[]
        #render=False
        
        if (self.render):
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1) #display env 

        self.resetEnvironment()

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    #def _step(self, action):
    def step(self, action):
        self.motionselector(action)
        #print(action)
        #p.stepSimulation()
        self.sim.step()

        xtarget=self.xcurrent
        ytarget=self.ycurrent
        ztarget=self.zcurrent
        #print("action",action,"target sent to IK:",self.xyz)
        
        currentjointstate=self.sawyer_robot.get_current_joint_states()
        current_pose=self.sawyer_robot.solve_forward_kinematics(currentjointstate[:-2])
        x_pose=current_pose[0][0][0]+self.xoffset#+0.114#+.114 #.1
        y_pose=current_pose[0][0][1]+self.yoffset#-0.02#-0.02
        z_pose=current_pose[0][0][2]+self.zoffset#+0.13#+0.13 #.1
        
        distToTarget=math.sqrt(pow((x_pose-xtarget),2) +pow((y_pose-ytarget),2)+pow((z_pose-ztarget),2))
        #print("xyz pose:",x_pose,y_pose,z_pose,"distToTarget",distToTarget)
        steptotargetcounter=0
        
        #while self.sawyer_robot.check_if_at_position(self.curr_action_config, .2) is False:  #replace this line with euclidian distance!!! make this 2mm precision
        wrist_jointinfo=p.getJointState(self.sawyerID,16) 
        wrist_forcetorque = [round(num, 3) for num in wrist_jointinfo[2]]
        
        while distToTarget>.01:
            #p.stepSimulation()
            self.sim.step()
            steptotargetcounter+=1
            
            wrist_jointinfo=p.getJointState(self.sawyerID,16) 
            wrist_forcetorque = [round(num, 3) for num in wrist_jointinfo[2]]
            
            #print("positioncheckcounter",positioncheckcounter)
            """
            if (#positioncheckcounter==40 or 
                    wrist_forcetorque[0]<-20 or wrist_forcetorque[0]>20
                    or wrist_forcetorque[1]<-20 or wrist_forcetorque[1]>20
                    or wrist_forcetorque[2]>2): #default z axis force is -18. limit of 2 is not a typo!
            """
            
            currentjointstate=self.sawyer_robot.get_current_joint_states()
            current_pose=self.sawyer_robot.solve_forward_kinematics(currentjointstate[:-2])
            x_pose=current_pose[0][0][0]+self.xoffset#+0.114#+.114 #.1
            y_pose=current_pose[0][0][1]+self.yoffset#-0.02#-0.02
            z_pose=current_pose[0][0][2]+self.zoffset#+0.13#+0.13 #.1
            
            distToTarget=math.sqrt(pow((x_pose-xtarget),2) +pow((y_pose-ytarget),2)+pow((z_pose-ztarget),2))
            #print("xyz pose:",x_pose,y_pose,z_pose,"distToTarget",distToTarget)
            
            #check action, then do opposite action. execute opposite action until we are in limit
            if steptotargetcounter==40:
                break
            if (wrist_forcetorque[2]<-20   or wrist_forcetorque[2]>20):
                break
                
            """
            #If force limit reached, reverse movement direction
            if (wrist_forcetorque[0]<-20 or wrist_forcetorque[0]>20) and action==0:
                temp_action=1
                print("force over limit. Performing action 1")
            if (wrist_forcetorque[0]<-20 or wrist_forcetorque[0]>20) and action==1:
                temp_action=0
                print("force over limit. Performing action 0")
            if (wrist_forcetorque[1]<-20 or wrist_forcetorque[1]>20) and action==2:
                temp_action=3
                print("force over limit. Performing action 3")
            if (wrist_forcetorque[1]<-20 or wrist_forcetorque[1]>20) and action==3:
                temp_action=2
                print("force over limit. Performing action 2")
            if (wrist_forcetorque[0]<-20 or wrist_forcetorque[0]>20 or
                wrist_forcetorque[1]<-20 or wrist_forcetorque[1]>20):   
            
                self.motionselector(temp_action)      
                self.sim.step()
                currentjointstate=self.sawyer_robot.get_current_joint_states()
                current_pose=self.sawyer_robot.solve_forward_kinematics(currentjointstate[:-2])
                x_pose=current_pose[0][0][0]+self.xoffset#+0.114#+.114 #.1
                y_pose=current_pose[0][0][1]+self.yoffset#-0.02#-0.02
                z_pose=current_pose[0][0][2]+self.zoffset#+0.13#+0.13 #.1
                break
            """  
            
            """   
            if (wrist_forcetorque[2]<-20 or wrist_forcetorque[2]>20):   
                self.zcurrent=round(self.zcurrent+.001,3) 
                
                self.xyz = [self.xcurrent, self.ycurrent, self.zcurrent]
                #print("current location",self.xyz,"action",action, )

                self.curr_action_config = self.sawyer_robot.solve_inverse_kinematics(self.xyz,self.orientation,target_in_local_coords=False)
                self.dofindex=[3, 8, 9, 10, 11, 13, 16]
                
                for idx, joint_idx in enumerate(self.dofindex):
                            #p.resetJointState(self.sawyerID, joint_idx, self.joint_config[idx])
                            p.setJointMotorControl2(self.sawyerID, joint_idx, p.POSITION_CONTROL, targetPosition=self.curr_action_config[idx], force=100)
                self.sim.step()
                break
            """    
                
                
                
                
                
            """
                print("action stopped")
                if wrist_forcetorque[0]<-20 or wrist_forcetorque[0]>20:
                    print("x axis force limit exceeded at:", wrist_forcetorque[0])
                if wrist_forcetorque[1]<-20 or wrist_forcetorque[1]>20:
                    print("y axis force limit exceeded at:", wrist_forcetorque[1])   
                if  wrist_forcetorque[2]>2:
                    print("z axis force limit exceeded at:", wrist_forcetorque[2])    
            """    
         

                #self.xyz = [self.xcurrent, self.ycurrent, self.zcurrent]
                #print("current location",self.xyz,"action",action, )
                #self.curr_action_config = self.sawyer_robot.solve_inverse_kinematics(self.xyz,self.orientation,target_in_local_coords=False)
                #self.sawyer_robot.move_to_joint_pos(self.curr_action_config)
                #self.sawyer_robot.move_to_joint_pos(currentjointstate)
                
        currentjointstate=self.sawyer_robot.get_current_joint_states()
        current_pose=self.sawyer_robot.solve_forward_kinematics(currentjointstate[:-2])
        """
        self.xcurrent=current_pose[0][0][0]+.114
        self.ycurrent=current_pose[0][0][1]-0.02
        self.zcurrent=current_pose[0][0][2]+0.13
        """       
                
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
    
        # the center of 1.5 hole is roughly 0.71  0.08 OR   ~0.78 in x and y
        #truepose=self.sawyer_robot.get_joint_pose_in_world_frame()
        #print("Truepose",truepose[0]
        movedist=0.001    #0.001
        
        """  
        if action==0:  
            self.xcurrent=round(self.xcurrent+ movedist,3)
        if action==1:  
            self.xcurrent=round(self.xcurrent- movedist,3)
        if action==2:  
            self.ycurrent=round(self.ycurrent+movedist,3)
        if action==3:  
            self.ycurrent=round(self.ycurrent-movedist,3)
        if action==4:  
            self.zcurrent=round(self.zcurrent+movedist,3)
        if action==5:  
            self.zcurrent=round(self.zcurrent-movedist,3)      
        """   
        
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
      
        self.xyz = [self.xcurrent, self.ycurrent, self.zcurrent]
        #print("current location",self.xyz,"action",action, )
    
        self.curr_action_config = self.sawyer_robot.solve_inverse_kinematics(self.xyz,self.orientation,target_in_local_coords=False)
        self.dofindex=[3, 8, 9, 10, 11, 13, 16]
        """"""
        for idx, joint_idx in enumerate(self.dofindex):
                    #p.resetJointState(self.sawyerID, joint_idx, self.joint_config[idx])
                    p.setJointMotorControl2(self.sawyerID, joint_idx, p.POSITION_CONTROL, targetPosition=self.curr_action_config[idx], force=100)
        
        #self.sawyer_robot.move_to_joint_pos(self.curr_action_config)
        #time.sleep(.3)
        
                
    def reset(self):    
        self._envStepCounter=0
        p.resetSimulation(0)
        self.resetEnvironment()

        #print(self.sawyer_robot.get_current_joint_states())
        
        #print("stepcounter=0")

        self._observation = self._compute_observation() #you *have* to compute and return the observation from reset()
        return np.array(self._observation)
    
    def resetEnvironment(self):
    
        ground_plane = SimObject("Ground", "plane.urdf", [0,0,0])
        #self.maxV = 24.6 # 235RPM = 24,609142453 rad/sec    
        self._envStepCounter = 0
        path = os.path.abspath(os.path.dirname(__file__))

        table = SimObject('table', os.path.join(path, "NEWtable.urdf"),  (0.9, 0.1, .47),(1.5708*2,0,0),fixed_base=1)
        
        sim_obj1 = SimObject('hole1', os.path.join(path, '1.5hole.urdf'),  (0.69, 0.1, .530),(0,0,0),fixed_base=1)  #1.5708 for 90 deg rotation
        #sim_obj2 = SimObject('hole2', os.path.join(path, '1.25hole.urdf'), (0.69, 0.3, .530),(0,0,0),fixed_base=1)
        #sim_obj3 = SimObject('hole3', os.path.join(path, '1.15hole.urdf'), (0.69, 0.5, .530),(0,0,0),fixed_base=1)    
        #sim_obj4 = SimObject('c_hole', os.path.join(path, 'C_hole.urdf'),  (0.69, -0.5, .530),(0,0,0),fixed_base=1)
        
        self.lrf_sensor = LaserRangeFinder(position_offset=[0.69, 0.1, .530 ],          
                                  orientation_offset=[0, -0.7068252, 0, 0.7073883 ] ,fixed_pose=False)
        
        #for world frame shift testing
        #testblock = SimObject('testblock', os.path.join(path, '1.25hole.urdf'), (0.69, 0.3, .570),(0,0,0),fixed_base=1)
        #self.testblockID=testblock.get_simulator_id()
        #p.removeBody(sim_obj4ID)
      
        self.lrf_sensor.set_range(0,0.15)#0.11)  .0381) #to be just inside hole1
        self.lrf_sensor.set_debug_mode(True) 
    
        

        #print(self.sawyer_robot._arm_dof_indices) #[3, 8, 9, 10, 11, 13, 16]  #these correspond to the links
        #print(self.sawyer_robot._arm_dof_names) 
        
        self._seed()
        random.seed()
        self.targetx=0.69#0.7
        self.targety=0.096#0.085
        #self.targetz=0.84# 0.789
    
        self.xstart=random.uniform(self.targetx-(.5*0.0127) , self.targetx+(0.5*0.0127) ) # 0.0127 m=0.5in
        self.ystart=random.uniform(self.targety-(.5*0.0127) , self.targety+(0.5*0.0127) )   #1*.001
        #self.xstart=self.targetx
        #self.ystart=self.targety
        
        #print("start location",self.xstart,self.ystart)
        self.zstart=0.82#0.87#0.95
        
        self.xcurrent=self.xstart
        self.ycurrent=self.ystart
        self.zcurrent=self.zstart
        self.xyz = [self.xcurrent, self.ycurrent, self.zcurrent]

        self.orientation= [0,1,0,0]#[-0.06,1,0,0] #[0,1,0,0] #rpy2quat  in cairo_planning/geometric/transformation
        self.sawyer_robot = SawyerMOD(robot_name="sawyer0",position=[0, 0, 0.8], fixed_base=1)
        self.sawyerID=self.sawyer_robot.get_simulator_id() #numeric code for robot
        #print(self.sawyer_robot._arm_dof_indices) #[3, 8, 9, 10, 11, 13, 16]  #these correspond to the links
        #print(self.sawyer_robot._arm_dof_names)   
        
        self.joint_config = self.sawyer_robot.solve_inverse_kinematics(self.xyz,self.orientation) #,target_in_local_coords=False
        
        self.dofindex=[3, 8, 9, 10, 11, 13, 16]
        for idx, joint_idx in enumerate(self.dofindex):
            p.resetJointState(self.sawyerID, joint_idx, self.joint_config[idx])
            p.setJointMotorControl2(self.sawyerID, joint_idx, p.POSITION_CONTROL, targetPosition=self.joint_config[idx], force=100)
            
        currentjointstate=self.sawyer_robot.get_current_joint_states()
        current_pose=self.sawyer_robot.solve_forward_kinematics(currentjointstate[:-2])
        x_pose=current_pose[0][0][0]#+0.114#+.114 #.1
        y_pose=current_pose[0][0][1]#-0.02#-0.02
        z_pose=current_pose[0][0][2]#+0.13#+0.13 #.1
        
        self.xoffset=self.xcurrent-x_pose
        self.yoffset=self.ycurrent-y_pose
        self.zoffset=self.zcurrent-z_pose
        
        #print("target passed to IK:", self.xyz)
        #print("starting xyz pose from FK:",x_pose,y_pose,z_pose)
        #print("offsets",self.xoffset,self.yoffset,self.zoffset)
        x_pose=current_pose[0][0][0]+self.xoffset#+0.114#+.114 #.1
        y_pose=current_pose[0][0][1]+self.yoffset#-0.02#-0.02
        z_pose=current_pose[0][0][2]+self.zoffset#+0.13#+0.13 #.1
        #print("corrected starting xyz pose from FK:",x_pose,y_pose,z_pose)
        
        
#         time.sleep(.4) 
#         self.sawyer_robot.move_to_joint_pos(self.joint_config)
#         time.sleep(.4)
        #print("initial config",self.joint_config)

        for x in self.dofindex:
            p.enableJointForceTorqueSensor(self.sawyerID,x,enableSensor=1)  #args: bodyID#, jointIndex,enablesensor
        self.actionlist=[]
    

    def _compute_observation(self):
        
        #truepose=self.sawyer_robot.get_joint_pose_in_world_frame()
        #self.xdifference=self.targetx-self.xcurrent
        #self.ydifference=self.targety-self.ycurrent
        #self.zdifference=self.targetz-self.zcurrent
        
        #return [self.xdifference,self.ydifference,self.zdifference]
        
        currentjointstate=self.sawyer_robot.get_current_joint_states()
        current_pose=self.sawyer_robot.solve_forward_kinematics(currentjointstate[:-2])

        x_pose=current_pose[0][0][0] +self.xoffset#+0.114#+.114 #.1
        y_pose=current_pose[0][0][1]+self.yoffset#-0.02#-0.02
        z_pose=current_pose[0][0][2]+self.zoffset#+0.13#+0.13 #.1
        #print("obs",[x_pose,y_pose,z_pose])
    
        #return[x_pose,y_pose,z_pose]
    
        wrist_jointinfo=p.getJointState(self.sawyerID,16) 
        wrist_forcetorque = [round(num, 3) for num in wrist_jointinfo[2]]
        #print(wrist_forcetorque)
        return[wrist_forcetorque[0],wrist_forcetorque[1],wrist_forcetorque[2],wrist_forcetorque[3],wrist_forcetorque[4],x_pose,y_pose]
    
        #return [self.xcurrent,self.ycurrent,self.zcurrent]
    
    def _compute_reward(self):
        self.dist = self.lrf_sensor.get_reading()  #distance from laser range finder
        #print("Laser dist=:",dist)
        #return 0.1 - abs(self.vt - self.vd) * 0.005  #+ reweard for standing still  #base this on last state
        
        self.currentreward=0
        """
        self.currentreward=-1*39.3701*math.sqrt(pow(self.xdifference,2) +pow(self.ydifference,2)+pow(self.zdifference,2))
        """
        
        #give reward if if z=0.82 or less
        
        currentjointstate=self.sawyer_robot.get_current_joint_states()
        current_pose=self.sawyer_robot.solve_forward_kinematics(currentjointstate[:-2])

        #x_pose=current_pose[0][0][0] +self.xoffset#+0.114#+.114 #.1
        #y_pose=current_pose[0][0][1]+self.yoffset#-0.02#-0.02
        z_pose=current_pose[0][0][2]+self.zoffset
        
        #if z_pose>0.82:
        #        self.currentreward=-1
       
        #else:
        
        if self.dist==math.inf:
                self.currentreward=-1
        else:
            self.currentreward=-1*(z_pose-0.794)*39.3701  #-1* inches off of table
        
            
        #self.currentreward=-1*(z_pose-0.794)*39.3701  #-1* inches off of table
        #print("z_pose",z_pose)
        """
        if self.dist==math.inf:
                self.currentreward=-10
        else:
            self.currentreward=-1*self.dist*39.3701
        """
        
        #self.currentreward=(self.currentreward/5)+1.2
        #print("reward",self.currentreward)
        #print("current xyz",[self.xcurrent,self.ycurrent,self.zcurrent],"goal:",[self.targetx,self.targety,self.targetz])
        #print("observation (xyz diff to goal)",self._observation,"reward",self.currentreward)
        #print("    ")
        
        #if self.dist>0 and self.dist<90:
            #self.sim.logger.info("Detector Pos: %s \t\t\t Det: %f" % (self.lrf_sensor1._position_offset, self.dist))
            #print("laser sensor contact at " ,self.xyz)  
            #f = open("contact_locations.txt", "a")
            #f.write(str(self.xyz))
            #f.close()
            #self.currentreward=self.currentreward+2  #bonus for hitting target dead-on    
            
        return self.currentreward  #negative magnitude. hopefully system tries to reduce this. 
    

    def _compute_done(self):
        #cubePos, _ = p.getBasePositionAndOrientation(self.botId)
        #return cubePos[2] < 0.15 or self._envStepCounter >= 1500  #default
        stepsPerEpisode=80 #did 20 before
        #if self.currentreward > -0.5:
        #    print("RESET! - reward over limit")
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

        #print(self.currentreward,"Target:",(self.targetx,self.targety,self.targetz) ," Current Position: ",self.xyz,)    
        
        return  self._envStepCounter >= stepsPerEpisode #self.currentreward > -0.001 or
    
    

    def _render(self, mode='human', close=False):
        pass


    """
    def clamp(n, minn, maxn):
        return max(min(maxn, n), minn)
    """  