
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
    

    def __init__(self, render=True):
        self._observation = []
        #self.action_space = spaces.Discrete(9)#Generates number between 0 and 9
        self.action_space = spaces.Discrete(4)#Generates number between 0 and 9

        #for the moment, action space set to min and max x y and z values to shift at every move. Eventually, add orientation changes!
        #self.action_space = spaces.Box(np.array([-0.01, -0.01, -0.01]), """ x, y, z min """
        #                                    np.array([0.01, 0.01, 0.01])) # x, y, z max
        
        #spaces.box is used for n-dimensional continuous domains
        #self.observation_space = spaces.Box(np.array([-math.pi, -math.pi, -5]), 
        #                                    np.array([math.pi, math.pi, 5])) # pitch, gyro, com.sp.

        self.observation_space = spaces.Box(np.array([-5, -5, -5]), 
                                            np.array([5, 5, 5])) # pitch, gyro, com.sp.

        
        use_ros = False
        use_real_time = True #True
        logger = Logger()
        self.sim = Simulator(logger=logger, use_ros=use_ros, use_real_time=use_real_time) # Initialize the Simulator
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) #disable explorer and camera views 
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0) 
        p.setPhysicsEngineParameter(enableFileCaching=0)
        p.setPhysicsEngineParameter(solverResidualThreshold=1e-30)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
        p.setGravity(0,0,-9.81)
        self.fig, (self.ax1) = plt.subplots(1,figsize=(8,16)) # was 8,8
        #p.setTimeStep(0.01) # sec
        
        #p.setPhysicsEngineParameter(numSolverIterations=100, numSubSteps=10) #numSolverIterations=100, numSubSteps=10) #  #make physics more accurate by iterating by smaller steps?
        #p.setPhysicsEngineParameter(solverResidualThreshold=1e-30)  # I am not sure what this does 
        
        p.resetDebugVisualizerCamera( cameraDistance=1.3, cameraYaw=92, cameraPitch=-37, cameraTargetPosition=[-0.001, 0.03, 0.03])  
        self.curr_action_config = None #when the sim starts, there is no current target. needs to call motion selector to get a target
        self.episodecounter=0
        self.rewardlist=[]
        self.render=render
        if (self.render):
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1) #display env 

        self.resetEnvironment()

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    #def _step(self, action):
    def step(self, action):
        #self._assign_throttle(action)  #for arm, increment by a small amount (Investigate killing action if Force>20)
        self.motionselector(action)
        
        positioncheckcounter=0
        while self.sawyer_robot.check_if_at_position(self.curr_action_config, .2) is False:
            #for multistep in range(100): #step forward 10 steps before observation
            p.stepSimulation()
            #positioncheckcounter+=1
            wrist_jointinfo=p.getJointState(self.sawyerID,16) 
            wrist_forcetorque = [round(num, 3) for num in wrist_jointinfo[2]]
            
            if (#positioncheckcounter==40 or 
                    wrist_forcetorque[0]<-20 or wrist_forcetorque[0]>20
                    or wrist_forcetorque[1]<-20 or wrist_forcetorque[1]>20
                    or wrist_forcetorque[2]>2): #default z axis force is -18. limit of 2 is not a typo!
                """
                print("action stopped")
                if wrist_forcetorque[0]<-20 or wrist_forcetorque[0]>20:
                    print("x axis force limit exceeded at:", wrist_forcetorque[0])
                if wrist_forcetorque[1]<-20 or wrist_forcetorque[1]>20:
                    print("y axis force limit exceeded at:", wrist_forcetorque[1])   
                if  wrist_forcetorque[2]>2:
                    print("z axis force limit exceeded at:", wrist_forcetorque[2])    
                """    
                currentjointstate=self.sawyer_robot.get_current_joint_states()
                fk_pose=self.sawyer_robot.solve_forward_kinematics(currentjointstate[:-2])
                self.xcurrent=fk_pose[0][0][0]+.114
                self.ycurrent=fk_pose[0][0][1]-0.02
                self.zcurrent=fk_pose[0][0][2]+0.12
                
                #self.xyz = [self.xcurrent, self.ycurrent, self.zcurrent]
                #print("current location",self.xyz,"action",action, )
                #self.curr_action_config = self.sawyer_robot.solve_inverse_kinematics(self.xyz,self.orientation,target_in_local_coords=False)
                #self.sawyer_robot.move_to_joint_pos(self.curr_action_config)
                self.sawyer_robot.move_to_joint_pos(currentjointstate)
                self.dofindex=[3, 8, 9, 10, 11, 13, 16]
                for idx, joint_idx in enumerate(self.dofindex):
                    p.resetJointState(self.sawyerID, joint_idx, self.joint_config[idx])
                    p.setJointMotorControl2(self.sawyerID, joint_idx, p.POSITION_CONTROL, targetPosition=self.joint_config[idx], force=100)
                p.stepSimulation()
                # 
                break
                
             #time.sleep(0.4)



        # we can still check if we violate a torque limit etc and if that happens, we can collect an observation, reward and make sure done is now true so we move onto a new action or terminate the current policy rollout.
        #time.sleep(.3)
        self._observation = self._compute_observation()  
        reward = self._compute_reward()
        done = self._compute_done()
        #print("step:", self._envStepCounter, "action:",action, "observation",[self.xcurrent,self.ycurrent,self.zcurrent]," reward",self.currentreward )
        self._envStepCounter += 1
        
        #dist2 = self.lrf_sensor2.get_reading()
        return np.array(self._observation), reward, done, {}

    def reset(self):    
      
        p.resetSimulation(0)
        self.resetEnvironment()

        #print(self.sawyer_robot.get_current_joint_states())
        self._envStepCounter=0
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
      
        self.lrf_sensor.set_range(0,0.11)#0.11)  .0381) #to be just inside hole1
        self.lrf_sensor.set_debug_mode(True) 
    
        self.fig, (self.ax1) = plt.subplots(1,figsize=(8,8))

        #print(self.sawyer_robot._arm_dof_indices) #[3, 8, 9, 10, 11, 13, 16]  #these correspond to the links
        #print(self.sawyer_robot._arm_dof_names) 
        
        self._seed()
        
        self.targetx=0.7
        self.targety=0.085
        self.targetz= 0.789
    
        #self.xstart=random.uniform(self.targetx-(.5*0.0127) , self.targetx+(0.5*0.0127) ) # 0.0127 m=0.5in
        #self.ystart=random.uniform(self.targety-(.5*0.0127) , self.targety+(0.5*0.0127) )
        self.xstart=random.uniform(self.targetx-(1*.001) , self.targetx+(1*.001) ) # 0.0127 m=0.5in
        self.ystart=random.uniform(self.targety-(1*.001) , self.targety+(1*.001) )
        self.zstart=0.87#0.95
        
        self.xcurrent=self.xstart
        self.ycurrent=self.ystart
        self.zcurrent=self.zstart
        self.xyz = [self.xcurrent, self.ycurrent, self.zcurrent]

        self.orientation= [-0.06,1,0,0] #[0,1,0,0] #rpy2quat  in cairo_planning/geometric/transformation
        self.sawyer_robot = SawyerMOD(robot_name="sawyer0",position=[0, 0, 0.8], fixed_base=1)
        self.sawyerID=self.sawyer_robot.get_simulator_id() #numeric code for robot
        #print(self.sawyer_robot._arm_dof_indices) #[3, 8, 9, 10, 11, 13, 16]  #these correspond to the links
        #print(self.sawyer_robot._arm_dof_names)   
        
        self.joint_config = self.sawyer_robot.solve_inverse_kinematics(self.xyz,self.orientation) #,target_in_local_coords=False
        
        self.dofindex=[3, 8, 9, 10, 11, 13, 16]
        for idx, joint_idx in enumerate(self.dofindex):
            p.resetJointState(self.sawyerID, joint_idx, self.joint_config[idx])
            p.setJointMotorControl2(self.sawyerID, joint_idx, p.POSITION_CONTROL, targetPosition=self.joint_config[idx], force=100)

#         time.sleep(.4) 
#         self.sawyer_robot.move_to_joint_pos(self.joint_config)
#         time.sleep(.4)
        #print("initial config",self.joint_config)

        for x in self.dofindex:
            p.enableJointForceTorqueSensor(self.sawyerID,x,enableSensor=1)  #args: bodyID#, jointIndex,enablesensor
        self.actionlist=[]
    
    def motionselector(self,action):
    
        # the center of 1.5 hole is roughly 0.71  0.08 OR   ~0.78 in x and y
  
        #print("action: ", action)
        
        #truepose=self.sawyer_robot.get_joint_pose_in_world_frame()
        #print("Truepose",truepose[0])
        
        
        """
        if action==0:  
            self.xcurrent=round(self.xcurrent+ 0.005,3)
        if action==1:  
            self.xcurrent=round(self.xcurrent- 0.005,3)
        if action==2:  
            self.ycurrent=round(self.ycurrent+0.005,3)
        if action==3:  
            self.ycurrent=round(self.ycurrent-0.005,3)
        if action==4:  
            self.zcurrent=round(self.zcurrent+0.005,3)
        if action==5:  
            self.zcurrent=round(self.zcurrent-0.005,3)      
        """   
        if action==0:  
            self.xcurrent=round(self.xcurrent+ 0.001,3)
            self.zcurrent=round(self.zcurrent-0.001,3) 
        if action==1:  
            self.xcurrent=round(self.xcurrent- 0.001,3)
            self.zcurrent=round(self.zcurrent-0.001,3) 
        if action==2:  
            self.ycurrent=round(self.ycurrent+0.001,3)
            self.zcurrent=round(self.zcurrent-0.001,3) 
        if action==3:  
            self.ycurrent=round(self.ycurrent-0.001,3)   
            self.zcurrent=round(self.zcurrent-0.001,3) 
            
            
        self.actionlist.append(action)
      
        self.xyz = [self.xcurrent, self.ycurrent, self.zcurrent]
        #print("current location",self.xyz,"action",action, )
    
        self.curr_action_config = self.sawyer_robot.solve_inverse_kinematics(self.xyz,self.orientation,target_in_local_coords=False)
        self.sawyer_robot.move_to_joint_pos(self.curr_action_config)
        #time.sleep(.3)
        
                
    def _compute_observation(self):
        
        #truepose=self.sawyer_robot.get_joint_pose_in_world_frame()
        self.xdifference=self.targetx-self.xcurrent
        self.ydifference=self.targety-self.ycurrent
        self.zdifference=self.targetz-self.zcurrent
        
        #return [self.xdifference,self.ydifference,self.zdifference]
        return [self.xcurrent,self.ycurrent,self.zcurrent]
    
    def _compute_reward(self):
        self.dist = self.lrf_sensor.get_reading()  #distance from laser range finder
        #print("Laser dist=:",dist)
        #return 0.1 - abs(self.vt - self.vd) * 0.005  #+ reweard for standing still  #base this on last state
        
        self.currentreward=0
        #self.currentreward=-1*39.3701*math.sqrt(pow(self.xdifference,2) +pow(self.ydifference,2)+pow(self.zdifference,2))
        if self.dist==math.inf:
                self.currentreward=-10
        else:
            self.currentreward=-1*self.dist*39.3701
        
        
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
            print("Ep:",self.episodecounter,"Dist (m)",self.dist, " Reward:",self.currentreward,"Target:",(self.targetx,self.targety,self.targetz) ," Final Position: ", self.xyz )
            print("action list",self.actionlist)
            self.episodecounter=self.episodecounter+1
       
            self.rewardlist.append(self.currentreward)
            
            self.ax1.cla() #clear axes 
            self.ax1.plot(self.rewardlist)
            
            plt.setp(self.ax1, xlim=(0, 500), ylim=(-10,0))

            display(self.fig)

            if len(self.rewardlist)>500:
                self.rewardlist.pop(0)
                
            
            clear_output(wait = True)   #uncomment to clear output at each reset 
            
       

        #print(self.currentreward,"Target:",(self.targetx,self.targety,self.targetz) ," Current Position: ",self.xyz,)    
        
        
        return self.currentreward > -0.5 or self._envStepCounter >= stepsPerEpisode
    
    

    def _render(self, mode='human', close=False):
        pass


    """
    def clamp(n, minn, maxn):
        return max(min(maxn, n), minn)
    """  