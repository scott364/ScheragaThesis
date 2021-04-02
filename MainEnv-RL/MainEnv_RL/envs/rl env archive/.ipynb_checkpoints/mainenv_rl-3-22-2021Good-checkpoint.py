
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
from .manipulatorsMOD import SawyerMOD
from .utils import load_configuration, save_config_to_configuration_file, manual_control, create_cuboid_obstacle
from pybullet_utils import bullet_client as bc
from pybullet_utils import urdfEditor as ed
import matplotlib.pyplot as plt
from IPython.display import display, clear_output
import random

#class BalancebotEnv(gym.Env):
class MainEnvRL(gym.Env):
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
        p.setGravity(0,0,-9.81)
        p.setTimeStep(0.01) # sec
        #p.setPhysicsEngineParameter(enableFileCaching=0) # I am not sure what this does 
        #p.setPhysicsEngineParameter(numSolverIterations=100, numSubSteps=10) #numSolverIterations=100, numSubSteps=10) #  #make physics more accurate by iterating by smaller steps?
        #p.setPhysicsEngineParameter(solverResidualThreshold=1e-30)  # I am not sure what this does 
        
        p.resetDebugVisualizerCamera( cameraDistance=1.3, cameraYaw=92, cameraPitch=-37, 
                                 cameraTargetPosition=[-0.001, 0.03, 0.03])  
        #p.resetDebugVisualizerCamera( cameraDistance=1.5, cameraYaw=-30, cameraPitch=-30, cameraTargetPosition=[0.0, 0.0, 0.25]) 
        ground_plane = SimObject("Ground", "plane.urdf", [0,0,0])
        #self.maxV = 24.6 # 235RPM = 24,609142453 rad/sec    
        self._envStepCounter = 0
        path = os.path.abspath(os.path.dirname(__file__))

        table = SimObject('table', os.path.join(path, "NEWtable.urdf"),  (0.9, 0.1, .47),(1.5708*2,0,0),fixed_base=1)
        sim_obj1 = SimObject('hole1', os.path.join(path, '1.5hole.urdf'),  (0.69, 0.1, .530),(0,0,0),fixed_base=1)  #1.5708 for 90 deg rotation
        sim_obj2 = SimObject('hole2', os.path.join(path, '1.25hole.urdf'), (0.69, 0.3, .530),(0,0,0),fixed_base=1)
        sim_obj3 = SimObject('hole3', os.path.join(path, '1.15hole.urdf'), (0.69, 0.5, .530),(0,0,0),fixed_base=1)    
        sim_obj4 = SimObject('c_hole', os.path.join(path, 'C_hole.urdf'),  (0.69, -0.5, .530),(0,0,0),fixed_base=1)
        
        self.lrf_sensor1 = LaserRangeFinder(position_offset=[0.69, 0.1, .530 ],          
                                  orientation_offset=[0, -0.7068252, 0, 0.7073883 ] ,fixed_pose=False)
      
        #self.lrf_sensor.set_range(0,0.0381) #to be just inside hole1
        self.lrf_sensor1.set_range(0,0.12) 
        self.lrf_sensor1.set_debug_mode(True) 
    
        self.fig, (self.ax1) = plt.subplots(1,figsize=(8,8))
        self.rewardlist=[]
        self.episodecounter=0
        
        #print(self.sawyer_robot._arm_dof_indices) #[3, 8, 9, 10, 11, 13, 16]  #these correspond to the links
        #print(self.sawyer_robot._arm_dof_names) 
        
        self._seed()
        #self.xstart=random.uniform(self.targetx-0.0127 , self.targetx+0.0127 )
        #self.ystart=random.uniform(self.targety-0.0127 , self.targety+0.0127 )
        # 0.0127 m=0.5in
        
        self.xstart=0.715
        self.ystart=0.045
        self.zstart=0.95
        
        self.targetx=0.715 #
        self.targety=0.045 
        self.targetz=0.95
        
        
        self.xcurrent=self.xstart
        self.ycurrent=self.ystart
        self.zcurrent=self.zstart
        self.xyz = [self.xcurrent, self.ycurrent, self.zcurrent]
        self.orientation= [0,1,0,0] #quaternion hand pointing down..ish?
        
        self.sawyer_robot = SawyerMOD(robot_name="sawyer0",position=[0, 0, 0.8], fixed_base=1)
        self.sawyerID=self.sawyer_robot.get_simulator_id() #numeric code for robot
        
        self.joint_config = self.sawyer_robot.solve_inverse_kinematics(self.xyz,self.orientation)
        
        
        self.dofindex=[3, 8, 9, 10, 11, 13, 16]
        for j in range(len(self.dofindex)):
            p.resetJointState(self.sawyerID, self.dofindex[j], self.joint_config[j])  
        time.sleep(.4) 
        self.sawyer_robot.move_to_joint_pos(self.joint_config)
        time.sleep(.4)
        
        
        self.dist=-1
        
        if (render):
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1) #display env 

        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
    

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    #def _step(self, action):
    def step(self, action):
        #self._assign_throttle(action)  #for arm, increment by a small amount (Investigate killing action if Force>20)
        
        self.motionselector(action)
        #for multistep in range(100): #step forward 10 steps before observation
            #p.stepSimulation()
            
        self._observation = self._compute_observation()  
        reward = self._compute_reward()
        done = self._compute_done()

        self._envStepCounter += 1
        self.dist = self.lrf_sensor1.get_reading()
        #dist2 = self.lrf_sensor2.get_reading()

        return np.array(self._observation), reward, done, {}

    #def _reset(self):
    def reset(self):    

        #corner 1location 0.6399999999999999    0.010000000000000024    0.9

        #corner 2 location 0.78    0.12000000000000001    0.9
        
        #self.xstart=0.7
        #self.ystart=0.08
        
        self.xcurrent=random.uniform(self.targetx-0.0127 , self.targetx+0.0127 )
        self.ycurrent=random.uniform(self.targety-0.0127 , self.targety+0.0127 )
        #self.zstart=0.9
        
        #self.xcurrent=self.xstart
        #self.ycurrent=self.ystart
        self.zcurrent=self.zstart
        self.xyz = [self.xcurrent, self.ycurrent, self.zcurrent]
        
        #self.joint_config = self.sawyer_robot.solve_inverse_kinematics(self.xyz,self.orientation)
        #sawyer_robot.move_to_joint_pos(joint_config)
        

        # print(self.joint_config)
        for k in range(len(self.dofindex)):
                #p.resetJointState(self.sawyerID, dofindex[y], self.jointPositions[y]) 
                p.resetJointState(self.sawyerID, self.dofindex[k], self.joint_config[k])  
        time.sleep(.4)
        self.sawyer_robot.move_to_joint_pos(self.joint_config)
        time.sleep(.4)        
                
                 
        #self.sawyer_robot.move_to_joint_pos(joint_config)
        
        #for x in range(len(dofindex)):
                    #jointinfo=p.getJointState(robotID,int(x)) 
                #p.resetJointState(self.sawyerID, dofindex[x], self.joint_config[x]) 
                
        """
        for multistep in range(1): #step forward 10 steps before observation
            self.joint_config = self.sawyer_robot.solve_inverse_kinematics(self.xyz,self.orientation)
            #print(self.joint_config)
            self.sawyer_robot.move_to_joint_pos(self.joint_config)
            time.sleep(.7)
            #p.stepSimulation()
        """
        
        #print(self.sawyer_robot.get_current_joint_states())
        self.dist=-1
        self._envStepCounter=0
        # you *have* to compute and return the observation from reset()
   
        self._observation = self._compute_observation()
        return np.array(self._observation)

    
    def motionselector(self,action):
    
    
        # the center of 1.5 hole is roughly 0.71  0.08 OR   ~0.78 in x and y
        """
        self.xcurrent=0.69
        self.ycurrent=0.1
        self.zcurrent=0.9
        """
        #if action==0:  #take no action- might use later
        if action==0:  
            self.xcurrent+=0.01
        if action==1:  
            self.xcurrent-=0.01
        if action==2:  
            self.ycurrent+=0.01
        if action==3:  
            self.ycurrent-=0.01
            
        """ 
        if action==4:  
            self.zcurrent+=0.01
        if action==5:  
            self.zcurrent+=0.01
        """   
        self.xyz = [self.xcurrent, self.ycurrent, self.zcurrent]
        #print("action:",action, "position",self.xyz)
        self.joint_config2 = self.sawyer_robot.solve_inverse_kinematics(self.xyz,self.orientation)
        self.sawyer_robot.move_to_joint_pos(self.joint_config2)
        time.sleep(.5)
                
    def _compute_observation(self):
        
        """
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.botId)
        cubeEuler = p.getEulerFromQuaternion(cubeOrn)
        linear, angular = p.getBaseVelocity(self.botId)
        return [cubeEuler[0],angular[0],self.vt]
        """
        
        self.xdifference=self.targetx-self.xcurrent
        self.ydifference=self.targety-self.ycurrent
        self.zdifference=self.targetz-self.zcurrent

        #return [self.xdifference,self.ydifference,self.zdifference]
        return [self.xcurrent,self.ycurrent,self.zcurrent]

    def _compute_reward(self):
        #dist = self.lrf_sensor.get_reading()  #distance from laser range finder
        #print("Laser dist=:",dist)
        #return 0.1 - abs(self.vt - self.vd) * 0.005  #+ reweard for standing still  #base this on last state
        self.currentreward=0
        self.currentreward=-1*39.3701*math.sqrt(pow(self.xdifference,2) +pow(self.ydifference,2)+pow(self.zdifference,2))
        
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
        if self.currentreward > 99:
            print("RESET! - reward over limit")
        if self._envStepCounter >= 20:    
            #print("RESET!-env counter at max", "final reward value:",self.currentreward)
            print("Ep:",self.episodecounter, "Reward:",self.currentreward,"Target:",(self.targetx,self.targety,self.targetz) ," Final Position: ", self.xyz )
            self.episodecounter=self.episodecounter+1
       
            self.rewardlist.append(self.currentreward)
            
            self.ax1.cla() #clear axes 
            self.ax1.plot(self.rewardlist)
            
            plt.setp(self.ax1, xlim=(0, 200), ylim=(-2.5,0))

            display(self.fig)

            if len(self.rewardlist)>200:
                self.rewardlist.pop(0)
                
            
            clear_output(wait = True)    
            

        #print(self.currentreward,"Target:",(self.targetx,self.targety,self.targetz) ," Current Position: ",self.xyz,)    
        
        
        return self.currentreward > -0.1 or self._envStepCounter >= 20
    
    

    def _render(self, mode='human', close=False):
        pass

    
"""
def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)
"""  