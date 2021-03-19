
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
import pybullet_data
import time
import matplotlib.pyplot as plt

#class BalancebotEnv(gym.Env):
class MainEnvRL(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second' : 50}

    def __init__(self, render=True):
        self._observation = []
        #self.action_space = spaces.Discrete(9)#Generates number between 0 and 9
        self.action_space = spaces.Discrete(3)#Generates number between 0 and 9
        
        
        #for the moment, action space set to min and max x y and z values to shift at every move. Eventually, add orientation changes!
        #self.action_space = spaces.Box(np.array([-0.01, -0.01, -0.01]), """ x, y, z min """
        #                                    np.array([0.01, 0.01, 0.01])) # x, y, z max
        
        #spaces.box is used for n-dimensional continuous domains
        #self.observation_space = spaces.Box(np.array([-math.pi, -math.pi, -5]), 
        #                                    np.array([math.pi, math.pi, 5])) # pitch, gyro, com.sp.
        
        self.observation_space = spaces.Box(np.array([-5, -5, -5]), 
                                            np.array([5, 5, 5])) # pitch, gyro, com.sp.
        
        
        use_ros = False
        use_real_time = True
        logger = Logger()
        sim = Simulator(logger=logger, use_ros=use_ros, use_real_time=use_real_time) # Initialize the Simulator
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
        self.vt = 0 
        self.vd = 0 #always zero
        self.maxV = 24.6 # 235RPM = 24,609142453 rad/sec    
        self._envStepCounter = 0
        path = os.path.abspath(os.path.dirname(__file__))

        table = SimObject('table', os.path.join(path, "NEWtable.urdf"),  (0.9, 0.1, .47),(1.5708*2,0,0),fixed_base=1)
        sim_obj1 = SimObject('hole1', os.path.join(path, '1.5hole.urdf'),  (0.69, 0.1, .530),(0,0,0),fixed_base=1)  #1.5708 for 90 deg rotation
        sim_obj2 = SimObject('hole2', os.path.join(path, '1.25hole.urdf'), (0.69, 0.3, .530),(0,0,0),fixed_base=1)
        sim_obj3 = SimObject('hole3', os.path.join(path, '1.15hole.urdf'), (0.69, 0.5, .530),(0,0,0),fixed_base=1)    
        sim_obj4 = SimObject('hole1', os.path.join(path, 'C_hole.urdf'),  (0.69, -0.5, .530),(0,0,0),fixed_base=1)
        
        self.lrf_sensor = LaserRangeFinder(position_offset=[0.69, 0.1, .530-(0.03)],
                                  orientation_offset=[0, -0.7068252, 0, 0.7073883 ] ,fixed_pose=False)
        self.lrf_sensor.set_range(0,0.0381) 
        self.lrf_sensor.set_debug_mode(True) 
        
        self.sawyer_robot = SawyerMOD(robot_name="sawyer0"
                       ,position=[0, 0, 0.8], fixed_base=1)
        self.sawyerID=self.sawyer_robot.get_simulator_id() #numeric code for robot
        
        #print("robotID=",self.sawyerID)
        
        #easier
        #jointPositions = [0.5009041352157391,0.2951560129283386,-1.2254625531233645,0.8216155636392285,-1.5183997750913025,-1.096785632443942,
        #                  -0.08396404586438821]
        
        #harder
        #jointPositions = [0.501636986629667, 0.29287604252770827,-1.2204323492623623, 0.8217620498244532,-1.5237524820023234,-1.0948021953074452,-0.08258911534820294]
        jointPositions =  [0.48186312289557237,0.21968516477708186,-1.0929915771839849,1.0268559328345195,-1.5765081889008141,-1.0299460541948502,-0.29512687917163566]
        
        dofindex=[3, 8, 9, 10, 11, 13, 16]
        for y in range(len(dofindex)):
            p.resetJointState(self.sawyerID, dofindex[y], jointPositions[y])

        """
        cubeStartPos = [2,0,0.001]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
        self.segwayID = p.loadURDF(os.path.join(path, "balancebot_simple.xml"),
                           cubeStartPos,
                           cubeStartOrientation)
        #print("balancebot ID=",self.segwayID )
        """    
        if (render):
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1) #display env 

        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
    
        self._seed()
        self.xcurrent=0.63
        self.ycurrent=0.02
        self.zcurrent=0.9
        
        self.targetx=0.71 
        self.targety=0.09
        self.targetz=0.9

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    #def _step(self, action):
    def step(self, action):
        #self._assign_throttle(action)  #for arm, increment by a small amount (Investigate killing action if Force>20)
        
        self.motionselector(action)
        for multistep in range(1): #step forward 10 steps before observation
            p.stepSimulation()
            time.sleep(.4)
        self._observation = self._compute_observation()  
        reward = self._compute_reward()
        done = self._compute_done()

        self._envStepCounter += 1

        return np.array(self._observation), reward, done, {}

    #def _reset(self):
    def reset(self):    
        #p.removeBody(self.segwayID)
        #easier
        #jointPositions = [0.5009041352157391,0.2951560129283386,-1.2254625531233645,0.8216155636392285,-1.5183997750913025,-1.096785632443942,-0.08396404586438821]
        #harder
        jointPositions =  [0.48186312289557237,0.21968516477708186,-1.0929915771839849,1.0268559328345195,-1.5765081889008141,-1.0299460541948502,-0.29512687917163566]
        
        dofindex=[3, 8, 9, 10, 11, 13, 16]
        for y in range(len(dofindex)):
                p.resetJointState(self.sawyerID, dofindex[y], jointPositions[y]) 
                
        """      
        self.vt = 0 
        self.vd = 0 #always zero
        self.maxV = 24.6 # 235RPM = 24,609142453 rad/sec    
        cubeStartPos = [2,0,0.001]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])    
        self._envStepCounter = 0
        path = os.path.abspath(os.path.dirname(__file__))
        self.botId = p.loadURDF(os.path.join(path, "balancebot_simple.xml"),
                           cubeStartPos,
                           cubeStartOrientation)
        """  
        
        self._envStepCounter=0
        # you *have* to compute and return the observation from reset()
        self._observation = self._compute_observation()
        
        
        # the center of 1.5 hole is at roughly 0.71  0.08 OR   ~0.78 in x and y
        self.a=0
        self.b=1
        self.c=0
        self.d=0
        self.orientation= [self.a,self.b,self.c,self.d] #quaternion hand pointing down..ish?
        self.xcurrent=0.63
        self.ycurrent=0.02
        self.zcurrent=0.9
        self.xyz = [self.xcurrent, self.ycurrent, self.zcurrent]
        
        joint_config = self.sawyer_robot.solve_inverse_kinematics(self.xyz,self.orientation)
        self.sawyer_robot.move_to_joint_pos(joint_config)
        
        return np.array(self._observation)
    """
    def _assign_throttle(self, action):
        dv = 0.1
        #print("action",action)
        deltav = [-10.*dv,-5.*dv, -2.*dv, 
                  -0.1*dv, 0, 0.1*dv, 
                  2.*dv,5.*dv, 10.*dv][action]
        vt = clamp(self.vt + deltav, -self.maxV, self.maxV)
        self.vt = vt

        p.setJointMotorControl2(bodyUniqueId=self.botId, 
                                jointIndex=0, 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocity=vt)
        p.setJointMotorControl2(bodyUniqueId=self.botId, 
                                jointIndex=1, 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocity=-vt)
    """
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

        joint_config = self.sawyer_robot.solve_inverse_kinematics(self.xyz,self.orientation)
        self.sawyer_robot.move_to_joint_pos(joint_config)
                
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
        
        
        
        return [self.xdifference,self.ydifference,self.zdifference]
        

    def _compute_reward(self):
        #dist = self.lrf_sensor.get_reading()  #distance from laser range finder
        #print("Laser dist=:",dist)
        #return 0.1 - abs(self.vt - self.vd) * 0.005  #+ reweard for standing still  #base this on last state
        self.currentreward=-1*math.sqrt(pow(self.xdifference,2)
                                         +pow(self.ydifference,2)
                                         +pow(self.zdifference,2))
        
        #print("current xyz",[self.xcurrent,self.ycurrent,self.zcurrent],"goal:",[self.targetx,self.targety,self.targetz])
        #print("observation (xyz diff to goal)",self._observation,"reward",self.currentreward)
        #print("    ")
        return self.currentreward  #negative magnitude. hopefully system tries to reduce this. 
    

    def _compute_done(self):
        #cubePos, _ = p.getBasePositionAndOrientation(self.botId)
        #return cubePos[2] < 0.15 or self._envStepCounter >= 1500  #default
        if self.currentreward > 99:
            print("RESET! - reward over limit")
        if self._envStepCounter >= 20:    
            #print("RESET!-env counter at max", "final reward value:",self.currentreward)
            print(self.currentreward)
        return self.currentreward > 99 or self._envStepCounter >= 20
    
    

    def _render(self, mode='human', close=False):
        pass

    
"""
def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)
"""  