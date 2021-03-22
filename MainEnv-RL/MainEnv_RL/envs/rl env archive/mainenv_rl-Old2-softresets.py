
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

from pybullet_utils import bullet_client as bc
from pybullet_utils import urdfEditor as ed
import pybullet_data
import time
import matplotlib.pyplot as plt

#https://stackoverflow.com/questions/6028000/how-to-read-a-static-file-from-inside-a-python-package
import pkgutil


from .manipulatorsMOD import SawyerMOD
from .utils import load_configuration, save_config_to_configuration_file, manual_control, create_cuboid_obstacle



#class BalancebotEnv(gym.Env):
class MainEnvRL(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second' : 50}

    def __init__(self, render=True):
        self._observation = []
        self.action_space = spaces.Discrete(9)
        self.observation_space = spaces.Box(np.array([-math.pi, -math.pi, -5]), 
                                            np.array([math.pi, math.pi, 5])) # pitch, gyro, com.sp.

        if (render):
            use_ros = False
            use_real_time = True
            logger = Logger()
            sim = Simulator(logger=logger, use_ros=use_ros, use_real_time=use_real_time) # Initialize the Simulator
            p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) #disable explorer and camera views 
            p.setGravity(0,0,-9.81)
        
            p.setTimeStep(0.01) # sec
            """
            ### Original Code
            self.physicsClient = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) #disable explorer and camera views
            ####
            """
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

            #sim_obj0 = SimObject('hole1', '1block.urdf',  (0.69, -0.2, .530),(0,0,0),fixed_base=0) 
            sim_obj1 = SimObject('hole1', os.path.join(path, '1.5hole.urdf'),  (0.69, 0.1, .530),(0,0,0),fixed_base=1)  #1.5708 for 90 deg rotation
            sim_obj2 = SimObject('hole2', os.path.join(path, '1.25hole.urdf'), (0.69, 0.3, .530),(0,0,0),fixed_base=1)
            sim_obj3 = SimObject('hole3', os.path.join(path, '1.15hole.urdf'), (0.69, 0.5, .530),(0,0,0),fixed_base=1)    
            sim_obj4 = SimObject('hole1', os.path.join(path, 'C_hole.urdf'),  (0.69, -0.5, .530),(0,0,0),fixed_base=1) 
            
            sawyer_robot = SawyerMOD(robot_name="sawyer0"
                           ,position=[0, 0, 0.8], fixed_base=1)
            robotID=sawyer_robot.get_simulator_id() #numeric code for robot
            print("robotID=",robotID)
            jointPositions = [0.5009041352157391,0.2951560129283386,-1.2254625531233645,0.8216155636392285,-1.5183997750913025,-1.096785632443942,-0.08396404586438821]
            #initialjointPositions=jointPositions
            dofindex=[3, 8, 9, 10, 11, 13, 16]
            for y in range(len(dofindex)):
                p.resetJointState(robotID, dofindex[y], jointPositions[y])

            
            cubeStartPos = [2,0,0.001]
            #cubeStartPos = [0.9,0.1,0.47]
            cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
            self.botId = p.loadURDF(os.path.join(path, "balancebot_simple.xml"),
                               cubeStartPos,
                               cubeStartOrientation)
            print("balancebot ID=",self.botId )

            #state1 = p.saveState()  #had scope issues from saving locally, as higher level gym accesses functions individually
             #p.restoreState(fileName="state.bullet")
            #file1 = open("restoreFile.txt", "w")
            #dumpStateToFile(file1)
            #file1.close()
            
        else:
            self.physicsClient = p.connect(p.DIRECT)  # non-graphical version

        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
        
        #path = os.path.abspath(os.path.dirname(__file__))
        #self.botId = p.loadURDF(os.path.join(path, "balancebot_simple.xml"),
        
        #print(tableURDF)
        
        self._seed()
        
        # paramId = p.addUserDebugParameter("My Param", 0, 100, 50)

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _step(self, action):
        self._assign_throttle(action)  #for arm, increment by a small amount (Investigate killing action if Force>20)
        p.stepSimulation()
        self._observation = self._compute_observation()  #investigate compputing a reward every 5 steps!
        reward = self._compute_reward()
        done = self._compute_done()

        self._envStepCounter += 1

        return np.array(self._observation), reward, done, {}

    def _reset(self):
        
        
        
        #p.setPhysicsEngineParameter(enableFileCaching=0)
        #p.setPhysicsEngineParameter(numSolverIterations=100, numSubSteps=10) #numSolverIterations=100, numSubSteps=10) #
        #p.setPhysicsEngineParameter(solverResidualThreshold=1e-30)  # I am not sure what this does 

        
        """
        table = SimObject('table', 'NEWtable.urdf',  (0.9, 0.1, .47),(1.5708*2,0,0),fixed_base=1)
        #print(p.getNumJoints(table.get_simulator_id())
        
        #Point the camera at the robot at the desired angle and distance
        #p.resetDebugVisualizerCamera( cameraDistance=1.5, cameraYaw=-30, cameraPitch=-30, cameraTargetPosition=[0.0, 0.0, 0.25])  


        #remember, for future objects, enable concave features in the URDF files using pybullet quckstart guide, like:
        #https://github.com/bulletphysics/bullet3/blob/master/data/samurai.urdf
        #sim_obj0 = SimObject('hole1', '1block.urdf',  (0.69, -0.2, .530),(0,0,0),fixed_base=0) 
        sim_obj1 = SimObject('hole1', '1.5hole.urdf',  (0.69, 0.1, .530),(0,0,0),fixed_base=1)  #1.5708 for 90 deg rotation
        sim_obj2 = SimObject('hole2', '1.25hole.urdf', (0.69, 0.3, .530),(0,0,0),fixed_base=1)
        sim_obj3 = SimObject('hole3', '1.15hole.urdf', (0.69, 0.5, .530),(0,0,0),fixed_base=1)    
        sim_obj4 = SimObject('hole1', 'C_hole.urdf',  (0.69, -0.5, .530),(0,0,0),fixed_base=1) 
        """
        p.removeBody(7)
        
        jointPositions = [0.5009041352157391,0.2951560129283386,-1.2254625531233645,0.8216155636392285,-1.5183997750913025,-1.096785632443942,-0.08396404586438821]
            #initialjointPositions=jointPositions
        dofindex=[3, 8, 9, 10, 11, 13, 16]
        robotID=6
        for y in range(len(dofindex)):
                p.resetJointState(robotID, dofindex[y], jointPositions[y])
        
        #########################
        # reset is called once at initialization of simulation
        #p.resetSimulation() # I am going to use save and restore state instead. Should hopefully save time for each reset. 
        #p.restoreState(fileName="state.bullet")
        
        
        
        #p.setGravity(0,0,-10) # m/s^2
        
        #planeId = p.loadURDF("plane.urdf")
        
        
        """
        #Instantiate robot at desired joint positions
        sawyer_robot = SawyerMOD(robot_name="sawyer0"
                           ,position=[0, 0, 0.8], fixed_base=1)
        robotID=sawyer_robot.get_simulator_id() #numeric code for robot
        jointPositions = [0.5009041352157391,0.2951560129283386,-1.2254625531233645,0.8216155636392285,-1.5183997750913025,-1.096785632443942,-0.08396404586438821]
        #initialjointPositions=jointPositions
        dofindex=[3, 8, 9, 10, 11, 13, 16]
        for y in range(len(dofindex)):
            p.resetJointState(robotID, dofindex[y], jointPositions[y])
        """   
            
        self.vt = 0 
        self.vd = 0 #always zero
        self.maxV = 24.6 # 235RPM = 24,609142453 rad/sec    
        cubeStartPos = [2,0,0.001]
        #cubeStartPos = [0.9,0.1,0.47]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])    
        self._envStepCounter = 0
        path = os.path.abspath(os.path.dirname(__file__))
        self.botId = p.loadURDF(os.path.join(path, "balancebot_simple.xml"),
                           cubeStartPos,
                           cubeStartOrientation)
        
        """
        cubeStartPos = [2,0,0.001]
        #cubeStartPos = [0.9,0.1,0.47]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])

        
        
      
        table = SimObject('table', os.path.join(path, "NEWtable.urdf"),  (0.9, 0.1, .47),(1.5708*2,0,0),fixed_base=1)
        
        #sim_obj0 = SimObject('hole1', '1block.urdf',  (0.69, -0.2, .530),(0,0,0),fixed_base=0) 
        sim_obj1 = SimObject('hole1', os.path.join(path, '1.5hole.urdf'),  (0.69, 0.1, .530),(0,0,0),fixed_base=1)  #1.5708 for 90 deg rotation
        #sim_obj2 = SimObject('hole2', os.path.join(path, '1.25hole.urdf'), (0.69, 0.3, .530),(0,0,0),fixed_base=1)
        #sim_obj3 = SimObject('hole3', os.path.join(path, '1.15hole.urdf'), (0.69, 0.5, .530),(0,0,0),fixed_base=1)    
        #sim_obj4 = SimObject('hole1', os.path.join(path, 'C_hole.urdf'),  (0.69, -0.5, .530),(0,0,0),fixed_base=1) 
        
        
        
        """

        # you *have* to compute and return the observation from reset()
        self._observation = self._compute_observation()
        return np.array(self._observation)

    def _assign_throttle(self, action):
        dv = 0.1
        deltav = [-10.*dv,-5.*dv, -2.*dv, -0.1*dv, 0, 0.1*dv, 2.*dv,5.*dv, 10.*dv][action]
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

    def _compute_observation(self):
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.botId)
        cubeEuler = p.getEulerFromQuaternion(cubeOrn)
        linear, angular = p.getBaseVelocity(self.botId)
        return [cubeEuler[0],angular[0],self.vt]

    def _compute_reward(self):
        return 0.1 - abs(self.vt - self.vd) * 0.005  #+ reqeard for standing still  #base this on last state

    def _compute_done(self):
        cubePos, _ = p.getBasePositionAndOrientation(self.botId)
        #return cubePos[2] < 0.15 or self._envStepCounter >= 1500
        return cubePos[2] < 0.15 or self._envStepCounter >= 1500

    def _render(self, mode='human', close=False):
        pass

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def dumpStateToFile(file):
    for i in range(p.getNumBodies()):
        pos, orn = p.getBasePositionAndOrientation(i)
        linVel, angVel = p.getBaseVelocity(i)
        txtPos = "pos=" + str(pos) + "\n"
        txtOrn = "orn=" + str(orn) + "\n"
        txtLinVel = "linVel" + str(linVel) + "\n"
        txtAngVel = "angVel" + str(angVel) + "\n"
        file.write(txtPos)
        file.write(txtOrn)
        file.write(txtLinVel)
        file.write(txtAngVel)