
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
        self.render=render
        
        use_ros = False
        use_real_time = True #True
        logger = Logger()
        self.sim = Simulator(logger=logger, use_ros=use_ros, use_real_time=use_real_time) # Initialize the Simulator
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) #disable explorer and camera views 
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0) 
        p.setPhysicsEngineParameter(enableFileCaching=0)
        p.setPhysicsEngineParameter(solverResidualThreshold=1e-30)
        
        p.setGravity(0,0,-9.81)
        self.fig, (self.ax1) = plt.subplots(1,figsize=(8,8))
        #p.setTimeStep(0.01) # sec
        
        #p.setPhysicsEngineParameter(numSolverIterations=100, numSubSteps=10) #numSolverIterations=100, numSubSteps=10) #  #make physics more accurate by iterating by smaller steps?
        #p.setPhysicsEngineParameter(solverResidualThreshold=1e-30)  # I am not sure what this does 
        
        p.resetDebugVisualizerCamera( cameraDistance=1.3, cameraYaw=92, cameraPitch=-37, cameraTargetPosition=[-0.001, 0.03, 0.03])  
        self.curr_action_config = None #when the sim starts, there is no current target. needs to call motion selector to get a target
        self.episodecounter=0
        """
        ground_plane = SimObject("Ground", "plane.urdf", [0,0,0])
        #self.maxV = 24.6 # 235RPM = 24,609142453 rad/sec    
        self._envStepCounter = 0
        path = os.path.abspath(os.path.dirname(__file__))

        table = SimObject('table', os.path.join(path, "NEWtable.urdf"),  (0.9, 0.1, .47),(1.5708*2,0,0),fixed_base=1)
        
        sim_obj1 = SimObject('hole1', os.path.join(path, '1.5hole.urdf'),  (0.69, 0.1, .530),(0,0,0),fixed_base=1)  #1.5708 for 90 deg rotation
        sim_obj2 = SimObject('hole2', os.path.join(path, '1.25hole.urdf'), (0.69, 0.3, .530),(0,0,0),fixed_base=1)
        sim_obj3 = SimObject('hole3', os.path.join(path, '1.15hole.urdf'), (0.69, 0.5, .530),(0,0,0),fixed_base=1)    
        sim_obj4 = SimObject('c_hole', os.path.join(path, 'C_hole.urdf'),  (0.69, -0.5, .530),(0,0,0),fixed_base=1)
        
        #sim_obj4ID=sim_obj4.get_simulator_id()
        #p.removeBody(sim_obj4ID)
        
        self.lrf_sensor1 = LaserRangeFinder(position_offset=[0.69, 0.1, .530 ],          
                                  orientation_offset=[0, -0.7068252, 0, 0.7073883 ] ,fixed_pose=False)
        
        #for world frame shift testing
        #testblock = SimObject('testblock', os.path.join(path, '1.25hole.urdf'), (0.69, 0.3, .570),(0,0,0),fixed_base=1)
        #self.testblockID=testblock.get_simulator_id()
        #p.removeBody(sim_obj4ID)
        
      
      
        #self.lrf_sensor.set_range(0,0.0381) #to be just inside hole1
        self.lrf_sensor1.set_range(0,0.11) 
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
        
        self.xstart=0.7#0.715
        self.ystart=0.08#0.035
        self.zstart=0.87#0.95
        
        self.targetx=0.7
        self.targety=0.08
        self.targetz=0.87
        
        
        self.xcurrent=self.xstart
        self.ycurrent=self.ystart
        self.zcurrent=self.zstart
        self.xyz = [self.xcurrent, self.ycurrent, self.zcurrent]
        #self.orientation= [0,1,0,0] #quaternion hand pointing down..ish?
        
        
        #rpy2quat  in cairo_planning/geometric/transformation
        
        self.orientation= [-0.06,1,0,0] #[0,1,0,0]
        #self.orientation=(0.012822343810135861, 0.9994838704682953, 0.018943326830012097, 0.022555055786720374)
        #self.orientation= [rpy2quatV2([0,0,0],degrees=True)]
        #print("orientation",self.orientation)
        
        self.sawyer_robot = SawyerMOD(robot_name="sawyer0",position=[0, 0, 0.8], fixed_base=1)
        self.sawyerID=self.sawyer_robot.get_simulator_id() #numeric code for robot
        #print(self.sawyer_robot._arm_dof_indices) #[3, 8, 9, 10, 11, 13, 16]  #these correspond to the links
        #print(self.sawyer_robot._arm_dof_names)   
        
        self.joint_config = self.sawyer_robot.solve_inverse_kinematics(self.xyz,self.orientation) #,target_in_local_coords=False
        #print(self.joint_config)
        
        self.dofindex=[3, 8, 9, 10, 11, 13, 16]
        for j in range(len(self.dofindex)):
            p.resetJointState(self.sawyerID, self.dofindex[j], self.joint_config[j])  
        time.sleep(.4) 
        self.sawyer_robot.move_to_joint_pos(self.joint_config)
        time.sleep(.4)
        print("initial config",self.joint_config)
        
       
        
        #self.target_truepose=self.sawyer_robot.get_joint_pose_in_world_frame()
        #print("world frame pose", self.target_truepose)
      
        
        self.joint_config = self.sawyer_robot.solve_inverse_kinematics(a[0],self.orientation) #,target_in_local_coords=False
        for j in range(len(self.dofindex)):
            p.resetJointState(self.sawyerID, self.dofindex[j], self.joint_config[j])  
        time.sleep(.4) 
        self.sawyer_robot.move_to_joint_pos(self.joint_config)
        time.sleep(.4)
        
        
        #print("offset to arm pos =", xyz[0]-a[0][0],xyz[1]-a[0][1],xyz[2]-a[0][2])

        # print("offset from arm pos to target =", 0.69-a[0][0],0.1-a[0][1],0.53-a[0][2]
              
        #"Correct" offset values: target-world frame pose      
        #0.012052132397680437 0.0003320223226237623 -0.2784160673552567
              
              
        for x in self.dofindex:
            p.enableJointForceTorqueSensor(self.sawyerID,x,enableSensor=1)  #args: bodyID#, jointIndex,enablesensor
    
        
        self.dist=-1
        
        if (self.render):
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1) #display env 

        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
        """
        self.resetEnvironment()

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    #def _step(self, action):
    def step(self, action):
        #self._assign_throttle(action)  #for arm, increment by a small amount (Investigate killing action if Force>20)
        self.motionselector(action)
#         while self.sawyer_robot.check_if_at_position(self.curr_action_config, .2) is False:
#             #for multistep in range(100): #step forward 10 steps before observation
#             #.stepSimulation()
#             time.sleep(0.4)
            # we can still check if we violate a torque limit etc and if that happens, we can collect an observation, reward and make sure done is now true so we move onto a new action or terminate the current policy rollout.
        time.sleep(.3)
        self._observation = self._compute_observation()  
        reward = self._compute_reward()
        done = self._compute_done()
        #print("step:", self._envStepCounter, "action:",action, "observation",[self.xcurrent,self.ycurrent,self.zcurrent]," reward",self.currentreward )
        self._envStepCounter += 1
        self.dist = self.lrf_sensor1.get_reading()
        #dist2 = self.lrf_sensor2.get_reading()
        return np.array(self._observation), reward, done, {}

        
        
    

    #def _reset(self):
    def reset(self):    
        
        """
        #world frame does not appear to shift after each reset
        p.removeBody(self.testblockID)
        path = os.path.abspath(os.path.dirname(__file__))
        testblock = SimObject('testblock', os.path.join(path, '1.25hole.urdf'), (0.69, 0.3, .570),(0,0,0),fixed_base=1)
        self.testblockID=testblock.get_simulator_id()
        """
        
        
        #corner 1location 0.6399999999999999    0.010000000000000024    0.9
        #corner 2 location 0.78    0.12000000000000001    0.9

        
        #self.xcurrent=random.uniform(self.targetx-0.0254, self.targetx+0.0254)
        #self.ycurrent=random.uniform(self.targety-0.0254, self.targety+0.0254 )
        #self.zstart=0.9
        
        
        
        
        
        """
        #p.removeBody(self.sawyerID)
        
        
        self.sawyer_robot = SawyerMOD(robot_name="sawyer0",position=[0, 0, 0.8], fixed_base=1)
        self.sawyerID=self.sawyer_robot.get_simulator_id() #numeric code for robot
  

        self.xstart=0.7#0.715
        self.ystart=0.08#0.035
        self.zstart=0.9#0.95
        
        
        self.xcurrent=self.xstart
        self.ycurrent=self.ystart
        self.zcurrent=self.zstart
        self.xyz = [self.xcurrent, self.ycurrent, self.zcurrent]
        
        self.orientation= [-0.06,1,0,0] #[0,1,0,0]
        
        self.joint_config = self.sawyer_robot.solve_inverse_kinematics(self.xyz,self.orientation) #,target_in_local_coords=False
        self.dofindex=[3, 8, 9, 10, 11, 13, 16]
        for j in range(len(self.dofindex)):
            p.resetJointState(self.sawyerID, self.dofindex[j], self.joint_config[j])  
        time.sleep(.4) 
        self.sawyer_robot.move_to_joint_pos(self.joint_config)
        time.sleep(.4)
        
        """
        #resetjointPositions =  [0.48186312289557237,0.21968516477708186,-1.0929915771839849,1.0268559328345195,-1.5765081889008141,-1.0299460541948502,-0.29512687917163566]
        
        
        #resetjointPositions =  [0.4271999066525195, 0.33274770695878236, -1.1222757800134693,
        #                        0.7002948907548097, -1.6505034723525929, -1.0668082127861727, -0.1398996263640491]
        
        
        #print("joints",range(p.getNumJoints(robotID)))     
        #gripperdof=list(sawyer_robot._gripper_dof_indices)
        #fingerPositions=list(sawyer_robot.get_gripper_pct_finger_positions(1))  
        """
        for y in range(len(self.dofindex)):
            p.resetJointState(self.sawyerID, self.dofindex[y], resetjointPositions[y])
        self.target_truepose=self.sawyer_robot.get_joint_pose_in_world_frame()
        print("------")
        print("world frame pose after true reset", self.target_truepose)
        """
        
        
        """
        self.xcurrent=0.7#0.715
        self.ycurrent=0.08#0.08#0.035
        self.zcurrent=0.87 #0.95
        #self.xyz = [self.xcurrent, self.ycurrent, self.zcurrent]
        self.xyz = [0.7, 0.08, 0.87] #[0.7, 0.08, 0.87] from true reset
        
        #print("Reset!  xyz:",self.xyz, "orientation:",self.orientation)
        self.orientation= [-0.06,1,0,0]
        self.joint_config = self.sawyer_robot.solve_inverse_kinematics(self.xyz,self.orientation) #,target_in_local_coords=False

        # print(self.joint_config)
        #print(self.joint_config)
        self.sawyer_robot.move_to_joint_pos(self.joint_config)
        time.sleep(2)   
        self.target_truepose=self.sawyer_robot.get_joint_pose_in_world_frame()
        print("world frame pose after reset to 0.7,0.8,0.87", self.target_truepose)
        
        
        for k in range(len(self.dofindex)):
                #p.resetJointState(self.sawyerID, dofindex[y], self.jointPositions[y]) 
                p.resetJointState(self.sawyerID, self.dofindex[k], self.joint_config[k])  
        

        print("joint config",self.joint_config)
        """
        p.resetSimulation(0)
        self.resetEnvironment()

        #print(self.sawyer_robot.get_current_joint_states())
        #self.dist=-1
        self._envStepCounter=0
        #print("stepcounter=0")
        
        # you *have* to compute and return the observation from reset()
   
        self._observation = self._compute_observation()
        return np.array(self._observation)
    
    def resetEnvironment(self):
    
        ground_plane = SimObject("Ground", "plane.urdf", [0,0,0])
        #self.maxV = 24.6 # 235RPM = 24,609142453 rad/sec    
        self._envStepCounter = 0
        path = os.path.abspath(os.path.dirname(__file__))

        table = SimObject('table', os.path.join(path, "NEWtable.urdf"),  (0.9, 0.1, .47),(1.5708*2,0,0),fixed_base=1)
        
        sim_obj1 = SimObject('hole1', os.path.join(path, '1.5hole.urdf'),  (0.69, 0.1, .530),(0,0,0),fixed_base=1)  #1.5708 for 90 deg rotation
        sim_obj2 = SimObject('hole2', os.path.join(path, '1.25hole.urdf'), (0.69, 0.3, .530),(0,0,0),fixed_base=1)
        sim_obj3 = SimObject('hole3', os.path.join(path, '1.15hole.urdf'), (0.69, 0.5, .530),(0,0,0),fixed_base=1)    
        sim_obj4 = SimObject('c_hole', os.path.join(path, 'C_hole.urdf'),  (0.69, -0.5, .530),(0,0,0),fixed_base=1)
        
        #sim_obj4ID=sim_obj4.get_simulator_id()
        #p.removeBody(sim_obj4ID)
        
        self.lrf_sensor1 = LaserRangeFinder(position_offset=[0.69, 0.1, .530 ],          
                                  orientation_offset=[0, -0.7068252, 0, 0.7073883 ] ,fixed_pose=False)
        
        #for world frame shift testing
        #testblock = SimObject('testblock', os.path.join(path, '1.25hole.urdf'), (0.69, 0.3, .570),(0,0,0),fixed_base=1)
        #self.testblockID=testblock.get_simulator_id()
        #p.removeBody(sim_obj4ID)
        
      
      
        #self.lrf_sensor.set_range(0,0.0381) #to be just inside hole1
        self.lrf_sensor1.set_range(0,0.11) 
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
        
        self.xstart=0.7#0.715
        self.ystart=0.08#0.035
        self.zstart=0.87#0.95
        
        self.targetx=0.7
        self.targety=0.08
        self.targetz=0.87
        
        
        self.xcurrent=self.xstart
        self.ycurrent=self.ystart
        self.zcurrent=self.zstart
        self.xyz = [self.xcurrent, self.ycurrent, self.zcurrent]
        #self.orientation= [0,1,0,0] #quaternion hand pointing down..ish?
        
        
        #rpy2quat  in cairo_planning/geometric/transformation
        
        self.orientation= [-0.06,1,0,0] #[0,1,0,0]
        #self.orientation=(0.012822343810135861, 0.9994838704682953, 0.018943326830012097, 0.022555055786720374)
        #self.orientation= [rpy2quatV2([0,0,0],degrees=True)]
        #print("orientation",self.orientation)
        
        self.sawyer_robot = SawyerMOD(robot_name="sawyer0",position=[0, 0, 0.8], fixed_base=1)
        self.sawyerID=self.sawyer_robot.get_simulator_id() #numeric code for robot
        #print(self.sawyer_robot._arm_dof_indices) #[3, 8, 9, 10, 11, 13, 16]  #these correspond to the links
        #print(self.sawyer_robot._arm_dof_names)   
        
        self.joint_config = self.sawyer_robot.solve_inverse_kinematics(self.xyz,self.orientation) #,target_in_local_coords=False
        #print(self.joint_config)
        
        self.dofindex=[3, 8, 9, 10, 11, 13, 16]
        for idx, joint_idx in enumerate(self.dofindex):
            p.resetJointState(self.sawyerID, joint_idx, self.joint_config[idx])
            p.setJointMotorControl2(self.sawyerID, joint_idx, p.POSITION_CONTROL, targetPosition=self.joint_config[idx], force=100)

#         time.sleep(.4) 
#         self.sawyer_robot.move_to_joint_pos(self.joint_config)
#         time.sleep(.4)
        #print("initial config",self.joint_config)
        
        #self.target_truepose=self.sawyer_robot.get_joint_pose_in_world_frame()
        #print("world frame pose", self.target_truepose)

              
        for x in self.dofindex:
            p.enableJointForceTorqueSensor(self.sawyerID,x,enableSensor=1)  #args: bodyID#, jointIndex,enablesensor
    
        
        self.dist=-1
        
        if (self.render):
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1) #display env 

        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
    
    def motionselector(self,action):
    
    
        # the center of 1.5 hole is roughly 0.71  0.08 OR   ~0.78 in x and y
        """
        self.xcurrent=0.69
        self.ycurrent=0.1
        self.zcurrent=0.9
        """
        #print("action: ", action)
        
        #truepose=self.sawyer_robot.get_joint_pose_in_world_frame()
        #print("Truepose",truepose[0])
        #self.xcurrent=truepose[0][0]+0.05
        #self.ycurrent=truepose[0][1]-0.024
        self.zcurrent=0.87
        #if action==0:  #take no action- might use later
        
        if action==0:  
            self.xcurrent=round(self.xcurrent+ 0.01,3)
        if action==1:  
            self.xcurrent=round(self.xcurrent- 0.01,3)
        if action==2:  
            self.ycurrent=round(self.ycurrent+0.01,3)
        if action==3:  
            self.ycurrent=round(self.ycurrent-0.01,3)
            
        """ 
        if action==4:  
            self.zcurrent+=0.01
        if action==5:  
            self.zcurrent+=0.01
        """   
        self.xyz = [self.xcurrent, self.ycurrent, self.zcurrent]
        #print("current location",self.xyz,"action",action, )
    
        self.curr_action_config = self.sawyer_robot.solve_inverse_kinematics(self.xyz,self.orientation,target_in_local_coords=False)
        self.sawyer_robot.move_to_joint_pos(self.curr_action_config)
        #time.sleep(.3)
        
                
    def _compute_observation(self):
        
        """
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.botId)
        cubeEuler = p.getEulerFromQuaternion(cubeOrn)
        linear, angular = p.getBaseVelocity(self.botId)
        return [cubeEuler[0],angular[0],self.vt]
        """
        #truepose=self.sawyer_robot.get_joint_pose_in_world_frame()
        """"""
        self.xdifference=self.targetx-self.xcurrent
        self.ydifference=self.targety-self.ycurrent
        self.zdifference=self.targetz-self.zcurrent
        
        """
        self.xdifference=self.target_truepose[0][0]-truepose[0][0]
        self.ydifference=self.target_truepose[0][1]-truepose[0][1]
        self.zdifference=self.target_truepose[0][2]-truepose[0][2]
        """
        
        #return [self.xdifference,self.ydifference,self.zdifference]
        return [self.xcurrent,self.ycurrent,self.zcurrent]
        #return [truepose[0][0],truepose[0][1],truepose[0][2]]
    

    def _compute_reward(self):
        #dist = self.lrf_sensor.get_reading()  #distance from laser range finder
        #print("Laser dist=:",dist)
        #return 0.1 - abs(self.vt - self.vd) * 0.005  #+ reweard for standing still  #base this on last state
        self.currentreward=0
        self.currentreward=-1*39.3701*math.sqrt(pow(self.xdifference,2) +pow(self.ydifference,2)+pow(self.zdifference,2))
        
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
        stepsPerEpisode=20 #did 20 before
        if self.currentreward > 99:
            print("RESET! - reward over limit")
        if self._envStepCounter >= stepsPerEpisode:    
            #print("RESET!-env counter at max", "final reward value:",self.currentreward)
            print("Ep:",self.episodecounter, "Reward:",self.currentreward,"Target:",(self.targetx,self.targety,self.targetz) ," Final Position: ", self.xyz )
            self.episodecounter=self.episodecounter+1
       
            self.rewardlist.append(self.currentreward)
            
            self.ax1.cla() #clear axes 
            self.ax1.plot(self.rewardlist)
            
            plt.setp(self.ax1, xlim=(0, 200), ylim=(-10,0))

            display(self.fig)

            if len(self.rewardlist)>200:
                self.rewardlist.pop(0)
                
            
            clear_output(wait = True)   #uncomment to clear output at each reset 
            
       

        #print(self.currentreward,"Target:",(self.targetx,self.targety,self.targetz) ," Current Position: ",self.xyz,)    
        
        
        return self.currentreward > 100 or self._envStepCounter >= stepsPerEpisode
    
    

    def _render(self, mode='human', close=False):
        pass


    """
    def clamp(n, minn, maxn):
        return max(min(maxn, n), minn)
    """  