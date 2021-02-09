import time
import os
import sys
import pybullet as p
from cairo_simulator.core.utils import ASSETS_PATH
from cairo_simulator.core.log import Logger
from cairo_simulator.core.link import *
from cairo_simulator.core.simulator import Simulator, SimObject
from cairo_simulator.devices.manipulators import *
from manipulatorsMOD import SawyerMOD
from utils import load_configuration, save_config_to_configuration_file, manual_control, create_cuboid_obstacle
from pybullet_utils import bullet_client as bc
from pybullet_utils import urdfEditor as ed
import pybullet_data
import time

use_ros = False
use_real_time = True
logger = Logger()
sim = Simulator(logger=logger, use_ros=use_ros, use_real_time=use_real_time) # Initialize the Simulator
p.setGravity(0,0,-9.81)
p.setPhysicsEngineParameter(enableFileCaching=0)
ground_plane = SimObject("Ground", "plane.urdf", [0,0,0])
table = SimObject('table', 'NEWtable.urdf',  (0.9, 0.1, .47),(1.5708*2,0,0),fixed_base=1)

p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) #disable explorer and camera views
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)


"""
#dofindex=[3, 8, 9, 10, 11, 13, 16]
[4, 9, 10, 11, 12, 14, 17]
#['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
"""


# URDF Combine V2--------------------------------
#https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_utils/examples/combineUrdf.py
p0 = bc.BulletClient(connection_mode=p.DIRECT)
p0.setAdditionalSearchPath(pybullet_data.getDataPath())
p1 = bc.BulletClient(connection_mode=p.DIRECT)
p1.setAdditionalSearchPath(pybullet_data.getDataPath())

#husky = p1.loadURDF("1block.urdf", flags=p0.URDF_USE_IMPLICIT_CYLINDER)
#husky = p1.loadURDF("C_hole.urdf", flags=p0.URDF_USE_IMPLICIT_CYLINDER)

husky = p1.loadURDF(ASSETS_PATH +'sawyer_description/urdf/sawyer_static_classic.urdf', flags=p0.URDF_USE_IMPLICIT_CYLINDER)

kuka = p0.loadURDF("1block.urdf")
ed0 = ed.UrdfEditor()
ed0.initializeFromBulletBody(husky, p1._client)
ed1 = ed.UrdfEditor()
ed1.initializeFromBulletBody(kuka, p0._client)

parentLinkIndex = 18
jointPivotXYZInParent = [0, 0, 0] 
jointPivotRPYInParent = [0, 0, 0]
jointPivotXYZInChild = [0, -0.15, 0] 
jointPivotRPYInChild = [0, 0, 0]

newjoint = ed0.joinUrdf(ed1, parentLinkIndex, jointPivotXYZInParent, jointPivotRPYInParent,
                        jointPivotXYZInChild, jointPivotRPYInChild, p0._client, p1._client)
newjoint.joint_type = p0.JOINT_FIXED

#ed0.saveUrdf(ASSETS_PATH +'sawyer_description/urdf/combined5.urdf')
#sawyer_robot = SawyerMOD(robot_name="sawyer0",position=[0, 0, 0.8], fixed_base=1)

#p1.loadURDF(ASSETS_PATH +'sawyer_description/urdf/sawyer_static_classic.urdf', flags=p0.URDF_USE_IMPLICIT_CYLINDER)
#p1.loadURDF(ASSETS_PATH +'sawyer_description/urdf/combined5.urdf', flags=p0.URDF_USE_IMPLICIT_CYLINDER)


#print(sawyer_robot._arm_dof_indices) # #these correspond to the links
#print(sawyer_robot._arm_dof_names) #['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']

"""

sawyer_robot = SawyerMOD(robot_name="sawyer0"
                       ,position=[0, 0, 0.8], fixed_base=1)
"""

orn = [0, 0, 0, 1]


ed0.createMultiBody(basePosition=[0, 0, .8], baseOrientation=orn)
"""
husky3 = SawyerMOD(robot_name="sawyer0", 
                         urdf_file='combined5.urdf'
                       ,position=[0, 0, 0.8], fixed_base=1)

"""

# URDF Combine V2--------------------------------

#manual_control(sawyer_robot)    
try:
    while True:
        sim.step()
        
        
except KeyboardInterrupt:
    p.disconnect()
    sys.exit(0)