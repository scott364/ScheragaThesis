########## INIT ####################################################################################
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from dashboard_client import DashboardClient
import time
from time import sleep
from math import degrees
import numpy as np



########## ROBOT CONNECTION ########################################################################
robotHost = "192.168.0.101"
ctrl = RTDEControl( robotHost )
rcvr = RTDEReceive( robotHost )
dash = DashboardClient( robotHost )
ctrl.endTeachMode() # Needed before asking the robot to do things programmatically
sleep( 1.0 )


########## TESTS ###################################################################################
F = rcvr.getActualTCPForce()
print( "UR FT estimate:", F )

V = rcvr.getActualTCPSpeed()
print( "Effector Velocity:", V )