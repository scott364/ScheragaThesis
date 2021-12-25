from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import time
from time import sleep
from math import degrees
import numpy as np

robotHost = "192.168.0.101"
ctrl = RTDEControl( robotHost )
rcvr = RTDEReceive( robotHost )

_RCVR_TEST = 1
_CTRL_TEST = 0

########## Receiver ################################################################################

if _RCVR_TEST:
    
    ctrl.endTeachMode() # Needed before asking the robot to do things programmatically
    sleep( 1.0 )
    
    print( "Receiver Connected?:", rcvr.isConnected(), ", with uptime", rcvr.getTimestamp(), "[s]" )

    init_q = rcvr.getActualQ()

    print( "Q:", [ degrees( q_i ) for q_i in init_q ], type( init_q ) )
    # 2021-06-14: Matches the joint angles given on the pendant

    init_x = rcvr.getActualTCPPose()

    print( "X:", init_x, type( init_x ) )
    # 2021-06-14: Does NOT match the TCP pose given on the pendant,
    #             Possibly giving the position of the wrist?

    init_t = rcvr.getJointTemperatures()
    print( "T:", init_t, type( init_t ) )

    status = rcvr.getRobotStatus()
    mode   = rcvr.getRobotMode()
    safety  = rcvr.getSafetyStatusBits()
    print( "status:", status, ", safety status bits:", safety, ", Mode:", mode )


########## Controller ##############################################################################
    
if _CTRL_TEST:
    
    ctrl.endTeachMode() # Needed before asking the robot to do things programmatically
    sleep( 1.0 )

    # Status Checks #
    print( "Controller Connected?:", ctrl.isConnected(), ", Is there a running program?:", ctrl.isProgramRunning(),
           ", Is the robot steady?:", ctrl.isSteady() )
    
    # Set TCP? #
    if 0:
        pose_0 = rcvr.getActualTCPPose()
        print( "TCP before setting:", pose_0 )
        print( "About to set the TCP ...", end = " " )
        tcpOffset = [  0.000,
                      -0.001,
                       0.276,
                       0.000, 
                       0.000, 
                       0.000, ]
        ctrl.setTcp( tcpOffset )
        sleep( 1.0 )
        pose_1 = rcvr.getActualTCPPose()
        print( "TCP after setting:", pose_1, ", Difference:", np.subtract( pose_1, pose_0 ) )
        
    # movel #
    target = rcvr.getActualTCPPose()
    target[2] += 0.150
    #             pose  , spd , acl, async
    ctrl.moveL( target, 0.25, 0.5, True )
    if 1:
        sleep( 0.2 )
        ctrl.stopL( 0.5 )
    
    