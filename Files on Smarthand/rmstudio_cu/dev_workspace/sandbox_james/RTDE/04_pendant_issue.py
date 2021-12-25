#%%%%% INIT %%%%%%%%%%%%%%%%%

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from dashboard_client import DashboardClient
import time
from time import sleep
print("Initialized!")

#%%%%% CONNECT %%%%%%%%%%%%%%

robotHost = "192.168.0.101"
ctrl = RTDEControl( robotHost )
rcvr = RTDEReceive( robotHost )
dash = DashboardClient( robotHost )
ctrl.endTeachMode() # Needed before asking the robot to do things programmatically
sleep( 1.0 )
print("Instantiated!")


#%%%%% DIAGNOSTICS %%%%%%%%%%

def decode_status( status ):
    print( "Robot status Bits 0-3:" ) 
    print( "Is power on?: __________ ", (status & 1) == 1 )
    print( "Is program running?: ___ ", (status & 2) == 2 )
    print( "Is teach button pressed?:", (status & 4) == 4 )
    print( "Is power button pressed?:", (status & 8) == 8 )


def recover_position_control( wait_s = 0.25, maxIter = 10, restart = 1 ):
    """ Attempt to return to position mode after touching the pendant """
    if restart:
        conn = ctrl.isConnected()
        print( "Controller connected?:", conn )
        if not ctrl.isSteady():
            bgn = time.time()
            if 0:
                ctrl.disconnect()
                ctrl.reconnect()
            else:
                ctrl.reuploadScript()
            print( "Restart duration:", time.time()-bgn, "[s]" )
            sleep( wait_s )
            print( "Controller steady?:", ctrl.isSteady() )
    else:
        for i in range( maxIter ):
            ctrl.endTeachMode()
            sleep( wait_s )
            if ctrl.isSteady(): # Break when in pos mode
                return True
    return ctrl.isSteady()


def full_report( unteach = 1 ):
    """ Print all the relevant modes of the robot """
    if unteach:
        print( "Recovered from pendant?:", recover_position_control(), '\n' )
    print( "Controller Connected?:", ctrl.isConnected(), ", Is there a running program?:", ctrl.isProgramRunning(),
           ", Is the robot steady?:", ctrl.isSteady() )
    status = rcvr.getRobotStatus()
    mode   = rcvr.getRobotMode()
    safety = rcvr.getSafetyStatusBits()
    print( "status:", status, 
           ", safety status bits:", safety, ", Mode:", mode , "\n" )
    decode_status( status )



#%%%%% BEFORE PENDANT %%%%%%%
full_report(0)

#%%%%% ATTEMPT TO MOVE %%%%%%
# SUCCESS #
target = rcvr.getActualTCPPose()
target[2] += 0.050
#             pose  , spd , acl, async
ctrl.moveL( target, 0.25, 0.5, True )


#%%%%% PENDANT %%%%%%%%%%%%%%
print("""
USE PENDANT TO MOVE THE ROBOT
""")
sleep( 10.0 )

#%%%%% ATTEMPT TO MOVE %%%%%%
# FAILURE #
target = rcvr.getActualTCPPose()
target[2] += 0.050
#             pose  , spd , acl, async
ctrl.moveL( target, 0.25, 0.5, True )

#%%%%% AFTER TEACH END %%%%%%
full_report(1)

#%%%%% ATTEMPT TO MOVE %%%%%%
# SUCCESS #
target = rcvr.getActualTCPPose()
target[2] += 0.050
#             pose  , spd , acl, async
ctrl.moveL( target, 0.25, 0.5, True )
# %%
