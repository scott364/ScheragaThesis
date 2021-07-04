"""
UR_RTDE_wrapper.py
James Watson, 2021-06 for Robotic Materials Inc.
Thin wrapper over `ur_rtde` by University of South Denmark, https://gitlab.com/sdurobotics/ur_rtde
"""

########## INIT ####################################################################################

##### Imports ###################################

## Standard ##
import time, math
from time import sleep
from math import degrees
import numpy as np

## RTDE ##
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from dashboard_client import DashboardClient

## Local ##
from rmlib.rmtools import pmath
from arm_utils import RunUntilAnyT



########## UTILITY FUNCTIONS ######################################################################


def decode_status( status ):
    print( "Robot status Bits 0-3:" ) 
    print( "Is power on?: __________ ", (status & 1) == 1 )
    print( "Is program running?: ___ ", (status & 2) == 2 )
    print( "Is teach button pressed?:", (status & 4) == 4 )
    print( "Is power button pressed?:", (status & 8) == 8 )


    
########## RTDEwrapper #############################################################################


class RTDEwrapper:
    """ Thin wrapper over `ur_rtde` by University of South Denmark, https://gitlab.com/sdurobotics/ur_rtde """
    
    
    def __init__( self , initDict , _ ):
        """ Create Controller and Receiver for 2-way comms between Jupyter and UR5 """
        
        ### Control Setup ###
        self._DISLIKE_DASH = 1 # Set this to T to minimize connection time with the dashboard,
        #                        interacting with the dash seems to cause Ubuntu to have random "internal errors"
        self._DISLIKE_SCRIPT = 1 # Similar story
        
        # Control Params #
        self.robot_arm_host           = initDict['ip_address'] # IP address of the robot on the subnet "192.168.0.101"
        self.arm_max_linear_speed     = initDict['max_linear_speed']
        self.arm_max_linear_accel     = initDict['max_linear_accel']
        self.arm_max_joint_speed      = initDict['max_joint_speed']
        self.arm_max_joint_accel      = initDict['max_joint_accel']
        self.arm_default_linear_speed = initDict['default_linear_speed']
        self.arm_default_linear_accel = initDict['default_linear_accel']
        self.arm_default_joint_speed  = initDict['default_joint_speed']
        self.arm_default_joint_accel  = initDict['default_joint_accel']
        # Connection Params #
        self.pause      = 0.005 # Pause time after disconnecting from the pendant
        self.loopPause  = 0.050 # Pause time before checking the force condition again
        self.loopHz     = 1.0 / self.loopPause
        self.T_slack    = 5.0 # - Slack time between programmatic commands when we assume the user touched the pendant
        self.lastProg   = 0.0 # - Last time a programmatic command was sent
        self.f_dashConn = 0 # --- Flag for whether this object is connected to the dashboard or not
        self.f_scrpConn = 0
        # Init Connection Objects #
        self.ctrl = RTDEControl( self.robot_arm_host ) # --- Controller
        self.rcvr = RTDEReceive( self.robot_arm_host ) # --- Receiver
        self.dash = DashboardClient( self.robot_arm_host ) # Dashboard
        if not self._DISLIKE_DASH:
            self.activate_dashboard(1)
        if not self._DISLIKE_SCRIPT:
            self.activate_script_client(1)
        self.runner = None
        
        ### Convenient Variables ###
        
        self.common_poses = {
            'home' : [0, math.radians(-90), 0, math.radians(-90), 0, 0],
            '-y' : [math.radians(-90), math.radians(-90), math.radians(-90), math.radians(-90), math.radians(90), 0],
            '+x' : [0, math.radians(-90), math.radians(-90), math.radians(-90), math.radians(90), 0],
            '+y' : [math.radians(90), math.radians(-90), math.radians(-90), math.radians(-90), math.radians(90), 0],
            '-x' : [math.radians(180), math.radians(-90), math.radians(-90), math.radians(-90), math.radians(90), 0],
            'cv_1' : [-1.4595659414874,-1.8198393026935,-0.9309876600848,-1.9611166159259,1.570393443107,0.1136082112789],
            'cv_2' : [1.796921849250, -1.744428459797, -0.720430199299, -2.24652368227, 1.570549249649, 0.2302896231412],
        }
        
    
    ##### Communication #########################
    
    
    def full_report( unteach = 1 ):
        """ Print all the relevant modes of the robot """
        print( "Controller Connected?:", self.ctrl.isConnected() ) 
        print( "Is the robot steady?: ", self.ctrl.isSteady()    )
        print( "Mode: _______________ ", self.rcvr.getRobotMode(), "\n" )
        decode_status( self.rcvr.getRobotStatus() )
    
    
    def recover_position_control( self, N_rerty = 5 ):
        """ Restore position control in case the pendant has been touched """
        # 0. Get current time
        now = time.time()
        # 1. Only disable if it has been a while since the last programmatic command
        if (now - self.lastProg) > self.T_slack:
            if not self.ctrl.isSteady():
                for i in range( N_rerty ):
                    self.ctrl.disconnect()
                    self.ctrl.reconnect()
                    sleep( self.pause )
                    if self.ctrl.isSteady():
                        break
            else:
                self.ctrl.endTeachMode()
                sleep( self.pause )
            self.lastProg = now
            return 1
        else:
            self.lastProg = now
            return 0
    
    
    def kill_rtde( self ):
        """ Disconnect from all clients, 2021-06-14: Legacy function """
        self.ctrl.disconnect()
        self.rcvr.disconnect()
        self.activate_dashboard(0)
    
    
    def revive_rtde( self ):
        """ Re-establish all broken connections """
        revived = 0
        if not self.ctrl.isConnected():
            self.ctrl.reconnect()
            revived = 1
        if not self.rcvr.isConnected():
            self.rcvr.reconnect()
            revived = 1
        if (not self._DISLIKE_DASH) and (not self.dash.isConnected()):
            self.activate_dashboard(1)
            revived = 1
        if revived:
            sleep( self.pause )
        return revived
    
    
    def hard_restart( self ):
        """ Shutdown and reconnect """
        self.kill_rtde()
        sleep( self.pause )
        self.revive_rtde()
            

    ##### Receiver Facilities ###################


    def get_status_bits( self ):
        """ Get robot status (bits?), 2021-06-14: Legacy function """
        self.recover_position_control()
        return bin( self.rcvr.getRobotStatus() )
    
    
    def get_safety_status_bits( self ):
        """ Get safety status bits, 2021-06-14: Legacy function """
        self.recover_position_control()
        return bin( self.rcvr.getSafetyStatusBits() )
    
    
    def get_tcp_pose_vec( self ):
        """ Gives the tcp position of the effector as a list: [x, y, z, rX, rY, rZ] """
        self.recover_position_control()
        return self.rcvr.getActualTCPPose()
    
    
    def get_tcp_pose( self ):
        """ Gives the tcp position of the effector as a homogeneous coordinate """
        return pmath.pose_vec_to_mtrx( self.get_tcp_pose_vec() )
    
    
    def get_tcp_force( self ):
        """ Gets the 6-axis force-torque magnitudes on the tcp as a list: [nx, ny, nz, rx, ry, rz] """
        self.recover_position_control()
        return self.rcvr.getActualTCPForce()
    
    
    def get_joint_angles( self ):
        """ Get the radian angles of each joint as a list: [base, shoulder, elbow, wrist_1, wrist_2, wrist_3] """
        self.recover_position_control()
        return self.rcvr.getActualQ()
    
    
    def get_joint_degrees( self ):
        """ Get the radian angles of each joint as a list: [base, shoulder, elbow, wrist_1, wrist_2, wrist_3] """
        return [ degrees( q_i ) for q_i in self.get_joint_angles() ]
        
    
    def enable_freedrive( self ):
        """ Remotely enable freedrive, 2021-06-14: Legacy function """
        self.ctrl.teachMode()
        sleep( self.pause )
        return True


    def disable_freedrive( self ):
        """ Remotely disable freedrive, 2021-06-14: Legacy function """
        self.ctrl.endTeachMode() # Needed before asking the robot to do things programmatically
        sleep( self.pause )
        return True
    
    
    ##### Controller Facilities #################


    def stop_move( self ):
        """ Stop all motion (Decelerate in task space) """
        self.ctrl.stopL( self.arm_max_linear_accel )
        
        
    def wait_until_robot_is_finished( self, stop_condition = 'dummy' ):
        """ Block until the status bits indicate the robot is not moving, or the stop condition is met """
        if stop_condition == 'dummy':
            stop_condition = self.dummy_stop
        # Wait for robot to START moving
        time.sleep( 0.1 )
#         while self.get_status_bits() == bin(3): # 2021-06-15: Unk problem with this condition!
        # Wait for robot to STOP moving
        while np.linalg.norm( self.rcvr.getActualTCPSpeed() ) > 0.001:
            if stop_condition():
                self.stop_move()
                break
            else:
                sleep( self.loopPause )
                
                
    def threaded_conditional_stop( self, stop_condition = 'dummy' ):
        """ Check the stop condition in a background thread until the robot has stopped moving """
        if stop_condition == 'dummy':
            stop_condition = self.dummy_stop
            
        def end_cond():
            nonlocal self
            return np.linalg.norm( self.rcvr.getActualTCPSpeed() ) <= 0.001
        
        def stop_cb():
            nonlocal self
            self.stop_move()
            
        sched = RunUntilAnyT( [stop_condition, end_cond], stop_cb, self.loopHz )
        time.sleep( 0.1 ) # Wait for the robot to start moving
        sched.run()

    
    def move_speed( self, target, move_type = 'j', speed = None, accel = None,
                    radius = 0, stop_condition = 'dummy', blocking = True ):
        """ Execute a motion to the given `target` using the `move_type` and dynamic params """
        self.recover_position_control()
        if move_type == 'j':
            if speed is None: speed = self.arm_default_joint_speed
            if accel is None: accel = self.arm_default_joint_accel
        elif move_type in ['l', 'p']:
            if speed is None: speed = self.arm_default_linear_speed
            if accel is None: accel = self.arm_default_linear_accel
        else:
            raise ValueError( 'move_speed: Move Type Not Supported:', move_type, " not in ('j','l','p')" )
        # Limit speed to max value
        if speed > self.arm_max_joint_speed:
            speed = self.arm_max_joint_speed
        # Limit acceleration to max value
        if accel > self.arm_max_joint_accel:
            accel = self.arm_max_joint_accel
        # Handle homogeneous coordinates
        if type( target ) == np.ndarray and target.shape == (4, 4):
            target = pmath.pose_mtrx_to_vec( target )
            
        if move_type == 'j':
            #           pose  , speed, accel, async
            self.ctrl.moveJ( target, speed, accel, True )
        elif move_type == 'l':
            #           pose  , speed, accel, async
            self.ctrl.moveL( target, speed, accel, True )
        elif move_type == 'p':
            #           pose  , speed, accel, async
            self.ctrl.moveP( target, speed, accel, True )
        else:
            raise RuntimeError( 'move_speed: THIS BRANCH UNREACHABLE:', move_type, " not in ('j','l','p')" )
        
        if blocking:
            self.wait_until_robot_is_finished( stop_condition = stop_condition )
        elif stop_condition != 'dummy':
            self.threaded_conditional_stop( stop_condition )
        # else non-blocking with no stop condition, do nothing
        
        return True
    
    
    def move_timed( self, target, move_type = 'j', time = 10, radius = 0, stop_condition = 'dummy', blocking = True ):
        raise NotImplementedError( "!!! FIXME: `move_timed` has NOT BEEN IMPLEMENTED !!!" )
    
    
    def move( self, target, move_type = 'j', speed_per = None, accel_per = None, radius = 0,
              stop_condition = 'dummy', blocking = True ):
        if move_type in ['j']:
            max_speed     = self.arm_max_joint_speed
            default_speed = self.arm_default_joint_speed
            max_accel     = self.arm_max_joint_accel
            default_accel = self.arm_default_joint_accel
        elif move_type in ['l', 'p']:
            max_speed     = self.arm_max_linear_speed
            default_speed = self.arm_default_linear_speed
            max_accel     = self.arm_max_linear_accel
            default_accel = self.arm_default_linear_accel
        else:
            raise Exception( 'Move Type Not Supported:', move_type )

        # Select speed value
        if speed_per is not None:
            speed = speed_per*max_speed
        else:
            speed = default_speed

        # Select acceleration value
        if accel_per is not None:
            accel = accel_per*max_accel
        else:
            accel = default_accel

        self.move_speed( target, move_type = move_type, speed = speed, accel = accel,
                         radius = radius, stop_condition = stop_condition, blocking = blocking )
        
        
    def set_joint_angles_speed( self, target, move_type = 'j', speed = None, accel = None,
                                stop_condition = 'dummy', blocking = True ):
        raise NotImplementedError( "!!! FIXME: `set_joint_angles_speed` has NOT BEEN IMPLEMENTED !!!" )
    
    
    def set_joint_angles_timed( self, target, time = 10, stop_condition = 'dummy', blocking = True ):
        raise NotImplementedError( "!!! FIXME: `set_joint_angles_timed` has NOT BEEN IMPLEMENTED !!!" )
    
    
    def set_joint_angles( self, target, move_type = 'j', speed_per = None, accel_per = None,
                          stop_condition = 'dummy', blocking = True ):
        raise NotImplementedError( "!!! FIXME: `set_joint_angles` has NOT BEEN IMPLEMENTED !!!" )
    
    
    def translate_tcp( self, translation_vec, dir_pose = 'origin', move_type = 'j', speed_per = None, accel_per = None,
                       stop_condition = 'dummy', blocking = True ):
        raise NotImplementedError( "!!! FIXME: `translate_tcp` has NOT BEEN IMPLEMENTED !!!" )
    
    
    def rotate_tcp( self, rotation_vec, dir_pose = 'origin', speed_per = None, accel_per = None,
                    stop_condition = 'dummy', blocking  = True ):
        raise NotImplementedError( "!!! FIXME: `rotate_tcp` has NOT BEEN IMPLEMENTED !!!" )
    
    
    def set_tool_digital_outputs( self, pin1, pin2 ):
        raise NotImplementedError( "!!! FIXME: `set_tool_digital_outputs` has NOT BEEN IMPLEMENTED !!!" )
    
    
    def move_to_common_pose( self, pose, move_type = 'j', speed_per = 0.25, accel_per = 0.25, radius = 0,
                             stop_condition = 'dummy', blocking = True ):
        raise NotImplementedError( "!!! FIXME: `move_to_common_pose` has NOT BEEN IMPLEMENTED !!!" )
        
    ##### Dashboard Client ######################


    def activate_dashboard( self, f_active ):
        """ Connect or disconnect from the dashboard, return T if dashboard is connected """
        if f_active:
            self.dash.connect()
            self.f_dashConn = 1
        else:
            self.dash.disconnect()
            self.f_dashConn = 0
        sleep( self.pause )
        return self.f_dashConn


    def power_on( self ):
        """ Power on the robot, 2021-06-14: Legacy function """
        self.recover_position_control()
        if self._DISLIKE_DASH:
            self.activate_dashboard(1)
        self.dash.powerOn()
        if self._DISLIKE_DASH:
            self.activate_dashboard(0)
            
            
    def power_off( self ):
        """ Power off the robot, 2021-06-14: Legacy function """
        self.recover_position_control()
        if self._DISLIKE_DASH:
            self.activate_dashboard(1)
        self.dash.powerOff()
        if self._DISLIKE_DASH:
            self.activate_dashboard(0)
            
            
    def brake_release( self ):
        """ Release brakes, 2021-06-14: Legacy function """
        self.recover_position_control()
        if self._DISLIKE_DASH:
            self.activate_dashboard(1)
        self.dash.brakeRelease()
        if self._DISLIKE_DASH:
            self.activate_dashboard(0)
            
            
    ##### Script Client #########################
            
            
    def activate_script_client( self, f_active ):
        """ Connect or disconnect from the script client, return T if dashboard is connected """
        if f_active:
            self.scrp.connect()
            self.f_scrpConn = 1
        else:
            self.scrp.disconnect()
            self.f_scrpConn = 0
        sleep( self.pause )
        return self.f_scrpConn
    
    
    def send_script_to_robot( self, script ):
        """ Send URscript to the robot to be executed """
        command = 'def fun():\n' + script + 'end\n'
        self.recover_position_control()
        if self._DISLIKE_SCRIPT:
            self.activate_script_client(1)
        self.scrp.sendScriptCommand( command )
        if self._DISLIKE_SCRIPT:
            self.activate_script_client(0)
            
    ##### Utility Functions #####################
            
            
    def dummy_stop( self ):
        """ Always return `False`, 2021-06-14: Legacy function """
        return False