#!/usr/bin/python
# ~~ assembly_trees.py ~~
# 2019 August , James Watson , for Robotic Materials
# Assembly primitives and tasks
# Goals:
# - Robustness
# - Reports pass/fail so that user can easily implement failure-tolerant assembly workflows


# ~~~ NOTES ~~~
# 2019-08-01: In the future allow for a failure type as well as PASS/FAIL?

# ~~~ Imports ~~~
# ~~ Standard ~~
import threading , math , time , datetime , os , pickle , builtins
from math import radians
from time import sleep
from datetime import datetime
from pytz import timezone
mst = timezone('MST')
# ~~ Special ~~
import cv2
import matplotlib.pyplot as plt
import numpy as np
from numpy import pi
import py_trees
import py_trees.blackboard as PTBlackboard
from py_trees.common import Status
from py_trees.tests import Timeout_Success
from dicttoxml import dicttoxml

# ~~ Local ~~
import utils
from utils import ( mindex , vec_unit , condition_false , combine_poses , is_matx_list , HeartRate , TimerQueue )
import pmath
from pmath import ( orient_error_between_poses , get_disance_between_poses , translate_pose , rotate_pose , pose_components , is_pose_mtrx ,
                    position_from_pose , transform_vectors )

# ~~ HACKS ~~
# from force_torque_actions import Force_Torque_Actions

import sys, os.path
SOURCEDIR = os.path.dirname( os.path.abspath( __file__ ) ) # URL, dir containing source file: http://stackoverflow.com/a/7783326
sys.path.insert( 0 , SOURCEDIR ) # Might need this to fetch a sibling lib in the same directory


# ~~ Constants ~~
builtins._DUMMYPOSE = np.zeros( (4,4) )
# ~ Speeds ~
builtins._DESCEND_SLOW   = 0.005 # [m/s] (movel)
builtins._SPEED_CAUTIOUS = 0.020 # [m/s] (movel)
# ~ Times ~
builtins._BIAS_PAUSE_TIME = 0.5 # [s]


def is_dummy_pose( pose ):
    """ Test if the pose is the dummy pose """
    return np.all( pose == _DUMMYPOSE )

def merge_obj_without_defaults( dstObj , srcObj ):
    """ Add all the attributes of `srcObj` to `dstObj`, excluding the __dunderscores__ attributes """
    # https://stackoverflow.com/questions/14839528/merge-two-objects-in-python
    addAttrib = [ elem for elem in dir( srcObj ) if not '__' in elem ] 
    for property in addAttrib:
        setattr( dstObj , property , getattr( srcObj , property )  )

def augment_RM( rmInstance ):
    """ Hack needed to add orphan functions back onto the RM studio object without disturbing the inheritance list """
    # TODO: UNHACK
    
    # == Sensing ==

    def base_wrist_force( self ):
        """ Return the force in the base frame """
        wrench   = self.ft.get_wrist_force()
        handPose = self.arm.get_tcp_pose()
        return transform_vectors( np.array( [ wrench[:3] ] ) , handPose )[0]
    
    rmInstance.base_wrist_force = base_wrist_force

    # __ End Sensing __
    
    # == Movements ==
    
    def align_tcp(self, lock_roll=False, lock_pitch=False, lock_yaw=False):
        """ 
        Alignes the gripper with the nearest rotational axis (principle cartesian axes).

        Parameters
        ----------
        lock_roll: bool
        lock_pitch: bool
        lock_yaw: bool
        """
#         pose = self.arm.get_tcp_pose()
        pose = self.arm.get_tcp_pose()
        rot_matrix = pose[0:3, 0:3]
        R = pmath.rotation_mtrx_to_rpy(rot_matrix)
        for i, value in enumerate(R):
            if(i == 0 and lock_pitch):
                continue
            if(i == 1 and lock_yaw):
                continue
            if(i == 2 and lock_roll):
                continue
            if value > -3.142 and value <= -2.36:
                R[i] = -3.14  # -180
            elif value > -2.36 and value <= -0.79:
                R[i] = -1.57  # -90
            elif value > -0.79 and value <= 0.79:
                R[i] = 0  # 0
            elif value > 0.79 and value <= 2.36:
                R[i] = 1.57  # 90
            elif value > 2.36 and value <= 3.142:
                R[i] = 3.14  # 180
            else:
                raise NameError('ERROR -James')
        rot_matrix = pmath.rpy_to_rotation_mtrx(R)
        pose[0:3, 0:3] = rot_matrix
        rtnVal = self.arm.move_speed( pose , 'l' , 0.125 , 0.35 , 0 , 'dummy' , False )
        return True
    
    rmInstance.arm.align_tcp = align_tcp
    
    # __ End Move __
    
#     add_obj = 
#     merge_obj_without_defaults( rmInstance , Force_Torque_Actions() ) # THIS DOESN'T WORK
    
    # == Assembly Globals ==
    
    builtins.ASMBB = PTBlackboard.Blackboard() # Hack the BB object into the built-in namespace
    
#     builtins._jogSpeedLin = 
    
    # __ End Globals __
    
    print( "Functions and Vars added to RMStudio!" )


def run_BT_until_done( rootNode , 
                       N = 1000 , tickPause = 1.0 , 
                       breakOnFailure = False , breakOnSuccess = False , 
                       Nverb = 50 , 
                       shortTime = 5.0 , N_ST_retry = 5 , retrySleep_s = 0.5 ):
    """ Tick root until `maxIter` is reached while printing to terminal """
    
    print( "### RUN BT" , rootNode.name , "###\n" )
    
#     for j in range( N_ST_retry ):
        
    bgn     = time.time()
    elapsed = 0.0

    # 0. Setup
    rootNode.setup_with_descendants()
    pacer = HeartRate( Hz = 1/tickPause ) # metronome
    # 1. Run
    for i in range( 1 , N+1 ):
        try:
            rootNode.tick_once()

            if Nverb > 0 and i % Nverb == 0:
                print("\n--------- Tick {0} ---------\n".format(i))
                print("\n")
                print( py_trees.display.unicode_tree( root        = rootNode ,
                                                      show_status = True     ) )

            if  breakOnFailure  and  ( rootNode.status == Status.FAILURE ):
                print( "Root node" , rootNode.name , "failed!\n" )
                break
            elif  breakOnSuccess  and  ( rootNode.status == Status.SUCCESS ):
                print( "Root node" , rootNode.name , "succeeded!\n" )
                break
            else:
                if Nverb > 0 and i % Nverb == 0:
                    print( "Root node" , rootNode.name , "status:" , rootNode.status )
                pacer.sleep()

        except KeyboardInterrupt:
            break
    if Nverb > 0:
        print( py_trees.display.unicode_tree( root        = rootNode ,
                                              show_status = True     ) )
    elapsed = time.time() - bgn
    print("\nRun completed! with status" , rootNode.status , "after" , elapsed , "seconds.\n\n")
    rootNode.terminate( rootNode.status ) # HACK required coz tree doesn't complete sometimes
        
#         
#         if elapsed >= shortTime:
#             print( "Run COMPLETED in" , elapsed , "seconds" )
#             break
#         else:
#             print( "Run SHORT-CIRCUIT at" , elapsed , "seconds, Attempt" , j+1 , "of" , N_ST_retry )
#             rootNode.terminate( rootNode.status )
#             if j+1 == N_ST_retry:
#                 print( "!! QUIT TREE !!" )
#                 rootNode.terminate( rootNode.status )
#                 sleep( 0.5 )
#                 rootNode.terminate( rootNode.status )
#             else:
#                 sleep( retrySleep_s )
            
    print( "\n### BT SESSION OVER ###" )
    
    
def BT_procedure_w_precondition( precond , behavList , rootName = "MAIN_SEQ" , procName = "PROC_SEQ" ):
    """ Return a NON-memory sequence that consists of a `precond` followed by a MEMORY sub-sequence consisting of `behavList` """
    # NOTE: Intent is for `precond` to be tested every tick, while `behavList` procedes in a linear fashion
    # NOTE: This function assumes that `behavList` is in the correct order
    # 1. Create the root node and add the precondition
    rtnSeq = py_trees.composites.Sequence( name = rootName , memory = 0 )
    rtnSeq.add_child( precond )
    # 2. Create the "procedure" sequence and add each of the behaviors
    prcSeq = py_trees.composites.Sequence( name = procName , memory = 1 )
    for behav in behavList:
        prcSeq.add_child( behav )
    # 3. Add the sub-sequence to root and return root
    rtnSeq.add_child( prcSeq )
    return rtnSeq


# ===== BEHAVIOR TREES ================================================================================================

builtins._EPSILON_POSE_POSN = 0.001 # - [mm]
builtins._EPSILON_ORNT_EROR = 1.0/180.0 # 1.0 is 180deg

# ==== Finger Width Condition ====

class Finger_Width_COND( py_trees.behaviour.Behaviour ):
    """ Return SUCCESS if the gripped mass meets or exceeds expectations, otherwise return FAILURE """
    # NOTE: This function assumes the robot is near enough to sea level for 'g' to apply
    
    def __init__( self , partWidth , margin = 0.25 , ctrl = None , _DEBUG = 1 ):
        """ Store the target mass """
        super().__init__( name = "Finger_Width_COND" )
        self.ctrl      = ctrl
        self.partWidth = partWidth
        self.margin    = margin
        self._DEBUG    = _DEBUG
        
    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Check that the arm is moving, goal reached, or failed
        """
        self.logger.debug( "  %s [Gripped_Mass_COND::update()]" % self.name )
        # 1. If the robot arm has stopped (ASSUMPTION: Robot has stopped because it has finished moving)
        
        width = self.ctrl.hand.get_finger_width()
        
        if width >= ( self.partWidth * ( 1.0 - self.margin ) ):
            return py_trees.common.Status.SUCCESS
        else:
            if self._DEBUG:
                print( "Finger_Width_COND - FAILED with desired " , self.partWidth , "-vs- actual" , width )
            return py_trees.common.Status.FAILURE    

# ____ End Finger Width ____

class TwistToggleRelGen:
    """ Generator for a toggled twist motion, Relative """
    
    def __init__( self , twist_deg = 10 , doubleSided = 1 , N = None , ctrl = None ):
        """ Set vars for the twist operation """
        self.ctrl     = ctrl
        self.twistRad = radians( twist_deg )
        self.dbblSdd  = doubleSided
        self.N        = N
        self.factor   = 1.0
        self.i        = 1
        
    def __call__( self ):
        """ Generate the next pose """
        # 1. Get the hand pose
        pose = self.ctrl.arm.get_tcp_pose()
        sleep( 0.5 )
        # 2. Calc the next pose
        turnRad = 0
        if self.dbblSdd:
            print( "double sided" )
            
            if self.N != None: 
                print( "specified N" )
                # B. If double sided && If iter-limited && if the last iteration, Then turn by hald
                if self.i == 1:
                    print( "the first iteration" )
                    turnRad = self.twistRad * 0.5
                elif self.N == self.i:
                    print( "the last iteration" )
                    turnRad = self.twistRad * 0.5
                # B. If double sided && If iter-limited && if surpassed last iteration, Then do not turn
                elif self.N < self.i:
                    print( "too many iteration" )
                    turnRad = 0.0
                # C. Else turn by full measure
                else:
                    print( "iteration:" , self.i )
                    turnRad = self.twistRad
            else:
                print( "no N given" )
                # A. If double sided && If the first iteration, Then turn by half
                if self.i == 1:
                    print( "the first iteration" )
                    turnRad = self.twistRad * 0.5
                else:
                    turnRad = self.twistRad
            
        # D. Else is not double sided, turn by full measure
        else:
            print( "NOT double sided" )
            turnRad = self.twistRad
        # 3. Rotate the pose
        turnRad *= self.factor
        print( "turnRad =" , turnRad )
        pose    = rotate_pose( pose , [ 0.0 , 0.0 , turnRad ] , dir_pose = 'origin' )
        # 4. Reverse the direction
        self.factor *= -1.0
        # 5. Incrment `i`
        self.i += 1
        # 6. Return the pose
        return pose
    
class Set_BB_Key( py_trees.behaviour.Behaviour ):
    """ Set key and always succeed """
    
    def __init__( self , key , val , name = "Set_BB_Key" ):
        """ 
        Minimal one-time initialisation, offline only: 
        """
        super().__init__( name = name )
        self.key = key
        self.val = val
        
#         print( "Set_BB_Key initialized with key:" , self.key )
        
    def update( self ):
        """ Set key and always succeed """
        ASMBB.set( self.key , self.val )
        #print( "Set" , self.key , "to" , self.val , ", What is the value?" , ASMBB.get( self.key ) )
        check = ASMBB.get( self.key )
        if check == self.val:
            return py_trees.common.Status.SUCCESS
        else:
            print( "Set_BB_Key: Could NOT set the value" )
            return py_trees.common.Status.FAILURE

# === Move Arm Behavior ===

# 1. Calculate absolute goal pose (Setup)
# 2. Move, respond with "RUNNING" while the move is being executed
# 3. If the UR has completed the move, check that the goal pose was reached
# 4. If goal reached set to SUCCESS, otherwise set to FAILURE

class Move_Arm( py_trees.behaviour.Behaviour ): # BASIC
    """ Move the arm to a pose, SUCCESS if pose reached, otherwise FAILURE """
    
    def __init__( self , 
                  pose = None , BB_key = None , mode = 'l' , speed = 0.125 , accel = 0.35 , ctrl = None , name = "Move_Arm" ,
                  blocking = 1 , 
                  _DEBUG = 0 ):
        """ 
        Minimal one-time initialisation, offline only: 
        Set the pose that the arm must reach 
        """
        self.pose     = pose
        self.BB_key   = BB_key
        self.mode     = mode
        self.speed    = speed
        self.accel    = accel
        self.blocking = blocking
        self.ctrl     = ctrl # Should be an RMStudio object, will raise an error if not set
        self._DEBUG   = _DEBUG
        super().__init__( name )

    def initialise( self ):
        """
        First time your behaviour is ticked or not RUNNING: 
        Send move command to UR
        """
        self.logger.debug( "  %s [Move_Arm::initialise()]" % self.name )
        
        # Fetch the value here, because the key may not exist at instantiation time
        if type( self.BB_key ) != type( None ):
            self.pose = ASMBB.get( self.BB_key )
        
        self.ctrl.arm.move_speed( self.pose , self.mode , self.speed , self.accel , 0 , 'dummy' , self.blocking )
        time.sleep( 0.1 )

    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Check that the arm is moving, goal reached, or failed
        """
        self.logger.debug( "  %s [Move_Arm::update()]" % self.name )
        # 1. If the robot arm has stopped
        if int( self.ctrl.arm.get_status_bits() , 2 ) == 1:
            # A. Calc error
            currPose = self.ctrl.arm.get_tcp_pose()
            transErr = get_disance_between_poses( currPose , self.pose )
            orienErr = orient_error_between_poses( currPose , self.pose )
            # B. If the goal has been reached
            if  ( transErr <= _EPSILON_POSE_POSN )  and  ( orienErr <= _EPSILON_ORNT_EROR ):
                if self._DEBUG: print( "Move_Arm: Goal REACHED!" )
                return py_trees.common.Status.SUCCESS
            # C. Otherwise the goal has not been reached
            else:
                if self._DEBUG: print( "Move_Arm: Goal NOT REACHED!" )
                return py_trees.common.Status.FAILURE
        # 2. Otherwise the robot arm has not stopped
        else: 
            return py_trees.common.Status.RUNNING

# ___ End Move Arm ___


# === Behavior Move Arm Relative ===

def _ALWAYS_FALSE():
    return False

class Move_Arm_Relative( py_trees.behaviour.Behaviour ): # BASIC
    """ Move the arm to a pose, SUCCESS if pose reached, otherwise FAILURE """
    
    def __init__( self , translation = [0.0,0.0,0.0] , rotation = [0.0,0.0,0.0] , 
                         mode = 'l' , speed = 0.125 , accel = 0.35 , stop_cond = None , frame = 'origin' ,
                         cond_success = 1 , bias_wrist = 1 ,
                         ctrl = None , name = "Move_Arm_Relative" , _DEBUG = 0 ):
        """ 
        Minimal one-time initialisation, offline only: 
        Set the pose that the arm must reach 
        """
        self.trans     = translation
        self.rottn     = rotation
        self.mode      = mode
        self.speed     = speed
        self.accel     = accel
        self.pose      = None
        self.stop_cond = stop_cond
        self.wrap_cond = None
        self.cond_met  = 0
        self.cond_sccs = cond_success
        self._DEBUG    = _DEBUG
        builtins._GLOB_FT_FLAG = 0
        
        if type( stop_cond ) != type( None ):
            self.cond_name  = str( stop_cond.__name__ )[:]
            self.usng_cond  = True
            self.bias_wrist = bias_wrist
        else:
            self.cond_name  = ""
            self.usng_cond  = False
            self.bias_wrist = False
        
        self.frame     = frame
        self.ctrl      = ctrl # Should be an RMStudio object, will raise an error if not set
        super().__init__( name )
        
    
    def _give_wrap1( self ):
        return self.stop_cond
    
    def _give_wrap2( self ):
        func = self._give_wrap1()
        def cond_w_flag():
            rtnVal = func()
            if rtnVal:
                if self._DEBUG:  print( "Force condition MET" )
                self.cond_met = 1
            else:
                if self._DEBUG:  print( "Force condition NOT met" )
            return rtnVal
        return cond_w_flag

    def initialise( self ):
        """-
        First time your behaviour is ticked or not RUNNING: 
        Send move command to UR
        """
        self.logger.debug( "  %s [Move_Arm::initialise()]" % self.name )
        # Calc the relative pose
        time.sleep( 0.1 )
        
        self.pose = self.ctrl.arm.get_tcp_pose()
        orig = self.pose.copy()
        if self._DEBUG:  print( "Translation:" , self.trans , ", Rotation:" , self.rottn , ", Frame:" , self.frame )
        self.pose = translate_pose( self.pose , self.trans , dir_pose = self.frame )
        self.pose = rotate_pose(    self.pose , self.rottn , dir_pose = self.frame )
        
        if self._DEBUG:  print( "Linear Diff B/n Poses:" , np.subtract( position_from_pose( self.pose ) , position_from_pose( orig ) ) )
        
        if self._DEBUG:  print( "Wrench BEFORE bias:" , self.ctrl.ft.get_wrist_force() )
        
        if self.bias_wrist:
            if self._DEBUG:  print( "Biasing wrist ..." )
            self.ctrl.ft.bias_wrist_force()
            time.sleep( _BIAS_PAUSE_TIME )
        
        if self._DEBUG:  print( "Wrench AFTER bias:" , self.ctrl.ft.get_wrist_force() )
        
        # Move arm
        if self.usng_cond:
            if self._DEBUG:  print( "Using the stop condition:" , self.cond_name )
            self.ctrl.arm.move_speed( self.pose , self.mode , self.speed , self.accel , 0 , self._give_wrap1() , True )
        else:
            if self._DEBUG:  print( "Moving without a force condition" )
            self.ctrl.arm.move_speed( self.pose , self.mode , self.speed , self.accel , 0 , _ALWAYS_FALSE , True )
        time.sleep( 0.1 )
        if self._DEBUG:  print( "Movement completed!" )

    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Check that the arm is moving, goal reached, or failed
        """
        self.cond_met = _GLOB_FT_FLAG
        
        self.logger.debug( "  %s [Move_Arm::update()]" % self.name )
        # 1. If the robot arm has stopped
        if int( self.ctrl.arm.get_status_bits() , 2 ) == 1:
            # A. Calc error
            currPose = self.ctrl.arm.get_tcp_pose()
            transErr = get_disance_between_poses( currPose , self.pose )
            orienErr = orient_error_between_poses( currPose , self.pose )
            # B. If the goal has been reached
            if self.usng_cond:
                if self.cond_sccs:
                    if self.cond_met:
                        if self._DEBUG:  print( "Ended FT move with condition MET --> SUCCESS" )
                        return py_trees.common.Status.SUCCESS
                    else:
                        if self._DEBUG:  print( "Ended FT move with condition UNMET --> FAILURE" )
                        return py_trees.common.Status.FAILURE
                else:
                    if self.cond_met:
                        if self._DEBUG:  print( "Ended FT move with condition MET --> FAILURE" )
                        return py_trees.common.Status.FAILURE
                    else:
                        if self._DEBUG:  print( "Ended FT move with condition UNMET --> SUCCESS" )
                        return py_trees.common.Status.SUCCESS
            else:
                if  ( transErr <= _EPSILON_POSE_POSN )  and  ( orienErr <= _EPSILON_ORNT_EROR ):
                    if self._DEBUG:  print( "Move_Arm_Relative: Ended at GOAL!" )
                    return py_trees.common.Status.SUCCESS
                # C. Otherwise the goal has not been reached
                else:
                    if self._DEBUG:  print( "Move_Arm_Relative: Goal NOT REACHED!" )
                    return py_trees.common.Status.FAILURE
        # 2. Otherwise the robot arm has not stopped
        else: 
            return py_trees.common.Status.RUNNING

# ___ End Move Arm Relative ___


class Run_Py_Func( py_trees.behaviour.Behaviour ):
    """ 
    Run an arbitrary Python function, 
    NOTE: Function signature must be `func_name( self , *args )` 
    NOTE: Function must return success or failure
    """
    
    def __init__( self , func , *args , name = "Run_Py_Func" ):
        """ Save the function and the arguments """
        super().__init__( name )
        self.func   = func
        self.args   = args
        self.result = False
        
    def initialise( self ):
        """
        First time your behaviour is ticked or not RUNNING: 
        Send move command to UR
        """
        self.result = False
        
    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Check that a proper pose was stored
        """
        if self.func( *self.args ):
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
           
class Xform_BB_Pose( py_trees.behaviour.Behaviour ):
    """ Operate on a backboard pose using vectors stored in the blackboard """
    
    def __init__( self , translation_BBkey , rotation_BBkey , pose_BBkey , 
                  transFrame = 'origin' , rottnFrame = 'origin' , name = "Xform_BB_Pose" , rotationFirst = 1 ):
        super().__init__( name )
        self.translation_BBkey = translation_BBkey
        self.rotation_BBkey    = rotation_BBkey
        self.pose_BBkey        = pose_BBkey
        self.transFrame        = transFrame
        self.rottnFrame        = rottnFrame
        self.rotationFirst     = rotationFirst
        
    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Check that a proper pose was stored
        """
        
        # 1. Fetch data
        tran = ASMBB.get( self.translation_BBkey )
        rotn = ASMBB.get( self.rotation_BBkey    )
        pose = ASMBB.get( self.pose_BBkey        )
        
        # 2. Apply transformations
        if self.rotationFirst:
            xformPose = rotate_pose(    pose = pose      , rotation_vec    = rotn , dir_pose = self.rottnFrame )
            xformPose = translate_pose( pose = xformPose , translation_vec = tran , dir_pose = self.transFrame )
        else:
            xformPose = translate_pose( pose = pose      , translation_vec = tran , dir_pose = self.transFrame )
            xformPose = rotate_pose(    pose = xformPose , rotation_vec    = rotn , dir_pose = self.rottnFrame )
        
        # 3. Update the pose
        ASMBB.set( self.pose_BBkey , xformPose )

# === Behavior Store Current Pose ===

class Store_Current_Pose( py_trees.behaviour.Behaviour ):
    """ Move the arm to a pose, SUCCESS if pose reached, otherwise FAILURE """
    
    N = 0
    
    def __init__( self , keyString = "lastPose_" , ctrl = None , setOnce = True , name = "Store_Current_Pose" ):
        """ 
        Minimal one-time initialisation, offline only: 
        Set the pose that the arm must reach 
        """
        self._DEBUG = 0
        super().__init__( name )
        self.ctrl = ctrl # Should be an RMStudio object, will raise an error if not set
        self.__class__.N += 1
        self.key = keyString
        if self._DEBUG: print( "Created `Store_Current_Pose` with key:" , self.key )
        self.mem = setOnce

    def initialise( self ):
        """
        First time your behaviour is ticked or not RUNNING: 
        Send move command to UR
        """
        self.logger.debug( "  %s [Move_Arm::initialise()]" % self.name )
        # Calc the relative pose
        time.sleep( 0.1 )
        self.pose = self.ctrl.arm.get_tcp_pose()
        
        if self.mem:
            if not ASMBB.exists( self.key ):
                ASMBB.set( self.key , self.pose )
        else:
            ASMBB.set( self.key , self.pose )

    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Check that a proper pose was stored
        """
        self.logger.debug( "  %s [Move_Arm::update()]" % self.name )
        # 1. If the robot arm has stopped
        if is_pose_mtrx( self.pose ):
            if self._DEBUG:  print( "Retrieve Pose with key:" , self.key , '\n' , ASMBB.get( self.key ) )
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

# ___ End Store Current Pose ___


# === Jog Safe Sequence ===
# TODO: ADD COLLISION DETECTION VIA FT SENSOR

class Jog_Safe( py_trees.composites.Sequence ): # BASIC
    """ Pick the drill up from the holster """
    # NOTE: This behavior should not, on its own, assume any gripper state 
    
    def __init__( self , endPose , 
                         zSAFE = 0.150 ,  
                         hover = False ,  
                         ctrl  = None  ): 
        """ Construct the subtree """
        super().__init__( name = "Jog_Safe" , memory = 1 )
        self.zSAFE   = zSAFE 
        self.hover   = hover
        self.ctrl    = ctrl
        
        self.targetP = endPose.copy()
        self.pose1up = _DUMMYPOSE
        self.pose2up = _DUMMYPOSE
        
        self.moveUp  = Move_Arm( self.pose1up , ctrl = ctrl )
        self.moveJg  = Move_Arm( self.pose2up , ctrl = ctrl )
        self.moveDn  = Move_Arm( self.targetP , ctrl = ctrl )
        
        self.add_child( self.moveUp )
        self.add_child( self.moveJg )
        if not hover:
            self.add_child( self.moveDn )
        
    def initialise( self ):
        """
        ( Ticked first time ) or ( ticked not RUNNING ): 
        Generate move waypoint, then move with condition
        """
        nowPose = self.ctrl.arm.get_tcp_pose()
        nowPart = pose_components( nowPose )
        nowZ    = nowPart['position'][2]
        
        self.pose1up = nowPose.copy()
#         print( self.zSAFE )
        self.pose1up[2,3] = self.zSAFE
        
        self.pose2up = self.targetP.copy()
        self.pose2up[2,3] = self.zSAFE
        
        self.moveUp.pose = self.pose1up.copy()
        self.moveJg.pose = self.pose2up.copy()

# ___ End Jog Safe ___
        

# === Move Rule with Stop Condition Behavior ===

class Move_Rule_w_Stop_Cond( py_trees.behaviour.Behaviour ):
    """ Use a generator to get a waypoint, and move there with a stop condition """
    # NOTE: It is the responsibility of the generator to profide appropriate waypoints for every iteration
    # NOTE: The generator must called without args and return the next appropriate pose (homogeneous coords)
    
    def __init__( self , generator , 
#                  setFunc , 
                 stop_cond , 
                 condSuccess = True , 
                 fetchInit = None ,
                 relativeRule = False , 
                 ctrl = None ):
        """ 
        Minimal one-time initialisation, offline only: 
        Set generator and condition, `condSuccess`: Meeting the condition triggers success
        """
        super().__init__( name = "Move_Rule_w_Stop_Cond" )
        self.genr = generator # - Pose generation functor that will be called
        self.stop = stop_cond # - Stop condition for each of the moves
        self.cMet = False # ----- Were any of the moves stopped by the force condition
        self.cWin = condSuccess # Does meeting the condition equal success? (False for met == False is SUCCESS)
        self.ctrl = ctrl # ------ Reference to the RM object
        self.pose = None # ------ Most recent pose returned by the generator
        self.last = None # ------ The last pose that we moved to
        self.ftch = fetchInit # - Assembly BB key to retrieve the starting pose
        self.pSet = False
        self.relativeRule = relativeRule
        self.offSet       = None
        self.trans   = None
        self.rottn   = None
        
    def initialise( self ):
        """
        ( Ticked first time ) or ( ticked not RUNNING ): 
        Generate move waypoint, then move with condition
        """
        self.logger.debug( "  %s [Move_Rule_w_Stop_Cond::initialise()]" % self.name )
        
        builtins._GLOB_FT_FLAG = 0
        
        # If there is a pose to fetch, do that now
        if self.ftch and not self.pSet:
#             print( "The generator" , self.genr.__class__.__name__ , "was reset" )
            self.genr.reset( ASMBB.get( self.ftch ) )
            self.pSet = True
        
    def give_cond( self ):
        return self.stop
        
    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Check that the arm is moving, condition met, or failed
        """
        self.logger.debug( "  %s [Move_Rule_w_Stop_Cond::update()]" % self.name )
        
        if not self.relativeRule:
            self.pose = self.genr()
        else:
            # Calc relative pose from current
            offsetMatx  = self.genr()
            transOffset = offsetMatx[0,:]
            rottnOffset = offsetMatx[1,:]
            self.pose   = self.ctrl.arm.get_tcp_pose()
            self.pose   = translate_pose( self.pose , transOffset , dir_pose='origin' )
            self.pose   = rotate_pose(    self.pose , rottnOffset , dir_pose='origin' )
        
#         print( "Pose Output:\n" , self.pose )
        
        speed = 0.7 / 2.0
        accel = 0.7
        
        # 2. Move w/ stop condition
        builtins._GLOB_FT_FLAG = 0
        self.ctrl.arm.move_speed( self.pose , 'j' , speed , accel , 0 , self.give_cond() , True ) # Must be BLOCKING!
        
        self.last = self.pose.copy()
        
        # 1. If the robot arm has stopped (ASSUMPTION: Robot has stopped because it has finished moving)
        if int( self.ctrl.arm.get_status_bits() , 2 ) == 1:
            # A. if the condition was met
            if builtins._GLOB_FT_FLAG:
#                 print( self.name , ": Stop condition MET!" )
                rtnStatus = py_trees.common.Status.SUCCESS if self.cWin else py_trees.common.Status.FAILURE
            # B. Else the robot ended its motion without touching anything
            else:
#                 print( self.name , ": Stop condition NOT met!" )
                rtnStatus = py_trees.common.Status.FAILURE if self.cWin else py_trees.common.Status.SUCCESS
#             builtins._GLOB_FT_FLAG = 0
            return rtnStatus
        # 2. Otherwise the robot arm has not stopped
        else: 
            return py_trees.common.Status.RUNNING

# ___ End Move Rule Until Condition ___


# === Gripper/Hand Behaviors ===

class Set_Fingers( py_trees.behaviour.Behaviour ): # BASIC
    """ Change the gripper open state """
    
    def __init__( self , openState = 1.0 , ctrl = None , waitSec = 0.5 , name = "Set_Fingers" ):
        """ 
        Minimal one-time initialisation, offline only: 
        Store the desired width
        """
        self.desired = openState #- 0.0: Close Gripper, 1.0: Open Gripper, 0.x: Desired width
        self.ctrl    = ctrl # ----- Should be an RMStudio object, will raise an error if not set
        self.bgnTime = None # ----- Time after calling the movement
        self.waitSec = waitSec # -- After this amount of time, consider the gripper state changed (gripper width unreliable)
        super().__init__( name )
        
    def initialise( self ):
        """
        ( Ticked first time ) or ( ticked not RUNNING ): 
        Generate move waypoint, then move with condition
        """
        # 0. Log the beginning of the motion
        self.bgnTime = time.time() 
        # 1. Start the motion
        if self.desired >= 1.0:
            self.ctrl.hand.release( blocking = False )
        elif self.desired == 0.0:
            # self.ctrl.hand.close_grip( blocking = False )
            self.ctrl.hand.grip( blocking = False )
            
        else:
            self.ctrl.hand.set_finger_width( self.desired , blocking = False )
        
    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Check that the arm is moving, goal reached, or failed
        """
        if time.time() - self.bgnTime >= self.waitSec:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
    
        
class Set_Grip_Torque( py_trees.behaviour.Behaviour ): # BASIC
    """ Change the gripper torque state """
    
    def __init__( self , 
                 torqState = 1.0 , 
                 DANGER_MODE = 0 ,
                 ctrl = None , name = "Set_Grip_Torque" ):
        """ 
        Minimal one-time initialisation, offline only: 
        Store the desired torque
        """
        self.desired     = torqState #- NOTE: At this time not supporting excess torque
        self.ctrl        = ctrl # ----- Should be an RMStudio object, will raise an error if not set
        self.bgnTime     = None # ----- Time after calling the movement
        self.waitSec     = 0.5 # ------ After this amount of time, consider the gripper state changed (gripper width unreliable)
        self.DANGER_MODE = DANGER_MODE
        super().__init__( name )
        
    def initialise( self ):
        """
        ( Ticked first time ) or ( ticked not RUNNING ): 
        Generate move waypoint, then move with condition
        """
        # 0. Log the beginning of the motion
        self.bgnTime = time.time() 
        # 1. Set the torque
        if not self.DANGER_MODE:
            self.ctrl.hand.set_finger_torque( self.desired )
        else:
            self.ctrl.hand.set_finger_torque_DANGER( self.desired )
        
    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Check that the arm is moving, goal reached, or failed
        """
        if time.time() - self.bgnTime >= self.waitSec:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

# ___ End Grippers ___
    

# ==== Grasp at Pose Sequence ====

class Grasp_at_Pose( py_trees.composites.Sequence ): # BASIC
    """ Move to grasp pose and pick """
    
    def __init__( self , graspPose , partWidth ,  zApproach=0.20, zClose=0.0 , wdthNarrow = 0.058 , maxIter=5 , zFree=0.150 , ctrl = None ):
        """ Construct the subtree """
        
        super().__init__( name = "Grasp_at_Pose" , memory = 1 )
        self.ctrl       = ctrl
        
        # ~~ Add Nodes ~~
        
        #  1. Calc the approach and retry poses
        self.graspPose    = graspPose
        self.approachPose = translate_pose( graspPose , [ 0.0 , 0.0 , zApproach ] , dir_pose='origin')
        self.nearDpthPose = translate_pose( graspPose , [ 0.0 , 0.0 , zClose    ] , dir_pose='origin')
        self.liftFreePose = translate_pose( graspPose , [ 0.0 , 0.0 , zFree     ] , dir_pose='origin')
        
        #  2. Approach, pre-contact, wide
        self.add_child(  Move_Arm( self.approachPose , ctrl = ctrl )  )
        
        # 2020-03-12: TEST EXEC OK
        
        #  3. Narrow the fingers
        self.add_child(  Set_Fingers( openState = wdthNarrow , ctrl = ctrl )  )
        
        # 2020-03-12: TEST EXEC OK
        
        #  4. Move to contact
        self.add_child(  Move_Arm( self.nearDpthPose , ctrl = ctrl )  )
#         self.add_child(  Bias_Wrist( ctrl = ctrl )  ) # NO FORCE CTRL HERE?
        
        #  5. Narrow grip to hold
        self.add_child(  Set_Fingers( openState = 0.0 , ctrl = ctrl )  )
        
        #  6. Close (hard close)
        self.add_child(  Set_Grip_Torque( torqState = 1.0 , ctrl = ctrl )  )
            
        # 7. Move up 
        self.add_child(  Move_Arm( self.liftFreePose , ctrl = ctrl )  )
        self.add_child(  Finger_Width_COND( partWidth = partWidth , ctrl = ctrl )  )
        
# ____ End Grasp at Pose ____


class Align_TCP( py_trees.behaviour.Behaviour ): # BASIC
    """ Align the TCP to the origin/base frame (nearest axes) """
    
    def __init__( self , ctrl = None ):
        """ Set the control object """
        super().__init__( name = "Align_TCP" )
        self.ctrl = ctrl
        
    def initialise( self ):
        """
        First time your behaviour is ticked or not RUNNING: 
        Send move command to UR
        """
        self.logger.debug( "  %s [Align_TCP::initialise()]" % self.name )
        self.ctrl.align_tcp()
        time.sleep( 0.1 )

    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Check that the arm is moving, goal reached, or failed
        """
        self.logger.debug( "  %s [Align_TCP::update()]" % self.name )
        # 1. If the robot arm has stopped
        if int( self.ctrl.arm.get_status_bits() , 2 ) == 1:
            return py_trees.common.Status.SUCCESS
        # 2. Otherwise the robot arm has not stopped
        else: 
            return py_trees.common.Status.RUNNING
        
