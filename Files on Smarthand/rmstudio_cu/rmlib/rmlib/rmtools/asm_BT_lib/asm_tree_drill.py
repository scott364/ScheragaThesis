from py_trees.decorators import FailureIsSuccess , SuccessIsFailure
from py_trees.timers import Timer

from asm_tree_Basic import *

from asm_tree_FT_based import (
    Bias_Wrist , Move_to_Contact , Maintain_Z_Pressure , SpiralStep
)

from asm_tree_cond_test import labeled_precondition , BB_Flag_COND, Hand_Thermal_Check

from asm_tree_logic_flow import Run_to_X_Failures_DECO , Negator_DECO , Always_Succeed


from pmath import is_pose_mtrx

##### Drill State ##### ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class MultiSemaphore( dict ):
    """ Only allow N semaphores in the collection to wait """
    
    def __init__( self , N_limit = 1 , *args , **kw ):
        """ `MultiSemaphor` is a type of dictionary """
        dict.__init__( self , *args , **kw )
        self.limit = N_limit
        self.Nwait = 0
        
    def add_sem( self , *keys ):
        """ Add a new semaphore(s) to the collection """
        for key in keys:
            self[ key ] = 0
        
    def sem_wait( self , key ):
        """ Only wait if the limit has not been reached """
        success = 0
        # 1. If this key is not already waiting, then check limit (error on bad key)
        if ( not self[ key ] ) and ( self.Nwait < self.limit ):
            self.Nwait += 1
            self[ key ] = 1
            success = 1
        return success
    
    def sem_post( self , key ):
        """ Only post if the key is waiting """
        success = 0
        # 1. If this key is already waiting, then post (error on bad key)
        if ( self[ key ] ):
            self.Nwait -= 1
            self[ key ] = 0
            success = 1
        return success
            
    def check_sem( self , key ):
        """ Return the wait status of the specified key """
        return self[ key ] # Error on bad key
    
    def n_waiting( self ):
        """ Return the total number of keys currently waiting """
        return self.Nwait
    
    def reset( self ):
        """ Post all semaphores """
        # WARNING: Only do if safe!
        for k in self.keys():
            self[k] = 0
        self.Nwait = 0
    
    def check_OUT( self , key ):
        """ Alias for wait """
        return self.sem_wait( key )
    
    def check_IN( self , key ):
        """ Alias for post """
        return self.sem_post( key )
    
    def check_STATUS( self , key ):
        """ Alias for semaphore status """
        return self.check_sem( key )
    
DrillSemaphore = MultiSemaphore()
DrillSemaphore.add_sem( 'M8' , 'M6' , 'M4' )
print( "DrillSemaphore created with:" , list( DrillSemaphore.keys() ) )

def check_out_drill( name ):
#     global DrillSemaphore
    status = DrillSemaphore.check_OUT( name )
    if 1:
        if status:
            print( name , "checked OUT!" )
        else:
            print( name , "checkout FAILED!" )
    return status

def check_in_drill( name ):
#     global DrillSemaphore
    return DrillSemaphore.check_IN( name )
    
def reset_drill_state():
    DrillSemaphore.reset()
    print( list( DrillSemaphore.keys() ) , "checked IN!" )
    
##### Behaviors ##### ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class COND_Drill_Checked_Out( py_trees.behaviour.Behaviour ):
    """ Return success if the drill specified at instantiation is checked out """
    
    def __init__( self , drillID , ctrl = None , name = "COND_Drill_Checked_Out" ):
        self.ctrl = ctrl # ----- Should be an RMStudio object, will raise an error if not set
        super().__init__( name )
        self.drillID = drillID
        
    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Check that goal reached, or failed
        """
#         global DrillSemaphore
        status = DrillSemaphore.check_STATUS( self.drillID )
        if status:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class Check_OUT_Drill( py_trees.behaviour.Behaviour ):
    """ Return success if the drill specified at instantiation is checked out """
    
    def __init__( self , drillID , ctrl = None , name = "Check_OUT_Drill" ):
        self.ctrl    = ctrl # ----- Should be an RMStudio object, will raise an error if not set
        self.drillID = drillID
        super().__init__( name )
        
    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Check that goal reached, or failed
        """
#         global DrillSemaphore
        status = check_out_drill( self.drillID )
        if status:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
        
class Check_IN_Drill( py_trees.behaviour.Behaviour ):
    """ Return success if the drill specified at instantiation is checked out """
    
    def __init__( self , drillID , ctrl = None , name = "Check_OUT_Drill" ):
        self.ctrl    = ctrl # ----- Should be an RMStudio object, will raise an error if not set
        self.drillID = drillID
        super().__init__( name )
        
    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Check that goal reached, or failed
        """
#         global DrillSemaphore
        status = check_in_drill( self.drillID )
        if status:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

# == Drill Attached Condition ==

class COND_Drill_Attached( py_trees.behaviour.Behaviour ):
    """ Move the arm until force or distance limit reached, SUCCESS if force condition triggered, otherwise FAILURE """
    
    def __init__( self , ctrl = None , name = "COND_Drill_Attached" ):
        """ 
        Minimal one-time initialisation, offline only: 
        Set the pose that represents the limit of free motion, set the direction of motion 
        """
        self.ctrl   = ctrl # ----- Should be an RMStudio object, will raise an error if not set
        super().__init__( name )
        
    # self.initialise : Inherit `Behaviour`
    
    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Check that goal reached, or failed
        """
        _DEBUG = 0
        
        self.logger.debug( "  %s [COND_Drill_Attached::update()]" % self.name )
        # 1. Return true if the drill is attached
        if self.ctrl.hand.is_drill_attatched():
            if _DEBUG:  print( "Drill attached!" )
            return py_trees.common.Status.SUCCESS
        # 2. Otherwise the drill is not attached
        else:
            if _DEBUG:  print( "Drill DETACHED!" )
            return py_trees.common.Status.FAILURE
        

# __ End Drill Attached __

class COND_Holding_Drill( py_trees.composites.Sequence ):
    """ Are we holding the drill and is it the correct one? """
    
    def __init__( self , drillID , ctrl = None ):
        """ Construct the subtree """
        # NOTE: It is the responsibility of the calling code to determine the starting pose
        # 2020-02-10: For now still considering DOWN to be the only valid insertion direction
        
        super().__init__( name = "COND_Holding_Drill" , memory = 1 )
        self.ctrl    = ctrl
        self.drillID = drillID
        
        self.add_child(
            COND_Drill_Checked_Out( drillID , ctrl = ctrl )
        )
        
        self.add_child(
            COND_Drill_Attached( ctrl = ctrl )
        )
        
class Halt_Drill( py_trees.behaviour.Behaviour ):
    """ Stop the drill, fail if the drill is not attached """
    
    def __init__( self , ctrl = None , name = "Halt_Drill" ):
        """ Attach the RMStudio object """
        self.ctrl = ctrl
        super().__init__( name )
        
    # def initialise: Do nothing
    
    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Return success if the drill is attached, otherwise return failure
        """
        rtnBool = self.ctrl.hand.is_drill_attatched()
        if rtnBool:
            print( "Halt_Drill: Ended with drill ATTACHED!" )
            return py_trees.common.Status.SUCCESS
        else:
            print( "Halt_Drill: Ended with drill DETACHED!" )
            return py_trees.common.Status.FAILURE
    
# === Halt Check Drill Behavior ===

# 1. Stop drilling, fail if not attached
# 2. Set the gripper to a lesser torque
# 3. Thermal check

class Halt_Check_Drill( py_trees.composites.Sequence ):
    """ Stop the drill and relax the hand, fail if there is a problem with drill or hand """
    
    def __init__( self , ctrl = None , Tdefault = 0.5 , name = "Halt_Check_Drill" ):
        """ Add the sequence nodes """
        super.__init__( name )
        self.ctrl = ctrl
        
        # 1. Stop drilling, fail if not attached
        self.add_child(  Halt_Drill( ctrl = ctrl )  )
        # 2. Set the gripper to a lesser torque
        self.add_child(  Set_Grip_Torque( 0.5 , ctrl = ctrl )  )
        # 3. Thermal check 
        self.add_child(  Hand_Thermal_Check( ctrl = ctrl )  )
    

# FIXME: HALT CHECK DRILL

# ___ End Halt Check Drill ___
    
    
# ==== Unhoster Drill Composite ====

# drillPose, zApproach=0.20, zClose=0.040, zRetry=0.040 , maxIter=5, drillPress=None

#  1. Calc the approach poses
#  2. Approach, pre-contact, wide
#  3. Set finger torque to squishy
#  4. Narrow the fingers
#  5. Move down
#  6. Narrow again
#  7. Move to contact
#  8. Close 
#  9. Lift
# 10. Check attached
# 11. If attached, move up, clear of holster
# 12. If not attached, recover
#    A. Move up slightly
#    B. Move down to contact
#    C. Close 
#    D. Check attached

class Unholster_Drill( py_trees.composites.Sequence ):
    """ Pick the drill up from the holster """
    
    def __init__( self , drillPose , drillID , betweenPose = None , 
                 zApproach=0.20, zClose=0.040, zRetry=0.030 , Tpliant = 0.3 , wdthNarrow = 0.038 , maxIter=5, drillPress=4.0 , zFree=0.150 , 
                 ctrl = None ):
        """ Construct the subtree """
        # NOTE: It is the responsibility of the calling code to determine the starting pose
        # 2020-02-10: For now still considering DOWN to be the only valid insertion direction
        
        super().__init__( name = "Unholster_Drill" , memory = 1 )
        self.ctrl    = ctrl
        self.drillID = drillID
        
        # ~~ Add Nodes ~~
        
        #  1. Calc the approach and retry poses
        self.drillPose    = drillPose
        self.approachPose = translate_pose( drillPose , [ 0.0 , 0.0 , zApproach ] , dir_pose='origin')
        self.nearDpthPose = translate_pose( drillPose , [ 0.0 , 0.0 , zClose    ] , dir_pose='origin')
        self.retryLftPose = translate_pose( drillPose , [ 0.0 , 0.0 , zRetry    ] , dir_pose='origin')
        self.pushPastPose = translate_pose( drillPose , [ 0.0 , 0.0 , -0.020    ] , dir_pose='origin')
        self.liftFreePose = translate_pose( drillPose , [ 0.0 , 0.0 , zFree     ] , dir_pose='origin')
        
        # 1.1. If there is an in-between pose, move there
        if is_matx_list( betweenPose ):
            self.add_child(
                Move_Arm( betweenPose , ctrl = ctrl )
            )
        
        #  2. Approach, pre-contact, wide
        self.add_child(
            Move_Arm( self.approachPose , ctrl = ctrl )
        )
        
        # 2020-03-12: TEST EXEC OK
        
        #  3. Set finger torque to squishy
        self.add_child(
            Set_Grip_Torque( torqState = Tpliant , ctrl = ctrl )
        )
        
        # 2020-03-12: TEST EXEC OK
        
        #  4. Narrow the fingers
        self.add_child(
            Set_Fingers( openState = wdthNarrow , ctrl = ctrl )
        )
        
        # 2020-03-12: TEST EXEC OK
        
        #  5. Move down
        self.add_child(
            Move_Arm( self.nearDpthPose , ctrl = ctrl )
        )
        
        # 2020-03-12: TEST EXEC OK
        
        #  6. Narrow again (soft close)
        self.add_child(
            Set_Fingers( openState = 0.0 , ctrl = ctrl )
        )
        
        # 2020-03-12: TEST EXEC OK
        
        #  7. Move to contact
        self.add_child(
            Move_to_Contact( self.pushPastPose , drillPress , speed = 0.0625 , accel = 0.080 , biasWrist = 1 , ctrl = ctrl )
        )
        
        # 2020-03-12: TEST EXEC OK
        
        #  8. Close (hard close)
        self.add_child(
            Set_Grip_Torque( torqState = 1.0 , ctrl = ctrl )
        )
        
        # 2020-03-12: TEST EXEC OK
        
        attachedRecover = py_trees.composites.Selector( name = "Attached_Recover"  )
        
        #  9. Check attached
        attachedRecover.add_child(
            COND_Drill_Attached( ctrl = ctrl )
        )
        
        # 2020-03-12: TEST EXEC OK
        
        # 11. If not attached, recover
        recoverySequenc = py_trees.composites.Sequence( name = "Recovery_Sequence" , memory = 1 )
        
        #    A. Open up slightly
        recoverySequenc.add_child(
            Set_Fingers( openState = wdthNarrow , ctrl = ctrl )
        )
        #    B. Loosen up slightly
        recoverySequenc.add_child(
            Set_Grip_Torque( torqState = Tpliant , ctrl = ctrl )
        )
        #    C. Move up slightly
        recoverySequenc.add_child(
            Move_Arm( self.retryLftPose , ctrl = ctrl )
        )
        
         #    E. Close 
        recoverySequenc.add_child(
            Set_Fingers( openState = 0.0 , ctrl = ctrl )
        )
        
        #    D. Move down to contact
        recoverySequenc.add_child(
            Move_to_Contact( self.pushPastPose , drillPress , speed = 0.0313 , accel = 0.040 , biasWrist = 1 , ctrl = ctrl )
        )
        #    F. Tighten grip
        recoverySequenc.add_child(
            Set_Grip_Torque( torqState = 1.0 , ctrl = ctrl )
        )
        
        recoverySequenc.add_child(
            COND_Drill_Attached( ctrl = ctrl )
        )
        
        
        attachedRecover.add_child(
            recoverySequenc
        )
        
        recLoop = Run_to_X_Failures_DECO(
            attachedRecover ,
            X_allowedFails = maxIter , 
            memory = 1
        )
        
        recOrQuit = py_trees.composites.Selector( name = "Recover_or_Quit"  )
        recOrQuit.add_child( recLoop )
        
        giveUp = py_trees.composites.Sequence( name = "Give_Up" , memory = 1 )
        giveUp.add_child(
            Set_Fingers( openState = wdthNarrow * 1.25 , ctrl = ctrl )
        )
        giveUp.add_child(
            Move_Arm( self.approachPose , ctrl = ctrl )
        )
        giveUp.add_child(
            Set_Fingers( openState = 1.0 , ctrl = ctrl )
        )
        
        recOrQuit.add_child( giveUp )
        
        # Add the recovery subtree
        self.add_child(  recOrQuit  )
        
        
        gotIt = py_trees.composites.Sequence( name = "Got_It" , memory = 1 )
        
        gotIt.add_child(
            COND_Drill_Attached( ctrl = ctrl )
        )
        
        gotIt.add_child(
            Check_OUT_Drill( drillID )
        )
        
        #  5. Success, have drill
        gotIt.add_child(
            Move_Arm( self.nearDpthPose , ctrl = ctrl )
        )
        
        gotIt.add_child(
            Move_Arm( self.liftFreePose , ctrl = ctrl )
        )
        
        if is_matx_list( betweenPose ):
            gotIt.add_child(
                Move_Arm( betweenPose , ctrl = ctrl )
            )
        
        self.add_child(  gotIt  )
        
        # 2020-03-12: TEST EXEC OK
        
        

# ____ End Unholster Drill ____


# ==== Holser Drill Sequence ====

class Holster_Drill( py_trees.composites.Sequence ):
    """ Pick the drill up from the holster """
    
    def __init__( self , drillPose ,  drillID , betweenPose = None , 
                  zApproach=0.20, zClose=0.040, Tpliant = 0.3 , wdthNarrow = 0.038 , maxIter=5, drillPress=4.0 , zFree=0.150 , driverLen = 0.100 ,
                  ctrl = None ):
        """ Construct the subtree """
        # NOTE: It is the responsibility of the calling code to determine the starting pose
        # 2020-02-10: For now still considering DOWN to be the only valid insertion direction
        
        super().__init__( name = "Holster_Drill" , memory = 1 )
        self.ctrl    = ctrl
        self.drillID = drillID
        
        # ~~ Add Nodes ~~
        
        # 0. Precondition: Drill must be attached
        hasDrill = COND_Holding_Drill( drillID , ctrl = ctrl )
        labeled_precondition( self , hasDrill )
        self.add_child(  hasDrill  )
        
        #  1. Calc the approach and retry poses
        self.drillPose    = drillPose
        self.highAbovPose = drillPose.copy()
        self.highAbovPose[2,3] = max( driverLen + 0.050 , zFree )
        self.approachPose = translate_pose( drillPose , [ 0.0 , 0.0 , zApproach ] , dir_pose='origin')
        self.nearDpthPose = translate_pose( drillPose , [ 0.0 , 0.0 , zClose    ] , dir_pose='origin')
        self.liftFreePose = translate_pose( drillPose , [ 0.0 , 0.0 , zFree     ] , dir_pose='origin')
        self.pushPastPose = translate_pose( drillPose , [ 0.0 , 0.0 , -0.020    ] , dir_pose='origin')
        
        self.add_child(
            COND_Drill_Attached( ctrl = ctrl )
        )
        
        # 1.1. If there is an in-between pose, move there
        if is_matx_list( betweenPose ):
            self.add_child(
                Jog_Safe( betweenPose , 
                          zSAFE = max( driverLen + 0.050 , zFree ) ,  
                          hover = 1 ,  
                          ctrl  = self.ctrl )
            )
            
        # 1.2. Pre-Approach,
        self.add_child(
            Move_Arm( self.highAbovPose , ctrl = ctrl )
        )
        
        #  2. Approach, pre-contact, 
        self.add_child(
            Move_Arm( self.approachPose , ctrl = ctrl )
        )
        
        #  5. Move down
        self.add_child(
            Move_Arm( self.nearDpthPose , ctrl = ctrl )
        )
        
        #  7. Move to contact
        self.add_child(
            Move_to_Contact( self.pushPastPose , drillPress , speed = 0.080 , accel = 0.06 , biasWrist = 1 , ctrl = ctrl )
        )
        
        #    B. Loosen up slightly
        self.add_child(
            Set_Grip_Torque( torqState = Tpliant , ctrl = ctrl )
        )
        
        #    A. Open up slightly
        self.add_child(
            Set_Fingers( openState = wdthNarrow , ctrl = ctrl )
        )
        
        #  5. Move down
        self.add_child(
            Move_Arm( self.nearDpthPose , ctrl = ctrl )
        )
        
        #    A. Open up slightly
        self.add_child(
            Set_Fingers( openState = wdthNarrow * 1.25 , ctrl = ctrl )
        )
        
        self.add_child(
            Move_Arm( self.liftFreePose , ctrl = ctrl )
        )
        
        self.add_child(
            Negator_DECO( 
                COND_Drill_Attached( ctrl = ctrl )
            )
        )
        
        # 1.1. If there is an in-between pose, move there
        if is_matx_list( betweenPose ):
            self.add_child(
                Jog_Safe( betweenPose , 
                          zSAFE = max( driverLen + 0.050 , zFree ) ,  
                          hover = 1 ,  
                          ctrl  = self.ctrl )
            )
        
        self.add_child(
            Set_Grip_Torque( torqState = 1.0 , ctrl = ctrl )
        )
        
        self.add_child(
            Check_IN_Drill( drillID )
        )

# ____ End Holster Drill ____

class Set_Drill_Torque( py_trees.behaviour.Behaviour ):
    """ Set the drill torque """

    def __init__( self , drillTorque , ctrl = None ):
        """ Attach the RMStudio object """
        super().__init__( name = "Set_Drill_Torque" )
        self.ctrl = ctrl
        self.torq = drillTorque
        
    # def initialise: Do nothing
      
    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Always return success
        """
        self.ctrl.hand.set_drill_torque( self.torq )
        sleep( 0.25 )
        return py_trees.common.Status.SUCCESS
    
class Turn_Drill_CCW( py_trees.behaviour.Behaviour ):
    """ Turn the drill Counter-ClockWise """

    def __init__( self , ctrl = None ):
        """ Attach the RMStudio object """
        super().__init__( name = "Turn_Drill_CCW" )
        self.ctrl = ctrl
        
    # def initialise: Do nothing
      
    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Always return success
        """
        self.ctrl.hand.turn_drill_ccw()
        sleep( 0.25 )
        return py_trees.common.Status.SUCCESS

class Turn_Drill_CW( py_trees.behaviour.Behaviour ):
    """ Turn the drill ClockWise """

    def __init__( self , ctrl = None ):
        """ Attach the RMStudio object """
        super().__init__( name = "Turn_Drill_CW" )
        self.ctrl = ctrl
        
    # def initialise: Do nothing
      
    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Always return success
        """
        self.ctrl.hand.turn_drill_cw()
        sleep( 0.25 )
        return py_trees.common.Status.SUCCESS

class Simple_Screw_Grab( py_trees.composites.Sequence ):
    """ Retrieve a screw in the simplest way possible """
        
    def __init__( self , screwPose , drillID , 
                  contactF = 2.0 , pressF = 3.0 , hghtAbove = 0.030 , zSAFE = 0.150 , timeOut_s=3.0 , descendSpeed = 0.040 , 
                  relieveT = False , 
                  delayRelief = 0.0 ,
                  ctrl = None ):
        """ Store the screw pose """
        super().__init__( name = "Simple_Screw_Grab" , memory = 1 )
        
        self._DEBUG = 1
        
        self.ctrl      = ctrl
        self.drillID   = drillID
        
        self.screwPose = screwPose
        self.abovePose = translate_pose( screwPose , translation_vec = [ 0.0 , 0.0 ,  hghtAbove     ] , dir_pose='origin' )
        self.belowPose = translate_pose( screwPose , translation_vec = [ 0.0 , 0.0 , -hghtAbove*2.0 ] , dir_pose='origin' )
        self.liftdPose = translate_pose( screwPose , translation_vec = [ 0.0 , 0.0 ,  hghtAbove*2.0 ] , dir_pose='origin' )
        
        if self._DEBUG:
            
            print( "screwPose, Valid?:" , is_pose_mtrx( self.screwPose )  )
            print( self.screwPose )
            print( "abovePose, Valid?:" , is_pose_mtrx( self.abovePose )  )
            print( self.abovePose )
            print( "belowPose, Valid?:" , is_pose_mtrx( self.belowPose )  )
            print( self.belowPose )
            print( "liftdPose, Valid?:" , is_pose_mtrx( self.liftdPose )  )
            print( self.liftdPose )
            
            self.add_child(  Always_Succeed()  )
            
        # 0. Precondition: Drill must be attached
        hasDrill = COND_Holding_Drill( drillID , ctrl = ctrl )
        labeled_precondition( self , hasDrill )
        self.add_child(  hasDrill  )

        # 1. Jog above screw
        self.add_child(
            Jog_Safe( screwPose , 
                      zSAFE = zSAFE ,  
                      hover = 1 ,  
                      ctrl  = self.ctrl )
        )

        # 2. Lower to the approach pose
        self.add_child(  Move_Arm( self.abovePose , ctrl = ctrl )  )

        # 3. Turn the drill on
        self.add_child(  Turn_Drill_CW( ctrl = ctrl )  )

        # 4. Move to contact
        self.add_child(  Move_to_Contact( self.belowPose , contactF , speed = 0.0400 , accel = 0.175 , biasWrist = 1 , ctrl = ctrl )  )

        # 5. Maintain pressure for X seconds
        self.add_child(
            Maintain_Z_Pressure( timeOut_s=timeOut_s , plungeDepth=0.050 , descendSpeed=0.010 , zPress=pressF , maxZStep=0.010 , biasWrist=0 ,
                                 stop_condition=condition_false , stepSleep=0.1 ,
                                 relieveT=relieveT , relieveStep=0.0002 , delayRelief = delayRelief , ctrl=self.ctrl )
        )

        self.add_child(  
            Move_Arm_Relative( translation = [ 0.0 , 0.0 , 0.003 ] , rotation = [ 0.0 , 0.0 , 0.0 ] , 
                               mode = 'l' , speed = 0.012 , accel = 0.15 , 
                               ctrl = self.ctrl , name = "Move_Arm_Relative" )
        )

        # 4. Move to contact
        self.add_child(  Move_to_Contact( self.belowPose , pressF , speed = descendSpeed * 0.5 , accel = 0.175 * 0.25 , biasWrist = 1 , ctrl = ctrl )  )

        # 6. Lift the drill
        self.add_child(  Move_Arm( self.liftdPose , ctrl = ctrl )  )

        # 7. Stop the drill
        self.add_child(  Halt_Drill( ctrl = ctrl )  )

class Drive_Screw( py_trees.composites.Sequence ):
    """ Simple Screwing In Procedure """
    
    def __init__( self , seatPose , driverLen = 0.100 , touch1 = 2.00 , drop1 = 1.50 , insert1 = 2.0 , latStopT1 = 1.20 ,
                  timeout2 = 50.0 , press2 = 3.0 , grabT2 = -0.20 ,
                  angleToStep = 15 , startRadius = 0.0005 , stepSize = 0.0002 , drillT = 50 , descendSpeed = 0.04 , 
                  delayRelief = 0.0 , 
                  disableBackup = 0 ,
                  ctrl = None ):
        """ Set params and build tree """
        super().__init__( name = "Drive_Screw" , memory = 0 )
        
        self.ctrl      = ctrl
        self.abovePose = translate_pose( seatPose , translation_vec = [ 0.0 , 0.0 ,  driverLen ] , dir_pose='origin' )
        self.belowPose = translate_pose( seatPose , translation_vec = [ 0.0 , 0.0 , -driverLen ] , dir_pose='origin' )
        
        # A. Precondition: Is the drill attached?
        attch = COND_Drill_Attached( ctrl = ctrl )
        labeled_precondition( self , attch )
        self.add_child(  attch  )
        
        # B. Subtree: Linear screw procedure
        screwSeq = py_trees.composites.Sequence( name = "Screw_Sequence" , memory = 1 )
        
        # 1. Set hand and drill torques
        screwSeq.add_child(  Set_Grip_Torque(  torqState   = 1.0    , ctrl = ctrl )  )
        screwSeq.add_child(  Set_Drill_Torque( drillTorque = drillT , ctrl = ctrl )  )
        
        # 2. Turn on the drill
        screwSeq.add_child(  Turn_Drill_CW( ctrl = ctrl )  )
        
#         # 3. Spiral Insert
#         screwSeq.add_child(
#             Spiral_Search( initPose = self.abovePose , poseKey = "" , 
#                   touch_force = touch1 , drop_force = drop1 , insert_force = insert1 , max_movement = 0.1 , 
#                   lateralStopTorque = latStopT1 , pushbackF = 25.0 , 
#                   spiralSpeed = 0.002 , descendSpeed = 0.003 , biasWrist = 0 ,
#                   degrees_to_step = angleToStep , start_radius = startRadius , step_size = stepSize , 
#                   ctrl = ctrl )
#         )

        screwSeq.add_child(
            Move_to_Contact( pose = self.belowPose , Fmag = 1.5 , biasWrist = 1 , 
                             mode = 'l' , speed = descendSpeed , accel = 0.175,
                             ctrl = self.ctrl )
        )
        
        # 3. Press while driving
        condFunc = self.ctrl.exceeds_Z_torque( grabT2 , absolute = 1 )
        screwSeq.add_child(
            Maintain_Z_Pressure( timeOut_s=timeout2 , plungeMove = 0 , 
                                 descendSpeed=0.009 , zPress=press2 , maxZStep=0.010 , biasWrist=0 ,
                                 stop_condition=condFunc , stepSleep=0.1 ,
                                 relieveT=1 , relieveStep=0.0002 , stepPress = stepSize , 
                                 delayRelief = delayRelief , 
                                 ctrl = ctrl )
        )
        
        
        if not disableBackup:
            # 4. Releive pressure
            screwSeq.add_child(  Turn_Drill_CCW( ctrl = ctrl )  )
            screwSeq.add_child(  Timer( duration = 0.05 )  )
        
        
        # 5. Stop
        screwSeq.add_child(  Halt_Drill( ctrl = ctrl )  )
        if 1:
            screwSeq.add_child(  Set_Grip_Torque( torqState = 0.5 , ctrl = ctrl )  )
        # 7. Lift
        screwSeq.add_child(  
            Move_Arm_Relative( 
                translation = [ 0.0 , 0.0 , 0.010 ] , 
                ctrl = ctrl 
            )
        )
        # 6. Add `screwSeq` to main tree
        self.add_child(  screwSeq  )
        
        # FIXME: ADD A WIN CONDITION - MET THE FINAL Z TORQUE
        
class Basic_Unscrew( py_trees.composites.Sequence ):
    """ Unscrew """
    # NOTE: This behavior assumes that the drill is held above a screw
    
    def __init__( self , plungeDist , touchForce , zPress , engageTO, engageTz , unscrewTO ,
                  stepSize = 0.0005 , ctrl = None , 
                  reverseEngageRelief = 0 , 
                  name = "Basic_Unscrew" ):
        """ Build a sequence that will unscrew """
        super().__init__( name , memory = 1 )
        self.ctrl = ctrl
        
        # NOTE: This behavior assumes that you are already above the screw
        
        # A. Precondition: Is the drill attached?
        attch = COND_Drill_Attached( ctrl = ctrl )
        labeled_precondition( self , attch )
        self.add_child(  attch  )
        
        # 0. Set hand and drill torques
        self.add_child(  Set_Grip_Torque(  torqState   = 1.0    , ctrl = ctrl )  )
        self.add_child(  Set_Drill_Torque( drillTorque = 75 , ctrl = ctrl )  )
        
        condFunc = ctrl.exceeds_Z_torque( engageTz , absolute = 1 )
        
        # 2. Move to Contact ( If specified )
        if plungeDist > 0.0:
            
            # 1. Turn on drill
            self.add_child(  Turn_Drill_CW( ctrl = ctrl )  )
            
            self.add_child( 
                Move_to_Contact( Fmag = touchForce , relMove = [ 0.0 , 0.0 , -plungeDist ] , biasWrist = 1 , 
                                 mode = 'l' , speed = 0.04 , accel = 0.17 , 
                                 ctrl = ctrl , _DEBUG = 0 )
            )
        
            # If we are skipping move to contact assume that the screw is engaged so skip maintain_z_press as well
        
            # 3. Maintain pressure with a torque condition
            self.add_child(
                Maintain_Z_Pressure( timeOut_s = engageTO , plungeMove = 0 , 
                                     descendSpeed = 0.009 , zPress = zPress , maxZStep = 0.010 , biasWrist = 0 ,
                                     stop_condition = condFunc , stepSleep = 0.1 ,
                                     relieveT = 1 , relieveStep = 0.0002 , stepPress = stepSize , ctrl = ctrl,
                                     reverseRelief = reverseEngageRelief )
            )

        # 4. Stop drill
        self.add_child(  Halt_Drill( ctrl = ctrl )  )
        
        # 5. self.set_drill_torque(200)
        self.add_child(  Set_Drill_Torque( drillTorque = 200 , ctrl = ctrl )  )
        
        # 6. self.turn_drill_ccw()
        self.add_child(  Turn_Drill_CCW( ctrl = ctrl )  )
        
        # 7. Maintain pressure with timeout
        self.add_child(
            Maintain_Z_Pressure( timeOut_s = unscrewTO , plungeMove = 0 , 
                                 descendSpeed = 0.009 , zPress = zPress , maxZStep = 0.010 , biasWrist = 0 ,
                                 stop_condition = condFunc , stepSleep = 0.1 ,
                                 relieveT = 1 , relieveStep = 0.0002 , stepPress = stepSize , ctrl = ctrl )
        )
        
        # 8. Move up
        self.add_child(
            Move_Arm_Relative( translation = [ 0.0 , 0.0 , plungeDist ] , rotation = [ 0.0 , 0.0 , 0.0 ] , 
                               mode = 'l' , speed = 0.125 , accel = 0.35 , stop_cond = None , frame = 'origin' ,
                               ctrl = ctrl )
        )

from pmath import bases_from_pose
        
class Probe_for_Screw( py_trees.composites.Sequence ):
    
    def __init__( self , 
                  screwTopPose , boardPose , approachZ , searchZpad , touchForce , zPress , engageTz ,
                  engageTO = 1.5 , 
                  degrees_to_step = 18.0, 
                  start_radius = 0.002 , step_size=0.001, max_angle=100000, max_radius=100000 , 
                  iterLim = 20 , stepSize = 0.0005 ,
                  ctrl = None , drillLen = 0.080 , deckZ = 0.038 ,
                  descendSpeed = 0.04 , drillTorque = 75 ,
                  name = "Probe_for_Screw" ):
        """ Create the probing sequence """
        super().__init__( name , memory = 1 )
        
        poseOffsetKey = "grabPoseDiff"
        
        screwTopWTool = screwTopPose.copy()
        screwTopWTool[2,3] = drillLen + deckZ
                
        searchBgnPose = translate_pose( screwTopWTool.copy() , [ 0.0 , 0.0 , searchZpad ] , dir_pose = 'origin' )
        
        self.genr = SpiralStep( initPose = searchBgnPose , 
                                degrees_to_step = degrees_to_step ,
                                start_radius = start_radius , step_size = step_size , 
                                max_angle = max_angle , max_radius = max_radius )
        
        xDir , yDir , _ = bases_from_pose( boardPose )
        
        # 0. Move above the screw
        if approachZ > 0.0:
            abovePose = translate_pose( screwTopWTool.copy() , [ 0.0 , 0.0 , approachZ ] , dir_pose = 'origin' )
            self.add_child(  
                Move_Arm( pose = abovePose , mode = 'l' , speed = 0.0625 , accel = 0.17 , ctrl = ctrl )  
            )
        
        # 0. Set hand and drill torques
        self.add_child(  Set_Grip_Torque(  torqState   = 1.0    , ctrl = ctrl )  )
        self.add_child(  Set_Drill_Torque( drillTorque = drillTorque , ctrl = ctrl )  )
        
        self.add_child(  
            Move_Arm( pose = searchBgnPose , mode = 'l' , speed = 0.0625 , accel = 0.17 , ctrl = ctrl )  
        )
        
        expectedLocationKey = "CalculatedPose"
        
        self.add_child( 
            Store_Current_Pose( keyString = expectedLocationKey , ctrl = ctrl , setOnce = False )
        )
        
        correctedKey = "OffsetPoseKey"
        
        correctiveSeq = py_trees.composites.Sequence( name = "Corrective_Seq" , memory = 1 )
        # A. Check to see if the previous operation was a success
        correctiveSeq.add_child(  BB_Flag_COND( BB_key = "screwGrabSuccess" )  )
        # B. If it was, store the correction
        def store_corrected_pose( self , *args ):
            expectedPose = ASMBB.get( expectedLocationKey )
            offset       = ASMBB.get( poseOffsetKey )
            storePose    = expectedPose.copy()
            storePose[0,3] += offset[0]
            storePose[1,3] += offset[1]
            ASMBB.set( correctedKey , storePose )
            return True
        correctiveSeq.add_child(  Run_Py_Func( store_corrected_pose , None )  )
        # C. Move to the corrected pose
        correctiveSeq.add_child(  Move_Arm( BB_key = correctedKey , ctrl = ctrl )  )
        
        self.add_child(  FailureIsSuccess(  correctiveSeq  )  )
        
        finderSeq = py_trees.composites.Sequence( name = "Finder_Seq" , memory = 1 )
        # NOTE: The finder sequence will succeed if the screw is engaged
        
        
        # 1. Turn on drill
        finderSeq.add_child(  Turn_Drill_CW( ctrl = ctrl )  )
        
        # A. Move to Contact
        finderSeq.add_child( 
            Move_to_Contact( Fmag = touchForce , relMove = [ 0.0 , 0.0 , -searchZpad * 4.0 ] , biasWrist = 1 , 
                             mode = 'l' , speed = descendSpeed , accel = 0.17 , 
                             ctrl = ctrl , _DEBUG = 0 )
        )
        
        # B. Maintain pressure
        condFunc = ctrl.exceeds_Z_torque( engageTz , absolute = 1 )
        finderSeq.add_child( 
            Maintain_Z_Pressure( timeOut_s = engageTO , plungeMove = 0 , 
                                 descendSpeed = 0.009 , zPress = zPress , maxZStep = 0.010 , biasWrist = 0 ,
                                 stop_condition = condFunc , stepSleep = 0.1 ,
                                 relieveT = 0 , relieveStep = 0.0002 , stepPress = stepSize , ctrl = ctrl,
                                 reverseRelief = 0 , ft_cond = 1 , cond_success = 1 )
        )
        
        finderSeq.add_child(  Halt_Drill( ctrl = ctrl )  )
        
        # NOTE: If `finderSeq` has made it this far, we have grabbed the screw
        lastGrabKey = "UnscrewSuccessPose"
        
        finderSeq.add_child( 
            Store_Current_Pose( keyString = lastGrabKey , ctrl = ctrl , setOnce = False )
        )
        
        def calc_offset_from_pose1_to_pose2( self , *args ):
            expectdPose = ASMBB.get( expectedLocationKey )
            grabbedPose = ASMBB.get( lastGrabKey )
            poseDiff    = np.subtract( grabbedPose , expectdPose )
            diffVec     = poseDiff[0:3,3]
            ASMBB.set( poseOffsetKey , diffVec )
            return True
            
        finderSeq.add_child(  Run_Py_Func( calc_offset_from_pose1_to_pose2 , None )  )
        finderSeq.add_child(  Set_BB_Key( "screwGrabSuccess" , True )  )
        
        finderSel = py_trees.composites.Selector( name = "Finder_Sel" , memory = 1 )
        finderSel.add_child(  finderSeq  )
        finderSel.add_child(  SuccessIsFailure(  Set_BB_Key( "screwGrabSuccess" , False )  )  )
        
        recoverSeq = py_trees.composites.Sequence( name = "Recover_Seq" , memory = 1 )
        # NOTE: The recover sequence will ALWAYS FAIL
        
        # C. Relmove Up
        recoverSeq.add_child(
            Move_Arm_Relative( translation = [ 0.0 , 0.0 , searchZpad ] , rotation = [ 0.0 , 0.0 , 0.0 ] , 
                               mode = 'l' , speed = 0.04 , accel = 0.12 , stop_cond = None , frame = 'origin' ,
                               ctrl = ctrl )
        )
        
        def always_false( *args ):
            return False
        
        # D. Next waypoint
        recoverSeq.add_child(
            Move_Rule_w_Stop_Cond( 
                self.genr , 
                always_false , 
                condSuccess = True ,
                ctrl = ctrl
            )
        )
        
        probeSel = py_trees.composites.Selector( name = "Probe_Sel" , memory = 1 )
        probeSel.add_child(  finderSel   )
        probeSel.add_child(  recoverSeq  )
        
        self.add_child(  
            Run_to_X_Failures_DECO( 
                probeSel ,
                X_allowedFails = iterLim ,
                memory = 0              ,
                name = "Probe_Loop"    
            )
        )
        
        # 1. Turn off drill
        self.add_child(  Halt_Drill( ctrl = ctrl )  )