##### Imports #####
from asm_tree_Basic import *


##### Functions #####

def labeled_precondition( parent , cond , addChild = False ):
    """ Add the behavior to a specific, labeled attribute of the parent """
    if not hasattr( cond , "labels" ):
        cond.labels = []
    cond.labels.append( 'PRECONDTION' )
    parent.preCond = cond
    if addChild:
        parent.add_child( cond )

        
##### Behaviors ##### ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# === Pose Reached Condition ===

class COND_At_TCP_Pose( py_trees.behaviour.Behaviour ):
    """ Move the arm until force or distance limit reached, SUCCESS if force condition triggered, otherwise FAILURE """
    
    def __init__( self , targetPose , posnMargin = 0.003 , orntMargin = 2.0/180 , ctrl = None , name = "COND_At_TCP_Pose" ):
        """ 
        Minimal one-time initialisation, offline only: 
        Set the pose that represents the limit of free motion, set the direction of motion 
        """
        self.pose   = targetPose # Check if the TCP is here
        self.posErr = posnMargin # Allowed translational deviation [mm] from pose
        self.ornErr = orntMargin # Allowed angular deviation [1=180deg] from pose
        self.ctrl   = ctrl # ----- Should be an RMStudio object, will raise an error if not set
        self._DEBUG = False
        super().__init__( name )
        
    # self.initialise : Inherit `Behaviour`
    
    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Check that goal reached, or failed
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
                if self._DEBUG:  print( "COND_At_TCP_Pose , SUCCEEDED: Reached the pose" )
                return py_trees.common.Status.SUCCESS
            # C. Otherwise the goal has not been reached
            else:
                if self._DEBUG:  print( "COND_At_TCP_Pose , FAILED: Was at pose\n" , currPose , "\nwhile trying to reach\n" , self.pose )
                return py_trees.common.Status.FAILURE
        # 2. Otherwise the robot arm has not stopped, assume not at 
        else: 
            return py_trees.common.Status.FAILURE


# ___ End Pose Reached ___

class Hand_Thermal_Check( py_trees.behaviour.Behaviour ):
    """ Check that the temperature limits of the hand motors have not been exceeded """
    
    def __init__( self , ctrl = None , name = "Hand_Thermal_Check" ):
        """ Attach the RMStudio object """
        self.ctrl = ctrl
        super().__init__( name )
        
    # def initialise: Do nothing
    
    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Return success if motor temperatures are okay, otherwise return failure
        """
        try:
            rtnBool = self.ctrl.hand.opencm_thermal_check()
            return py_trees.common.Status.SUCCESS
        except:
            return py_trees.common.Status.FAILURE



        
class At_Z_Level_COND( py_trees.behaviour.Behaviour ):
    """ Return True if the TCP is within margin of some z-height """
    # TODO: GENERALIZE TO AT_PLANE_COND
    
    def __init__( self , zLevel , margin = 0.010 , ctrl = None ):
        """ Set the level and the margin """
        super().__init__( name = "At_Z_Level_COND" )
        self.zLevel = zLevel
        self.margin = margin
        self.ctrl   = ctrl
        
    def update( self ):
        """ Return True if the TCP is within margin of some z-height, Otherwise return False """
        currPose = self.ctrl.arm.get_tcp_pose()
        currZlvl = currPose[2,3]
        if abs( currZlvl - self.zLevel ) <= self.margin:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE



class Test_TCP_Extent( py_trees.composites.Sequence ):
    """ Test how far beyond the TCP the end of an object is """
    
    def __init__( self , testPose , zSAFE = 0.150 , zLevel = 0.010 , zMargin = 0.005 , testZpress = 2.0 , testDepth = 0.100 , ctrl = None ):
        """ Store the safe test space """
        super().__init__( name = "Test_TCP_Extent" , memory = 1 )
        self.ctrl         = ctrl
        self.pushPastPose = translate_pose( testPose , translation_vec = [ 0.0 , 0.0 , -testDepth ] , dir_pose='origin' )
        
        # 1. Move above the test point
        self.add_child(
            Jog_Safe( testPose , 
                      zSAFE = zSAFE ,  
                      hover = 1 ,  
                      ctrl  = self.ctrl )
        )
        
        # 2. Move to contact onto the test point
        self.add_child(  Move_to_Contact( self.pushPastPose , testZpress , speed = 0.0625 , accel = 0.175 , biasWrist = 1 , ctrl = ctrl )  )
        self.add_child(  At_Z_Level_COND( zLevel , zMargin , ctrl = ctrl )  )
        
        # 2020-03-28: Tested OK
        
class BB_Flag_COND( py_trees.behaviour.Behaviour ):
    """ Evaluate a BB value as truth, a way for BTs to flip each other's switches """
    
    def __init__( self , BB_key , name = "BB_Flag_COND"  ):
        """ Store the key """
        super().__init__( name )
        self.BB_key = BB_key
        
    def update( self ):
        """ Return True if BB key can be interpreted as true, otherwise return false """
        if ASMBB.exists( self.BB_key ):
            if ASMBB.get( self.BB_key ):
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        else:
            ASMBB.set( self.BB_key , False )
            return py_trees.common.Status.FAILURE