##### Imports #####
from rmlib.rmtools.asm_BT_lib.asm_tree_Basic import *
from rmlib.rmtools.asm_BT_lib.asm_tree_cond_test import *
from rmlib.rmtools.asm_BT_lib.asm_tree_logic_flow import Run_to_X_Failures_DECO , Negator_DECO
from utils import nowTimeStamp
import os , inspect , time
from math import floor , ceil

from rmlib.rmtools.utils import condition_false

##### Functions #####
        
def status_as_label( status ):
    """ Return the status as a label """
    try:
        return {  
            py_trees.common.Status.FAILURE : 0 , 
            py_trees.common.Status.SUCCESS : 1 , 
        }[ status ]
    except:
        return None
    
def get_XML_outfile_namer( pathPrefix = "" , namePrefix = "XML" ):
    """ Return a function that generates filenames based on the current time """
    def gen_tstamp_fname():
        """ Return a filename that incorporates the enclosed prefixes """
        return str(  os.path.join( pathPrefix ,  namePrefix+nowTimeStamp()+".xml" )  )
    return gen_tstamp_fname
        
##### Behaviors ##### ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class Tamp_Alternating( py_trees.composites.Sequence ):
    """ Attempt to remedy a jammed insertion """
    
    def __init__( self , zGoal , zMargin = 0.005 , tap_force = 3.0 , plungeDistance = 0.070 , tapWidth = 0.020 ,
                  twist_deg = 10 , doubleSided = 1 , retract_m = None , 
                  descendSpeed = 0.0125 , Ntries = 6 , TzStop = 1.8 , ctrl = None ):
        """ Build the subtree """
        super().__init__( name = "Tamp_Alternating" , memory = 1 )
        
        
        # -2. Retract if requested
        if retract_m != None:
             self.add_child(  Move_Arm_Relative( translation = [ 0 , 0 , retract_m ] , speed = descendSpeed , ctrl = ctrl )  )
            
        
        # -1. set finger width
        self.add_child(  Set_Fingers( openState = tapWidth , ctrl = ctrl )  )
        
        # 0. Instantiate tamp subtree
        pushTwist = py_trees.composites.Sequence( name = "Push_and_Twist_Seq" , memory = 1 )

        # 1. Retract
        pushTwist.add_child(  Move_Arm_Relative( translation = [ 0 , 0 , 0.25*plungeDistance ] , speed = descendSpeed , ctrl = ctrl )  )
        
        # 2. Twist
#         def cond_Tz( *args ):
#             """ Stop at the prescribed wrist torque """
#             wrench = ctrl.ft.get_wrist_force()
#             if abs( wrench[5] ) > TzStop:
#                 return 1
#             else:
#                 return 0
        def cond_Tz( *args ):
            return 0
        
        if 1:
            pushTwist.add_child(  Move_Rule_w_Stop_Cond( 
                TwistToggleRelGen( twist_deg = twist_deg , doubleSided = doubleSided , N = Ntries , ctrl = ctrl ) , 
                stop_cond    = cond_Tz , 
                condSuccess  = False   , 
                relativeRule = 0       , 
                ctrl         = ctrl 
            )  )
        
        # 3. Tamp
        pushTwist.add_child(  Move_to_Contact( relMove = [ 0 , 0 , -plungeDistance ] , Fmag = tap_force , speed = descendSpeed , biasWrist = 1 , ctrl = ctrl )  )
        
        # 4. Check z level
        pushTwist.add_child(  At_Z_Level_COND( zGoal , margin = zMargin , ctrl = ctrl )   )
        
        # 5. In a loop
        self.add_child(  
            Run_to_X_Failures_DECO(  
                pushTwist ,
                X_allowedFails = Ntries
            )
        )

class centering_XY_offset:
    """ Get a pose that will  """
    
    def __init__( self , ctrl , marginT=0.01 , moveStep=0.002 ):
        """ Calculate a move that relieves torque on the wrist, does not move """
        # NOTE: This function assumes that the gripper is oriented vertically
        self.ctrl     = ctrl
        self.marginT  = marginT
        self.moveStep = moveStep

    def __call__( self ):
        wrench = self.ctrl.ft.get_wrist_force()
        diffX  = wrench[4] # Translate perpendicular to torque axis
        diffY  = wrench[3]
        handPose = self.ctrl.arm.get_tcp_pose()
        goPose = handPose.copy()
        if abs(diffX) > self.marginT:
#             self.logger.debug("Centering X!")
            if diffX > 0:  # If more right, then move left
                goPose = translate_pose(goPose, translation_vec = [ +self.moveStep , 0.0 , 0.0 ], dir_pose='self')
            else:  # else more left, go right
                goPose = translate_pose(goPose, translation_vec = [ -self.moveStep , 0.0 , 0.0 ], dir_pose='self')
        if abs(diffY) > self.marginT:
#             self.logger.debug("Centering Y!")
            if diffY > 0:  # If more right, then move left
                goPose = translate_pose(goPose, translation_vec = [ 0.0 , -self.moveStep , 0.0 ], dir_pose='self')
            else:  # else more left, go right
                goPose = translate_pose(goPose, translation_vec = [ 0.0 , +self.moveStep , 0.0 ], dir_pose='self')
        return goPose
    
class Minimize_Wrist_Torque_XY( py_trees.composites.Sequence ):
    """ Make XY Moves until XY torques are minimized """
    
    def __init__( self , name = "Minimize_Wrist_Torque_XY" , numTrials = 10 , marginT=0.01 , moveStep=0.002  , ctrl = None ):
        """ Set params """
        super().__init__( name = name , memory = True )
        self.ctrl = ctrl
        self.marginT = marginT
        self.moveStep = moveStep
        
        # 1. Mark where we began the operations
        poseRecorder = Store_Current_Pose( ctrl = self.ctrl )
        self.add_child(
            poseRecorder
        ) # NOTE: `Store_Current_Pose` is store-once by default
        self.poseKey = poseRecorder.key # Get the dictionary key where this was stored
        
        # 2. Init the generator
        self.genr = centering_XY_offset( ctrl = self.ctrl , marginT=marginT , moveStep=moveStep )
        
        forceFunc = self.ctrl.ft.get_wrist_force
        margin    = self.marginT
        
        def minimized( *args ):
            wrench = forceFunc()
#             print( "Minimized is running, Wrench is:" , wrench )
            diffX  = wrench[4] # Translate perpendicular to torque axis
            diffY  = wrench[3]
            if abs(diffX) <= margin and abs(diffY) <= margin:
#                 print( "torques MINIMIZES" )
                builtins._GLOB_FT_FLAG = 1
                return 1
            else:
                return 0
        
        # 3. Spiral Search is a series of small arm movements under a condition
        self.add_child(
            Run_to_X_Failures_DECO(
                Move_Rule_w_Stop_Cond( self.genr , 
                                       minimized , 
                                       condSuccess = True , 
                                       fetchInit = "" , 
                                       ctrl = self.ctrl 
                                     ) , 
                X_allowedFails = numTrials
            )
        )
    
    def initialise( self ):
        builtins._GLOB_FT_FLAG = 0

class Record_Classify( py_trees.behaviour.Behaviour ):
    """ Node to monitor forces and return FAILURE as soon as it detects one based on a previously trained classifier """
    
    def __init__( self , recordFlagKey = "RECORD_KEY" , dataKey = "RECORDING" , recordHz = 10 ,
                  ctrl = None , name = "Record_Classify" ): # TODO: ADD CLASSIFIER
        """ 
        Minimal one-time initialisation, offline only: 
        """
        # 0. Init super
        super().__init__( name )
        # 1. Set params
        self.ctrl      = ctrl
        self.recording = False
        self.data      = []
        self.key       = dataKey
        self.flagKey   = recordFlagKey
        self._EXTRA    = False
#         ASMBB.set( self.key , [] )

        #print( "Record_Classify created with record flag" , self.flagKey , "and data key" , self.key )

        def get_ft_datum():
            """ Get a force snapshot """
            data = ASMBB.get( self.key )
#             print( "Fetched" , data )
            data.append(  ( datetime.now(tz=mst) , self.ctrl.ft.get_wrist_force() )  )
#             print( "Appended" , data )
            ASMBB.set( self.key , data )
        #print( "Created worker:" , get_ft_datum )
        
        self.workQ = TimerQueue( workFunc = get_ft_datum , updateHz = recordHz )
        
#         print( "Record_Classify init with data key" , self.key , "and flag key" , self.flagKey )
        
    def initialise( self ):
        """
        First time your behaviour is ticked or not RUNNING: 
        Set flag to begin recording
        """
        self.logger.debug( "  %s [Record_Classify::initialise()]" % self.name )
        
    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Take a data point
        """
        self.logger.debug( "  %s [Record_Classify::update()]" % self.name )
        
        if self._EXTRA:  print( "Flag" , self.flagKey , "Value:" , ASMBB.get( self.flagKey ) , end="~ " )
        
        recBool = ASMBB.get( self.flagKey )
        
        if recBool:
            if self._EXTRA:  print( "Flag" , self.flagKey , "is now TRUE, Recording?" , self.recording )
            if not self.recording:
                if self._EXTRA:  print( "About to record" )
                self.recording = True
                self.workQ.start()
        else:
            if self._EXTRA:  print( "Flag" , self.flagKey , "is now FALSE, Recording?" , self.recording )
            if self.recording:
                self.recording = False
                self.workQ.stop()
                sleep( 0.25 ) # REQUIRED HACK : Allows thread to stop and for recording to happen # REQUIRED
                self.workQ.stop()
                sleep( 0.25 ) # REQUIRED HACK : Allows thread to stop and for recording to happen # REQUIRED
                
#             data = ASMBB.get( self.key )
#             data.append(  ( datetime.now(tz=mst) , self.ctrl.ft.get_wrist_force() )  )
#             ASMBB.set( self.key , data )
                
        if 0:
            print( "\tParent Status:" , self.parent.status , "Recording:" , self.recording )
            print( "\tThere are" , len(  ASMBB.get( self.key ) ) , "data points"  )
            print( "\t" , self.ctrl.ft.get_wrist_force() )
            print( "\t" ,  ASMBB.get( self.key ) )
            
        return py_trees.common.Status.SUCCESS

class Sequence_Recorder( py_trees.composites.Sequence ):
    """ A sequence that records its results """
    
    def __init__( self , memory = 0 , dataKey = "RECORD" , flagKey = "RECORD_KEY" , outfilePath = "output.xml" , outFileGenFunc = None ,  
                  name = "Sequence_Recorder" ):
        """ Create the composite and inherit """
        # 0. Super Init
        super().__init__( memory = memory , name = name )
        # 1. Set params and keys
        self.dataKey     = dataKey
        self.flagKey     = flagKey
        self.outfile     = outfilePath
        self.outFileGenF = outFileGenFunc
        
        #print( "Sequence_Recorder created with record flag" , self.flagKey , "and data key" , self.dataKey )
        
    def initialise( self ):
        """ Get the blackboard ready to record data """
        # 2. Set or generate output filename
        if self.outFileGenF != None:
            self.outfile = self.outFileGenF()
        # 3. Set 
        #print( "Sequence_Recorder: Clearing the keys!" )
        ASMBB.set( self.flagKey , False )
        ASMBB.set( self.dataKey , []    )
        
    def terminate( self , new_status ):
        """ Write to file while recording the final status """
#         print( "about to TERMINATE" )
        w_data = {
            "data"  : ASMBB.get( self.dataKey ) , # ---- 1. Fetch data
            "label" : status_as_label( self.status ) , # 2. Fetch label
        }
        print( "about to WRITE to" , self.outfile , end = "" )
        # 3. Open file
        with open( self.outfile , 'w' ) as outf:
            # 4. Write label && data
            outf.write(  dicttoxml( w_data ).decode()  )
        print( ", writing COMPLETE" )

# ==== Tilt Insert Sequence ====

# * Sequence
#   0. Conditional: At beginning pose
#   1. Offset move
#   2. Move down to contact
#   3. Offset move
#   4. Push insert Z with force limit ( Move down to contact, with a greater limit )


class Tilt_Insert( py_trees.composites.Sequence ):
    """ For largish dia. (>= 10mm), cylindrical peg-in-hole problems """
    
    def __init__( self , beginPose ,
                  tilt_angle_deg = 5 , part_offset = 0 , dia = 0 , 
                  touch_force = 1 , insert_force = 2, max_movement = 0.1 , biasWrist = 1 , offset_dir = np.array([1.0,0.0,0.0]), 
                  posnMargin = 0.003 , orntMargin = 2.0/180 , finalInsertHandZ = None , recordFlagKey = "RECORD_KEY" , dropSpeed = 0.125 , 
                  ctrl = None , hackFactor1 = 1.35 , hackFactor2 = 0.10 ):
        """ Construct the subtree """
        # NOTE: It is the responsibility of the calling code to determine the starting pose
        # 2020-02-10: For now still considering DOWN to be the only valid insertion direction
        super().__init__( name = "Tilt_Insert" , memory = 1 )
        
        print( "Tilt_Insert: dropSpeed =" , dropSpeed )
        
        self.ctrl = ctrl
        self.recordFlagKey = recordFlagKey
        
        #print( "Tilt_Insert created with record flag" , self.recordFlagKey )
        
        # ~~ Add Nodes ~~
        
        # 0. Conditional: At beginning pose
        self.add_child(
            COND_At_TCP_Pose( beginPose , posnMargin = posnMargin , orntMargin = orntMargin , ctrl = ctrl )
        )
        
        # 1. Offset move
        # A. Calculate the end pose
        x_offset = ( part_offset ) * math.sin( math.radians( tilt_angle_deg ) ) - ( dia / 2 ) * math.cos( math.radians( tilt_angle_deg ) ) 
        print( "Calculated an X offset:" , x_offset )
        dX       = translate_pose( beginPose , offset_dir * x_offset , dir_pose='self') # origin
        dX       = rotate_pose( dX , [ 0.0 , math.radians( tilt_angle_deg ) , 0.0 ] , dir_pose='self') # origin
        self.add_child(
            Move_Arm( dX , speed = dropSpeed , ctrl = ctrl )
        )
        
        
        self.add_child(
            Set_Grip_Torque( torqState = 1.0 , ctrl = ctrl )
        )
        
        # 2. Move down to contact
        endPose = translate_pose( dX , [ 0.0 , 0.0 , -max_movement ] , dir_pose='origin')
        self.add_child(
            Move_to_Contact( endPose , touch_force , biasWrist = biasWrist , speed = dropSpeed , accel = 0.175 , ctrl = ctrl )
        )
        
#         self.add_child(  Set_BB_Key( self.recordFlagKey , True )  )
        
#         hackFactor = 1.45

#         rotDeg_1 = -tiltAngleDeg * hackFactor1
#         rotDeg_2 =  tiltAngleDeg * ( hackFactor1 - 1.0 )
#         trnMtr_1 = -x_offset * hackFactor1

#         rotDeg_1 =  tilt_angle_deg * hackFactor1
#         rotDeg_2 = -tilt_angle_deg * ( hackFactor1 - 1.0 )
#         trnMtr_1 = offset_dir *  x_offset * hackFactor1
#         trnMtr_2 = 0.0
    
        print( "Offset Rotate 1:" ,  )
    
        # 3. Offset move
        self.add_child(
            Move_Arm_Relative( 
                translation = np.multiply( offset_dir , -1.0 * x_offset ) , #trnMtr_1 , 
#                 rotation = [ 0.0 , math.radians( tilt_angle_deg * ( 1.0- hackFactor1 ) ) , 0.0 ] , 
                rotation = [ 0.0 , math.radians( tilt_angle_deg ) + hackFactor1 , 0.0 ] , 
                frame = 'self' ,
                speed = 0.025 , # HACK
                ctrl = ctrl 
            )
        )
        
        self.add_child(
            Move_Arm_Relative( 
                translation = np.multiply( offset_dir , -1.0 * x_offset ) , #trnMtr_1 , 
#                 rotation = [ 0.0 , math.radians( tilt_angle_deg * ( 1.0- hackFactor1 ) ) , 0.0 ] , 
                rotation = [ 0.0 , hackFactor1 , 0.0 ] , 
                frame = 'self' ,
                speed = 0.025 , # HACK
                ctrl = ctrl 
            )
        )
        
#         self.add_child(
#             Move_Arm_Relative( 
#                 translation = np.multiply( offset_dir , -1.0 * x_offset ) , #trnMtr_1 , 
# #                 rotation = [ 0.0 , math.radians( tilt_angle_deg * ( 1.0- hackFactor1 ) ) , 0.0 ] , 
#                 rotation = [ 0.0 , -hackFactor1 , 0.0 ] , 
#                 frame = 'self' ,
#                 speed = 0.025 , # HACK
#                 ctrl = ctrl 
#             )
#         )
        
#         self.add_child(
#             Move_Arm_Relative( 
#                 translation = np.multiply( offset_dir , -1.0 * x_offset ) , #trnMtr_1 , 
# #                 rotation = [ 0.0 , math.radians( tilt_angle_deg * ( 1.0- hackFactor1 ) ) , 0.0 ] , 
#                 rotation = [ 0.0 , 1.0 * hackFactor1 * math.radians( tilt_angle_deg ) , 0.0 ] , 
#                 frame = 'self' ,
#                 speed = 0.025 , # HACK
#                 ctrl = ctrl 
#             )
#         )
        
#         print( "Offset Rotate 2:" ,  )
        
#         self.add_child(
#             Move_Arm_Relative( 
#                 rotation = [ 0.0 , math.radians( rotDeg_2 ) , 0.0 ] , 
#                 frame = 'self' ,
#                 speed = 0.025 , # HACK
#                 ctrl = ctrl 
#             )
#         )
        
        # 4. Push insert Z with force limit ( Move down to contact, with a greater limit )
        endPose = translate_pose( dX , [ 0.0 , 0.0 , -max_movement ] , dir_pose='origin')
        self.add_child(
            # NOTE: Do not want to bias wrist if we are already in contact
            Move_to_Contact( Fmag = insert_force , relMove = [ 0.0 , 0.0 , -max_movement ] , 
                             biasWrist = False , speed = dropSpeed , accel = 0.175 , 
                             ctrl = ctrl ) 
        )
        
#         self.add_child(  Set_BB_Key( self.recordFlagKey , False )  )
        
        if finalInsertHandZ != None:
            # 5. Check for z
            self.add_child(  At_Z_Level_COND( finalInsertHandZ , margin = 0.010 , ctrl = ctrl )  )

# ____ End Tilt Insert ____

# === Spiral Search Composite ===

# TODO: Accelerating Spiral Search : wider and wider loops
# TODO: Adaptive Spiral Search : re-center on candidate snags

class SpiralStep:
    """ Move Rule class: Returns each step in a spiral with each call """
    
    def set_init( self , initPose = _DUMMYPOSE ):
        """ Set the initial pose """
        self.initPose = initPose
        self.currZlvl = initPose[2,3]
    
    def __init__( self , initPose = _DUMMYPOSE , 
                  degrees_to_step = 18.0, start_radius = 0.002 , step_size=0.001, max_angle=100000, max_radius=100000 ,
                  ctrl = None , pressMaintain = 2.0 , chaseStep = 0.00005 , N_chaseMax = 15 ):
        """ Set the center of the spiral """
        self.set_init( initPose )
        self.stepAngl   = degrees_to_step
        self.initRadi   = start_radius
        self.stepRadi   = step_size
        self.maxAngle   = max_angle
        self.mxRadius   = max_radius
        self.r          = start_radius
        self.phi        = 0.0
        self.currPose   = self.initPose.copy()
        self.N_chaseMax = N_chaseMax
        self.chaseCount = 0
        if type( ctrl ) != type( None ):
            self.pressMode     = 1
            self.ctrl          = ctrl
            self.pressMaintain = pressMaintain
            self.chaseStep     = chaseStep
            self.chaseON       = 0
            print( "SpiralStep created in CHASE MODE" )
        else:
            self.pressMode = 0
        
    def reset( self , initPose = None ):
        """ Reset the spiral, possibly with new pose """
        self.r   = self.initRadi
        self.phi = 0.0
        if is_matx_list( initPose ):
            self.initPose   = initPose
            self.currPose   = self.initPose.copy()
            self.currZlvl   = initPose[2,3]
            self.chaseCount = 0
        
    def __call__( self ):
        """ Get the next pose """
        assert ( is_pose_mtrx( self.initPose ) ) , "SpiralStep generator called with an invalid start pose!\n" + str( self.initPose )
        self.phi += self.stepAngl
        x = np.cos( np.deg2rad( self.phi ) ) * self.r
        y = np.sin( np.deg2rad( self.phi ) ) * self.r
        self.r += self.stepRadi
        
        if self.pressMode:
            if not self.chaseON:
                self.currPose = translate_pose( self.initPose , translation_vec = [ x , y , 0 ] )
            zPress = self.ctrl.ft.get_wrist_force()[2]
            if zPress < -self.pressMaintain:
                self.currZlvl += self.chaseStep
                self.chaseON   = 0
            else:
                self.currZlvl -= self.chaseStep
                self.chaseCount += 1
                if self.chaseCount >= self.N_chaseMax:
                    self.chaseON = 0
                else:
                    self.chaseON   = 1
        elif ( self.phi < self.maxAngle and self.r < self.mxRadius ):
            self.currPose = translate_pose( self.initPose , translation_vec = [ x , y , 0 ] )
                
        self.currPose[2,3] = self.currZlvl
                
        return self.currPose
    
# class Spiral_Search( py_trees.composites.Selector ):

class Spiral_Search( py_trees.composites.Sequence ):
    """ Move in a spiral until some force condition is met """
    
    def __init__( self , initPose = _DUMMYPOSE , poseKey = "" , 
                  touch_force = 1 , drop_force = 1 , insert_force = 2 , max_movement = 0.1 , 
                  lateralStopTorque = 0.9 , pushbackF = 20.0 , 
                  spiralSpeed = 0.002 , descendSpeed = None , biasWrist = 1 ,
                  degrees_to_step = 15 , start_radius = 0.0005 , step_size = 0.00005 , 
                  chaseMode = 0  , chaseStep = 0.00005 , 
                  ctrl = None ):
        """ Store params """
        
        if chaseMode:
            print( "Spiral_Search created in CHASE MODE" )
        
        super().__init__( name = "Spiral_Search" , memory = 1 )
        
        # 0. Set params
        self.initPose        = initPose
        self.poseKey         = poseKey
        self.degrees_to_step = degrees_to_step
        self.start_radius    = start_radius
        self.step_size       = step_size
        self.max_angle       = 10 * math.pi
        self.max_radius      = 0.040
        self.ctrl            = ctrl
        if chaseMode:
            self.genr = SpiralStep( initPose = initPose , 
                                    degrees_to_step = degrees_to_step, start_radius = self.start_radius , step_size=self.step_size, 
                                    max_angle=self.max_angle, max_radius=self.max_radius ,
                                    ctrl = self.ctrl , pressMaintain = touch_force , chaseStep = chaseStep )
        else:
            self.genr = SpiralStep( initPose = initPose , 
                  degrees_to_step = degrees_to_step, start_radius = self.start_radius , step_size=self.step_size, max_angle=self.max_angle, 
                  max_radius=self.max_radius ,
                  ctrl = None , pressMaintain = 2.0 , chaseStep = 0.00005 )
        
    
        # 2. Spiral
        def insert_detector( *args ):
            """ Return true if either the later force or the lateral torque has been exceeded """

            wrench = self.ctrl.ft.get_wrist_force()
            if max( abs( wrench[3] ) , abs( wrench[4] ) ) > lateralStopTorque:
                self.logger.debug( "b:" + " STOPPING spiral on lateral torque limit of " + str( lateralStopTorque ) 
                                    + " > (one of) actual " + str( abs( wrench[3] ) ) + ' ' + str( abs( wrench[4] ) ) )
                return 1
            elif  ( wrench[2] < -pushbackF ): 
                self.logger.debug( "b:" + " STOPPING spiral on Pushback force " + str( -pushbackF ) + " > actual " + str( wrench[2] ) )
                return 1
            elif  ( drop_force > 0 )  and  ( wrench[2] > -drop_force ): 
                self.logger.debug( "b:" + " STOPPING spiral onDrop force " + str( wrench[2] ) + " > actual " + str( -drop_force ) )
                return 1
            else:
                return 0
    
        # 1. Add the Spiral Rule
        self.add_child(
            Move_Rule_w_Stop_Cond( self.genr , 
                                   insert_detector , 
                                   condSuccess = True , 
                                   ctrl = self.ctrl
                                 )
        )
        
        # 2. Add the fail condition, Moved too far
        self.add_child(
            COND_At_TCP_Pose( self.initPose , posnMargin = max_movement , orntMargin = 1.0 , ctrl = self.ctrl )
        )
        
    def initialise( self ):
        """
        ( Ticked first time ) or ( ticked not RUNNING ): 
        Set the generator to the appropriate initial pose, if one has been store in the dictionary
        """
        if self.poseKey:
            self.genr.set_init(  ASMBB.get( self.poseKey )  )
    
# ___ End Spiral Search ___


# ==== Spiral Insert Sequence ====

# 1. Move to contact
# 2. Spiral search : Condition met or failed
# 3. Insert w/ force
# 4. TODO: Optionally tamp

class Spiral_Insert( py_trees.composites.Sequence ):
    """ Spiral search + Cylindrical Insertion, useful for both Peg-in-Hole and Hole-on-Peg """
    
    def __init__( self , beginPose ,
                  touch_force = 1 , drop_force = 1 , insert_force = 2 , max_movement = 0.1 , 
                  lateralStopTorque = 0.9 , pushbackF = 20.0 , spiralSpeed = 0.002 , descendSpeed = None , biasWrist = 1 ,
                  degrees_to_step = 15 , maxAngle = 100 * 360 , startRadius = 0.0005 , stepSize = 0.00005 ,  max_radius=100000 ,
                  posnMargin = 0.003 , orntMargin = 2.0/180 , 
                  reliefStep_m = 0.002 , reliefMargin_Nm = 0.100 , reliefN = 10 , 
                  finalInsertHandZ = None , suppressDrop = 0 , suppressLateral = 0 , stepPress = 0 , 
                  recordFlagKey = "RECORD_KEY" , ctrl = None,
                  chaseMode = 0  , chaseStep = 0.00005 , N_chaseMax = 15 ):
        """ Construct the subtree """
        # NOTE: It is the responsibility of the calling code to determine the starting pose
        # 2020-02-10: For now still considering DOWN to be the only valid insertion direction
        
        super().__init__( name = "Spiral_Insert" , memory = True )
        self.ctrl          = ctrl
        self.recordFlagKey = recordFlagKey
        print( "Spiral_Insert created with record flag" , self.recordFlagKey )
        
        if chaseMode:
            print( "Spiral_Insert created in CHASE MODE" )
        
        # ~~ Add Nodes ~~
        
        # 0. Conditional: At beginning pose
        self.add_child(
            COND_At_TCP_Pose( beginPose , posnMargin = posnMargin , orntMargin = orntMargin , ctrl = self.ctrl )
        )
        
        # 2020-03-08: OKAY CREATE , OKAY EXEC
        
        # 1. Move down to contact
        endPose = translate_pose( beginPose , [ 0.0 , 0.0 , -max_movement ] , dir_pose='origin')
        
        if descendSpeed:
            self.add_child(
                Move_to_Contact( endPose , touch_force , speed = descendSpeed , biasWrist = biasWrist , ctrl = ctrl )
            )
        else:
            self.add_child(
                Move_to_Contact( endPose , touch_force , speed = 0.125 , biasWrist = biasWrist , ctrl = ctrl )
            )
        
        # 2020-03-08: OKAY CREATE , OKAY EXEC
        
        # 2. Mark where part made contact and store it in the dictionary
        poseRecorder = Store_Current_Pose( ctrl = self.ctrl )
        self.add_child(
            poseRecorder
        ) # NOTE: `Store_Current_Pose` is store-once by default
        self.poseKey = poseRecorder.key # Get the dictionary key where this was stored
        
        # 2020-03-08: OKAY CREATE , OKAY EXEC
        
        self.add_child(  Set_BB_Key( self.recordFlagKey , True )  )
        
        # 3. Spiral search behavior
        
        # A. Spiral search Stop condition 
        def insert_detector( *args ):
            """ Return true if either the later force or the lateral torque has been exceeded """
            _DEBUG = 1
            wrench = self.ctrl.ft.get_wrist_force()
            if ( max( abs( wrench[3] ) , abs( wrench[4] ) ) > lateralStopTorque )  and  ( not suppressLateral ):
                if _DEBUG:  
                    print( "insert_detector:" + " STOPPING spiral on lateral torque limit of " + str( lateralStopTorque ) \
                            + " > (one of) actual " + str( abs( wrench[3] ) ) + ' ' + str( abs( wrench[4] ) ) )
                builtins._GLOB_FT_FLAG = 1
                return 1
            elif  ( wrench[2] < -pushbackF ): 
                if _DEBUG:  
                    print( "insert_detector:" + " STOPPING spiral on Pushback force " + str( -pushbackF ) + " > actual " + str( wrench[2] ) )
                builtins._GLOB_FT_FLAG = 1
                return 1
            elif  ( drop_force > 0 )  and  ( wrench[2] > -drop_force )  and  ( not suppressDrop ): 
                if _DEBUG:  
                    print( "insert_detector:" + " STOPPING spiral onDrop force " + str( wrench[2] ) + " > actual " + str( -drop_force ) )
                builtins._GLOB_FT_FLAG = 1
                return 1
            else:
                return 0
    
        # B. Calc the number of times that the spiral mini motions can fail before the behavior fails
        allowedF = int( math.ceil( ( max_radius - startRadius ) / stepSize ) )
        print( "Max Spiral Steps = (" , max_radius , "-" , startRadius , ") /" , stepSize , "=" , allowedF )
        
        
    
        # C. Init the generator
        if chaseMode:
            self.genr = SpiralStep( _DUMMYPOSE , degrees_to_step, startRadius , stepSize , 
                                                 maxAngle , max_radius ,
                                                 self.ctrl , pressMaintain = touch_force , 
                                                 chaseStep = chaseStep , N_chaseMax = N_chaseMax )
        else:
            self.genr = SpiralStep( _DUMMYPOSE , degrees_to_step, startRadius , stepSize , 
                                                 maxAngle , max_radius )
        
        
        spiralPress = py_trees.composites.Selector( memory = 1 )
        
        # The pressure step must happen before the spiral step because the spiral step fails every time the stop condition is not met
#         if stepPress:
            
#             spiralPress.add_child(
#                 Negator_DECO(
#                     Move_to_Contact( Fmag = touch_force * 1.00 , relMove = [ 0 , 0 , -spiralSpeed*0.75 ] , biasWrist = 0 , ctrl = self.ctrl )
#                 )
#             )
        
        spiralPress.add_child(
            Move_Rule_w_Stop_Cond( self.genr , 
                                   insert_detector , 
                                   condSuccess = True , 
                                   fetchInit = self.poseKey , 
                                   ctrl = self.ctrl )
        )
        
        
        
        
        # C. Spiral Search is a series of small arm movements under a condition
        self.add_child( 
            Run_to_X_Failures_DECO(
                spiralPress , 
                X_allowedFails = allowedF
            )
        )
        
        # 2020-03-09: OKAY CREATE , OKAY EXEC
        
        
        
        self.add_child(
            Minimize_Wrist_Torque_XY( name = "Minimize_Wrist_Torque_XY" , 
                                      numTrials = reliefN , marginT = reliefMargin_Nm , moveStep = reliefStep_m , 
                                      ctrl = self.ctrl )
        )
        
        
        
        self.add_child(
            Move_to_Contact( Fmag = insert_force , relMove = [ 0 , 0 , -max_movement ] , biasWrist = 0 , ctrl = self.ctrl )
        )
        
        self.add_child(  Set_BB_Key( self.recordFlagKey , False )  )
        
         # 2020-03-11: OKAY CREATE , OKAY EXEC
            
        
            
        if finalInsertHandZ != None:
            # 5. Check for z
            self.add_child(  At_Z_Level_COND( finalInsertHandZ , margin = 0.010 , ctrl = ctrl )  )
        
        # 4. Optionally tamp
        # FIXME: TAMP
        
    def initialise( self ):
        builtins._GLOB_FT_FLAG = 0

# ____ End Spiral Insert ____
  
# === Bias Wrist Behavior ===

class Bias_Wrist( py_trees.behaviour.Behaviour ):
    """ Bias the wrist FT sensor in preparation for taking a reading """
    # NOTE: Minimally blocks and always succeeds
    
    def __init__( self , waitSec = 0.75 , ctrl = None ):
        """ Set the number of seconds to wait """
        super().__init__( name = "Bias_Wrist" )
        self.waitSec = waitSec
        self.ctrl    = ctrl
        
    def update( self ):
        """ Minimally blocks and always succeeds """
        self.ctrl.ft.bias_wrist_force()
        sleep( self.waitSec )
#         print( "~~~ BIAS WRIST! ~~~" )
        return py_trees.common.Status.SUCCESS

# ___ End Bias Wrist ___

# === Move to Contact Behavior ===

# 1. Calculate absolute goal pose (Setup)
# 2. Move, respond with "RUNNING" while the move is being executed
# 3. If the UR has completed the move, check that the force stop was triggered
# 4. If force criterion reached set to SUCCESS, otherwise set to FAILURE

class Move_to_Contact( py_trees.behaviour.Behaviour ): 
    """ Move the arm until force or distance limit reached, SUCCESS if force condition triggered, otherwise FAILURE """
    # NOTE: It is the responsibility of the the composite behavior (e.g. the Inserts) to provide an appropriate target for motion,
    #       as well as a direction of motion
    # NOTE: If a `relMove` is passed to the constructor, then the `pose` will be overridden
    
    def __init__( self , pose = _DUMMYPOSE , Fmag = 1.0 , relMove = None , biasWrist = 1 , 
                         mode = 'l' , speed = 0.125 , accel = 0.35 , 
                         pull = 0 , 
                         ctrl = None , _DEBUG = 0 , name = "Move_to_Contact" ):
        """ 
        Minimal one-time initialisation, offline only: 
        Set the pose that represents the limit of free motion, set the direction of motion 
        """
        super().__init__( name )
        self.pose    = pose # ---- Limit of free motion for this action, reaching this is FAILURE
        self.ctrl    = ctrl # ---- Should be an RMStudio object, will raise an error if not set
        self.Fstp    = Fmag # ---- Force magnitude that ends motion and signifies
        self.biasW   = biasWrist # Should the wrist sensor be biased before the motion begins?
        self.mode    = mode
        self.speed   = speed
        self.accel   = accel
        self.relMove = relMove # - Translate some vector from the present location rather than having a pose target
        self._DEBUG  = _DEBUG
        self.pull    = pull
        
        if self._DEBUG: print( "Move_to_Contact: arg   =" , speed )
        if self._DEBUG: print( "Move_to_Contact: speed =" , self.speed )

    def initialise( self ):
        """
        First time your behaviour is ticked or not RUNNING: 
        Send move command to UR
        """
        self.logger.debug( "  %s [Move_Arm::initialise()]" % self.name )
        
        self.badFlag = False
        
        # 0. Calc direction of motion
        bgnPose  = self.ctrl.arm.get_tcp_pose()
        bgnParts = pose_components( bgnPose )
        
        # 1. Check the beginning pose
        if not is_pose_mtrx( bgnPose ):
            self.badFlag = True
            if self._DEBUG: print( "FAILED TO RETRIEVE INIT POSITION: CANNOT MOVE" )
        
        # 2. If this is a relative move, then calc the endpoint based on the present point
        if( is_matx_list( self.relMove ) ):
            endPose   = translate_pose( bgnPose , self.relMove , dir_pose='origin')
            if not is_pose_mtrx( endPose ):
                endPose = bgnPose
                self.badFlag = True
                if self._DEBUG: print( "FAILED TO CALC ENDPOINT MATX: CANNOT MOVE" )
            endParts  = pose_components( endPose )
            self.pose = endPose.copy()
        # 3. else is an absolute move, check
        else:
            endParts  = pose_components( self.pose )
            if not is_pose_mtrx( self.pose ):
                endPose = bgnPose
                self.badFlag = True
                if self._DEBUG: print( "RECEIVED BADENDPOINT MATX: CANNOT MOVE" )
        
        
        rm        = self.ctrl
        self.cMet = False
        builtins._GLOB_FT_FLAG = 0
        
        # 1. Setup stop condition
        def stop_cond():
            """ Returns true when force opposing direction of motion exceeds the limit """
            
            # A. Get the wrist force 
            wrench = rm.ft.get_wrist_force()
            mag    = wrench[2]
                        
            if self._DEBUG:
                print( "mag:" , mag , "Fstp:" , -self.Fstp )
    
            if self.badFlag:
                print( "Preventing something bad" )
                return True
    
            # C. If the force opposes motion and its magnitude more negative than condition, then
            if self.pull:
                bCond = mag >=  self.Fstp
            else:
                bCond = mag <= -self.Fstp
                
            if bCond:
                # D. Set flag and return true
                builtins._GLOB_FT_FLAG = 1
                return True
            else:
                return False
        
        if self.biasW:
            self.ctrl.ft.bias_wrist_force()
            
        if self._DEBUG: print( "Move_to_Contact: about to MOVE with speed =" , self.speed )
        self.ctrl.arm.move_speed( self.pose , self.mode , self.speed , self.accel , 0 , stop_cond , True )
        

    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Check that the arm is moving, goal reached, or failed
        """
        self.logger.debug( "  %s [Move_Arm::update()]" % self.name )
        # 1. If the robot arm has stopped (ASSUMPTION: Robot has stopped because it has finished moving)
        if int( self.ctrl.arm.get_status_bits() , 2 ) == 1:
            # A. if the condition was met
            if builtins._GLOB_FT_FLAG:
                if self._DEBUG: print( "Move_to_Contact: The condition was met!" )
                return py_trees.common.Status.SUCCESS
            # B. Else the robot ended its motion without touching anything
            else:
                if self._DEBUG: print( "Move_to_Contact: Ended WITHOUT meeting condition!" )
                return py_trees.common.Status.FAILURE
        # 2. Otherwise the robot arm has not stopped
        else: 
#             print( "Movement is running!" )
            return py_trees.common.Status.RUNNING

# ___ End Move Arm ___

class Maintain_Z_Pressure( py_trees.behaviour.Behaviour ):
    """ Hold down the TCP at a certain pressure for a set duration """
    
    def __init__( self , 
                  timeOut_s , plungeMove = 1 , plungeDepth=0.050 , descendSpeed=0.010 , zPress=2.0 , maxZStep=0.010 , biasWrist=1 ,
                  stop_condition=condition_false , stepSleep=0.1 ,
                  relieveT=False , relieveStep=0.0002 , stepPress = 0.0002 , ctrl = None , 
                  reverseRelief = 0 , ft_cond = 0 , cond_success = 1 , delayRelief = 0.0 ):
        """ Set up the behaviour """
        super().__init__( name = "Maintain_Z_Pressure" )
        self.ctrl           = ctrl
        self.timeOut_s      = timeOut_s
        self.plungeMove     = plungeMove
        self.plungeDepth    = plungeDepth
        self.descendSpeed   = descendSpeed
        self.zPress         = zPress
        self.maxZStep       = maxZStep
        self.biasWrist      = biasWrist
        self.stop_condition = stop_condition
        self.stepSleep      = stepSleep
        self.relieveT       = relieveT
        self.relieveStep    = relieveStep
        self.stepPress      = stepPress
        self.reverseRelief  = reverseRelief
        self.ft_cond        = ft_cond
        self.cond_success   = cond_success
        self.delayRelief    = delayRelief
        
    def _give_condition( self ):
        print( inspect.getsource( self.stop_condition ) )
        return self.stop_condition
    
    def initialise( self ):
        """ Performs the behavior , BLOCKS """
        # WARNING: BEHAVIOR IS BLOCKING!
        
        builtins._GLOB_FT_FLAG = 0
        
        bgn = time.time()
        
        self.ctrl.maintain_z_pressure( timeOut_s      = self.timeOut_s         , 
                                       plungeMove     = self.plungeMove        ,
                                       plungeDepth    = self.plungeDepth       , 
                                       descendSpeed   = self.descendSpeed      , 
                                       zPress         = self.zPress            , 
                                       maxZStep       = self.maxZStep          , 
                                       biasWrist      = self.biasWrist         ,
                                       stop_condition = self._give_condition() , 
                                       stepSleep      = self.stepSleep         ,
                                       relieveT       = self.relieveT          , 
                                       relieveStep    = self.relieveStep       ,
                                       stepPress      = self.stepPress         ,
                                       reverseRelief  = self.reverseRelief     ,
                                       delayRelief    = self.delayRelief       )
        
        print( "Maintain_Z_Pressure: self.ctrl.maintain_z_pressure ran for" , time.time() - bgn , "seconds!" )
    
    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Always return success
        """
        if self.ft_cond:
            if self.cond_success:
                if builtins._GLOB_FT_FLAG:
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.FAILURE
            else:
                if builtins._GLOB_FT_FLAG:
                    return py_trees.common.Status.FAILURE
                else:
                    return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.SUCCESS

class Maintain_Y_Pressure( py_trees.behaviour.Behaviour ):
    """ Hold down the TCP at a certain pressure for a set duration """
    
    def __init__( self , 
                  timeOut_s , plungeMove = 1 , plungeDepth=0.050 , descendSpeed=0.010 , yPress=2.0 , maxYStep=0.010 , biasWrist=1 ,
                  stop_condition=condition_false , stepSleep=0.1 ,
                  relieveT=False , relieveStep=0.0002 , stepPress = 0.0002 , ctrl = None , 
                  reverseRelief = 0 , ft_cond = 0 , cond_success = 1 , delayRelief = 0.0 ):
        """ Set up the behaviour """
        super().__init__( name = "Maintain_Y_Pressure" )
        self.ctrl           = ctrl
        self.timeOut_s      = timeOut_s
        self.plungeMove     = plungeMove
        self.plungeDepth    = plungeDepth
        self.descendSpeed   = descendSpeed
        self.yPress         = yPress
        self.maxYStep       = maxYStep
        self.biasWrist      = biasWrist
        self.stop_condition = stop_condition
        self.stepSleep      = stepSleep
        self.relieveT       = relieveT
        self.relieveStep    = relieveStep
        self.stepPress      = stepPress
        self.reverseRelief  = reverseRelief
        self.ft_cond        = ft_cond
        self.cond_success   = cond_success
        self.delayRelief    = delayRelief
        
    def _give_condition( self ):
        print( inspect.getsource( self.stop_condition ) )
        return self.stop_condition
    
    def initialise( self ):
        """ Performs the behavior , BLOCKS """
        # WARNING: BEHAVIOR IS BLOCKING!
        
        builtins._GLOB_FT_FLAG = 0
        
        bgn = time.time()
        
        self.ctrl.maintain_y_pressure( timeOut_s      = self.timeOut_s         , 
                                       plungeMove     = self.plungeMove        ,
                                       plungeDepth    = self.plungeDepth       , 
                                       descendSpeed   = self.descendSpeed      , 
                                       yPress         = self.yPress            , 
                                       maxYStep       = self.maxYStep          , 
                                       biasWrist      = self.biasWrist         ,
                                       stop_condition = self._give_condition() , 
                                       stepSleep      = self.stepSleep         ,
                                       relieveT       = self.relieveT          , 
                                       relieveStep    = self.relieveStep       ,
                                       stepPress      = self.stepPress         ,
                                       reverseRelief  = self.reverseRelief     ,
                                       delayRelief    = self.delayRelief       )
        
        print( "Maintain_Y_Pressure: self.ctrl.maintain_y_pressure ran for" , time.time() - bgn , "seconds!" )
    
    def update( self ):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Always return success
        """
        if self.ft_cond:
            if self.cond_success:
                if builtins._GLOB_FT_FLAG:
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.FAILURE
            else:
                if builtins._GLOB_FT_FLAG:
                    return py_trees.common.Status.FAILURE
                else:
                    return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.SUCCESS

        
class Set_Down_at_Pose( py_trees.composites.Sequence ):
    """ Set something down at a pose, gently """
    
    def __init__( self , setDownPose , objMinWidth = 0.010 , distAway = 0.020 , setForce = 2.0 , finalOpen = 1.0 , ySAFE = 0.150 , ctrl = None ):
        super().__init__( name = "Set_Down_at_Pose" , memory = 0 )
        self.ctrl      = ctrl
        self.awayPose = translate_pose( setDownPose , translation_vec = [ 0.0 ,  distAway, 0.0 ] , dir_pose='origin' )
        self.throughPose = translate_pose( setDownPose , translation_vec = [ 0.0 , -distAway, 0.0 ] , dir_pose='origin' )
        
        # 0. Precondition: Be holding an object
        pCond = Finger_Width_COND( partWidth = objMinWidth , ctrl = ctrl )
        labeled_precondition( self , pCond )
        self.add_child( pCond ) # Checked continuously
        
        
        setDownSeq = py_trees.composites.Sequence( name = "Set_Down_Seq" , memory = 1 )
        # 2. Jog over the position
        setDownSeq.add_child(  Jog_Safe( self.abovePose , 
                                   ySAFE = ySAFE ,  
                                   hover = 1 ,  
                                   ctrl  = self.ctrl )  )
        
        # 3. Move to the approach pose
        setDownSeq.add_child(  Move_Arm( self.abovePose , ctrl = ctrl )  )
        
        # 4. Set down with (gentle) force
        setDownSeq.add_child(  Move_to_Contact( self.belowPose , setForce , speed = 0.0625 , accel = 0.175 , biasWrist = 1 , ctrl = ctrl )  )
        
        # 5. Open hand to preset
        setDownSeq.add_child(  Set_Fingers( finalOpen , ctrl = ctrl )  )
        
        # Add sub-seq
        self.add_child( setDownSeq )


class Spin_to_Stop( py_trees.composites.Sequence ):
    """ Try N times to twist the wrist to some wrist Z torque condition """
    
    def __init__( self , centralPose , partDia , gripMargin = 0.010 , stopTorq = 0.25 , twistDeg = 15 , Ntries = 5 , ctrl = None ):
        """ Set up a repeating sequence to  """
        # NOTE: This behavior assumes that the hand is already at `centralPose`
        # NOTE: This behavior assumes that the hand is already open
        super().__init__( name = "Spin_to_Stop" , memory = 1 )
        self.ctrl      = ctrl
        self.beginPose = centralPose.copy()
        self.twistPose = rotate_pose( self.beginPose , [ 0.0 , 0.0 , radians( twistDeg ) ] , dir_pose='self')
        
        
        # 0. This behavior assumes that the hand is already at `centralPose`
        pCond = COND_At_TCP_Pose( centralPose , ctrl = ctrl )
        labeled_precondition( self , pCond , addChild = 1 )
        
        # Loop will go until torq resistance
        twistLoop = py_trees.composites.Sequence( name = "Twist_Loop" , memory = 1 )
        
        # 1. Go to central pose, Assume the gripper is open
        twistLoop.add_child(  Move_Arm( centralPose , ctrl = ctrl )  )
        
        # 2. Bias wrist
        twistLoop.add_child(  Bias_Wrist( ctrl = ctrl )  )
        
        # 3. Grasp
        twistLoop.add_child(  Set_Fingers( openState = 0.0 , ctrl = ctrl )  )
        
        twistTry = py_trees.composites.Selector( name = "Twist_Loop" )
        
        # 4. Twist with force condition, This will succeed when the force condition is met, stopping the loop
        twistTry.add_child(
            Move_Arm_to_FT_Cond( self.twistPose , stopCond = ctrl.exceeds_Z_torque( stopTorq ) , biasWrist = 0 , 
                                 mode = 'j' , speed = 0.125 , accel = 0.35 , F_cond_is_success = 1 ,
                                 ctrl = ctrl )
        )
        
        # 3. Grasp
        twistTry.add_child( SuccessIsFailure( Set_Fingers( openState = partDia + gripMargin , ctrl = ctrl ) ) )
        
        twistLoop.add_child(  twistTry   )
        
        # 4. Allow this many twists
        self.add_child(  Run_to_X_Failures_DECO( twistLoop , X_allowedFails = Ntries )  )
        
        self.add_child(  Set_Fingers( openState = 1.0 , ctrl = ctrl )  )

class SpinPressGen:
    """ Class that generates a pose that rotates about the wrist while maintaining some -Z pressure on the wrist """
    
    def __init__( self , angleStep_deg , zPress , stepPress , ctrl = None ):
        """ Set vars for the pressure spin operation """
        # NOTE: This generator assumes that the hand is already in contact with the piece that undergoes spinning pressure
        self.ctrl         = ctrl
        self.angleStepRad = radians( angleStep_deg )
        self.zPress       = zPress
        self.stepPress    = stepPress
        
    # Pressure condition
    def Zforce_cond( self , wrench ):
        if wrench[2] < -self.zPress:
            return 1
        else:
            return 0
    
    def __call__( self ):
        """ This is a functor """
        # 1. Get the current pose
        currPose = self.ctrl.arm.get_tcp_pose()
        # 2. Get the current pressure
        crWrench = self.ctrl.ft.get_wrist_force()
        # 3. Twist the current pose
        trgtPose = rotate_pose( currPose , [ 0.0 , 0.0 , self.angleStepRad ] , dir_pose='origin')
        # 4. Determine if the pressure is above or below the threshold
        if self.Zforce_cond( crWrench ):
            stepZ =  self.stepPress * 0.5 # Don't back off as much, screw is running away
        else:
            stepZ = -self.stepPress # Chase hard
        # 5. Apply pressure constraint to the next pose
        trgtPose = translate_pose( trgtPose , translation_vec = [ 0.0 , 0.0 , stepZ ] , dir_pose='origin' )
        # N. Return
        return trgtPose
        
class Spin_Press( py_trees.composites.Sequence ):
    """ Preload the arm, twist, then push again """
    
    def __init__( self , 
                  wideDia , plungeDist ,
                  preloadPress = 3.0 , press2 = 5.0 , stopTorq = 0.10 , 
                  gripMargin = 0.010 , twist_deg = 15.0 , preLoadTech = 1 , 
                  spinStep_deg = 15.0 / 50.0 ,
                  bias_wrist = 1 ,
                  stepPress  = 0.0005 ,
                  suppressLastPush = 0 ,
                  suppressRelease = 0 ,
                  ctrl = None ):
        """ Set up a repeating sequence to  """
        # NOTE: This behavior assumes that the hand is already at `centralPose`
        # NOTE: This behavior assumes that the hand is already open
        super().__init__( name = "Spin_to_Stop" , memory = 1 )
        self.ctrl     = ctrl
        self.gen      = SpinPressGen( angleStep_deg = spinStep_deg , zPress = preloadPress , stepPress = stepPress , ctrl = ctrl )
                
        # 0. This behavior assumes that the hand is already at `centralPose`
        
        # 1. Grasp
        self.add_child(  Set_Fingers( openState = 0.0 , ctrl = ctrl )  )
        
        if bias_wrist:
            # 2. Bias wrist
            self.add_child(  Bias_Wrist( ctrl = ctrl )  )
        
        # If using the preload technique
        if preLoadTech:
        
            # 3. Preload
            self.add_child(  
                Move_to_Contact( Fmag = preloadPress , relMove = [ 0.0 , 0.0 , -plungeDist ] , 
                                 speed = 0.0625 , accel = 0.175 , biasWrist = 1 , ctrl = ctrl )  
            )

            # 4. Twist with force condition
            self.add_child(
                Move_Arm_Relative( translation = [ 0.0 , 0.0 , 0.0 ] , rotation = [ 0.0 , 0.0 , radians( twist_deg ) ] , 
                             mode = 'l' , speed = 0.125 , accel = 0.35 , stop_cond = ctrl.exceeds_Z_torque( stopTorq ) ,
                             frame = 'self' ,
                             ctrl = ctrl ) 
            )

        # else spin while continually applying pressure as needed
        else:
            allowedN = int( ceil( twist_deg / spinStep_deg ) )
            
            self.add_child( 
                Run_to_X_Failures_DECO(
                    Move_Rule_w_Stop_Cond(  
                         self.gen , 
                         stop_cond    = ctrl.exceeds_Z_torque( stopTorq ) , 
                         condSuccess  = True , 
                         fetchInit    = None ,
                         relativeRule = False , 
                         ctrl         = ctrl
                    ) , 
                    X_allowedFails = allowedN
                )
            )

        # 5. Push Again
        if not suppressLastPush:
            self.add_child(
                Move_to_Contact( Fmag = press2 , relMove = [ 0.0 , 0.0 , -plungeDist ] , 
                                 speed = 0.0625 , accel = 0.175 , biasWrist = 1 , ctrl = ctrl )  
            )
            
        # 6. Open
        if not suppressRelease:
            self.add_child(  Set_Fingers( openState = wideDia , ctrl = ctrl )  )
        else:
            self.add_child(  Set_Fingers( openState = 0.0 , ctrl = ctrl )  )
