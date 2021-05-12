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
                         ctrl = None , _DEBUG = 1 , name = "Move_to_Contact" ):
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
        
        print( "Move_to_Contact: arg   =" , speed )
        print( "Move_to_Contact: speed =" , self.speed )

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
            print( "FAILED TO RETRIEVE INIT POSITION: CANNOT MOVE" )
        
        # 2. If this is a relative move, then calc the endpoint based on the present point
        if( is_matx_list( self.relMove ) ):
            endPose   = translate_pose( bgnPose , self.relMove , dir_pose='origin')
            if not is_pose_mtrx( endPose ):
                endPose = bgnPose
                self.badFlag = True
                print( "FAILED TO CALC ENDPOINT MATX: CANNOT MOVE" )
            endParts  = pose_components( endPose )
            self.pose = endPose.copy()
        # 3. else is an absolute move, check
        else:
            endParts  = pose_components( self.pose )
            if not is_pose_mtrx( self.pose ):
                endPose = bgnPose
                self.badFlag = True
                print( "RECEIVED BADENDPOINT MATX: CANNOT MOVE" )
        
        # 4. Calculate the direction of motion, and check for errors
        direction = vec_unit( endParts["position"] - bgnParts["position"] )
        if np.isnan( direction ).any():
            self.badFlag = True
            print( "COMPUTED A BAD MOVEMENT DIRECTION" , direction , ": CANNOT MOVE" )
        #print( "Moving in direction:" , direction )
        self.drct = direction
        rm        = self.ctrl
        self.cMet = False
        
#         print( "Beginning Pose:\n" , bgnParts["position"] )
#         print( "Moving in direction:" , direction )
        
        
        # 1. Setup stop condition
        def stop_cond():
            """ Returns true when force opposing direction of motion exceeds the limit """
            
            # A. Get the wrist force in the base frame
#             Fdir = self.ctrl.base_wrist_force( self.ctrl )[:3]
            Fdir = self.ctrl.base_wrist_force( )[:3]
            if np.isnan( Fdir ).any():
                self.badFlag = True
                print( "COMPUTED A BAD FORCE VECTOR" , Fdir , ": CANNOT MOVE" )
            
            # B. Dot the force vector with the direction of motion
            mag = direction.dot( Fdir )
#             print( mag )
            if np.isnan( mag ).any():
                self.badFlag = True
                print( "COMPUTED A BAD DOT PRODUCT: CANNOT MOVE" )
            
#             print( "Stop force:" , self.Fstp )
#             print( "direction dot force:" , mag )
#             print( "Cond met?:" , mag <= -self.Fstp )
            
            if self._DEBUG:
                print( "mag:" , mag , "Fstp:" , -self.Fstp )
    
            if self.badFlag:
                print( "Preventing something bad" )
                return True
    
            # C. If the force opposes motion and its magnitude more negative than condition, then
            if mag <= -self.Fstp:
                # D. Set flag and return true
                self.cMet = True
                return True
            else:
#                 self.cMet = False
                return False
        
        if self.biasW:
            self.ctrl.ft.bias_wrist_force()
            
            
        
#         print( stop_cond , type( stop_cond ) )
#         print( stop_cond() )
        print( "Move_to_Contact: about to MOVE with speed =" , self.speed )
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
            if self.cMet:
                print( "Move_to_Contact: The condition was met!" )
                return py_trees.common.Status.SUCCESS
            # B. Else the robot ended its motion without touching anything
            else:
                print( "Move_to_Contact: Ended WITHOUT meeting condition!" )
                return py_trees.common.Status.FAILURE
        # 2. Otherwise the robot arm has not stopped
        else: 
#             print( "Movement is running!" )
            return py_trees.common.Status.RUNNING

# ___ End Move Arm ___