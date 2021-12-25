import builtins
import math
import os
import time

import numpy as np
import py_trees
import rmlib.rmtools as rm
from rmlib.rmtools import utils

import transforms3d 
from transforms3d.axangles import axangle2aff

from rmlib.rmtools.asm_BT_lib.asm_tree_Basic import (
    Finger_Width_COND,
    Jog_Safe,
    Move_Arm,
    Move_Arm_Relative,
    Move_Rule_w_Stop_Cond,
    Set_BB_Key,
    Set_Fingers,
    Store_Current_Pose,
)
from rmlib.rmtools.asm_BT_lib.asm_tree_cond_test import (
    COND_At_TCP_Pose,
    labeled_precondition,
)
from rmlib.rmtools.asm_BT_lib.asm_tree_logic_flow import Run_to_X_Failures_DECO, Negator_DECO

##### Functions #####


def status_as_label(status):
    """Return the status as a label"""
    try:
        return {
            py_trees.common.Status.FAILURE: 0,
            py_trees.common.Status.SUCCESS: 1,
        }[status]
    except:
        return None


def get_XML_outfile_namer(pathPrefix="", namePrefix="XML"):
    """Return a function that generates filenames based on the current time"""

    def gen_tstamp_fname():
        """Return a filename that incorporates the enclosed prefixes"""
        return str(os.path.join(pathPrefix, namePrefix + utils.nowTimeStamp() + ".xml"))

    return gen_tstamp_fname


class At_Z_Level_COND(py_trees.behaviour.Behaviour):
    """Return True if the TCP is within margin of some z-height"""

    # TODO: GENERALIZE TO AT_PLANE_COND

    def __init__(self, zLevel, margin=0.010, ctrl=None):
        """Set the level and the margin"""
        super().__init__(name="At_Z_Level_COND")
        self.zLevel = zLevel
        self.margin = margin
        self.ctrl = ctrl

    def update(self):
        """Return True if the TCP is within margin of some z-height, Otherwise return False"""
        currPose = self.ctrl.arm.get_tcp_pose()
        currZlvl = currPose[2, 3]
        if abs(currZlvl - self.zLevel) <= self.margin:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class Move_Arm_Transform(py_trees.behaviour.Behaviour):
    """Move the arm to a pose, SUCCESS if pose reached, otherwise FAILURE"""

    def __init__(
        self,
        pose_func=None,
        BB_key=None,
        mode="l",
        speed=0.125,
        accel=0.35,
        ctrl=None,
        name="Move_Arm_Transform",
        blocking=1,
        _DEBUG=0,
    ):
        """
        Minimal one-time initialisation, offline only:
        Set the pose that the arm must reach
        """
        self.pose_func = pose_func
        self.BB_key = BB_key
        self.mode = mode
        self.speed = speed
        self.accel = accel
        self.blocking = blocking
        self.ctrl = ctrl  # Should be an RMStudio object, will raise an error if not set
        self._DEBUG = _DEBUG
        super().__init__(name)

    def initialise(self):
        """
        First time your behaviour is ticked or not RUNNING:
        Send move command to UR
        """
        self.logger.debug("  %s [Move_Arm::initialise()]" % self.name)

        # Fetch the value here, because the key may not exist at instantiation time
        if type(self.BB_key) != type(None):
            self.pose = ASMBB.get(self.BB_key)

        curr_pose = self.ctrl.arm.get_tcp_pose()
        self.pose = self.pose_func(curr_pose)
        self.ctrl.arm.move_speed(self.pose, self.mode, self.speed, self.accel, 0, "dummy", self.blocking)
        time.sleep(0.1)

    def update(self):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Check that the arm is moving, goal reached, or failed
        """
        self.logger.debug("  %s [Move_Arm::update()]" % self.name)
        # 1. If the robot arm has stopped
        if int(self.ctrl.arm.get_status_bits(), 2) == 1:
            # A. Calc error
            currPose = self.ctrl.arm.get_tcp_pose()
            transErr = rm.poses.get_distance_between_poses(currPose, self.pose)
            orienErr = rm.poses.orient_error_between_poses(currPose, self.pose)
            # B. If the goal has been reached
            if (transErr <= _EPSILON_POSE_POSN) and (orienErr <= _EPSILON_ORNT_EROR):
                if self._DEBUG:
                    print("Move_Arm: Goal REACHED!")
                return py_trees.common.Status.SUCCESS
            # C. Otherwise the goal has not been reached
            else:
                if self._DEBUG:
                    print("Move_Arm: Goal NOT REACHED!")
                return py_trees.common.Status.FAILURE
        # 2. Otherwise the robot arm has not stopped
        else:
            return py_trees.common.Status.RUNNING




class SpiralStep:
    """Move Rule class: Returns each step in a spiral with each call"""

    def set_init(self, initPose=_DUMMYPOSE):
        """Set the initial pose"""
        self.initPose = initPose
        self.currZlvl = initPose[2, 3]

    def __init__(
        self,
        initPose=_DUMMYPOSE,
        degrees_to_step=18.0,
        start_radius=0.002,
        step_size=0.001,
        max_angle=100000,
        max_radius=100000,
        ctrl=None,
        pressMaintain=2.0,
        chaseStep=0.00005,
        N_chaseMax=15,
    ):
        """Set the center of the spiral"""
        self.set_init(initPose)
        self.stepAngl = degrees_to_step
        self.initRadi = start_radius
        self.stepRadi = step_size
        self.maxAngle = max_angle
        self.mxRadius = max_radius
        self.r = start_radius
        self.phi = 0.0
        self.currPose = self.initPose.copy()
        self.N_chaseMax = N_chaseMax
        self.chaseCount = 0
        if ctrl is not None:
            self.pressMode = 1
            self.ctrl = ctrl
            self.pressMaintain = pressMaintain
            self.chaseStep = chaseStep
            self.chaseON = 0
            print("SpiralStep created in CHASE MODE")
        else:
            self.pressMode = 0

    def reset(self, initPose=None):
        """Reset the spiral, possibly with new pose"""
        self.r = self.initRadi
        self.phi = 0.0
        if utils.is_matx_list(initPose):
            self.initPose = initPose
            self.currPose = self.initPose.copy()
            self.currZlvl = initPose[2, 3]
            self.chaseCount = 0

    def __call__(self):
        """Get the next pose"""
        assert rm.poses.is_pose_mtrx(
            self.initPose
        ), "SpiralStep generator called with an invalid start pose!\n" + str(
            self.initPose
        )
        self.phi += self.stepAngl
        x = np.cos(np.deg2rad(self.phi)) * self.r
        y = np.sin(np.deg2rad(self.phi)) * self.r
        self.r += self.stepRadi

        if self.pressMode:
            if not self.chaseON:
                self.currPose = rm.poses.translate_pose(
                    self.initPose, translation_vec=[x, y, 0]
                )
            zPress = self.ctrl.arm.get_biased_force()[2]
            if zPress < -self.pressMaintain:
                self.currZlvl += self.chaseStep
                self.chaseON = 0
            else:
                self.currZlvl -= self.chaseStep
                self.chaseCount += 1
                if self.chaseCount >= self.N_chaseMax:
                    self.chaseON = 0
                else:
                    self.chaseON = 1
        elif self.phi < self.maxAngle and self.r < self.mxRadius:
            self.currPose = rm.poses.translate_pose(
                self.initPose, translation_vec=[x, y, 0]
            )

        self.currPose[2, 3] = self.currZlvl

        return self.currPose


class Spiral_Search(py_trees.composites.Sequence):
    """Move in a spiral until some force condition is met"""

    def __init__(
        self,
        initPose=_DUMMYPOSE,
        poseKey="",
        touch_force=1,
        drop_force=1,
        insert_force=2,
        max_movement=0.1,
        lateralStopTorque=0.9,
        pushbackF=20.0,
        spiralSpeed=0.002,
        descendSpeed=None,
        biasWrist=1,
        degrees_to_step=15,
        start_radius=0.0005,
        step_size=0.00005,
        chaseMode=0,
        chaseStep=0.00005,
        ctrl=None,
    ):
        """Store params"""

        if chaseMode:
            print("Spiral_Search created in CHASE MODE")

        super().__init__(name="Spiral_Search", memory=1)

        # 0. Set params
        self.initPose = initPose
        self.poseKey = poseKey
        self.degrees_to_step = degrees_to_step
        self.start_radius = start_radius
        self.step_size = step_size
        self.max_angle = 10 * math.pi
        self.max_radius = 0.040
        self.ctrl = ctrl
        if chaseMode:
            self.genr = SpiralStep(
                initPose=initPose,
                degrees_to_step=degrees_to_step,
                start_radius=self.start_radius,
                step_size=self.step_size,
                max_angle=self.max_angle,
                max_radius=self.max_radius,
                ctrl=self.ctrl,
                pressMaintain=touch_force,
                chaseStep=chaseStep,
            )
        else:
            self.genr = SpiralStep(
                initPose=initPose,
                degrees_to_step=degrees_to_step,
                start_radius=self.start_radius,
                step_size=self.step_size,
                max_angle=self.max_angle,
                max_radius=self.max_radius,
                ctrl=None,
                pressMaintain=2.0,
                chaseStep=0.00005,
            )

        # 2. Spiral
        def insert_detector(*args):
            """Return true if either the later force or the lateral torque has been exceeded"""

            wrench = self.ctrl.arm.get_biased_force()
            if max(abs(wrench[3]), abs(wrench[4])) > lateralStopTorque:
                self.logger.debug(
                    "b:"
                    + " STOPPING spiral on lateral torque limit of "
                    + str(lateralStopTorque)
                    + " > (one of) actual "
                    + str(abs(wrench[3]))
                    + " "
                    + str(abs(wrench[4]))
                )
                return 1
            elif wrench[2] < -pushbackF:
                self.logger.debug(
                    "b:"
                    + " STOPPING spiral on Pushback force "
                    + str(-pushbackF)
                    + " > actual "
                    + str(wrench[2])
                )
                return 1
            elif (drop_force > 0) and (wrench[2] > -drop_force):
                self.logger.debug(
                    "b:"
                    + " STOPPING spiral onDrop force "
                    + str(wrench[2])
                    + " > actual "
                    + str(-drop_force)
                )
                return 1
            else:
                return 0

        # 1. Add the Spiral Rule
        self.add_child(
            Move_Rule_w_Stop_Cond(
                self.genr, insert_detector, condSuccess=True, ctrl=self.ctrl
            )
        )

        # 2. Add the fail condition, Moved too far
        self.add_child(
            COND_At_TCP_Pose(
                self.initPose, posnMargin=max_movement, orntMargin=1.0, ctrl=self.ctrl
            )
        )

    def initialise(self):
        """
        ( Ticked first time ) or ( ticked not RUNNING ):
        Set the generator to the appropriate initial pose, if one has been store in the dictionary
        """
        if self.poseKey:
            self.genr.set_init(ASMBB.get(self.poseKey))


# ___ End Spiral Search ___


# ==== Spiral Insert Sequence ====

# 1. Move to contact
# 2. Spiral search : Condition met or failed
# 3. Insert w/ force
# 4. TODO: Optionally tamp


class Spiral_Insert(py_trees.composites.Sequence):
    """Spiral search + Cylindrical Insertion, useful for both Peg-in-Hole and Hole-on-Peg"""

    def __init__(
        self,
        beginPose,
        touch_force=1,
        drop_force=1,
        insert_force=2,
        max_movement=0.1,
        lateralStopTorque=0.9,
        pushbackF=20.0,
        spiralSpeed=0.002,
        descendSpeed=0.025,
        biasWrist=1,
        degrees_to_step=15,
        maxAngle=100 * 360,
        startRadius=0.0005,
        stepSize=0.00005,
        max_radius=100000,
        posnMargin=0.003,
        orntMargin=2.0 / 180,
        reliefStep_m=0.002,
        reliefMargin_Nm=0.100,
        reliefN=10,
        finalInsertHandZ=None,
        suppressDrop=False,
        suppressLateral=0,
        stepPress=0,
        recordFlagKey="RECORD_KEY",
        ctrl=None,
        chaseMode=0,
        chaseStep=0.00005,
        N_chaseMax=15,
    ):
        """Construct the subtree"""
        # NOTE: It is the responsibility of the calling code to determine the starting pose
        # 2020-02-10: For now still considering DOWN to be the only valid insertion direction

        super().__init__(name="Spiral_Insert", memory=True)
        self.ctrl = ctrl
        self.recordFlagKey = recordFlagKey
        print("Spiral_Insert created with record flag", self.recordFlagKey)

        if chaseMode:
            print("Spiral_Insert created in CHASE MODE")

        # ~~ Add Nodes ~~

        # 0. Conditional: At beginning pose
        self.add_child(
            COND_At_TCP_Pose(
                beginPose, posnMargin=posnMargin, orntMargin=orntMargin, ctrl=self.ctrl
            )
        )

        # 2020-03-08: OKAY CREATE , OKAY EXEC

        # 1. Move down to contact
        endPose = rm.poses.translate_pose(
            beginPose, [0.0, 0.0, -max_movement], dir_pose="origin"
        )

        self.add_child(
            Move_to_Contact(
                endPose,
                touch_force,
                speed=descendSpeed,
                biasWrist=biasWrist,
                ctrl=ctrl,
            )
        )

        # 2. Mark where part made contact and store it in the dictionary
        poseRecorder = Store_Current_Pose(ctrl=self.ctrl)
        self.add_child(
            poseRecorder
        )  # NOTE: `Store_Current_Pose` is store-once by default
        self.poseKey = poseRecorder.key  # Get the dictionary key where this was stored

        self.add_child(Set_BB_Key(self.recordFlagKey, True))

        # 3. Spiral search behavior
        
        tilt_angle_deg = -5
        tilt_angle = math.radians(tilt_angle_deg)
        def get_transform_pose(current_pose):
            M = axangle2aff([1, 0, 0], tilt_angle)
            new_pose = np.dot(M, current_pose)
            return new_pose


        tilt = Move_Arm_Transform(
            pose_func = get_transform_pose, # function that returns 4x4 homogeneous coord pose
            mode  = 'l',  # { 'l': linear in task space, 'j': linear in joint space }
            speed = 0.125, 
            accel = 0.35, 
            ctrl  = self.ctrl) 
        self.add_child(tilt)
        
        # A. Spiral search Stop condition
        def insert_detector(*args):
            """Return true if either the later force or the lateral torque has been exceeded"""
            _DEBUG = 1
            wrench = self.ctrl.arm.get_biased_force()
            if (max(abs(wrench[3]), abs(wrench[4])) > lateralStopTorque) and (
                not suppressLateral
            ):
                if _DEBUG:
                    print(
                        "insert_detector:"
                        + " STOPPING spiral on lateral torque limit of "
                        + str(lateralStopTorque)
                        + " > (one of) actual "
                        + str(abs(wrench[3]))
                        + " "
                        + str(abs(wrench[4]))
                    )
                builtins._GLOB_FT_FLAG = 1
                return 1
            elif wrench[2] < -pushbackF:
                if _DEBUG:
                    print(
                        "insert_detector:"
                        + " STOPPING spiral on Pushback force "
                        + str(-pushbackF)
                        + " > actual "
                        + str(wrench[2])
                    )
                builtins._GLOB_FT_FLAG = 1
                return 1
            elif (drop_force > 0) and (wrench[2] > -drop_force) and (not suppressDrop):
                if _DEBUG:
                    print(
                        "insert_detector:"
                        + " STOPPING spiral onDrop force "
                        + str(wrench[2])
                        + " > actual "
                        + str(-drop_force)
                    )
                builtins._GLOB_FT_FLAG = 1
                return 1
            else:
                return 0

        # B. Calc the number of times that the spiral mini motions can fail before the behavior fails
        allowedF = int(math.ceil((max_radius - startRadius) / stepSize))
        print(
            "Max Spiral Steps = (",
            max_radius,
            "-",
            startRadius,
            ") /",
            stepSize,
            "=",
            allowedF,
        )

        # C. Init the generator
        if chaseMode:
            self.genr = SpiralStep(
                _DUMMYPOSE,
                degrees_to_step,
                startRadius,
                stepSize,
                maxAngle,
                max_radius,
                self.ctrl,
                pressMaintain=touch_force,
                chaseStep=chaseStep,
                N_chaseMax=N_chaseMax,
            )
        else:
            self.genr = SpiralStep(
                _DUMMYPOSE, degrees_to_step, startRadius, stepSize, maxAngle, max_radius
            )

        spiralPress = py_trees.composites.Selector(memory=1)

        # The pressure step must happen before the spiral step because the spiral step fails every time the stop condition is not met
        #         if stepPress:

        # spiralPress.add_child(
        #     Negator_DECO(
        #         Move_to_Contact( Fmag = touch_force * 1.00 , relMove = [ 0 , 0 , -spiralSpeed*0.75 ] , biasWrist = 0 , ctrl = self.ctrl )
        #     )
        # )

        spiralPress.add_child(
            Move_Rule_w_Stop_Cond(
                self.genr,
                insert_detector,
                condSuccess=True,
                fetchInit=self.poseKey,
                ctrl=self.ctrl,
            )
        )

        # C. Spiral Search is a series of small arm movements under a condition
        self.add_child(Run_to_X_Failures_DECO(spiralPress, X_allowedFails=allowedF))

        # 2020-03-09: OKAY CREATE , OKAY EXEC

        self.add_child(
            Minimize_Wrist_Torque_XY(
                name="Minimize_Wrist_Torque_XY",
                numTrials=reliefN,
                marginT=reliefMargin_Nm,
                moveStep=reliefStep_m,
                ctrl=self.ctrl,
            )
        )

        self.add_child(
            Move_to_Contact(
                Fmag=insert_force,
                relMove=[0, 0, -max_movement],
                biasWrist=0,
                ctrl=self.ctrl,
            )
        )

        self.add_child(Set_BB_Key(self.recordFlagKey, False))

        # 2020-03-11: OKAY CREATE , OKAY EXEC

        if finalInsertHandZ:
            # 5. Check for z
            self.add_child(At_Z_Level_COND(finalInsertHandZ, margin=0.010, ctrl=ctrl))

        # 4. Optionally tamp
        # FIXME: TAMP

    def initialise(self):
        builtins._GLOB_FT_FLAG = 0


# ____ End Spiral Insert ____


# === Move to Contact Behavior ===

# 1. Calculate absolute goal pose (Setup)
# 2. Move, respond with "RUNNING" while the move is being executed
# 3. If the UR has completed the move, check that the force stop was triggered
# 4. If force criterion reached set to SUCCESS, otherwise set to FAILURE


class Move_to_Contact(py_trees.behaviour.Behaviour):
    """Move the arm until force or distance limit reached, SUCCESS if force condition triggered, otherwise FAILURE"""

    # NOTE: It is the responsibility of the the composite behavior (e.g. the Inserts) to provide an appropriate target for motion,
    #       as well as a direction of motion
    # NOTE: If a `relMove` is passed to the constructor, then the `pose` will be overridden

    def __init__(
        self,
        pose=_DUMMYPOSE,
        Fmag=1.0,
        relMove=None,
        biasWrist=1,
        mode="l",
        speed=0.125,
        accel=0.35,
        pull=0,
        ctrl=None,
        _DEBUG=1,
        name="Move_to_Contact",
    ):
        """
        Minimal one-time initialisation, offline only:
        Set the pose that represents the limit of free motion, set the direction of motion
        """
        super().__init__(name)
        self.pose = (
            pose  # ---- Limit of free motion for this action, reaching this is FAILURE
        )
        self.ctrl = (
            ctrl  # ---- Should be an RMStudio object, will raise an error if not set
        )
        self.Fstp = Fmag  # ---- Force magnitude that ends motion and signifies
        self.biasW = (
            biasWrist  # Should the wrist sensor be biased before the motion begins?
        )
        self.mode = mode
        self.speed = speed
        self.accel = accel
        self.relMove = relMove  # - Translate some vector from the present location rather than having a pose target
        self._DEBUG = _DEBUG
        self.pull = pull

        if self._DEBUG:
            print("Move_to_Contact: arg   =", speed)
        if self._DEBUG:
            print("Move_to_Contact: speed =", self.speed)

    def initialise(self):
        """
        First time your behaviour is ticked or not RUNNING:
        Send move command to UR
        """
        self.logger.debug("  %s [Move_Arm::initialise()]" % self.name)

        self.badFlag = False

        # 0. Calc direction of motion
        bgnPose = self.ctrl.arm.get_tcp_pose()
        bgnParts = rm.poses.pose_components(bgnPose)

        # 1. Check the beginning pose
        if not rm.poses.is_pose_mtrx(bgnPose):
            self.badFlag = True
            if self._DEBUG:
                print("FAILED TO RETRIEVE INIT POSITION: CANNOT MOVE")

        # 2. If this is a relative move, then calc the endpoint based on the present point
        if utils.is_matx_list(self.relMove):
            endPose = rm.poses.translate_pose(bgnPose, self.relMove, dir_pose="origin")
            if not rm.poses.is_pose_mtrx(endPose):
                endPose = bgnPose
                self.badFlag = True
                if self._DEBUG:
                    print("FAILED TO CALC ENDPOINT MATX: CANNOT MOVE")
            endParts = rm.poses.pose_components(endPose)
            self.pose = endPose.copy()
        # 3. else is an absolute move, check
        else:
            endParts = rm.poses.pose_components(self.pose)
            if not rm.poses.is_pose_mtrx(self.pose):
                endPose = bgnPose
                self.badFlag = True
                if self._DEBUG:
                    print("RECEIVED BADENDPOINT MATX: CANNOT MOVE")

        self.cMet = False
        builtins._GLOB_FT_FLAG = 0

        # 1. Setup stop condition
        def stop_cond():
            """Returns true when force opposing direction of motion exceeds the limit"""

            # A. Get the wrist force
            wrench = self.ctrl.arm.get_biased_force()
            mag = wrench[2]

            if self._DEBUG:
                print("mag:", mag, "Fstp:", -self.Fstp)

            if self.badFlag:
                print("Preventing something bad")
                return True

            # C. If the force opposes motion and its magnitude more negative than condition, then
            if self.pull:
                bCond = mag >= self.Fstp
            else:
                bCond = mag <= -self.Fstp

            if bCond:
                # D. Set flag and return true
                builtins._GLOB_FT_FLAG = 1
                return True
            else:
                return False

        if self._DEBUG:
            print("Move_to_Contact: about to MOVE with speed =", self.speed)

        self.ctrl.arm.move_speed(
            self.pose, self.mode, self.speed, self.accel, 0, stop_cond, True
        )

    def update(self):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Check that the arm is moving, goal reached, or failed
        """
        self.logger.debug("  %s [Move_Arm::update()]" % self.name)
        # 1. If the robot arm has stopped (ASSUMPTION: Robot has stopped because it has finished moving)
        if int(self.ctrl.arm.get_status_bits(), 2) == 1:
            # A. if the condition was met
            if builtins._GLOB_FT_FLAG:
                if self._DEBUG:
                    print("Move_to_Contact: The condition was met!")
                return py_trees.common.Status.SUCCESS
            # B. Else the robot ended its motion without touching anything
            else:
                if self._DEBUG:
                    print("Move_to_Contact: Ended WITHOUT meeting condition!")
                return py_trees.common.Status.FAILURE
        # 2. Otherwise the robot arm has not stopped
        else:
            #             print( "Movement is running!" )
            return py_trees.common.Status.RUNNING


# ___ End Move Arm ___


class centering_XY_offset:
    """Get a pose that will"""

    def __init__(self, ctrl, marginT=0.01, moveStep=0.002):
        """Calculate a move that relieves torque on the wrist, does not move"""
        # NOTE: This function assumes that the gripper is oriented vertically
        self.ctrl = ctrl
        self.marginT = marginT
        self.moveStep = moveStep

    def __call__(self):
        wrench = self.ctrl.arm.get_biased_force()
        diffX = wrench[4]  # Translate perpendicular to torque axis
        diffY = wrench[3]
        handPose = self.ctrl.arm.get_tcp_pose()
        goPose = handPose.copy()
        if abs(diffX) > self.marginT:
            #             self.logger.debug("Centering X!")
            if diffX > 0:  # If more right, then move left
                goPose = rm.poses.translate_pose(
                    goPose, translation_vec=[+self.moveStep, 0.0, 0.0], dir_pose="self"
                )
            else:  # else more left, go right
                goPose = rm.poses.translate_pose(
                    goPose, translation_vec=[-self.moveStep, 0.0, 0.0], dir_pose="self"
                )
        if abs(diffY) > self.marginT:
            #             self.logger.debug("Centering Y!")
            if diffY > 0:  # If more right, then move left
                goPose = rm.poses.translate_pose(
                    goPose, translation_vec=[0.0, -self.moveStep, 0.0], dir_pose="self"
                )
            else:  # else more left, go right
                goPose = rm.poses.translate_pose(
                    goPose, translation_vec=[0.0, +self.moveStep, 0.0], dir_pose="self"
                )
        return goPose


class Minimize_Wrist_Torque_XY(py_trees.composites.Sequence):
    """Make XY Moves until XY torques are minimized"""

    def __init__(
        self,
        name="Minimize_Wrist_Torque_XY",
        numTrials=10,
        marginT=0.01,
        moveStep=0.002,
        ctrl=None,
    ):
        """Set params"""
        super().__init__(name=name, memory=True)
        self.ctrl = ctrl
        self.marginT = marginT
        self.moveStep = moveStep

        # 1. Mark where we began the operations
        poseRecorder = Store_Current_Pose(ctrl=self.ctrl)
        self.add_child(
            poseRecorder
        )  # NOTE: `Store_Current_Pose` is store-once by default
        self.poseKey = poseRecorder.key  # Get the dictionary key where this was stored

        # 2. Init the generator
        self.genr = centering_XY_offset(
            ctrl=self.ctrl, marginT=marginT, moveStep=moveStep
        )

        forceFunc = self.ctrl.arm.get_biased_force
        margin = self.marginT

        def minimized(*args):
            wrench = forceFunc()
            #             print( "Minimized is running, Wrench is:" , wrench )
            diffX = wrench[4]  # Translate perpendicular to torque axis
            diffY = wrench[3]
            if abs(diffX) <= margin and abs(diffY) <= margin:
                #                 print( "torques MINIMIZES" )
                builtins._GLOB_FT_FLAG = 1
                return 1
            else:
                return 0

        # 3. Spiral Search is a series of small arm movements under a condition
        self.add_child(
            Run_to_X_Failures_DECO(
                Move_Rule_w_Stop_Cond(
                    self.genr, minimized, condSuccess=True, fetchInit="", ctrl=self.ctrl
                ),
                X_allowedFails=numTrials,
            )
        )

    def initialise(self):
        builtins._GLOB_FT_FLAG = 0


class Maintain_Z_Pressure(py_trees.behaviour.Behaviour):
    """Hold down the TCP at a certain pressure for a set duration"""

    def __init__(
        self,
        timeOut_s,
        plungeMove=1,
        plungeDepth=0.050,
        descendSpeed=0.010,
        zPress=2.0,
        maxZStep=0.010,
        biasWrist=1,
        stop_condition=utils.condition_false,
        stepSleep=0.1,
        relieveT=False,
        relieveStep=0.0002,
        stepPress=0.0002,
        ctrl=None,
        reverseRelief=0,
        ft_cond=0,
        cond_success=1,
        delayRelief=0.0,
    ):
        """Set up the behaviour"""
        super().__init__(name="Maintain_Z_Pressure")
        self.ctrl = ctrl
        self.timeOut_s = timeOut_s
        self.plungeMove = plungeMove
        self.plungeDepth = plungeDepth
        self.descendSpeed = descendSpeed
        self.zPress = zPress
        self.maxZStep = maxZStep
        self.biasWrist = biasWrist
        self.stop_condition = stop_condition
        self.stepSleep = stepSleep
        self.relieveT = relieveT
        self.relieveStep = relieveStep
        self.stepPress = stepPress
        self.reverseRelief = reverseRelief
        self.ft_cond = ft_cond
        self.cond_success = cond_success
        self.delayRelief = delayRelief

    def _give_condition(self):
        return self.stop_condition

    def initialise(self):
        """Performs the behavior , BLOCKS"""
        # WARNING: BEHAVIOR IS BLOCKING!

        builtins._GLOB_FT_FLAG = 0

        bgn = time.time()

        self.ctrl.maintain_z_pressure(
            timeOut_s=self.timeOut_s,
            plungeMove=self.plungeMove,
            plungeDepth=self.plungeDepth,
            descendSpeed=self.descendSpeed,
            zPress=self.zPress,
            maxZStep=self.maxZStep,
            biasWrist=self.biasWrist,
            stop_condition=self._give_condition(),
            stepSleep=self.stepSleep,
            relieveT=self.relieveT,
            relieveStep=self.relieveStep,
            stepPress=self.stepPress,
            reverseRelief=self.reverseRelief,
            delayRelief=self.delayRelief,
        )

        print(
            "Maintain_Z_Pressure: self.ctrl.maintain_z_pressure ran for",
            time.time() - bgn,
            "seconds!",
        )

    def update(self):
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


class Maintain_Y_Pressure(py_trees.behaviour.Behaviour):
    """Hold down the TCP at a certain pressure for a set duration"""

    def __init__(
        self,
        timeOut_s,
        plungeMove=1,
        plungeDepth=0.050,
        descendSpeed=0.010,
        yPress=2.0,
        maxYStep=0.010,
        biasWrist=1,
        stop_condition=utils.condition_false,
        stepSleep=0.1,
        relieveT=False,
        relieveStep=0.0002,
        stepPress=0.0002,
        ctrl=None,
        reverseRelief=0,
        ft_cond=0,
        cond_success=1,
        delayRelief=0.0,
    ):
        """Set up the behaviour"""
        super().__init__(name="Maintain_Y_Pressure")
        self.ctrl = ctrl
        self.timeOut_s = timeOut_s
        self.plungeMove = plungeMove
        self.plungeDepth = plungeDepth
        self.descendSpeed = descendSpeed
        self.yPress = yPress
        self.maxYStep = maxYStep
        self.biasWrist = biasWrist
        self.stop_condition = stop_condition
        self.stepSleep = stepSleep
        self.relieveT = relieveT
        self.relieveStep = relieveStep
        self.stepPress = stepPress
        self.reverseRelief = reverseRelief
        self.ft_cond = ft_cond
        self.cond_success = cond_success
        self.delayRelief = delayRelief

    def _give_condition(self):
        return self.stop_condition

    def initialise(self):
        """Performs the behavior , BLOCKS"""
        # WARNING: BEHAVIOR IS BLOCKING!

        builtins._GLOB_FT_FLAG = 0

        bgn = time.time()

        self.ctrl.maintain_y_pressure(
            timeOut_s=self.timeOut_s,
            plungeMove=self.plungeMove,
            plungeDepth=self.plungeDepth,
            descendSpeed=self.descendSpeed,
            yPress=self.yPress,
            maxYStep=self.maxYStep,
            biasWrist=self.biasWrist,
            stop_condition=self._give_condition(),
            stepSleep=self.stepSleep,
            relieveT=self.relieveT,
            relieveStep=self.relieveStep,
            stepPress=self.stepPress,
            reverseRelief=self.reverseRelief,
            delayRelief=self.delayRelief,
        )

        print(
            "Maintain_Y_Pressure: self.ctrl.maintain_y_pressure ran for",
            time.time() - bgn,
            "seconds!",
        )

    def update(self):
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


class Set_Down_at_Pose(py_trees.composites.Sequence):
    """Set something down at a pose, gently"""

    def __init__(
        self,
        setDownPose,
        objMinWidth=0.010,
        distAway=0.020,
        setForce=2.0,
        finalOpen=1.0,
        ySAFE=0.150,
        ctrl=None,
    ):
        super().__init__(name="Set_Down_at_Pose", memory=0)
        self.ctrl = ctrl
        self.awayPose = rm.poses.translate_pose(
            setDownPose, translation_vec=[0.0, distAway, 0.0], dir_pose="origin"
        )
        self.throughPose = rm.poses.translate_pose(
            setDownPose, translation_vec=[0.0, -distAway, 0.0], dir_pose="origin"
        )

        # 0. Precondition: Be holding an object
        pCond = Finger_Width_COND(partWidth=objMinWidth, ctrl=ctrl)
        labeled_precondition(self, pCond)
        self.add_child(pCond)  # Checked continuously

        setDownSeq = py_trees.composites.Sequence(name="Set_Down_Seq", memory=1)
        # 2. Jog over the position
        setDownSeq.add_child(
            Jog_Safe(self.abovePose, ySAFE=ySAFE, hover=1, ctrl=self.ctrl)
        )

        # 3. Move to the approach pose
        setDownSeq.add_child(Move_Arm(self.abovePose, ctrl=ctrl))

        # 4. Set down with (gentle) force
        setDownSeq.add_child(
            Move_to_Contact(
                self.belowPose,
                setForce,
                speed=0.0625,
                accel=0.175,
                biasWrist=1,
                ctrl=ctrl,
            )
        )

        # 5. Open hand to preset
        setDownSeq.add_child(Set_Fingers(finalOpen, ctrl=ctrl))

        # Add sub-seq
        self.add_child(setDownSeq)


class SpinPressGen:
    """Class that generates a pose that rotates about the wrist while maintaining some -Z pressure on the wrist"""

    def __init__(self, angleStep_deg, zPress, stepPress, ctrl=None):
        """Set vars for the pressure spin operation"""
        # NOTE: This generator assumes that the hand is already in contact with the piece that undergoes spinning pressure
        self.ctrl = ctrl
        self.angleStepRad = math.radians(angleStep_deg)
        self.zPress = zPress
        self.stepPress = stepPress

    # Pressure condition
    def Zforce_cond(self, wrench):
        if wrench[2] < -self.zPress:
            return 1
        else:
            return 0

    def __call__(self):
        """This is a functor"""
        # 1. Get the current pose
        currPose = self.ctrl.arm.get_tcp_pose()
        # 2. Get the current pressure
        crWrench = self.ctrl.arm.get_biased_force()
        # 3. Twist the current pose
        trgtPose = utils.rotate_pose(
            currPose, [0.0, 0.0, self.angleStepRad], dir_pose="origin"
        )
        # 4. Determine if the pressure is above or below the threshold
        if self.Zforce_cond(crWrench):
            stepZ = (
                self.stepPress * 0.5
            )  # Don't back off as much, screw is running away
        else:
            stepZ = -self.stepPress  # Chase hard
        # 5. Apply pressure constraint to the next pose
        trgtPose = rm.poses.translate_pose(
            trgtPose, translation_vec=[0.0, 0.0, stepZ], dir_pose="origin"
        )
        # N. Return
        return trgtPose


class Spin_Press(py_trees.composites.Sequence):
    """Preload the arm, twist, then push again"""

    def __init__(
        self,
        wideDia,
        plungeDist,
        preloadPress=3.0,
        press2=5.0,
        stopTorq=0.10,
        gripMargin=0.010,
        twist_deg=15.0,
        preLoadTech=1,
        spinStep_deg=15.0 / 50.0,
        bias_wrist=1,
        stepPress=0.0005,
        suppressLastPush=0,
        suppressRelease=0,
        ctrl=None,
    ):
        """Set up a repeating sequence to"""
        # NOTE: This behavior assumes that the hand is already at `centralPose`
        # NOTE: This behavior assumes that the hand is already open
        super().__init__(name="Spin_to_Stop", memory=1)
        self.ctrl = ctrl
        self.gen = SpinPressGen(
            angleStep_deg=spinStep_deg,
            zPress=preloadPress,
            stepPress=stepPress,
            ctrl=ctrl,
        )

        # 0. This behavior assumes that the hand is already at `centralPose`

        # 1. Grasp
        self.add_child(Set_Fingers(openState=0.0, ctrl=ctrl))

        # If using the preload technique
        if preLoadTech:

            # 3. Preload
            self.add_child(
                Move_to_Contact(
                    Fmag=preloadPress,
                    relMove=[0.0, 0.0, -plungeDist],
                    speed=0.0625,
                    accel=0.175,
                    biasWrist=1,
                    ctrl=ctrl,
                )
            )

            # 4. Twist with force condition
            self.add_child(
                Move_Arm_Relative(
                    translation=[0.0, 0.0, 0.0],
                    rotation=[0.0, 0.0, math.radians(twist_deg)],
                    mode="l",
                    speed=0.125,
                    accel=0.35,
                    stop_cond=ctrl.exceeds_Z_torque(stopTorq),
                    frame="self",
                    ctrl=ctrl,
                )
            )

        # else spin while continually applying pressure as needed
        else:
            allowedN = int(math.ceil(twist_deg / spinStep_deg))

            self.add_child(
                Run_to_X_Failures_DECO(
                    Move_Rule_w_Stop_Cond(
                        self.gen,
                        stop_cond=ctrl.exceeds_Z_torque(stopTorq),
                        condSuccess=True,
                        fetchInit=None,
                        relativeRule=False,
                        ctrl=ctrl,
                    ),
                    X_allowedFails=allowedN,
                )
            )

        # 5. Push Again
        if not suppressLastPush:
            self.add_child(
                Move_to_Contact(
                    Fmag=press2,
                    relMove=[0.0, 0.0, -plungeDist],
                    speed=0.0625,
                    accel=0.175,
                    biasWrist=1,
                    ctrl=ctrl,
                )
            )

        # 6. Open
        if not suppressRelease:
            self.add_child(Set_Fingers(openState=wideDia, ctrl=ctrl))
        else:
            self.add_child(Set_Fingers(openState=0.0, ctrl=ctrl))
