import math
import time
import numpy as np

import rmlib.rmtools as rm
from rmlib.arms.ur5 import UR5
from rmlib.hands.smarthand import SmartHand

from pprint import pprint

default_arm_config = {
    "ip_address": "128.138.224.180",
    "xmlrpc_port": "8003",
    "max_linear_speed": 0.25,
    "max_linear_accel": 1.2,
    "max_joint_speed": 1.05,
    "max_joint_accel": 1.4,
    "default_linear_speed": 0.1,
    "default_joint_speed": 0.7,
    "default_linear_accel": 0.8,
    "default_joint_accel": 0.8,
}

default_hand_config = {
    "finger_length": 0.0415,
    "finger_width_outer": 0.015,
    "finger_width_inner": 0.0,
    "finger_depth": 0.014,
}


class Robot:
    
    _DEBUG = 1
    
    def __init__(self, arm_config=None, hand_config=None):
        
        if hand_config is None:
            hand_config = default_hand_config
        
        if arm_config is None:
            arm_config = default_arm_config
        else:
            ## HACK ##
            hand_config = arm_config['my_components']['hand_config']
            ## HACK ##
            arm_config  = arm_config['my_components']['arm_config']
            
            
        
            
        if self._DEBUG:
            print("### ARM CONFIG ###")
            pprint( arm_config )
            print("### HAND CONFIG ###")
            pprint( hand_config )
            
            
        self.arm = UR5(arm_config)
        self.hand = SmartHand(hand_config)
        # self.camera = RealSense()
        print("Robot Ready!")

    # Capturing
    def get_and_view_color(self):
        img = self.camera.get_color_image()
        rm.viewer.show_image(img)

    def get_and_view_depth(self):
        img = self.camera.get_depth_image()
        rm.viewer.show_image(img)

    def get_and_view_cloud(self):
        cloud = self.camera.get_cloud()
        rm.viewer.show_cloud(cloud)

    def get_capture(self):
        capture = rm.capture.Capture()
        capture.color_image, capture.depth_image, capture.cloud = self.camera.get_all()
        capture.camera_pose = self.camera_pose()
        capture.camera_info = self.camera.get_info_dict()
        return capture

    # Poses
    def camera_pose(self, cam="pc"):
        tcp_pose = self.arm.get_tcp_pose()
        if cam == "pc":
            camera_pose = tcp_pose.dot(self.camera.tcp_to_pc_cam_pose)
        elif cam == "ci":
            camera_pose = tcp_pose.dot(self.camera.tcp_to_ci_cam_pose)
        else:
            raise Exception("cam: not supported")
        return camera_pose

    def limit_rotation_from_tcp(self, pose, frame="base"):
        if frame == "base":
            Tbc = self.camera_pose()
            Tco = rm.poses.invert_pose(Tbc).dot(pose)
            rpy = rm.poses.rotation_mtrx_to_rpy(Tco[0:3, 0:3])
        elif frame == "tool":
            rpy = rm.poses.rotation_mtrx_to_rpy(pose.copy()[0:3, 0:3])
        else:
            return None

        if abs(math.degrees(rpy[2])) > 90:
            pose = rm.poses.rotate_pose(
                pose, [0, 0, math.radians(180)], dir_pose="self"
            )
            pose_new = rm.poses.rotate_pose(pose, [rpy[0], rpy[1], 0], dir_pose="self")
        else:
            pose_new = rm.poses.rotate_pose(
                pose, [-rpy[0], -rpy[1], 0], dir_pose="self"
            )

        return pose_new

    # Movements
    def move_tcp_over_pose(self, pose, distance, speedl=None, rotate=False):
        if rotate == False:
            pose = rm.poses.combine_rot_and_trans_from_poses(
                rot_pose=self.arm.get_tcp_pose(), trans_pose=pose
            )
        else:
            pose = self.limit_rotation_from_tcp(pose, frame="base")

        pose = rm.poses.translate_pose(pose, translation_vec=[0, 0, -distance])
        self.arm.move_speed(pose, move_type="l", speed=speedl)

    def pick_part(
        self,
        grasp_pose,
        approach_dist,
        approach_width=0.1,
        approach_speedl=None,
        pick_speedl=None,
        retreat_speedl=None,
        force_limit=None,
    ):

        # Move to approach pose
        pose = rm.poses.translate_pose(
            grasp_pose, translation_vec=[0.0, 0.0, -approach_dist]
        )
        self.arm.move_speed(pose, move_type="l", speed=approach_speedl)

        # Set gripper width
        if self.hand.CAN_SET_WIDTH:
            self.hand.set_finger_width(approach_width)

        # Move to pose
        if force_limit is not None:
            # Gets the static force in the z axis of the gripper
            static_force = abs(self.arm.get_tcp_force()[2])

            def stop_condition():
                curr_force_z = self.arm.get_tcp_force()[2]
                biased_force = static_force - abs(curr_force_z)
                if abs(biased_force) > force_limit:
                    return True
                return False

            success = self.arm.move_speed(
                grasp_pose,
                move_type="l",
                speed=pick_speedl,
                stop_condition=stop_condition,
            )
        else:
            success = self.arm.move_speed(grasp_pose, move_type="l", speed=pick_speedl)

        # Grip
        if success:
            gripped = self.hand.grip()

        time.sleep(1)

        # Move back to approach pose
        self.move_tcp_over_pose(
            grasp_pose, approach_dist, rotate=True, speedl=retreat_speedl
        )

        # Return if object was picked
        if success:
            return gripped
        else:
            return False

    def place_part(
        self,
        place_pose,
        approach_dist,
        approach_speedl=None,
        drop_speedl=None,
        retreat_speedl=None,
        force_limit=None,
    ):

        # Move to approach pose
        pose = rm.poses.translate_pose(
            place_pose, translation_vec=[0.0, 0.0, -approach_dist]
        )
        self.arm.move_speed(pose, move_type="l", speed=approach_speedl)

        # Move to pose
        if force_limit is not None:

            # Gets the static force in the z axis of the gripper
            static_force = abs(self.arm.get_tcp_force()[2])

            def stop_condition():
                curr_force_z = self.arm.get_tcp_force()[2]
                biased_force = static_force - abs(curr_force_z)
                if abs(biased_force) > force_limit:
                    return True
                return False

            success = self.arm.move_speed(
                place_pose,
                move_type="l",
                speed=drop_speedl,
                stop_condition=stop_condition,
            )
        else:
            success = self.arm.move_speed(place_pose, move_type="l", speed=drop_speedl)

        # Release grip
        if success is True:
            released = self.hand.release()

        time.sleep(1)

        # Move back to approach pose
        self.move_tcp_over_pose(
            place_pose, approach_dist, rotate=True, speedl=retreat_speedl
        )

        if success:
            return released
        else:
            return False

    # Behavior Trees requirements
    def base_wrist_force(self):
        """Return the force in the base frame"""
        wrench = self.arm.get_unbiased_force()
        handPose = self.arm.get_tcp_pose()
        return rm.utils.transform_vectors(np.array([wrench[:3]]), handPose)[0]


    def align_tcp(self, lock_roll=False, lock_pitch=False, lock_yaw=False):
        """
        Alignes the gripper with the nearest rotational axis (principle cartesian axes).

        Parameters
        ----------
        lock_roll: bool
        lock_pitch: bool
        lock_yaw: bool
        """
        pose = self.arm.get_tcp_pose()
        rot_matrix = pose[0:3, 0:3]
        R = rm.poses.rotation_mtrx_to_rpy(rot_matrix)
        for i, value in enumerate(R):
            if i == 0 and lock_pitch:
                continue
            if i == 1 and lock_yaw:
                continue
            if i == 2 and lock_roll:
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
                raise NameError("ERROR -James")
        rot_matrix = rm.poses.rpy_to_rotation_mtrx(R)
        pose[0:3, 0:3] = rot_matrix
        rtnVal = self.arm.move(pose)
        return rtnVal
