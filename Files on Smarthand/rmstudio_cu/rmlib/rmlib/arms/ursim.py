import subprocess
import socket
import netifaces as ni
import xmlrpc.client
import time
import numpy as np
import math

class URSim:
    def __init__(self, ursim_config, _):
#         self.robot_arm_ip = self.rm_config['robot_arm']['ip_address']
#         self.arm_max_linear_speed = self.rm_config['robot_arm']['max_linear_speed']
#         self.arm_max_linear_accel = self.rm_config['robot_arm']['max_linear_accel']
#         self.arm_max_joint_speed = self.rm_config['robot_arm']['max_joint_speed']
#         self.arm_max_joint_accel = self.rm_config['robot_arm']['max_joint_accel']

#         self.arm_default_linear_speed = self.rm_config['robot_arm']['default_linear_speed']
#         self.arm_default_linear_accel = self.rm_config['robot_arm']['default_linear_accel']
#         self.arm_default_joint_speed = self.rm_config['robot_arm']['default_joint_speed']
#         self.arm_default_joint_accel = self.rm_config['robot_arm']['default_joint_accel']

        self.dashboard_stepSize_translate = 0.08
        self.dashboard_stepSize_rotate = 0.08
        self.dashboard_stepSize_joints = 0.2

        self.tcp = np.subtract(np.divide(np.random.rand(6),5),0.1)
        self.jas = np.subtract(np.multiply(np.random.rand(6),2*np.pi),np.pi)
        self.connected = True
        
    def dummy_stop(self):
        return False

    def get_tcp_pose_vec(self):
        """
        Gives the tcp position of the effector.

        Return
        ------
        tcp_pose: [6,] list
            The pose of the effector.\n
            [x, y, z, rX, rY, rZ]

        """
        self.tcp += np.divide(np.subtract(np.random.rand(6),0.5),1000)
        return self.tcp.tolist()

    def get_tcp_pose(self):
        """
        Gives the tcp position of the effector.

        Return
        ------
        tcp_pose: [6,] list
            The pose of the effector.\n
            [x, y, z, rX, rY, rZ]

        """
        self.tcp += np.divide(np.subtract(np.random.rand(6),0.5),1000)
        return self.pose_vec_to_mtrx(self.tcp)

    def get_tcp_force(self):
        """
        Gets the 6-axis force magnitudes on the tcp.

        Return
        ------
        tcp_force: [6,] list
            The force on the effector.\n
            [nx, ny, nz, rx, ry, rz]

        """
        return [0,0,0,0,0,0]

    def get_joint_angles(self):
        """
        Get the configuration of the robot in joint space.

        Return
        ------
        joint angles: [6,] list
            The radian angles of each joint. \n
            [base, shoulder, elbow, wrist_1, wrist_2, wrist_3]

        """
        self.jas += np.divide(np.subtract(np.random.rand(6),0.5),50)
        return self.jas.tolist()
    
    def enable_freedrive(self):
        return True

    def disable_freedrive(self):
        return True

    
    def move_speed(self, target, move_type='j', speed=None, accel=None, radius=0, stop_condition='dummy', blocking=True):
        return True

    def move_timed(self, target, move_type='j', time=10, radius=0, stop_condition='dummy', blocking=True):
        return True

    def move(self, target, move_type='j', speed_per=None, accel_per=None, radius=0, stop_condition='dummy', blocking=True):
        return True
        

    def set_joint_angles_speed(self, target, move_type='j', speed=None, accel=None, stop_condition='dummy', blocking=True):
        return True

    def set_joint_angles_timed(self, target, time=10, stop_condition='dummy', blocking=True):
        return True

    def set_joint_angles(self, target, move_type='j', speed_per=None, accel_per=None, stop_condition='dummy', blocking=True):
        return True


    def translate_tcp(self, translation_vec, dir_pose='origin', move_type='j', speed_per=None, accel_per=None, stop_condition='dummy', blocking=True):
        return True


    def rotate_tcp(self, rotation_vec, dir_pose='tcp', speed_per=None, accel_per=None, stop_condition='dummy', blocking=True):
        return True

