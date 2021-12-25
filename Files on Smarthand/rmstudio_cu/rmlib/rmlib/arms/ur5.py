import math
import os
import socket
import time
import xmlrpc.client

import netifaces as ni
import numpy as np
import rmlib.rmtools as rm


class UR5:
    def __init__(self, ur5_config):
        self.robot_arm_ip = ur5_config["ip_address"]
        self.xmlrpc_port = ur5_config["xmlrpc_port"]
        self.arm_max_linear_speed = ur5_config["max_linear_speed"]
        self.arm_max_linear_accel = ur5_config["max_linear_accel"]
        self.arm_max_joint_speed = ur5_config["max_joint_speed"]
        self.arm_max_joint_accel = ur5_config["max_joint_accel"]

        self.arm_default_linear_speed = ur5_config["default_linear_speed"]
        self.arm_default_linear_accel = ur5_config["default_linear_accel"]
        self.arm_default_joint_speed = ur5_config["default_joint_speed"]
        self.arm_default_joint_accel = ur5_config["default_joint_accel"]

        self.common_poses = {
            "home": [0, math.radians(-90), 0, math.radians(-90), 0, 0],
            "-y": [
                math.radians(-90),
                math.radians(-90),
                math.radians(-90),
                math.radians(-90),
                math.radians(90),
                0,
            ],
            "+x": [
                0,
                math.radians(-90),
                math.radians(-90),
                math.radians(-90),
                math.radians(90),
                0,
            ],
            "+y": [
                math.radians(90),
                math.radians(-90),
                math.radians(-90),
                math.radians(-90),
                math.radians(90),
                0,
            ],
            "-x": [
                math.radians(180),
                math.radians(-90),
                math.radians(-90),
                math.radians(-90),
                math.radians(90),
                0,
            ],
            "cv_1": [
                -1.4595659414874,
                -1.8198393026935,
                -0.9309876600848,
                -1.9611166159259,
                1.570393443107,
                0.1136082112789,
            ],
            "cv_2": [
                1.796921849250,
                -1.744428459797,
                -0.720430199299,
                -2.24652368227,
                1.570549249649,
                0.2302896231412,
            ],
        }

        # Start rtde
        self.start_rtde()

        self.save_force_bias()

        # Start sockets to ur5
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.settimeout(5)

        self.s_dashboard = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s_dashboard.settimeout(5)

        try:
            self.s.connect((self.robot_arm_ip, 30003))
            self.s_dashboard.connect((self.robot_arm_ip, 29999))
            self.connected = True
        except Exception:
            print("Error Connecting to Arm at IP Address: " + self.robot_arm_ip)
            self.connected = False

        self.s.settimeout(None)
        time.sleep(0.05)

    def start_rtde(self):
        # Connect to rtde server as xmlrpc client
        ip = ni.ifaddresses("eth0")[ni.AF_INET][0]["addr"]
        param = f"http://{ip}:{self.xmlrpc_port}/RPC2"
        self.proxy = xmlrpc.client.ServerProxy(param)

    def power_on(self):
        self.s_dashboard.send("power on\n".encode())

    def power_off(self):
        self.s_dashboard.send("power off\n".encode())

    def brake_release(self):
        self.s_dashboard.send("brake release\n".encode())

    def dummy_stop(self):
        return False

    def get_status_bits(self):
        state = self.get_state()
        return bin(state["robot_status_bits"])

    def get_safety_status_bits(self):
        state = self.get_state()
        return bin(state["safety_status_bits"])

    def get_tcp_pose_vec(self):
        """
        Gives the tcp position of the effector.

        Return
        ------
        tcp_pose: [6,] list
            The pose of the effector.\n
            [x, y, z, rX, rY, rZ]

        """
        state = self.get_state()
        return state["actual_TCP_pose"]

    def get_tcp_pose(self):
        """
        Gives the tcp position of the effector.

        Return
        ------
        tcp_pose: [6,] list
            The pose of the effector.\n
            [x, y, z, rX, rY, rZ]

        """
        return rm.poses.pose_vec_to_mtrx(self.get_tcp_pose_vec())

    def get_joint_angles(self):
        """
        Get the configuration of the robot in joint space.

        Return
        ------
        joint angles: [6,] list
            The radian angles of each joint. \n
            [base, shoulder, elbow, wrist_1, wrist_2, wrist_3]

        """
        state = self.get_state()
        return state["actual_q"]

    def get_payload(self):
        state = self.get_state()
        return state["payload"]
    
    def get_state(self):
        state = self.proxy.get_state()
        if state is None:
            time.sleep(0.005)
            state = self.proxy.get_state()
        return state

    def save_force_bias(self):
        wrench = self.get_raw_tcp_force()
        print("saving force bias: {}".format(wrench))
        self.force_bias = wrench

    def get_raw_tcp_force(self):
        state = self.get_state()
        return state["ft_raw_wrench"]

    def get_biased_force(self):
        wrench = self.get_raw_tcp_force()
        biased_wrench = [a - b for a, b in zip(wrench, self.force_bias)]
        return biased_wrench
    
    def bias_wrist_force(self):
        return self.save_force_bias()
    
    def get_wrist_force(self):
        return self.get_biased_force()
        

    def enable_freedrive(self):
        self.send_command_to_robot(("freedrive_mode()\n"))
        return True

    def disable_freedrive(self):
        self.send_command_to_robot(("end_freedrive_mode()\n"))
        return True

    def send_command_to_robot(self, command):
        self.s.send(command.encode())

    def send_script_to_robot(self, script):
        command = "def fun():\n" + script + "end\n"
        self.send_command_to_robot(command)

    def wait_until_robot_is_finished(self, stop_condition="dummy"):
        if stop_condition == "dummy":
            stop_condition = self.dummy_stop

        # Wait for robot to start moving
        time.sleep(0.1)

        while self.get_status_bits() == bin(3):
            should_stop = stop_condition()
            if should_stop:
                self.move(self.get_tcp_pose())
                return should_stop

    def move_speed(
        self,
        target,
        move_type="j",
        speed=None,
        accel=None,
        radius=0,
        stop_condition="dummy",
        blocking=True,
    ):
        if move_type in ["j"]:
            if speed is None:
                speed = self.arm_default_joint_speed
            if accel is None:
                accel = self.arm_default_joint_accel
        elif move_type in ["l", "p"]:
            if speed is None:
                speed = self.arm_default_linear_speed
            if accel is None:
                accel = self.arm_default_linear_accel
        else:
            raise Exception("Move Type Not Supported:", move_type)

        # Limit speed to max value
        if speed > self.arm_max_joint_speed:
            speed = self.arm_max_joint_speed

        # Limit acceleration to max value
        if accel > self.arm_max_joint_accel:
            accel = self.arm_max_joint_accel

        if type(target) == np.ndarray and target.shape == (4, 4):
            target = rm.poses.pose_mtrx_to_vec(target)

        # Removed the p and r
        movement = (
            "move{0}(p{1}, a={2}, v={3}, r={4})".format(
                move_type, target, accel, speed, radius
            )
            + "\n"
        )
        self.send_command_to_robot(movement)

        if blocking:
            return self.wait_until_robot_is_finished(stop_condition=stop_condition)

        return True

    def move_timed(
        self,
        target,
        move_type="j",
        time=10,
        radius=0,
        stop_condition="dummy",
        blocking=True,
    ):
        if type(target) == np.ndarray and target.shape == (4, 4):
            target = rm.poses.pose_mtrx_to_vec(target)

        movement = (
            "move{0}(p{1}, t={2}, r={3})".format(move_type, target, time, radius) + "\n"
        )
        self.send_command_to_robot(movement)

        if blocking:
            return self.wait_until_robot_is_finished(stop_condition=stop_condition)

        return True

    def move(
        self,
        target,
        move_type="j",
        speed_per=None,
        accel_per=None,
        radius=0,
        stop_condition="dummy",
        blocking=True,
    ):
        if move_type in ["j"]:
            max_speed = self.arm_max_joint_speed
            default_speed = self.arm_default_joint_speed
            max_accel = self.arm_max_joint_accel
            default_accel = self.arm_default_joint_accel
        elif move_type in ["l", "p"]:
            max_speed = self.arm_max_linear_speed
            default_speed = self.arm_default_linear_speed
            max_accel = self.arm_max_linear_accel
            default_accel = self.arm_default_linear_accel
        else:
            raise Exception("Move Type Not Supported:", move_type)

        # Select speed value
        if speed_per is not None:
            speed = speed_per * max_speed
        else:
            speed = default_speed

        # Select acceleration value
        if accel_per is not None:
            accel = accel_per * max_accel
        else:
            accel = default_accel

        self.move_speed(
            target,
            move_type=move_type,
            speed=speed,
            accel=accel,
            radius=radius,
            stop_condition=stop_condition,
            blocking=blocking,
        )

    def set_joint_angles_speed(
        self,
        target,
        move_type="j",
        speed=None,
        accel=None,
        stop_condition="dummy",
        blocking=True,
    ):
        # Limit speed to max value
        if speed > self.arm_max_joint_speed:
            speed = self.arm_max_joint_speed
        # Limit acceleration to max value
        if accel > self.arm_max_joint_accel:
            accel = self.arm_max_joint_accel

        if isinstance(target, np.ndarray):
            target = target.tolist()
        target = [float(val) for val in target]

        movement = (
            "move{0}({1}, a={2}, v={3})".format(move_type, target, accel, speed) + "\n"
        )
        self.send_command_to_robot(movement)

        if blocking:
            return self.wait_until_robot_is_finished(stop_condition=stop_condition)
        return True

    def set_joint_angles_timed(
        self, target, time=10, stop_condition="dummy", blocking=True
    ):
        if isinstance(target, np.ndarray):
            target = target.tolist()
        target = [float(val) for val in target]

        movement = "move{0}({1}, t={2})".format(move_type, target, time) + "\n"
        self.send_command_to_robot(movement)

        if blocking:
            return self.wait_until_robot_is_finished(stop_condition=stop_condition)
        return True

    def set_joint_angles(
        self,
        target,
        move_type="j",
        speed_per=None,
        accel_per=None,
        stop_condition="dummy",
        blocking=True,
    ):
        if move_type in ["j"]:
            max_speed = self.arm_max_joint_speed
            default_speed = self.arm_default_joint_speed
            max_accel = self.arm_max_joint_accel
            default_accel = self.arm_default_joint_accel
        elif move_type in ["l", "p"]:
            max_speed = self.arm_max_linear_speed
            default_speed = self.arm_default_linear_speed
            max_accel = self.arm_max_linear_accel
            default_accel = self.arm_default_linear_accel
        else:
            raise Exception("Move Type Not Supported:", move_type)

        # Select speed value
        if speed_per is not None:
            speed = speed_per * max_speed
        else:
            speed = default_speed

        # Select acceleration value
        if accel_per is not None:
            accel = accel_per * max_accel
        else:
            accel = default_accel

        self.set_joint_angles_speed(
            target,
            move_type=move_type,
            speed=speed,
            accel=accel,
            stop_condition=stop_condition,
            blocking=blocking,
        )

    def translate_tcp(
        self,
        translation_vec,
        dir_pose="origin",
        move_type="j",
        speed_per=None,
        accel_per=None,
        stop_condition="dummy",
        blocking=True,
    ):
        current_pose = self.get_tcp_pose()
        new_pose = rm.poses.translate_pose(
            current_pose, translation_vec, dir_pose=dir_pose
        )
        self.move(
            new_pose,
            move_type=move_type,
            speed_per=speed_per,
            accel_per=accel_per,
            stop_condition=stop_condition,
            blocking=blocking,
        )

    def rotate_tcp(
        self,
        rotation_vec,
        dir_pose="origin",
        speed_per=None,
        accel_per=None,
        stop_condition="dummy",
        blocking=True,
    ):
        current_pose = self.get_tcp_pose()
        new_pose = rm.poses.rotate_pose(current_pose, rotation_vec, dir_pose=dir_pose)
        self.move(
            new_pose,
            move_type="j",
            speed_per=speed_per,
            accel_per=accel_per,
            stop_condition=stop_condition,
            blocking=blocking,
        )

    def set_tool_digital_outputs(self, pin1, pin2):
        script = "set_tool_digital_out(0,{})\nset_tool_digital_out(1,{})\n".format(
            bool(pin1), bool(pin2)
        )
        self.send_script_to_robot(script)

    def move_to_common_pose(
        self,
        pose,
        move_type="j",
        speed_per=0.25,
        accel_per=0.25,
        radius=0,
        stop_condition="dummy",
        blocking=True,
    ):
        target = self.common_poses[pose]
        self.set_joint_angles(
            target,
            move_type=move_type,
            speed_per=speed_per,
            accel_per=accel_per,
            stop_condition="dummy",
            blocking=blocking,
        )
