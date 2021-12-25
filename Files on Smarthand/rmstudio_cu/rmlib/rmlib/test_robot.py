import datetime

import numpy as np

import rmlib

import rmlib.rmtools as rm
from rmlib.robot import Robot
import py_trees

from py_trees.composites import Sequence, Selector

from rmlib.rmtools.asm_BT_lib.asm_tree_Basic import Move_Arm, run_BT_until_done
from rmlib.rmtools.asm_BT_lib.asm_tree_New import Spiral_Insert, Move_Arm_Relative

from rmlib.rmtools.asm_BT_lib.asm_tree_logic_flow import Force_Query_DECO

start_pose = np.array(
    [
        [-0.99621024, -0.08461295, 0.02014445, -0.11411503],
        [-0.08621264, 0.99125162, -0.09993801, -0.34670874],
        [-0.01151217, -0.10129597, -0.99478972, 0.4849095],
        [0.0, 0.0, 0.0, 1.0],
    ]
)

high_pose = np.array(
    [
        [-0.9896123, 0.00437867, 0.14369523, -0.10349781],
        [-0.02732312, 0.97558874, -0.21789918, -0.33015346],
        [-0.14114155, -0.21956191, -0.96533498, 0.67858697],
        [0.0, 0.0, 0.0, 1.0],
    ]
)


def move_arm(pose, robot):
    moveNode = Move_Arm(
        pose=pose,  # 4x4 homogeneous coord pose
        mode="l",  # { 'l': linear in task space, 'j': linear in joint space }
        speed=0.125,
        accel=0.35,
        ctrl=robot,
    )
    run_BT_until_done(moveNode)


def main():
    robot = Robot()

    move_arm(start_pose, robot)
    move_arm(high_pose, robot)
    move_arm(start_pose, robot)

    insert = Spiral_Insert(start_pose, 
        ctrl=robot, 
        suppressDrop=True,
        suppressLateral=True,
        chaseMode=0)

    run_BT_until_done(insert)

    #TODO fix spiral step to keep the tilt


if __name__ == "__main__":
    main()
