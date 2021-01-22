import pybullet as p
import time
import rospy
import numpy as np
# the following imports and template file can be found at
# https://github.com/cairo-robotics/cairo_simulator.git
from cairo_simulator.Simulator import Simulator, SimObject, ASSETS_PATH
from cairo_simulator.Manipulators import Sawyer


MAGNIFIER_CENTER_TO_GLASS_CENTER_OFFSET = [0.0, -0.045, 0.0]
ROBOT_ARM_TO_GRIPPER_POSITION_OFFSET = [0.0, 0.0, 0.14]
ROBOT_GRIPPER_TO_MAGNIFIER_CENTER_OFFSET = [0.0, -0.13, 0.0]

IK_OFFSET = np.add(np.add(
        MAGNIFIER_CENTER_TO_GLASS_CENTER_OFFSET,
        ROBOT_GRIPPER_TO_MAGNIFIER_CENTER_OFFSET
    ),
    ROBOT_ARM_TO_GRIPPER_POSITION_OFFSET
)


def solve_inverse_kinematics(sawyer, target_pos, target_orn, offset=IK_OFFSET):
    """
    target_pos - 3D position
    target_orn - quaternion
    offset - 3D vector
    """
    ik_vector = [(a - b) for a, b in zip(target_pos, p.rotateVector(target_orn, offset))]
    return sawyer.solve_inverse_kinematics(ik_vector, target_orn)


def get_magnifier_point(solder_pos, head_pos, percent_from_head=0.5):
    assert 0 <= percent_from_head <= 1
    sf = percent_from_head
    hf = 1 - percent_from_head

    return [sp*sf + hp*hf for sp, hp in zip(solder_pos, head_pos)]


def get_magnifier_poistion(magnifier_point, default_orientation, mag_position_offset=MAGNIFIER_CENTER_TO_GLASS_CENTER_OFFSET):
    return [(i-j) for i, j in zip(magnifier_point, p.rotateVector(p.getQuaternionFromEuler(default_orientation), mag_position_offset))]


def main():
    rospy.init_node("CAIRO_Sawyer_Simulator")
    use_real_time = True

    sim = Simulator()  # Initialize the Simulator

    # Add a table and a Sawyer robot
    table = SimObject("Table", ASSETS_PATH + 'table.sdf', (0.9, 0, 0),
                      (0, 0, 1.5708))  # Table rotated 90deg along z-axis
    sawyer_robot = Sawyer("sawyer0", 0.35, 0, 0.8)

    magnifier_urdf = "cairo_simulator/assets/Magnifier_urdf/urdf/Magnifier.SLDPRT.urdf"
    magnifier_pos_ini = [0, 0, 0]
    magnifier_orn_ini = [0, 0, 0]
    magnifier = SimObject('magnifier', magnifier_urdf, magnifier_pos_ini, magnifier_orn_ini)

    # object that simulates the soldering iron
    iron_ID = p.loadURDF("cairo_simulator/assets/Soldering Iron_urdf/urdf/Soldering Iron.SLDPRT.urdf", [1.15, 0, 0.605],
                         p.getQuaternionFromEuler([0, 0, 0]))

    # object that will simulate the human head
    head_ID = p.loadURDF('cairo_simulator/assets/7.6IN_Diam_Sphere.SLDPRT/urdf/7.6IN_Diam_Sphere.SLDPRT.urdf',
                         [1.5, 0, 1], [0, 0, 0, 1])

    joint_config = sawyer_robot.solve_inverse_kinematics([0.9, 0, 1.5], [0, 0, 0, 1])
    # sawyer_robot.move_to_joint_pos(joint_config)

    # get position and orientation of cube0
    ironPos, ironOrn = p.getBasePositionAndOrientation(iron_ID)

    # initial yaw orientation of human head
    yaw = 0.4

    # Loop until someone shuts us down
    placement = 0.40
    start_time = time.time()
    while rospy.is_shutdown() is not True:
        sim.step()
        # apply force on the soldering iron to move it along y axis
        # or across the table
        force = 5.75 * np.array([0, 1, 0])
        step_pos, step_orn = p.getBasePositionAndOrientation(iron_ID)

        # Apply force to the object/soldering iron
        # make sure its position its position is not too far such that the human
        # head cannot turn that much
        
        if 6 >= time.time() - start_time >= 5:
            s = time.time() - start_time - 5
            placement = min(0.65, 0.4 + 0.25 * s)
        
        if step_pos[0] < 1.155 and step_pos[1] < 0.20:
            p.applyExternalForce(objectUniqueId=iron_ID, linkIndex=-1,
                                 forceObj=force, posObj=ironPos, flags=p.WORLD_FRAME)

            # change the yaw every step to simulate the rotation of head movement.
            if yaw > -0.3:
                yaw -= 2e-6

            # reset orientation at every step to prevent it from falling flat
            step_iron_pos, step_orn = p.getBasePositionAndOrientation(iron_ID)

            # calculate mid point between soldering iron and human head
            x1 = 1.50  # sphere x-coordinate
            y1 = 0  # sphere y-coordinate
            z1 = 1  # sphere z-coordinate

            x2 = step_iron_pos[0]  # iron x-coordinate
            y2 = step_iron_pos[1]  # iron y-coordinate
            z2 = step_iron_pos[2]  # iron z-coordinate

            midpoint = get_magnifier_point([x2, y2, z2], [x1, y1, z1], placement)

        # reset the position of the head object every step to prevent it from
        # falling due to gravity
        p.resetBasePositionAndOrientation(bodyUniqueId=head_ID, posObj=(1.50, 0, 1), ornObj=(0, 0, yaw, 1))

        # reset the orientation only of soldering iron every step to prevent it from
        # falling due to gravity and to simulate it being held by a human hand
        p.resetBasePositionAndOrientation(bodyUniqueId=iron_ID, posObj=step_iron_pos,
                                          ornObj=p.getQuaternionFromEuler([1, 0, 0]))

        # maintain the cube at the midpoint of head and soldering iron
        # joint_config = sawyer_robot.solve_inverse_kinematics(midpoint, p.getQuaternionFromEuler([-1, 0, 0]))

        default_orientation = [np.pi, -3*np.pi/4, 0]
        magnifier_pos = get_magnifier_poistion(midpoint, default_orientation)
        for _ in range(100):
            joint_config = solve_inverse_kinematics(
                sawyer_robot,
                target_pos=midpoint,
                target_orn=p.getQuaternionFromEuler(default_orientation)
            )
            joint_config += [0.0, 0.0]
            sawyer_robot.move_to_joint_pos(joint_config)

        p.resetBasePositionAndOrientation(bodyUniqueId=magnifier.get_simulator_id(),
                                          posObj=magnifier_pos,
                                          ornObj=p.getQuaternionFromEuler(default_orientation))

    p.disconnect()


if __name__ == "__main__":
    main()
