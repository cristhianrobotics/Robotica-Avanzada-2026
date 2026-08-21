#!/usr/bin/env python3
import rclpy
import threading
import numpy as np
import os
import PyKDL as kdl
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import *
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from ament_index_python.packages import get_package_share_directory


def main():
    rclpy.init()
    node = rclpy.create_node('ik_gazebo')
    robot_client = ActionClient(node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    print("Waiting for server...")
    robot_client.wait_for_server()
    print("Connected to server")

    # Joint names
    jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    # --- Cinematica inversa con PyKDL ---
    package_path = get_package_share_directory('avlab1')
    urdf_path = os.path.join(package_path, 'urdf', 'ur5_robot.urdf')
    robot = URDF.from_xml_file(urdf_path)
    ok, tree = treeFromUrdfModel(robot)
    chain = tree.getChain("base_link", "ee_link")

    ik_solver = kdl.ChainIkSolverPos_LMA(chain)

    # Pose deseada (posicion y orientacion objetivo)
    target_pose = kdl.Frame(kdl.Rotation.RPY(0, 0.0, 0), kdl.Vector(0.0, 0.0, 0.9))

    q_init = kdl.JntArray(chain.getNrOfJoints())
    q_out = kdl.JntArray(chain.getNrOfJoints())
    result = ik_solver.CartToJnt(q_init, target_pose, q_out)
    print(f"Codigo de resultado del solver: {result}")

    Q0 = [q_out[i] for i in range(q_out.rows())]
    print("Resultado IK (valores articulares):")
    print(np.round(Q0, 3))

    # --- Enviar la meta al controlador (estilo command_gazebo) ---
    g = FollowJointTrajectory.Goal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = jnames
    g.trajectory.points = [
        JointTrajectoryPoint(
            positions=Q0,
            velocities=[0.0] * len(jnames),
            time_from_start=rclpy.duration.Duration(seconds=3).to_msg()
        )
    ]
    robot_client.send_goal_async(g)

    print("Meta enviada. El robot deberia moverse a la pose deseada en unos segundos.")

    # Mantener el nodo vivo un momento para que se ejecute la trayectoria
    rate = node.create_rate(1)
    for _ in range(5):
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()