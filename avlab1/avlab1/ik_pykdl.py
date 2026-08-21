#!/usr/bin/env python3
import rclpy
import threading
import numpy as np
import PyKDL as kdl
from markers import *
from labfunctions import *
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from ament_index_python.packages import get_package_share_directory
import os

def main():
    rclpy.init()
    node = rclpy.create_node('InverseKinematicsPyKDL')
    pub = node.create_publisher(JointState, 'joint_states', 10)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    marker = FrameMarker(node)

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
    target_pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0.3, 0.0, 0.3))

    q_init = kdl.JntArray(chain.getNrOfJoints())
    q_out = kdl.JntArray(chain.getNrOfJoints())
    ik_solver.CartToJnt(q_init, target_pose, q_out)

    q = np.array([q_out[i] for i in range(q_out.rows())])
    print("Resultado IK (valores articulares):")
    print(np.round(q, 3))

    # Efector final respecto a la base (usando la fkine que ya tienes, para verificar)
    T = fkine_ur5(q)
    print("Pose resultante (verificacion con fkine):")
    print(np.round(T, 3))

    x0 = TF2xyzquat(T)
    marker.setPose(x0)

    # Mensaje JointState
    jstate = JointState()
    jstate.header.stamp = node.get_clock().now().to_msg()
    jstate.name = jnames
    jstate.position = q.tolist()

    rate = node.create_rate(20)

    while rclpy.ok():
        jstate.header.stamp = node.get_clock().now().to_msg()
        pub.publish(jstate)
        marker.publish()
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()