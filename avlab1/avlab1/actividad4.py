#!/usr/bin/env python3
import rclpy
import threading
import numpy as np
import os
import PyKDL as kdl
from labfunctions import *
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import *
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from ament_index_python.packages import get_package_share_directory

# --- Parametros de la tarea ---
OFFSET_HERRAMIENTA = 0.05   # 5 cm por debajo del ultimo link
ALTURA_MINIMA = 0.10        # restriccion de seguridad: nunca bajar de 10 cm


def calcular_punto_herramienta(T_ee):
    """A partir de la pose del ultimo link (matriz 4x4), calcula la
    posicion del punto de la herramienta (offset en Z local)."""
    R = T_ee[0:3, 0:3]
    p_link = T_ee[0:3, 3]
    offset_local = np.array([0.0, 0.0, -OFFSET_HERRAMIENTA])
    p_tool = p_link + R @ offset_local
    return p_tool


def calcular_pose_link_deseada(p_tool_fijo, R_deseada):
    """Dado el punto fijo de la herramienta y una orientacion deseada,
    calcula donde debe estar el link del robot para que la herramienta
    no se mueva de su posicion."""
    offset_local = np.array([0.0, 0.0, -OFFSET_HERRAMIENTA])
    p_link = p_tool_fijo - R_deseada @ offset_local
    return p_link


def verificar_altura(p_link):
    """Verifica que el link no baje de la altura minima permitida."""
    return p_link[2] >= ALTURA_MINIMA


def numpy_a_kdl_rotation(R):
    """Convierte una matriz de rotacion numpy 3x3 a kdl.Rotation."""
    return kdl.Rotation(
        R[0, 0], R[0, 1], R[0, 2],
        R[1, 0], R[1, 1], R[1, 2],
        R[2, 0], R[2, 1], R[2, 2]
    )


def kdl_a_numpy_rotation(R_kdl):
    """Convierte un kdl.Rotation a matriz numpy 3x3."""
    return np.array([
        [R_kdl[0, 0], R_kdl[0, 1], R_kdl[0, 2]],
        [R_kdl[1, 0], R_kdl[1, 1], R_kdl[1, 2]],
        [R_kdl[2, 0], R_kdl[2, 1], R_kdl[2, 2]]
    ])


def main():
    rclpy.init()
    node = rclpy.create_node('reorient_ee')
    robot_client = ActionClient(node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    print("Waiting for server...")
    robot_client.wait_for_server()
    print("Connected to server")

    jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    # --- Cargar el modelo del robot para PyKDL ---
    package_path = get_package_share_directory('avlab1')
    urdf_path = os.path.join(package_path, 'urdf', 'ur5_robot.urdf')
    robot = URDF.from_xml_file(urdf_path)
    ok, tree = treeFromUrdfModel(robot)

    # CORREGIDO: el link final se llama "tool0", no "ee_link"
    chain = tree.getChain("base_link", "tool0")
    print("Numero de articulaciones en la cadena:", chain.getNrOfJoints())

    if chain.getNrOfJoints() == 0:
        print("ERROR: la cadena cinematica esta vacia. Revisa los nombres de los links.")
        node.destroy_node()
        rclpy.shutdown()
        return

    ik_solver = kdl.ChainIkSolverPos_LMA(chain)

    # --- 1. Configuracion articular inicial conocida ---
    q_actual = np.array([0.0, -np.pi/4, 0.0, np.pi/4, 0.0, 0.0])
    T_ee_inicial = fkine_ur5(q_actual)

    # --- 2. Calcular el punto fijo de la herramienta (NUNCA debe moverse) ---
    p_tool_fijo = calcular_punto_herramienta(T_ee_inicial)
    R_inicial = T_ee_inicial[0:3, 0:3]

    # --- Verificaciones de depuracion ---
    print("R_inicial:")
    print(np.round(R_inicial, 4))
    print("Determinante (debe ser 1.0):", np.linalg.det(R_inicial))
    print("R @ R.T (debe ser identidad):")
    print(np.round(R_inicial @ R_inicial.T, 4))
    print("Punto fijo de la herramienta:", np.round(p_tool_fijo, 3))

    # --- 3. Generar la secuencia de orientaciones (rotacion sobre Z local) ---
    angulos = np.linspace(0, np.pi, 20)  # 20 pasos, de 0 a 90 grados

    puntos_trayectoria = []
    tiempo_paso = 0.3  # segundos entre cada punto
    tiempo_acumulado = 0.0

    for ang in angulos:
        R_delta = kdl_a_numpy_rotation(kdl.Rotation.RotZ(ang))
        R_deseada = R_inicial @ R_delta

        p_link_deseado = calcular_pose_link_deseada(p_tool_fijo, R_deseada)

        if not verificar_altura(p_link_deseado):
            print(f"ADVERTENCIA: angulo {np.degrees(ang):.1f} grados viola la altura minima, se omite")
            continue

        target_frame = kdl.Frame(
            numpy_a_kdl_rotation(R_deseada),
            kdl.Vector(*p_link_deseado)
        )
        q_init = kdl.JntArray(chain.getNrOfJoints())
        q_out = kdl.JntArray(chain.getNrOfJoints())
        resultado = ik_solver.CartToJnt(q_init, target_frame, q_out)

        if resultado < 0:
            print(f"IK no convergio para angulo {np.degrees(ang):.1f} grados (codigo {resultado}), se omite")
            continue

        q_resultado = [q_out[i] for i in range(q_out.rows())]

        tiempo_acumulado += tiempo_paso
        punto = JointTrajectoryPoint(
            positions=q_resultado,
            velocities=[0.0] * len(jnames),
            time_from_start=rclpy.duration.Duration(seconds=tiempo_acumulado).to_msg()
        )
        puntos_trayectoria.append(punto)

    print(f"Se generaron {len(puntos_trayectoria)} puntos validos de trayectoria")

    if len(puntos_trayectoria) == 0:
        print("No se genero ninguna trayectoria valida. No se envia nada al robot.")
        node.destroy_node()
        rclpy.shutdown()
        return

    # --- 4. Enviar la trayectoria completa a Gazebo ---
    g = FollowJointTrajectory.Goal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = jnames
    g.trajectory.points = puntos_trayectoria
    robot_client.send_goal_async(g)

    print("Trayectoria enviada. El robot deberia cambiar de orientacion sin mover el punto de la herramienta.")

    try:
        rate = node.create_rate(1)
        for _ in range(int(tiempo_acumulado) + 3):
            if not rclpy.ok():
                break
            rate.sleep()
    except Exception as e:
        print(f"Aviso durante la espera: {e}")

    rclpy.shutdown()
    thread.join(timeout=2.0)

if __name__ == '__main__':
    main()