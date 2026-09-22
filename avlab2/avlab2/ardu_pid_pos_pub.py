#!/usr/bin/env python3
import rclpy
import threading
from std_msgs.msg import String


def main():

  rclpy.init()

  node = rclpy.create_node('publicador_posicion')

  publisher = node.create_publisher(
    String,
    'led_control',
    10
  )

  thread = threading.Thread(
    target=rclpy.spin,
    args=(node, ),
    daemon=True
  )

  thread.start()

  msg = String()

  try:

    while rclpy.ok():

      posicion = float(
        input("Ingrese posicion deseada en grados: ")
      )

      msg.data = str(posicion)

      publisher.publish(msg)

      node.get_logger().info(
        f'Publishing posicion: {msg.data} grados'
      )

  except KeyboardInterrupt:

    node.get_logger().info(
      'Publisher stopped by user'
    )

  node.destroy_node()

  rclpy.shutdown()


if __name__ == '__main__':
  main()