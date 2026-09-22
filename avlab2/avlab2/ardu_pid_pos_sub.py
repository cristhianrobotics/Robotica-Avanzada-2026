#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String


def callback(msg):

  print(msg.data)


def main():

  rclpy.init()

  node = rclpy.create_node(
    'suscriptor_posicion'
  )

  subscription = node.create_subscription(
    String,
    'motor_estado',
    callback,
    10
  )

  node.get_logger().info(
    'Esperando datos del PID de posicion...'
  )

  try:

    rclpy.spin(node)

  except KeyboardInterrupt:

    pass

  node.destroy_node()

  rclpy.shutdown()


if __name__ == '__main__':
  main()