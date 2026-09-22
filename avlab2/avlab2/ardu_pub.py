#!/usr/bin/env python3

import rclpy
import threading
from std_msgs.msg import String

# info del tópico
# ros2 topic info /led_control

# Este nodo publica comandos en el tópico ROS 2 "led_control"
# Type: std_msgs/msg/String, es un mensaje de tipo String

# std_msgs  → paquete de mensajes estándar de ROS 2
# msg       → carpeta/tipo de interfaces de mensaje
# String    → tipo de mensaje específico

def main():

  rclpy.init()

  node = rclpy.create_node('publicador')

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

      valor = int(input("Ingrese un valor entre -100 y 100: "))

      if valor >= -100 and valor <= 100:

        msg.data = str(valor)

        publisher.publish(msg)

        node.get_logger().info(
          f'Publishing: {msg.data}'
        )

      else:

        print("Valor invalido. Ingrese entre -100 y 100.")

  except KeyboardInterrupt:

    node.get_logger().info('Publisher stopped by user')

  node.destroy_node()

  rclpy.shutdown()


if __name__ == '__main__':
  main()