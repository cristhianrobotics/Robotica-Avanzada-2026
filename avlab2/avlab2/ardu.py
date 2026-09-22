#!/usr/bin/env python3
import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String


# Primero cargar al Arduino el código Escalamiento_Ardu.ino.
# Después se puede cerrar Arduino IDE.
#
# Este nodo recibe comandos desde el tópico ROS 2 "led_control"
# y los envía al Arduino mediante comunicación serial.
#
# Flujo:
# ROS 2 -> ardu.py -> Serial USB -> Arduino -> L298N -> Motor


# Comunicación serial con Arduino
arduinoData = serial.Serial('/dev/ttyACM0', 9600)


class SerialNode(Node):

    def __init__(self):

        super().__init__("led_node_ros")

        # Suscriptor ROS 2
        self.subscription = self.create_subscription(
            String,
            "led_control",
            self.command_callback,
            10
        )

        self.get_logger().info("Serial node initialized")
        self.get_logger().info("Esperando comandos en /led_control...")


    def command_callback(self, msg):

        # Mostrar el comando recibido desde ROS 2
        self.get_logger().info(
            "Sending command: %s" % msg.data
        )

        # Agregar retorno de carro para que Arduino pueda leer el dato
        myCmd = msg.data + "\r"

        # Enviar el comando al Arduino por Serial
        arduinoData.write(myCmd.encode())


def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    arduinoData.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()