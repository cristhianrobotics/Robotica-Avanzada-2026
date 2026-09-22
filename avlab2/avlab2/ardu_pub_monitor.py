#!/usr/bin/env python3
import rclpy
import serial

from rclpy.node import Node
from std_msgs.msg import String


arduinoData = serial.Serial(
    '/dev/ttyACM0',
    9600,
    timeout=0.01
)


class SerialNode(Node):

    def __init__(self):

        super().__init__("led_node_ros")

        # ==================================================
        # SUSCRIPTOR
        # ROS 2 -> Arduino
        # ==================================================

        self.subscription = self.create_subscription(
            String,
            "led_control",
            self.command_callback,
            10
        )


        # ==================================================
        # PUBLICADOR
        # Arduino -> ROS 2
        # ==================================================

        self.publisher_estado = self.create_publisher(
            String,
            "motor_estado",
            10
        )


        # Revisar puerto serial periódicamente
        self.timer = self.create_timer(
            0.05,
            self.read_arduino
        )


        self.get_logger().info("Serial node initialized")


    # ======================================================
    # RECIBE REFERENCIA DESDE ROS 2
    # ======================================================

    def command_callback(self, msg):

        self.get_logger().info(
            "Sending command: %s" % msg.data
        )

        myCmd = msg.data + "\r"

        arduinoData.write(myCmd.encode())


    # ======================================================
    # RECIBE DATOS DESDE ARDUINO
    # ======================================================

    def read_arduino(self):

        if arduinoData.in_waiting > 0:

            linea = arduinoData.readline().decode(
                'utf-8',
                errors='ignore'
            ).strip()

            if linea:

                msg = String()

                msg.data = linea

                self.publisher_estado.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    arduinoData.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()