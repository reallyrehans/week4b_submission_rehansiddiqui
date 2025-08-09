#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time


class RoverControlUser(Node):
    def __init__(self):
        super().__init__("rover_control_user")

        self.control_pub = self.create_publisher(
            Float64MultiArray, "/rover/control_input", 10
        )

        self.get_logger().info("User Rover Control Started")
        self.get_logger().info(
            "Publishing 8-element test commands to /rover/control_input"
        )
        # fl, fr, rl, rr | steer then drive

        wheel_speed = 1
        turn_omega = 3.14 / 6

        # Moving forward 2 metres
        self.get_logger().info("Moving 2m forward")
        self.command_rover(drive=[wheel_speed] * 4)
        time.sleep(2 / wheel_speed)
        self.command_rover()  # Default all 0

        # Turning left pi/2
        self.get_logger().info("Turning pi/2 left")
        self.command_rover(steer=[turn_omega] * 4)
        time.sleep((3.14 / 2) / turn_omega)
        self.command_rover()  # Default all 0

        # Moving forward 2 metres
        self.get_logger().info("Movisng 2m forward")
        self.command_rover(drive=[wheel_speed] * 4)
        time.sleep(2 / wheel_speed)
        self.command_rover()  # Default all 0

        # Turning right pi/2
        self.get_logger().info("Turning pi/2 right")
        self.command_rover(steer=[-turn_omega] * 4)
        time.sleep((3.14 / 2) / turn_omega)
        self.command_rover()  # Default all 0

        # Independent rotate
        self.get_logger().info("Independent - steering wheels")
        self.command_rover(steer=[-turn_omega, turn_omega, turn_omega, -turn_omega])
        time.sleep((3.14 / 4) / turn_omega)
        self.get_logger().info("Independent - driving wheels")
        self.command_rover(drive=[-wheel_speed, wheel_speed, -wheel_speed, wheel_speed])
        time.sleep(4)
        self.get_logger().info("Independent - steering wheels back")
        self.command_rover(steer=[turn_omega, -turn_omega, -turn_omega, turn_omega])
        time.sleep((3.14 / 4) / turn_omega)
        self.get_logger().info("Independent - FINISH")
        self.command_rover()

        # Differential rotate
        self.get_logger().info("Differential rotate")
        diff_wspeed = 50
        self.command_rover(drive=[diff_wspeed, -diff_wspeed, diff_wspeed, -diff_wspeed])
        time.sleep(4)
        self.command_rover()  # Default all 0
        self.get_logger().info("Differential rotate FINISH")

    def command_rover(self, steer=[0] * 4, drive=[0] * 4):
        # fl, fr, rl, rr | steer then drive
        self.control_pub.publish(Float64MultiArray(data=steer + drive))


def main(args=None):
    rclpy.init(args=args)
    tester = RoverControlUser()
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
