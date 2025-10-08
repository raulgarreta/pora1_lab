#!/usr/bin/env python3

import rclpy                    # ROS2 client library
from rclpy.node import Node     # ROS2 node baseclass
from std_msgs.msg import String


class ConstantControlNode(Node):
    def __init__(self) -> None:
        # give it a default node name
        super().__init__("constant_control_node")

        self.pub = self.create_publisher(String, "/constant_control" , 10)

        self.timer = self.create_timer(0.2, self.say_hello)
        self.counter = 0

    def say_hello(self):
        msg = String()
        msg.data = f"sending contsant control... {self.counter}"
        self.pub.publish(msg)
        self.counter += 1


if __name__ == "__main__":
    rclpy.init()            # initialize ROS client library
    node = ConstantControlNode()    # create the node instance
    rclpy.spin(node)        # call ROS2 default scheduler
    rclpy.shutdown()        # clean up after node exits



