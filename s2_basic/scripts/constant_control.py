#!/usr/bin/env python3

import rclpy                    # ROS2 client library
from rclpy.node import Node     # ROS2 node baseclass
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

class ConstantControlNode(Node):
    def __init__(self) -> None:
        # give it a default node name
        super().__init__("constant_control_node")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel" , 10)

        self.cmd_timer = self.create_timer(0.2, self.send_cmd)

        self.kill_sub = self.create_subscription(Bool, "/kill", self.kill_callback, 10)
        

    def send_cmd(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.linear.y = 2.0
        msg.linear.z = 3.0
        msg.angular.x = 4.0
        msg.angular.y = 5.0
        msg.angular.z = 6.0
        self.cmd_pub.publish(msg)
        

    def kill_callback(self, msg: Bool):
        if msg.data:
            # send a Twist with zero velocity
            self.cmd_pub.publish(Twist())
            # stop timer
            self.cmd_timer.cancel()


if __name__ == "__main__":
    rclpy.init()            # initialize ROS client library
    node = ConstantControlNode()    # create the node instance
    rclpy.spin(node)        # call ROS2 default scheduler
    rclpy.shutdown()        # clean up after node exits



