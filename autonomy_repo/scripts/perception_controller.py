#!/usr/bin/env python3

import numpy as np
import rclpy

from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState
from std_msgs.msg import Bool


class PerceptionController(BaseHeadingController):

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        
        self.declare_parameter("kp",2.0)

        self.declare_parameter("active",2.0)
        self.image_detected = False

        # Create subscriber
        self.subscription = self.create_subscription(
            Bool,                      # message type
            '/detector_bool',          # topic
            self.detector_callback,    # callback function
            10                         # QoS depth
        )

        self.subscription  # prevent unused variable warning

    def detector_callback(self, msg: Bool):
        # self.get_logger().info(f"Detector state: {msg.data}")
        self.image_detected = msg.data
    
    @property
    def set_kp(self) -> float:
        return self.get_parameter("kp").value

    @property
    def active(self) -> bool:
        return self.get_parameter("active").value
    
    def compute_control_with_goal(self, state: TurtleBotState, goal: TurtleBotState) -> TurtleBotControl:
        # error = wrap_angle(goal.theta - state.theta)
        # omega = self.get_parameter("kp").value * error
        # if self.active:
        #     omega = 0.2
        # else:
        #     omega = 0.0
        if not self.image_detected:
            omega = 0.2
        else:
            omega = 0.0
        return TurtleBotControl(v=0.0, omega=omega)
        


if __name__ == "__main__":
    rclpy.init()
    node = PerceptionController("perception_controller")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    