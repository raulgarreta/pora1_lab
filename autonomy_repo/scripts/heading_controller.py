#!/usr/bin/env python3

import numpy as np
import rclpy

from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState


class HeadingController(BaseHeadingController):

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.kp = 0.2

    def compute_control_with_goal(self, state: TurtleBotState, goal: TurtleBotState) -> TurtleBotControl:
        error = wrap_angle(goal.theta - state.theta)
        omega = self.kp * error
        return TurtleBotControl(v=0.0, omega=omega)
        


if __name__ == "__main__":
    rclpy.init()
    node = HeadingController("heading_controller")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    