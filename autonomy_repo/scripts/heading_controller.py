#!/usr/bin/env python3

import numpy as np
import rclpy

from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState


class HeadingController(BaseHeadingController):

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        
        self.declare_parameter("kp",2.0)
        self.declare_parameter("theta_thresh",0.01)
    
    @property
    def set_kp(self) -> float:
        return self.get_parameter("kp").value
    
    def compute_control_with_goal(self, state: TurtleBotState, goal: TurtleBotState) -> TurtleBotControl:
        error = wrap_angle(goal.theta - state.theta)
        self.get_logger().debug(f"Error: {error}")
        self.get_logger().debug(f"Goal theta: {goal.theta}")
        self.get_logger().debug(f"State theta: {state.theta}")
        if abs(error) < self.get_parameter("theta_thresh").value:
            return TurtleBotControl(v=0.0, omega=0.0)
        omega = self.get_parameter("kp").value * error
        return TurtleBotControl(v=0.0, omega=omega)
        


if __name__ == "__main__":
    rclpy.init()
    node = HeadingController("heading_controller")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    