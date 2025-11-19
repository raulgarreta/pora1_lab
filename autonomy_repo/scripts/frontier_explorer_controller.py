#!/usr/bin/env python3

import numpy as np
import rclpy

from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState
from std_msgs.msg import Bool
from asl_tb3_lib.grids import snap_to_grid, StochOccupancyGrid2D
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


def explore(occupancy):
    """ returns potential states to explore
    Args:
        occupancy (StochasticOccupancyGrid2D): Represents the known, unknown, occupied, and unoccupied states. See class in first section of notebook.

    Returns:
        frontier_states (np.ndarray): state-vectors in (x, y) coordinates of potential states to explore. Shape is (N, 2), where N is the number of possible states to explore.

    HINTS:
    - Function `convolve2d` may be helpful in producing the number of unknown, and number of occupied states in a window of a specified cell
    - Note the distinction between physical states and grid cells. Most operations can be done on grid cells, and converted to physical states at the end of the function with `occupancy.grid2state()`
    """

    window_size = 13    # defines the window side-length for neighborhood of cells to consider for heuristics
    ########################### Code starts here ###########################
    # define the kernel for convolution that gets percentage with 1s in the window
    perc_kernel = 1/(window_size**2)*np.ones((window_size, window_size))
    # apply convolution to get percentage of unknown, occupied, and unoccupied cells in the window
    unknown = convolve2d(occupancy.probs < 0, perc_kernel, mode='same')
    occupied = convolve2d(occupancy.probs > 0, perc_kernel, mode='same')
    unoccupied = convolve2d(occupancy.probs == 0, perc_kernel, mode='same')
    # get the indices that comply with the exploration heuristics
    frontier_states = np.argwhere(np.transpose((unknown >= 0.2) & (occupied == 0) & (unoccupied >= 0.3)))
    frontier_states = occupancy.grid2state(frontier_states)

    # get the closest frontier state to the current state
    closest_frontier_state_idx = np.argmin(np.linalg.norm(frontier_states - current_state, axis=1))
    closest_frontier_state = frontier_states[closest_frontier_state_idx]
    print("Closest frontier state: ", closest_frontier_state)
    print(f"Distance: {np.linalg.norm(closest_frontier_state - current_state):.2f}")
    ########################### Code ends here ###########################
    return closest_frontier_state


class FrontierExplorerController(Node):

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        
        self.declare_parameter("active", False)
        self.image_detected = False
        self.nav_success = True
        self.state = None
        self.map = None

        # Create subscriber
        self.subscription = self.create_subscription(
            Bool,                      # message type
            '/nav_success',          # topic
            self.nav_success_callback,    # callback function
            10                         # QoS depth
        )

        self.cmd_nav_sub = self.create_subscription(TurtleBotState, "/state", self.state_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)

        self.cmd_nav_pub = self.create_publisher(TurtleBotState, "/cmd_nav", 10)

    def state_callback(self, msg: TurtleBotState) -> None:
        self.state = msg

        
    def map_callback(self, msg: OccupancyGrid) -> None:
        """ Callback triggered when the map is updated

        Args:
            msg (OccupancyGrid): updated map message
        """
        self.occupancy = StochOccupancyGrid2D(
            resolution=msg.info.resolution,
            size_xy=np.array([msg.info.width, msg.info.height]),
            origin_xy=np.array([msg.info.origin.position.x, msg.info.origin.position.y]),
            window_size=9,
            probs=msg.data,
        )

    def nav_success_callback(self, msg: Bool):
        # self.get_logger().info(f"Detector state: {msg.data}")
        self.nav_success = msg.data

        # send the next goal to the navigator

        goal = explore(self.occupancy)
        self.cmd_nav_pub.publish(TurtleBotState(x=goal[0], y=goal[1], theta=0.0))
    
    @property
    def active(self) -> bool:
        return self.get_parameter("active").value
    
    

if __name__ == "__main__":
    rclpy.init()
    node = FrontierExplorerController("frontier_explorer_controller")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    