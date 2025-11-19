#!/usr/bin/env python3

import rclpy                    # ROS2 client library
from rclpy.node import Node     # ROS2 node baseclass
import rclpy.parameter          # ROS2 parameter support
import numpy as np
from numpy import linalg
import scipy.interpolate
import typing as T

from asl_tb3_lib.navigation import BaseNavigator, TrajectoryPlan
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_lib.tf_utils import quaternion_to_yaw
from asl_tb3_lib.grids import StochOccupancyGrid2D
from asl_tb3_msgs.msg import TurtleBotState, TurtleBotControl


class NavigationNode(BaseNavigator):
    def __init__(self) -> None:
        # give it a default node name
        super().__init__("navigation_node")
        self.kp = 0.5  # Increased from 0.1 for faster heading control
        self.v_treshold = 0.01  # Increased from 0.0001 for better singularity handling
        self.V_prev = 0.
        self.om_prev = 0.
        self.t_prev = 0.
        self.spline_alpha=0.05
        self.v_desired=0.15
        self.kpx = 2.0  # Increased from 1.0 for more responsive tracking
        self.kpy = 2.0  # Increased from 1.0 for more responsive tracking
        self.kdx = 2.0  # Increased from 1.0 for better damping
        self.kdy = 2.0  # Increased from 1.0 for better damping
        
        # Override default navigation thresholds for better performance
        self.set_parameters([
            rclpy.parameter.Parameter('theta_start_thresh', rclpy.Parameter.Type.DOUBLE, 0.1),  # More lenient alignment threshold
            rclpy.parameter.Parameter('plan_thresh', rclpy.Parameter.Type.DOUBLE, 0.5),  # Increased tolerance before replanning
            rclpy.parameter.Parameter('near_thresh', rclpy.Parameter.Type.DOUBLE, 0.15),  # Slightly larger goal proximity threshold
        ])


    def compute_heading_control(self,
        state: TurtleBotState,
        goal: TurtleBotState
    ) -> TurtleBotControl:
        """ Compute only orientation target (used for NavMode.ALIGN and NavMode.Park)

        Returns:
            TurtleBotControl: control target
        """
        error = wrap_angle(goal.theta - state.theta)
        omega = self.kp * error
        return TurtleBotControl(v=0.0, omega=omega)
        

    def compute_trajectory_tracking_control(self,
        state: TurtleBotState,
        plan: TrajectoryPlan,
        t: float,
    ) -> TurtleBotControl:
        """ Compute control target using a trajectory tracking controller

        Args:
            state (TurtleBotState): current robot state
            plan (TrajectoryPlan): planned trajectory
            t (float): current timestep

        Returns:
            TurtleBotControl: control command
        """

        dt = t - self.t_prev
        
        # Get desired state from the trajectory plan
        desired_state = plan.desired_state(t)
        
        # Get desired position and derivatives from spline
        x_d = desired_state.x
        y_d = desired_state.y
        xd_d = scipy.interpolate.splev(t, plan.path_x_spline, der=1)
        yd_d = scipy.interpolate.splev(t, plan.path_y_spline, der=1)
        xdd_d = scipy.interpolate.splev(t, plan.path_x_spline, der=2)
        ydd_d = scipy.interpolate.splev(t, plan.path_y_spline, der=2)

        # Initialize velocity if starting from zero
        if abs(self.V_prev) < self.v_treshold:
            # Use desired velocity magnitude as initial guess
            v_desired_mag = np.sqrt(xd_d**2 + yd_d**2)
            self.V_prev = max(self.v_treshold, min(v_desired_mag, self.v_desired))

        x = state.x
        y = state.y

        # current velocity in x and y
        xd = self.V_prev*np.cos(state.theta)
        yd = self.V_prev*np.sin(state.theta)

        # compute virtual controls
        u = np.array([xdd_d + self.kpx*(x_d-x) + self.kdx*(xd_d-xd),
                      ydd_d + self.kpy*(y_d-y) + self.kdy*(yd_d-yd)])

        # compute real controls
        J = np.array([[np.cos(state.theta), -self.V_prev*np.sin(state.theta)],
                      [np.sin(state.theta), self.V_prev*np.cos(state.theta)]])
        a, om = linalg.solve(J, u)
        V = self.V_prev + a*dt
        
        # Apply control limits to prevent excessive velocities
        V_max = 0.3  # Maximum linear velocity
        om_max = 1.0  # Maximum angular velocity (reduced from 1.5 to prevent spinning)
        V = np.clip(V, 0.0, V_max)  # Only allow forward motion during tracking
        om = np.clip(om, -om_max, om_max)
        
        # Ensure minimum forward velocity to make progress
        if V < 0.05:
            V = 0.05

        # save the commands that were applied and the time
        self.t_prev = t
        self.V_prev = V
        self.om_prev = om

        
        return TurtleBotControl(v=V, omega=om)


    def reset(self) -> None:
        self.V_prev = 0.
        self.om_prev = 0.
        self.t_prev = 0.

        

    def compute_trajectory_plan(self,
        state: TurtleBotState,
        goal: TurtleBotState,
        occupancy: StochOccupancyGrid2D,
        resolution: float,
        horizon: float,
    ) -> T.Optional[TrajectoryPlan]:
        """ Compute a trajectory plan using A* and cubic spline fitting

        Args:
            state (TurtleBotState): state
            goal (TurtleBotState): goal
            occupancy (StochOccupancyGrid2D): occupancy
            resolution (float): resolution
            horizon (float): horizon

        Returns:
            T.Optional[TrajectoryPlan]:
        """

        from P1_astar import AStar

        # Compute local planning bounds based on horizon
        # Create a planning window centered around the robot's current position
        x_min = state.x - horizon / 2
        x_max = state.x + horizon / 2
        y_min = state.y - horizon / 2
        y_max = state.y + horizon / 2

        # Clip to actual map bounds to avoid going outside the map
        # statespace_lo = np.maximum(
        #     [x_min, y_min],
        #     occupancy.statespace_lo
        # )
        # statespace_hi = np.minimum(
        #     [x_max, y_max],
        #     occupancy.statespace_hi
        # )
        statespace_lo = np.array([x_min, y_min])
        statespace_hi = np.array([x_max, y_max])

        # create the A* planner with the limited bounds
        planner = AStar(
            statespace_lo=statespace_lo,
            statespace_hi=statespace_hi,
            x_init=np.array([state.x, state.y]),
            x_goal=np.array([goal.x, goal.y]),
            occupancy=occupancy,
            resolution=resolution,
            dist_norm=2,
        )

        if not planner.solve():
            return None
        
        path = np.asarray(planner.path)

        self.reset()

        # Compute and set the following variables:
        #   1. ts: 
        #      Compute an array of time stamps for each planned waypoint assuming some constant 
        #      velocity between waypoints. 
        #
        #   2. path_x_spline, path_y_spline:
        #      Fit cubic splines to the x and y coordinates of the path separately
        #      with respect to the computed time stamp array.
        #      Hint: Use scipy.interpolate.splrep
        
        ##### YOUR CODE STARTS HERE #####
        dx = np.diff(path[:, 0])
        dy = np.diff(path[:, 1])
        ts = np.insert(np.cumsum(np.sqrt(dx**2 + dy**2)) / self.v_desired, 0, 0)
        path_x_spline = scipy.interpolate.splrep(ts, path[:,0], s=self.spline_alpha)
        path_y_spline = scipy.interpolate.splrep(ts, path[:,1], s=self.spline_alpha)
        ###### YOUR CODE END HERE ######
        
        return TrajectoryPlan(
            path=path,
            path_x_spline=path_x_spline,
            path_y_spline=path_y_spline,
            duration=ts[-1],
        )



if __name__ == "__main__":
    rclpy.init()            # initialize ROS client library
    node = NavigationNode()    # create the node instance
    rclpy.spin(node)        # call ROS2 default scheduler
    rclpy.shutdown()        # clean up after node exits