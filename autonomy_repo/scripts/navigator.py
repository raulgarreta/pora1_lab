#!/usr/bin/env python3

import rclpy                    # ROS2 client library
from rclpy.node import Node     # ROS2 node baseclass
import numpy as np
from numpy import linalg
import scipy.interpolate

from asl_tb3_lib.navigation import BaseNavigator, TrajectoryPlan
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_lib.tf_utils import quaternion_to_yaw
from asl_tb3_msgs.msg import TurtleBotState, TurtleBotControl


class NavigationNode(BaseNavigator):
    def __init__(self) -> None:
        # give it a default node name
        super().__init__("navigation_node")
        self.kp = 0.2
        self.v_treshold = 0.0001
        self.V_prev = 0.
        self.om_prev = 0.
        self.t_prev = 0.
        self.v_desired=0.15
        self.spline_alpha=0.05



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
        # x_d, xd_d, xdd_d, y_d, yd_d, ydd_d = self.get_desired_state(t)
        x_d, xd_d, xdd_d, y_d, yd_d, ydd_d = plan.desired_state(t)

        ########## Code starts here ##########
        # avoid singularity
        if abs(self.V_prev) < self.v_treshold:
            self.V_prev = self.v_treshold

        x = state.x
        y = state.y
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
        ########## Code ends here ##########

        # apply control limits
        # V = np.clip(V, -self.V_max, self.V_max)
        # om = np.clip(om, -self.om_max, self.om_max)

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

        # create the A* planner
        planner = AStar(
            statespace_lo=occupancy.statespace_lo,
            statespace_hi=occupancy.statespace_hi,
            x_init=state,
            x_goal=goal,
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