"""
Created on November 1, 2018

@author: juandhv (Juan David Hernandez Vega, juandhv@rice.edu)

Purpose: Alternative esc base controller 
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

# ROS messages
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from esc_move_base_msgs.msg import Path2D
from tf_transformations import euler_from_quaternion


class Controller(Node):
    """
    Controller class
    """

    def __init__(self):
        """
        Constructor
        """
        super().__init__("esc_move_base_control")

        # =======================================================================
        # Initial values
        # =======================================================================
        self.current_position_ = np.zeros(2)
        self.current_orientation_ = 0.0

        self.desired_position_ = np.zeros(2)
        self.desired_orientation_ = 0.0

        self.solution_path_wps_ = []

        # =======================================================================
        # Path trimmer variables
        # =======================================================================
        self.yaw = 0

        # =======================================================================
        # Get parameters
        # =======================================================================
        self.declare_parameter("max_vel", 0.1)
        self.declare_parameter("min_vel", 0.05)
        self.declare_parameter("max_turn_rate", 0.5)
        self.declare_parameter("min_turn_rate", 0.5)
        self.declare_parameter("drift_turning_vel", 0.0)
        self.declare_parameter("controller_hz", 100.0)
        self.declare_parameter("odometry_topic", "odometry_topic")
        self.declare_parameter("control_path_topic", "control_path_topic")
        self.declare_parameter("control_output_topic", "control_output_topic")
        self.declare_parameter("control_active_topic", "control_active_topic")
        self.declare_parameter("xy_goal_tolerance", 0.2)
        self.declare_parameter("yaw_goal_tolerance", 0.2)

        self.max_vel_ = self.get_parameter("max_vel").get_parameter_value().double_value
        self.min_vel_ = self.get_parameter("min_vel").get_parameter_value().double_value
        self.max_turn_rate_ = (
            self.get_parameter("max_turn_rate").get_parameter_value().double_value
        )
        self.min_turn_rate_ = (
            self.get_parameter("min_turn_rate").get_parameter_value().double_value
        )
        self.drift_turning_vel_ = (
            self.get_parameter("drift_turning_vel").get_parameter_value().double_value
        )
        self.controller_hz_ = (
            self.get_parameter("controller_hz").get_parameter_value().double_value
        )
        self.odometry_topic_ = (
            self.get_parameter("odometry_topic").get_parameter_value().string_value
        )
        self.control_path_topic_ = (
            self.get_parameter("control_path_topic").get_parameter_value().string_value
        )
        self.control_output_topic_ = (
            self.get_parameter("control_output_topic")
            .get_parameter_value()
            .string_value
        )
        self.control_active_topic_ = (
            self.get_parameter("control_active_topic")
            .get_parameter_value()
            .string_value
        )
        self.xy_goal_tolerance_ = (
            self.get_parameter("xy_goal_tolerance").get_parameter_value().double_value
        )
        self.yaw_goal_tolerance_ = (
            self.get_parameter("yaw_goal_tolerance").get_parameter_value().double_value
        )

        # =======================================================================
        # Subscribers
        # =======================================================================
        # Navigation data (feedback)
        self.odometry_sub_ = self.create_subscription(
            Odometry, self.odometry_topic_, self.odomCallback, 10
        )
        self.control_path_sub_ = self.create_subscription(
            Path2D, self.control_path_topic_, self.receiveControlPathCallback, 10
        )

        # =======================================================================
        # Publishers
        # =======================================================================
        self.control_output_pub_ = self.create_publisher(
            Twist, self.control_output_topic_, 10
        )
        self.control_active_pub_ = self.create_publisher(
            Bool, self.control_active_topic_, 10
        )
        self.controller_state = 0

        self.timer = self.create_timer(1 / self.controller_hz_, self.controlBaseEsc)

    def odomCallback(self, odometry_msg):
        """
        Callback to robot's odometry data
        """
        self.current_position_[0] = odometry_msg.pose.pose.position.x
        self.current_position_[1] = odometry_msg.pose.pose.position.y
        (r, p, self.yaw) = euler_from_quaternion(
            [
                odometry_msg.pose.pose.orientation.x,
                odometry_msg.pose.pose.orientation.y,
                odometry_msg.pose.pose.orientation.z,
                odometry_msg.pose.pose.orientation.w,
            ]
        )
        self.current_orientation_ = wrapAngle(self.yaw)
        return

    def receiveControlPathCallback(self, path_2d_msg):
        """Callback to receive path (list of waypoints)"""

        self.get_logger().warning("RECEIVED PATH =====")
        self.solution_path_wps_ = []

        waypoint_distances = np.array([])
        min_list_index = []
        actual_next_waypoint_index = None

        waypoints_list = path_2d_msg.waypoints
        # print(waypoints_list)
        # print("length:", len(waypoints_list))

        if len(waypoints_list) <= 3:
            self.solution_path_wps_.append(
                [waypoints_list[-1].x, waypoints_list[-1].y, waypoints_list[-1].theta]
            )
            return

        for waypoint in waypoints_list:
            waypoint_distances = np.append(
                waypoint_distances,
                [
                    euclidian_distance(
                        self.current_position_[0],
                        self.current_position_[1],
                        waypoint.x,
                        waypoint.y,
                    )
                ],
            )
        # print(len(waypoint_distances))
        for i in range(0, 3):
            min_list_index.append(
                np.where(waypoint_distances == np.amin(waypoint_distances))[0][0]
            )
            waypoint_distances[min_list_index[i]] = float("inf")
        # print(min_list_index)
        current_waypoint_angle = float("inf")
        for i in min_list_index:
            new_waypoint_angle = self.robotAngleToPoint(
                path_2d_msg.waypoints[i].x, path_2d_msg.waypoints[i].y
            )
            if new_waypoint_angle < current_waypoint_angle:
                actual_next_waypoint_index = i
                current_waypoint_angle = new_waypoint_angle

        if actual_next_waypoint_index is not None:
            waypoints_list = waypoints_list[actual_next_waypoint_index + 1 :]

        for i in range(1, len(waypoints_list)):
            waypoint = waypoints_list[i]
            distance_to_wp = math.sqrt(
                math.pow(waypoint.x - self.current_position_[0], 2.0)
                + math.pow(waypoint.y - self.current_position_[1], 2.0)
            )
            if distance_to_wp > self.xy_goal_tolerance_ or i == (
                len(waypoints_list) - 1
            ):
                self.solution_path_wps_.append([waypoint.x, waypoint.y, waypoint.theta])
        return

    def robotAngleToPoint(self, x, y):

        tetha_robot_waypoint = np.arctan2(
            y - self.current_position_[1], x - self.current_position_[0]
        )

        if tetha_robot_waypoint < 0:
            tetha_robot_waypoint = 2 * math.pi + tetha_robot_waypoint

        robotAngle = self.yaw

        if robotAngle < 0:
            robotAngle = 2 * math.pi + robotAngle

        if tetha_robot_waypoint > (robotAngle + math.pi):
            tetha_robot_waypoint = abs(robotAngle + 2 * math.pi - tetha_robot_waypoint)
        elif robotAngle > (tetha_robot_waypoint + math.pi):
            tetha_robot_waypoint = abs(tetha_robot_waypoint + 2 * math.pi - robotAngle)
        else:
            tetha_robot_waypoint = abs(tetha_robot_waypoint - robotAngle)

        return tetha_robot_waypoint

    def controlBaseEsc(self):
        """Control loop"""
        # print(self.solution_path_wps_)
        if len(self.solution_path_wps_) > 0:
            # print(self.solution_path_wps_)
            self.desired_position_[0] = self.solution_path_wps_[0][0]
            self.desired_position_[1] = self.solution_path_wps_[0][1]
            inc_x = self.desired_position_[0] - self.current_position_[0]
            inc_y = self.desired_position_[1] - self.current_position_[1]
            distance_to_goal = math.sqrt(math.pow(inc_x, 2.0) + math.pow(inc_y, 2.0))
            control_input = Twist()
            control_input.angular.x = 0.0
            control_input.angular.y = 0.0
            control_input.angular.z = 0.0
            control_input.linear.x = 0.0
            control_input.linear.y = 0.0
            control_input.linear.z = 0.0

            if distance_to_goal >= 0.4:
                self.controller_state = 0
                self.desired_orientation_ = wrapAngle(math.atan2(inc_y, inc_x))
                yaw_error = wrapAngle(
                    self.desired_orientation_ - self.current_orientation_
                )

                if abs(yaw_error) > 0.2:
                    self.get_logger().debug(
                        "orienting towards the next waypoint: " + str(yaw_error),
                    )
                    control_input.angular.z = yaw_error * self.max_turn_rate_
                else:
                    self.get_logger().debug(
                        "moving towards the next waypoint: " + str(yaw_error)
                    )
                    control_input.angular.z = yaw_error * self.max_turn_rate_

                    liner_speed = abs(distance_to_goal) * 0.5
                    if liner_speed < self.min_vel_:
                        control_input.linear.x = self.min_vel_
                    elif liner_speed > self.max_vel_:
                        control_input.linear.x = self.max_vel_
                    else:
                        control_input.linear.x = liner_speed

                if control_input.angular.z < -self.max_turn_rate_:
                    control_input.angular.z = -self.max_turn_rate_
                elif control_input.angular.z > self.max_turn_rate_:
                    control_input.angular.z = self.max_turn_rate_
                self.control_output_pub_.publish(control_input)

            else:
                if len(self.solution_path_wps_) > 1:
                    del self.solution_path_wps_[0]
                else:
                    if (
                        distance_to_goal >= self.xy_goal_tolerance_
                        and self.controller_state != 2
                    ):
                        self.controller_state = 1
                        self.desired_orientation_ = wrapAngle(math.atan2(inc_y, inc_x))
                        yaw_error = wrapAngle(
                            self.desired_orientation_ - self.current_orientation_
                        )
                        if abs(yaw_error) > 0.2 and abs(yaw_error) < 1.57:
                            self.get_logger().debug(
                                "final approach: orienting towards a waypoint (forward)"
                            )
                            if yaw_error > 0.0:
                                control_input.angular.z = self.min_turn_rate_
                            else:
                                control_input.angular.z = -self.min_turn_rate_
                            control_input.linear.x = self.drift_turning_vel_
                        elif abs(yaw_error) > 1.57 and abs(yaw_error) < 2.94:
                            self.get_logger().debug(
                                "final approach: orienting towards a waypoint (backward)",
                            )
                            if yaw_error > 0.0:
                                control_input.angular.z = -self.min_turn_rate_
                            else:
                                control_input.angular.z = self.min_turn_rate_

                            control_input.linear.x = -self.drift_turning_vel_
                        elif abs(yaw_error) < 0.2:
                            self.get_logger().debug(
                                "final approach: moving towards a waypoint (forward)"
                            )

                            control_input.linear.x = 0.05
                        else:
                            self.get_logger().debug(
                                "final approach: moving towards a waypoint (backward)",
                            )

                            control_input.linear.x = -0.05

                        self.control_output_pub_.publish(control_input)
                    else:
                        self.controller_state = 2
                        self.get_logger().debug(
                            "final approach, final orientation",
                        )
                        yaw_error = wrapAngle(
                            self.solution_path_wps_[0][2] - self.current_orientation_
                        )

                        if abs(yaw_error) < self.yaw_goal_tolerance_:
                            if len(self.solution_path_wps_) > 0:
                                del self.solution_path_wps_[0]
                        else:
                            control_input.angular.z = yaw_error * self.max_turn_rate_
                            if yaw_error < 0.0:
                                control_input.linear.x = self.drift_turning_vel_
                                control_input.angular.z = -self.min_turn_rate_ * 5
                            else:
                                control_input.linear.x = -self.drift_turning_vel_
                                control_input.angular.z = self.min_turn_rate_ * 5
                            self.control_output_pub_.publish(control_input)

            # self.get_logger().debug("%s: yaw_error: %f", self.get_name(), yaw_error)
            # self.get_logger().debug("%s: distance_to_goal: %f", self.get_name(), distance_to_goal)
            # self.get_logger().debug("%s: control_input.linear.x %f", self.get_name(), control_input.linear.x)
            # self.get_logger().debug("%s: control_input.angular.z %f\n", self.get_name(), control_input.angular.z)
            self.control_active_pub_.publish(Bool(data=True))
        else:
            control_input = Twist()
            self.control_output_pub_.publish(control_input)
            self.control_active_pub_.publish(Bool(data=False))
        return


def euclidian_distance(x1, y1, x2, y2):
    return np.sqrt(np.power(x2 - x1, 2) + np.power(y2 - y1, 2))


def wrapAngle(angle):
    """wrapAngle
    Calculates angles values between 0 and 2pi"""
    return angle + (2.0 * math.pi * math.floor((math.pi - angle) / (2.0 * math.pi)))


def main(args=None):
    rclpy.init(args=args)
    controller_node = Controller()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
