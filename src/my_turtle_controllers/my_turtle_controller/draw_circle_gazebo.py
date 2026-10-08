#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from my_robot_interfaces.action import NavigatePath
from my_robot_interfaces.srv import SetSpeed


class DrawCircleGazeboNode(Node):
    """Drives a TurtleBot3 in Gazebo in a circle and accepts path-following action goals."""

    def __init__(self):
        super().__init__('draw_circle_gazebo_node')

        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_subscriber_ = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10
        )
        self.timer_ = self.create_timer(0.1, self._publish_circle_motion)

        self._action_server = ActionServer(
            self, NavigatePath, 'navigate_path', self._execute_path
        )
        self._set_speed_service = self.create_service(
            SetSpeed, 'set_speed', self._set_speed_callback
        )

        self._linear_speed = 0.2
        self._angular_speed = 0.5
        self._executing_path = False
        self._current_x: float | None = None
        self._current_y: float | None = None
        self._current_yaw: float | None = None

        self._saved_waypoints: list = []
        self._saved_index: int = 0

        self.get_logger().info('Draw circle Gazebo node ready.')

    # ------------------------------------------------------------------ #
    # Callbacks                                                            #
    # ------------------------------------------------------------------ #

    def _odom_callback(self, msg: Odometry):
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self._current_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

    def _set_speed_callback(self, request: SetSpeed.Request, response: SetSpeed.Response):
        if request.linear_speed > 0.0:
            self._linear_speed = request.linear_speed
        if request.angular_speed > 0.0:
            self._angular_speed = request.angular_speed
        response.success = True
        response.message = (
            f'Speed updated: linear={self._linear_speed:.2f}, '
            f'angular={self._angular_speed:.2f}'
        )
        self.get_logger().info(response.message)
        return response

    def _publish_circle_motion(self):
        if self._executing_path:
            return
        msg = TwistStamped()
        msg.twist.linear.x = self._linear_speed
        msg.twist.angular.z = self._angular_speed
        self.publisher_.publish(msg)

    # ------------------------------------------------------------------ #
    # Navigation                                                           #
    # ------------------------------------------------------------------ #

    def _navigate_to_waypoint(
        self,
        goal_handle,
        target_x: float,
        target_y: float,
        index: int,
        total: int,
        linear_speed: float,
        timeout: float,
    ) -> str:
        """Drive toward one waypoint. Returns 'reached', 'cancelled', or 'timeout'."""
        linear_kp = 1.5
        angular_kp = 4.0
        distance_tolerance = 0.15
        step_sleep = 0.1
        start_time = time.time()

        while True:
            if goal_handle.is_cancel_requested:
                return 'cancelled'

            if timeout > 0.0 and (time.time() - start_time) > timeout:
                self.get_logger().warn(f'Waypoint {index + 1} timed out — skipping.')
                return 'timeout'

            if self._current_x is None:
                time.sleep(step_sleep)
                continue

            dx = target_x - self._current_x
            dy = target_y - self._current_y
            distance = math.hypot(dx, dy)

            if distance < distance_tolerance:
                break

            target_angle = math.atan2(dy, dx)
            angle_error = target_angle - self._current_yaw
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            twist = TwistStamped()
            twist.twist.linear.x = min(linear_kp * distance, linear_speed)
            twist.twist.angular.z = angular_kp * angle_error
            self.publisher_.publish(twist)

            feedback = NavigatePath.Feedback()
            feedback.current_index = index
            feedback.total_waypoints = total
            feedback.remaining_distance = float(distance)
            feedback.current_x = float(self._current_x)
            feedback.current_y = float(self._current_y)
            goal_handle.publish_feedback(feedback)

            self.get_logger().info(
                f'WP {index + 1}/{total} dist={distance:.2f} '
                f'pos=({self._current_x:.2f}, {self._current_y:.2f})'
            )
            time.sleep(step_sleep)

        self.publisher_.publish(TwistStamped())
        return 'reached'

    def _execute_path(self, goal_handle):
        request = goal_handle.request

        linear_speed = request.linear_speed if request.linear_speed > 0.0 else self._linear_speed
        timeout = request.waypoint_timeout

        if request.resume and self._saved_waypoints:
            waypoints = self._saved_waypoints
            start_index = self._saved_index
            self.get_logger().info(f'Resuming from waypoint {start_index + 1}.')
        else:
            waypoints = list(request.waypoints)
            start_index = 0

        if not waypoints:
            goal_handle.abort()
            result = NavigatePath.Result()
            result.success = False
            result.message = 'No waypoints supplied.'
            result.waypoints_completed = 0
            return result

        self._executing_path = True
        self.get_logger().info(
            f'Navigating {len(waypoints)} waypoints '
            f'at linear={linear_speed:.2f}, timeout={timeout:.1f}s.'
        )

        completed = 0
        for index in range(start_index, len(waypoints)):
            waypoint = waypoints[index]

            if goal_handle.is_cancel_requested:
                self._saved_waypoints = waypoints
                self._saved_index = index
                goal_handle.canceled()
                self._executing_path = False
                result = NavigatePath.Result()
                result.success = False
                result.message = f'Cancelled at waypoint {index + 1}. Send goal with resume=true to continue.'
                result.waypoints_completed = completed
                return result

            self.get_logger().info(
                f'Moving to waypoint {index + 1}/{len(waypoints)}: '
                f'({waypoint.position.x:.2f}, {waypoint.position.y:.2f})'
            )

            reason = self._navigate_to_waypoint(
                goal_handle,
                waypoint.position.x,
                waypoint.position.y,
                index,
                len(waypoints),
                linear_speed,
                timeout,
            )

            if reason == 'cancelled':
                self._saved_waypoints = waypoints
                self._saved_index = index
                goal_handle.canceled()
                self._executing_path = False
                result = NavigatePath.Result()
                result.success = False
                result.message = f'Cancelled at waypoint {index + 1}. Send goal with resume=true to continue.'
                result.waypoints_completed = completed
                return result

            if reason == 'timeout':
                continue

            completed += 1

        self._executing_path = False
        self._saved_waypoints = []
        self._saved_index = 0
        goal_handle.succeed()
        result = NavigatePath.Result()
        result.success = True
        result.message = f'Path completed. {completed} waypoints reached.'
        result.waypoints_completed = completed
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleGazeboNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
