#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from turtlesim.msg import Pose

from my_robot_interfaces.action import NavigatePath


class DrawCircleNode(Node):
    """Publishes a circular motion and exposes a path-following action."""

    def __init__(self):
        super().__init__('draw_circle_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber_ = self.create_subscription(
            Pose, '/turtle1/pose', self._pose_callback, 10
        )
        self.timer_ = self.create_timer(0.1, self._publish_circle_motion)

        self._action_server = ActionServer(
            self,
            NavigatePath,
            'navigate_path',
            self._execute_path,
        )

        self._linear_speed = 2.0
        self._angular_speed = 2.0
        self._executing_path = False
        self._current_pose: Pose | None = None

        self.get_logger().info('Draw circle node ready.')

    def _pose_callback(self, msg: Pose):
        self._current_pose = msg

    def _publish_circle_motion(self):
        if self._executing_path:
            return
        msg = Twist()
        msg.linear.x = self._linear_speed
        msg.angular.z = self._angular_speed
        self.publisher_.publish(msg)

    def _navigate_to_waypoint(
        self,
        goal_handle,
        target_x: float,
        target_y: float,
        index: int,
        total: int,
    ) -> bool:
        """Drive the turtle to (target_x, target_y) with proportional control.

        Returns True when the waypoint is reached, False if cancelled.
        """
        linear_kp = 1.5
        angular_kp = 4.0
        distance_tolerance = 0.3
        step_sleep = 0.1

        while True:
            if goal_handle.is_cancel_requested:
                return False

            if self._current_pose is None:
                time.sleep(step_sleep)
                continue

            dx = target_x - self._current_pose.x
            dy = target_y - self._current_pose.y
            distance = math.hypot(dx, dy)

            if distance < distance_tolerance:
                break

            target_angle = math.atan2(dy, dx)
            angle_error = target_angle - self._current_pose.theta
            # Normalise to [-pi, pi]
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            twist = Twist()
            twist.linear.x = min(linear_kp * distance, self._linear_speed)
            twist.angular.z = angular_kp * angle_error
            self.publisher_.publish(twist)

            feedback = NavigatePath.Feedback()
            feedback.current_index = index
            feedback.remaining_distance = float(distance)
            goal_handle.publish_feedback(feedback)

            self.get_logger().info(
                f'Waypoint {index + 1}/{total}: '
                f'distance={distance:.2f}, angle_error={angle_error:.2f}'
            )
            time.sleep(step_sleep)

        # Come to a stop at the waypoint
        self.publisher_.publish(Twist())
        return True

    def _execute_path(self, goal_handle):
        request = goal_handle.request
        waypoints = request.waypoints

        if not waypoints:
            self.get_logger().warn('No waypoints provided.')
            goal_handle.abort()
            result = NavigatePath.Result()
            result.success = False
            result.message = 'No waypoints supplied.'
            return result

        self._executing_path = True
        self.get_logger().info(f'Navigating through {len(waypoints)} waypoints.')

        for index, waypoint in enumerate(waypoints):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._executing_path = False
                result = NavigatePath.Result()
                result.success = False
                result.message = 'Path execution cancelled.'
                return result

            target_x = waypoint.position.x
            target_y = waypoint.position.y
            self.get_logger().info(
                f'Moving to waypoint {index + 1}/{len(waypoints)}: '
                f'({target_x:.2f}, {target_y:.2f})'
            )

            reached = self._navigate_to_waypoint(
                goal_handle, target_x, target_y, index, len(waypoints)
            )

            if not reached:
                goal_handle.canceled()
                self._executing_path = False
                result = NavigatePath.Result()
                result.success = False
                result.message = 'Path execution cancelled.'
                return result

        self._executing_path = False
        goal_handle.succeed()
        result = NavigatePath.Result()
        result.success = True
        result.message = 'Path completed successfully.'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()

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
