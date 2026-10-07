#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Spawn

from my_robot_interfaces.action import NavigatePath
from my_robot_interfaces.srv import SetSpeed


class DrawCircleNode(Node):
    """Drives a turtle in a circle and accepts path-following action goals.

    Features:
    - Circular motion paused during path execution
    - Proportional control navigation to waypoints
    - Per-goal speed override
    - Per-waypoint timeout (skips waypoint on timeout)
    - Cancel and resume support
    - /set_speed service to change circle speed at runtime
    - turtle_name parameter for running multiple instances
    """

    def __init__(self):
        super().__init__('draw_circle_node')

        # Feature 6: turtle_name parameter enables multiple turtle instances
        self.declare_parameter('turtle_name', 'turtle1')
        self._turtle_name = (
            self.get_parameter('turtle_name').get_parameter_value().string_value
        )

        self.publisher_ = self.create_publisher(
            Twist, f'/{self._turtle_name}/cmd_vel', 10
        )
        self.pose_subscriber_ = self.create_subscription(
            Pose, f'/{self._turtle_name}/pose', self._pose_callback, 10
        )
        self.timer_ = self.create_timer(0.1, self._publish_circle_motion)

        self._action_server = ActionServer(
            self,
            NavigatePath,
            f'{self._turtle_name}/navigate_path',
            self._execute_path,
        )

        # Feature 5: service to change circle speed at runtime
        self._set_speed_service = self.create_service(
            SetSpeed,
            f'{self._turtle_name}/set_speed',
            self._set_speed_callback,
        )

        self._linear_speed = 2.0
        self._angular_speed = 2.0
        self._executing_path = False
        self._current_pose: Pose | None = None

        # Feature 2: cancel/resume state
        self._saved_waypoints: list = []
        self._saved_index: int = 0

        # Feature 6: spawn turtle if not turtle1
        if self._turtle_name != 'turtle1':
            self._spawn_turtle()

        self.get_logger().info(f'Draw circle node ready for [{self._turtle_name}].')

    # ------------------------------------------------------------------ #
    # Callbacks                                                            #
    # ------------------------------------------------------------------ #

    def _pose_callback(self, msg: Pose):
        self._current_pose = msg

    # Feature 5: set speed service
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
        msg = Twist()
        msg.linear.x = self._linear_speed
        msg.angular.z = self._angular_speed
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
        distance_tolerance = 0.3
        step_sleep = 0.1
        start_time = time.time()

        while True:
            if goal_handle.is_cancel_requested:
                return 'cancelled'

            # Feature 4: waypoint timeout
            if timeout > 0.0 and (time.time() - start_time) > timeout:
                self.get_logger().warn(
                    f'[{self._turtle_name}] Waypoint {index + 1} timed out — skipping.'
                )
                return 'timeout'

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
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            twist = Twist()
            twist.linear.x = min(linear_kp * distance, linear_speed)
            twist.angular.z = angular_kp * angle_error
            self.publisher_.publish(twist)

            # Feature 1: publish current pose in feedback
            feedback = NavigatePath.Feedback()
            feedback.current_index = index
            feedback.total_waypoints = total
            feedback.remaining_distance = float(distance)
            feedback.current_x = float(self._current_pose.x)
            feedback.current_y = float(self._current_pose.y)
            goal_handle.publish_feedback(feedback)

            self.get_logger().info(
                f'[{self._turtle_name}] WP {index + 1}/{total} '
                f'dist={distance:.2f} '
                f'pos=({self._current_pose.x:.2f}, {self._current_pose.y:.2f})'
            )
            time.sleep(step_sleep)

        self.publisher_.publish(Twist())
        return 'reached'

    def _execute_path(self, goal_handle):
        request = goal_handle.request

        # Feature 3: use per-goal speed or fall back to node default
        linear_speed = request.linear_speed if request.linear_speed > 0.0 else self._linear_speed
        timeout = request.waypoint_timeout

        # Feature 2: resume from last cancelled point
        if request.resume and self._saved_waypoints:
            waypoints = self._saved_waypoints
            start_index = self._saved_index
            self.get_logger().info(
                f'[{self._turtle_name}] Resuming from waypoint {start_index + 1}.'
            )
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
            f'[{self._turtle_name}] Navigating {len(waypoints)} waypoints '
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
                result.message = (
                    f'Cancelled at waypoint {index + 1}. '
                    f'Send goal with resume=true to continue.'
                )
                result.waypoints_completed = completed
                return result

            self.get_logger().info(
                f'[{self._turtle_name}] Moving to waypoint {index + 1}/{len(waypoints)}: '
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
                result.message = (
                    f'Cancelled at waypoint {index + 1}. '
                    f'Send goal with resume=true to continue.'
                )
                result.waypoints_completed = completed
                return result

            if reason == 'timeout':
                # Feature 4: skip timed-out waypoint and continue
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

    # ------------------------------------------------------------------ #
    # Feature 6: spawn helper                                              #
    # ------------------------------------------------------------------ #

    def _spawn_turtle(self):
        client = self.create_client(Spawn, '/spawn')
        self.get_logger().info(f'Waiting for /spawn service to spawn [{self._turtle_name}]...')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Spawn service not available.')
            return
        req = Spawn.Request()
        req.x = 3.0
        req.y = 3.0
        req.theta = 0.0
        req.name = self._turtle_name
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        self.get_logger().info(f'Spawned [{self._turtle_name}].')


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
