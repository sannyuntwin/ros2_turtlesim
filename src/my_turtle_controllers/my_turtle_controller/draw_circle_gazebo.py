#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from my_robot_interfaces.action import NavigatePath
from my_robot_interfaces.srv import SetSpeed

# LiDAR thresholds (metres)
_DANGER_DIST = 0.35   # stop linear motion and turn away
_CAUTION_DIST = 0.60  # start scaling down linear speed


class DrawCircleGazeboNode(Node):
    """Drives a TurtleBot3 in Gazebo in a circle and accepts path-following action goals.

    Obstacle avoidance: subscribes to /scan (LiDAR). When an obstacle enters
    the forward arc the robot slows down (caution zone) or turns away (danger zone).
    This applies both during free circling and during waypoint navigation.
    """

    def __init__(self):
        super().__init__('draw_circle_gazebo_node')

        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.waypoints_pub_ = self.create_publisher(MarkerArray, '/waypoints', 10)
        self.odom_subscriber_ = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10
        )
        self.scan_subscriber_ = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, 10
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
        self._latest_scan: LaserScan | None = None

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

    def _scan_callback(self, msg: LaserScan):
        self._latest_scan = msg

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
        fwd = self._scan_range(-math.pi / 6, math.pi / 6)
        if fwd < _DANGER_DIST:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = self._avoidance_angular()
            self.get_logger().warn(f'Obstacle at {fwd:.2f} m — turning to avoid.')
        else:
            msg.twist.linear.x = self._linear_speed
            msg.twist.angular.z = self._angular_speed
        self.publisher_.publish(msg)

    # ------------------------------------------------------------------ #
    # Obstacle avoidance helpers                                           #
    # ------------------------------------------------------------------ #

    def _scan_range(self, angle_min_rad: float, angle_max_rad: float) -> float:
        """Return the minimum valid range reading in the given angular window."""
        if self._latest_scan is None:
            return float('inf')
        scan = self._latest_scan
        min_r = float('inf')
        for i, r in enumerate(scan.ranges):
            if not (scan.range_min <= r <= scan.range_max):
                continue
            angle = scan.angle_min + i * scan.angle_increment
            if angle_min_rad <= angle <= angle_max_rad:
                min_r = min(min_r, r)
        return min_r

    def _avoidance_angular(self) -> float:
        """Turn away from the closer side of the forward arc."""
        arc = math.pi / 6
        left = self._scan_range(0.0, arc)
        right = self._scan_range(-arc, 0.0)
        # Turn away from the closer obstacle
        return -self._angular_speed * 2.5 if left < right else self._angular_speed * 2.5

    # ------------------------------------------------------------------ #
    # Waypoint marker visualisation                                        #
    # ------------------------------------------------------------------ #

    def _publish_waypoint_markers(self, waypoints: list, start_index: int = 0):
        """Publish sphere markers for all waypoints; completed ones are grey."""
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, wp in enumerate(waypoints):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = stamp
            m.ns = 'waypoints'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wp.position.x
            m.pose.position.y = wp.position.y
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15
            if i < start_index:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.5, 0.5, 0.5, 0.5
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 1.0, 1.0
            arr.markers.append(m)
        self.waypoints_pub_.publish(arr)

    def _clear_waypoint_markers(self):
        """Remove all waypoint markers from RViz."""
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'waypoints'
        m.action = Marker.DELETEALL
        arr.markers.append(m)
        self.waypoints_pub_.publish(arr)

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
        angular_speed_cap: float,
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

            raw_angular = angular_kp * angle_error
            twist = TwistStamped()
            twist.twist.linear.x = min(linear_kp * distance, linear_speed)
            twist.twist.angular.z = (
                max(-angular_speed_cap, min(raw_angular, angular_speed_cap))
                if angular_speed_cap > 0.0
                else raw_angular
            )

            # Obstacle avoidance: override or scale the computed twist
            fwd = self._scan_range(-math.pi / 6, math.pi / 6)
            if fwd < _DANGER_DIST:
                # Obstacle too close: stop and steer away
                twist.twist.linear.x = 0.0
                twist.twist.angular.z = self._avoidance_angular()
                self.get_logger().warn(
                    f'WP {index + 1}/{total} — obstacle at {fwd:.2f} m, avoiding.'
                )
            elif fwd < _CAUTION_DIST:
                # Caution zone: scale linear speed down proportionally
                scale = (fwd - _DANGER_DIST) / (_CAUTION_DIST - _DANGER_DIST)
                twist.twist.linear.x *= scale

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
        angular_speed_cap = request.angular_speed if request.angular_speed > 0.0 else 0.0
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
        self._publish_waypoint_markers(waypoints, start_index)

        completed = 0
        for index in range(start_index, len(waypoints)):
            waypoint = waypoints[index]

            if goal_handle.is_cancel_requested:
                self._saved_waypoints = waypoints
                self._saved_index = index
                goal_handle.canceled()
                self._executing_path = False
                self._clear_waypoint_markers()
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
                angular_speed_cap,
                timeout,
            )

            if reason == 'cancelled':
                self._saved_waypoints = waypoints
                self._saved_index = index
                goal_handle.canceled()
                self._executing_path = False
                self._clear_waypoint_markers()
                result = NavigatePath.Result()
                result.success = False
                result.message = f'Cancelled at waypoint {index + 1}. Send goal with resume=true to continue.'
                result.waypoints_completed = completed
                return result

            if reason == 'timeout':
                continue

            completed += 1
            # Mark this waypoint grey now that it is reached
            self._publish_waypoint_markers(waypoints, index + 1)

        self._executing_path = False
        self._saved_waypoints = []
        self._saved_index = 0
        self._clear_waypoint_markers()
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
