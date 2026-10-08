#!/usr/bin/env python3

import math
import csv
import os
from datetime import datetime

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus


class WaypointMission(Node):

    def __init__(self):
        super().__init__('waypoint_mission')

        self._client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.waypoints = [
            (-0.302, -0.845, -0.52),
            (5.839, -4.351, 0.48),
            (13.509, -0.372, 2.60),
            (4.559, 5.021, -2.20),
            (0.708, -0.294, 0.00),
        ]

        self.current_waypoint = 0
        self.current_goal_handle = None

        # ================================
        # METRICS SETUP
        # ================================

        self.metrics_dir = os.path.expanduser(
            '~/ares_ws/metrics'
        )

        os.makedirs(
            self.metrics_dir,
            exist_ok=True
        )

        timestamp = datetime.now().strftime(
            '%Y%m%d_%H%M%S'
        )

        self.csv_file = os.path.join(
            self.metrics_dir,
            f'mission_{timestamp}.csv'
        )

        self.mission_start_time = (
            self.get_clock().now()
        )

        self.waypoint_start_time = None

        self.csv_handle = open(
            self.csv_file,
            'w',
            newline=''
        )

        self.csv_writer = csv.writer(
            self.csv_handle
        )

        self.csv_writer.writerow([
            'waypoint',
            'status',
            'time_sec'
        ])

        self.get_logger().info(
            f'Metrics file: {self.csv_file}'
        )

        self.get_logger().info(
            'Waiting for NavigateToPose action server...'
        )

        self._client.wait_for_server()

        self.get_logger().info(
            'Nav2 action server available'
        )

        self.send_next_goal()

    def yaw_to_quaternion(self, yaw):

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        return qz, qw

    def send_next_goal(self):

        if self.current_waypoint >= len(self.waypoints):

            mission_end_time = self.get_clock().now()

            total_time = (
                mission_end_time -
                self.mission_start_time
            ).nanoseconds / 1e9

            self.get_logger().info(
                '================================'
            )

            self.get_logger().info(
                'MISSION COMPLETE'
            )

            self.get_logger().info(
                f'Total mission time: '
                f'{total_time:.2f} sec'
            )

            self.get_logger().info(
                f'Metrics saved to: '
                f'{self.csv_file}'
            )

            self.get_logger().info(
                '================================'
            )

            self.csv_writer.writerow([
                'TOTAL',
                'MISSION_COMPLETE',
                f'{total_time:.3f}'
            ])

            self.csv_handle.flush()

            return

        x, y, yaw = (
            self.waypoints[
                self.current_waypoint
            ]
        )

        goal_msg = NavigateToPose.Goal()

        pose = PoseStamped()

        pose.header.frame_id = 'map'

        pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        qz, qw = self.yaw_to_quaternion(
            yaw
        )

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal_msg.pose = pose

        self.waypoint_start_time = (
            self.get_clock().now()
        )

        self.get_logger().info(
            '--------------------------------'
        )

        self.get_logger().info(
            f'Sending waypoint '
            f'{self.current_waypoint + 1}/'
            f'{len(self.waypoints)}'
        )

        self.get_logger().info(
            f'x={x:.3f}, '
            f'y={y:.3f}, '
            f'yaw={yaw:.2f}'
        )

        future = (
            self._client.send_goal_async(
                goal_msg
            )
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(
        self,
        future
    ):

        try:

            goal_handle = future.result()

        except Exception as e:

            self.get_logger().error(
                f'Failed to send goal: {e}'
            )

            return

        if not goal_handle.accepted:

            waypoint_number = (
                self.current_waypoint + 1
            )

            self.get_logger().error(
                f'Waypoint '
                f'{waypoint_number} '
                f'REJECTED'
            )

            self.write_waypoint_metric(
                waypoint_number,
                'REJECTED'
            )

            return

        self.current_goal_handle = (
            goal_handle
        )

        self.get_logger().info(
            f'Waypoint '
            f'{self.current_waypoint + 1} '
            f'goal ACCEPTED'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.result_callback
        )

    def result_callback(
        self,
        future
    ):

        try:

            result = future.result()

        except Exception as e:

            self.get_logger().error(
                f'Failed to receive '
                f'navigation result: {e}'
            )

            return

        status = result.status

        waypoint_number = (
            self.current_waypoint + 1
        )

        if (
            status ==
            GoalStatus.STATUS_SUCCEEDED
        ):

            self.write_waypoint_metric(
                waypoint_number,
                'SUCCEEDED'
            )

            self.get_logger().info(
                '================================'
            )

            self.get_logger().info(
                f'Waypoint '
                f'{waypoint_number} '
                f'SUCCEEDED'
            )

            self.get_logger().info(
                '================================'
            )

            self.current_waypoint += 1

            self.current_goal_handle = None

            self.send_next_goal()

        elif (
            status ==
            GoalStatus.STATUS_ABORTED
        ):

            self.write_waypoint_metric(
                waypoint_number,
                'ABORTED'
            )

            self.get_logger().error(
                f'Waypoint '
                f'{waypoint_number} '
                f'ABORTED'
            )

        elif (
            status ==
            GoalStatus.STATUS_CANCELED
        ):

            self.write_waypoint_metric(
                waypoint_number,
                'CANCELED'
            )

            self.get_logger().warning(
                f'Waypoint '
                f'{waypoint_number} '
                f'CANCELED'
            )

        else:

            self.write_waypoint_metric(
                waypoint_number,
                f'UNKNOWN_{status}'
            )

            self.get_logger().error(
                f'Waypoint '
                f'{waypoint_number} '
                f'finished with '
                f'unknown status '
                f'{status}'
            )

    def write_waypoint_metric(
        self,
        waypoint_number,
        status
    ):

        end_time = (
            self.get_clock().now()
        )

        if (
            self.waypoint_start_time
            is not None
        ):

            elapsed = (
                end_time -
                self.waypoint_start_time
            ).nanoseconds / 1e9

        else:

            elapsed = 0.0

        self.csv_writer.writerow([
            waypoint_number,
            status,
            f'{elapsed:.3f}'
        ])

        self.csv_handle.flush()

        self.get_logger().info(
            f'Waypoint '
            f'{waypoint_number} time: '
            f'{elapsed:.2f} sec'
        )

    def destroy_node(self):

        if hasattr(
            self,
            'csv_handle'
        ):

            if not self.csv_handle.closed:

                self.csv_handle.close()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = WaypointMission()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().warning(
            'Waypoint mission '
            'interrupted by user'
        )

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':
    main()