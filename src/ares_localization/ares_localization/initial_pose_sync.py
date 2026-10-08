#!/usr/bin/env python3

import math
import re
import subprocess
import time

import rclpy
from rclpy.node import Node

from nav2_msgs.srv import SetInitialPose
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPoseSync(Node):

    def __init__(self):
        super().__init__('ares_initial_pose_sync')

        # ============================================================
        # WORLD -> MAP CALIBRATION
        #
        # We will replace these with the correct calibrated values.
        # For now these are placeholders.
        # ============================================================

        self.theta = math.radians(-102.82064156775586)
        self.tx = 6.921941458432655
        self.ty = -0.3260294232732468

        self.client = self.create_client(
            SetInitialPose,
            '/set_initial_pose'
        )

        self.get_logger().info(
            'Waiting for AMCL /set_initial_pose service...'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting...')

        self.get_logger().info('AMCL service available')

        world_pose = self.get_gazebo_pose()

        if world_pose is None:
            self.get_logger().error(
                'Could not obtain ares_jackal Gazebo pose'
            )
            return

        x_world, y_world, yaw_world = world_pose

        self.get_logger().info(
            f'Gazebo pose: '
            f'x={x_world:.3f}, '
            f'y={y_world:.3f}, '
            f'yaw={math.degrees(yaw_world):.2f} deg'
        )

        x_map, y_map, yaw_map = self.world_to_map(
            x_world,
            y_world,
            yaw_world
        )

        self.get_logger().info(
            f'Calculated map pose: '
            f'x={x_map:.3f}, '
            f'y={y_map:.3f}, '
            f'yaw={math.degrees(yaw_map):.2f} deg'
        )

        self.send_initial_pose(
            x_map,
            y_map,
            yaw_map
        )

    def get_gazebo_pose(self):
        """
        Read /world/ares_world/dynamic_pose/info
        and extract the ares_jackal world pose.
        """

        command = [
            'gz',
            'topic',
            '-e',
            '-t',
            '/world/ares_world/dynamic_pose/info'
        ]

        try:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            start_time = time.time()
            buffer = []

            while time.time() - start_time < 5.0:

                line = process.stdout.readline()

                if not line:
                    continue

                buffer.append(line)

                text = ''.join(buffer)

                # Keep buffer manageable
                if len(buffer) > 200:
                    buffer = buffer[-200:]

                if 'name: "ares_jackal"' not in text:
                    continue

                # Find latest robot block
                idx = text.rfind('name: "ares_jackal"')
                robot_text = text[idx:]

                # Wait until orientation block is complete
                if 'orientation {' not in robot_text:
                    continue

                pos_match = re.search(
                    r'position\s*\{'
                    r'.*?x:\s*([-+eE0-9.]+)'
                    r'.*?y:\s*([-+eE0-9.]+)',
                    robot_text,
                    re.DOTALL
                )

                quat_match = re.search(
                    r'orientation\s*\{'
                    r'.*?z:\s*([-+eE0-9.]+)'
                    r'.*?w:\s*([-+eE0-9.]+)',
                    robot_text,
                    re.DOTALL
                )

                if pos_match and quat_match:

                    x = float(pos_match.group(1))
                    y = float(pos_match.group(2))

                    qz = float(quat_match.group(1))
                    qw = float(quat_match.group(2))

                    yaw = 2.0 * math.atan2(qz, qw)

                    process.terminate()

                    return x, y, yaw

            process.terminate()

        except Exception as e:
            self.get_logger().error(
                f'Gazebo pose read failed: {e}'
            )

        return None

    def world_to_map(self, xw, yw, yaww):

        c = math.cos(self.theta)
        s = math.sin(self.theta)

        xm = c * xw - s * yw + self.tx
        ym = s * xw + c * yw + self.ty

        yawm = yaww + self.theta

        # Normalize yaw to [-pi, pi]
        yawm = math.atan2(
            math.sin(yawm),
            math.cos(yawm)
        )

        return xm, ym, yawm

    def send_initial_pose(self, x, y, yaw):

        request = SetInitialPose.Request()

        pose = PoseWithCovarianceStamped()

        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = 0.0

        pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # x variance
        pose.pose.covariance[0] = 0.10

        # y variance
        pose.pose.covariance[7] = 0.10

        # yaw variance
        pose.pose.covariance[35] = 0.05

        request.pose = pose

        future = self.client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        if future.result() is not None:
            self.get_logger().info(
                'Initial AMCL pose sent successfully'
            )
        else:
            self.get_logger().error(
                'Failed to set AMCL initial pose'
            )


def main(args=None):

    rclpy.init(args=args)

    node = InitialPoseSync()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
