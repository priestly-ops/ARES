#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, Twist


class CmdVelAdapter(Node):
    def __init__(self):
        super().__init__('cmd_vel_adapter')

        self.sub = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.pub = self.create_publisher(
            Twist,
            '/ares/cmd_vel',
            10
        )

        self.get_logger().info(
            'Converting /cmd_vel TwistStamped -> /ares/cmd_vel Twist'
        )

    def cmd_callback(self, msg):
        out = Twist()

        out.linear.x = msg.twist.linear.x
        out.linear.y = msg.twist.linear.y
        out.linear.z = msg.twist.linear.z

        out.angular.x = msg.twist.angular.x
        out.angular.y = msg.twist.angular.y
        out.angular.z = msg.twist.angular.z

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)

    node = CmdVelAdapter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()