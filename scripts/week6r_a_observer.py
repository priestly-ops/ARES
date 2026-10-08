#!/usr/bin/env python3
"""Passive healthy qualification evidence recorder; never publishes."""
import json
import signal
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from std_msgs.msg import String


def main():
    rclpy.init()
    node = Node('week6r_a_passive_observer')
    output = Path(sys.argv[1]).open('x', buffering=1)
    states = {}
    subscriptions = []

    def write(kind, topic, value):
        output.write(json.dumps(dict(wall_time=time.time(), kind=kind,
                                    topic=topic, value=value)) + '\n')

    def state(topic, msg):
        if states.get(topic) != msg.data:
            write('transition', topic, dict(previous=states.get(topic), state=msg.data))
            states[topic] = msg.data

    for topic in ('/ares/trust/gnss', '/ares/trust/imu', '/ares/trust/odometry',
                  '/ares/system_trust_state', '/ares/recovery_state',
                  '/ares/estimator_health', '/ares/gnss_fault_status'):
        subscriptions.append(node.create_subscription(
            String, topic, lambda msg, topic=topic: state(topic, msg), 100))

    def log(msg):
        write('rosout', msg.name, dict(message=msg.msg, level=msg.level,
                                     stamp_sec=msg.stamp.sec + msg.stamp.nanosec / 1e9))

    subscriptions.append(node.create_subscription(Log, '/rosout', log, 1000))
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    write('observer', 'status', 'ready')
    try:
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        write('observer', 'status', 'stopped')
        node.destroy_node()
        rclpy.shutdown()
        output.close()


if __name__ == '__main__':
    main()
