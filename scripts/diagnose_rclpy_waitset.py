#!/usr/bin/env python3
"""Isolate rclpy entities while diagnosing ARES executor CPU usage.

This probe is deliberately read-only: it never publishes robot commands and it
never subscribes to ground truth.  Fast DDS is selected before importing ROS.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import sys
import traceback
from typing import Any


os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'

import message_filters  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
import rclpy  # noqa: E402
from rclpy._rclpy_pybind11 import RCLError  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from tf2_ros import (  # noqa: E402
    Buffer,
    TransformBroadcaster,
    TransformListener,
)


MODES = (
    'idle',
    'plain_sub',
    'message_filter_sub',
    'message_filter_cache',
    'timer',
    'tf',
    'clock_sub',
    'diagnostics',
    'source',
)


class WaitSetProbe(Node):
    """Hold only the entities selected by one diagnostic mode."""

    def __init__(self, mode: str, topic: str, use_sim_time: bool,
                 odom_rate_hz: float, clock_rate_hz: float,
                 tf_rate_hz: float) -> None:
        super().__init__(
            f'ares_waitset_probe_{mode}',
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    'use_sim_time',
                    rclpy.Parameter.Type.BOOL,
                    use_sim_time,
                )
            ],
        )
        self.mode = mode
        self.callback_count = 0
        self.odom_count = 0
        self.clock_count = 0
        self.tf_count = 0
        self.entities: list[Any] = []

        if mode == 'plain_sub':
            self.entities.append(self.create_subscription(
                Odometry, topic, self._callback, 50))
        elif mode == 'message_filter_sub':
            subscriber = message_filters.Subscriber(
                self, Odometry, topic, 50)
            subscriber.registerCallback(self._callback)
            self.entities.append(subscriber)
        elif mode == 'message_filter_cache':
            subscriber = message_filters.Subscriber(
                self, Odometry, topic, 50)
            cache = message_filters.Cache(
                subscriber, cache_size=102, allow_headerless=False)
            self.entities.extend((subscriber, cache))
        elif mode == 'timer':
            self.entities.append(self.create_timer(0.1, self._timer_callback))
        elif mode == 'tf':
            buffer = Buffer()
            listener = TransformListener(buffer, self)
            self.entities.extend((buffer, listener))
        elif mode == 'clock_sub':
            self.entities.append(self.create_subscription(
                Clock, '/clock', self._callback, 10))
        elif mode == 'diagnostics':
            # Import lazily so every other mode remains independent of ARES.
            source_package = (
                Path(__file__).resolve().parents[1] /
                'src' / 'ares_reliability'
            )
            sys.path.insert(0, str(source_package))
            from ares_reliability.diagnostics import MonitorDiagnostics
            self.entities.append(MonitorDiagnostics(
                self,
                'ARES rclpy wait-set diagnostic probe',
                '/ares/diagnostics/rclpy_waitset_probe',
            ))
        elif mode == 'source':
            domain_id = os.environ.get('ROS_DOMAIN_ID', '0')
            if domain_id in ('', '0'):
                raise RuntimeError(
                    'source mode requires a nonzero ROS_DOMAIN_ID so it '
                    'cannot affect the running robot')
            if odom_rate_hz > 0.0:
                self.odom_publisher = self.create_publisher(
                    Odometry, topic, 50)
                self.entities.append(self.create_timer(
                    1.0 / odom_rate_hz, self._publish_odom))
            if clock_rate_hz > 0.0:
                self.clock_publisher = self.create_publisher(
                    Clock, '/clock', 10)
                self.entities.append(self.create_timer(
                    1.0 / clock_rate_hz, self._publish_clock))
            if tf_rate_hz > 0.0:
                self.tf_broadcaster = TransformBroadcaster(self)
                self.entities.append(self.create_timer(
                    1.0 / tf_rate_hz, self._publish_tf))
            if (odom_rate_hz <= 0.0 and clock_rate_hz <= 0.0 and
                    tf_rate_hz <= 0.0):
                raise ValueError(
                    'source mode requires --odom-rate-hz or --clock-rate-hz')
        elif mode != 'idle':
            raise ValueError(f'unsupported mode: {mode}')

    def _callback(self, _message: Any) -> None:
        self.callback_count += 1

    def _timer_callback(self) -> None:
        self.callback_count += 1

    def _publish_odom(self) -> None:
        self.odom_count += 1
        message = Odometry()
        message.header.stamp.sec = self.odom_count // 30
        message.header.stamp.nanosec = (
            self.odom_count % 30) * (1_000_000_000 // 30)
        self.odom_publisher.publish(message)

    def _publish_clock(self) -> None:
        self.clock_count += 1
        message = Clock()
        message.clock.sec = self.clock_count // 1000
        message.clock.nanosec = (self.clock_count % 1000) * 1_000_000
        self.clock_publisher.publish(message)

    def _publish_tf(self) -> None:
        self.tf_count += 1
        transform = TransformStamped()
        transform.header.stamp.sec = self.tf_count // 30
        transform.header.stamp.nanosec = (
            self.tf_count % 30) * (1_000_000_000 // 30)
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=MODES)
    parser.add_argument(
        '--topic', default='/ares/odom_operational',
        help='Odometry topic used by subscription modes.',
    )
    parser.add_argument(
        '--use-sim-time', action='store_true',
        help='Enable the implicit rclpy /clock time source subscription.',
    )
    parser.add_argument('--odom-rate-hz', type=float, default=0.0)
    parser.add_argument('--clock-rate-hz', type=float, default=0.0)
    parser.add_argument('--tf-rate-hz', type=float, default=0.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    node: WaitSetProbe | None = None
    try:
        rclpy.init()
        node = WaitSetProbe(
            args.mode, args.topic, args.use_sim_time,
            args.odom_rate_hz, args.clock_rate_hz, args.tf_rate_hz)
        print(
            f'READY mode={args.mode} pid={os.getpid()} '
            f'use_sim_time={str(args.use_sim_time).lower()}',
            flush=True,
        )
        rclpy.spin(node)
        return 0
    except KeyboardInterrupt:
        return 0
    except ExternalShutdownException:
        return 0
    except RCLError as error:
        # Lyrical may surface a context-invalid wait/publish race after its
        # SIGINT handler has already shut the context down.
        if not rclpy.ok():
            return 0
        print(
            f'ERROR mode={args.mode} type={type(error).__name__}: {error}',
            file=sys.stderr,
            flush=True,
        )
        traceback.print_exc()
        return 1
    except BaseException as error:
        print(
            f'ERROR mode={args.mode} type={type(error).__name__}: {error}',
            file=sys.stderr,
            flush=True,
        )
        traceback.print_exc()
        return 1
    finally:
        if node is not None:
            print(
                f'STOP mode={args.mode} callbacks={node.callback_count} '
                f'odom_published={node.odom_count} '
                f'clock_published={node.clock_count} '
                f'tf_published={node.tf_count}',
                flush=True,
            )
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    # rclpy installs its own SIGINT handling; retain a conventional SIGTERM
    # exit for automated probes without interfering with Ctrl+C cleanup.
    signal.signal(signal.SIGTERM, lambda _signum, _frame: rclpy.shutdown())
    raise SystemExit(main())
