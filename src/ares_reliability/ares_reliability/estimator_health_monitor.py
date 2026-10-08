#!/usr/bin/env python3
"""Monitor the experimental EKF without conflating estimator and sensor faults."""

from collections import deque
import json
import math
from typing import Any, Optional

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def odometry_health(
    message: Odometry,
    previous_stamp: Optional[float] = None,
    previous_position: Optional[tuple[float, float]] = None,
    reference_position: Optional[tuple[float, float]] = None,
    jump_threshold_m: float = 2.0,
) -> dict[str, Any]:
    """Return finite, covariance, continuity, and reference-error evidence."""
    stamp = (float(message.header.stamp.sec) +
             float(message.header.stamp.nanosec) / 1.0e9)
    x = float(message.pose.pose.position.x)
    y = float(message.pose.pose.position.y)
    covariance = [float(value) for value in message.pose.covariance]
    values = [
        x, y, float(message.pose.pose.position.z),
        float(message.pose.pose.orientation.x),
        float(message.pose.pose.orientation.y),
        float(message.pose.pose.orientation.z),
        float(message.pose.pose.orientation.w),
        float(message.twist.twist.linear.x),
        float(message.twist.twist.linear.y),
        float(message.twist.twist.angular.z),
        *covariance,
    ]
    finite = all(math.isfinite(value) for value in values)
    pose_jump = (
        math.hypot(x - previous_position[0], y - previous_position[1])
        if previous_position is not None and finite else 0.0)
    reference_error = (
        math.hypot(x - reference_position[0], y - reference_position[1])
        if reference_position is not None and finite else None)
    timestamp_regression = (
        previous_stamp is not None and stamp < previous_stamp)
    covariance_trace = sum(
        covariance[index] for index in (0, 7, 14, 21, 28, 35))
    covariance_max = max(covariance) if covariance else 0.0
    status = 'HEALTHY'
    if not finite:
        status = 'INVALID'
    elif timestamp_regression:
        status = 'TIME_REGRESSION'
    elif pose_jump > jump_threshold_m:
        status = 'POSE_JUMP'
    return {
        'status': status,
        'finite': finite,
        'stamp_sec': stamp,
        'x_m': x,
        'y_m': y,
        'pose_jump_m': pose_jump,
        'timestamp_regression': timestamp_regression,
        'covariance_trace': covariance_trace,
        'covariance_max': covariance_max,
        'reference_error_m': reference_error,
    }


class EstimatorHealthMonitor(Node):
    """Publish health evidence for `/odometry/trust_fused`."""

    def __init__(self) -> None:
        super().__init__('estimator_health_monitor')
        self.output_topic = str(self.declare_parameter(
            'output_topic', '/odometry/trust_fused').value)
        self.reference_topic = str(self.declare_parameter(
            'reference_topic', '/odometry/filtered').value)
        self.stale_timeout_sec = float(self.declare_parameter(
            'stale_timeout_sec', 1.0).value)
        self.jump_threshold_m = float(self.declare_parameter(
            'jump_threshold_m', 2.0).value)
        self.previous_stamp: Optional[float] = None
        self.previous_position: Optional[tuple[float, float]] = None
        self.reference_position: Optional[tuple[float, float]] = None
        self.gnss_position: Optional[tuple[float, float]] = None
        self.gnss_covariance_x: Optional[float] = None
        self.gnss_covariance_y: Optional[float] = None
        self.gnss_innovation_m: Optional[float] = None
        self.gnss_measurement_count = 0
        self.last_receive_sec: Optional[float] = None
        self.first_receive_sec: Optional[float] = None
        self.intervals: deque[float] = deque(maxlen=60)
        self.message_count = 0
        self.restart_count = 0
        self.publisher = self.create_publisher(
            String, '/ares/estimator_health', 20)
        self.create_subscription(
            Odometry, self.output_topic, self.output_callback, 50)
        self.create_subscription(
            Odometry, self.reference_topic, self.reference_callback, 50)
        self.create_subscription(
            Odometry, '/odometry/gps_trusted', self.gnss_callback, 20)
        self.create_timer(0.25, self.timeout_callback)

    def reference_callback(self, message: Odometry) -> None:
        self.reference_position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )

    def gnss_callback(self, message: Odometry) -> None:
        """Capture converted GNSS covariance and pre-update displacement."""
        self.gnss_position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )
        self.gnss_covariance_x = float(message.pose.covariance[0])
        self.gnss_covariance_y = float(message.pose.covariance[7])
        self.gnss_innovation_m = (
            math.hypot(
                self.gnss_position[0] - self.previous_position[0],
                self.gnss_position[1] - self.previous_position[1],
            ) if self.previous_position is not None else None)
        self.gnss_measurement_count += 1

    def output_callback(self, message: Odometry) -> None:
        now = self.get_clock().now().nanoseconds / 1.0e9
        evidence = odometry_health(
            message,
            self.previous_stamp,
            self.previous_position,
            self.reference_position,
            self.jump_threshold_m,
        )
        stamp = float(evidence['stamp_sec'])
        if self.previous_stamp is not None and stamp > self.previous_stamp:
            self.intervals.append(stamp - self.previous_stamp)
        if evidence['timestamp_regression']:
            self.restart_count += 1
        self.previous_stamp = stamp
        self.previous_position = (
            float(evidence['x_m']), float(evidence['y_m']))
        self.last_receive_sec = now
        if self.first_receive_sec is None:
            self.first_receive_sec = now
        self.message_count += 1
        interval_sum = sum(self.intervals)
        evidence.update({
            'publication_rate_hz': (
                len(self.intervals) / interval_sum
                if self.intervals and interval_sum > 0.0 else None),
            'freshness_sec': max(0.0, now - stamp),
            'message_count': self.message_count,
            'restart_count': self.restart_count,
            'uptime_sec': max(0.0, now - self.first_receive_sec),
            'stale': False,
            'output_topic': self.output_topic,
            'reference_topic': self.reference_topic,
            'publishes_tf': False,
            'gnss_measurement_x_m': (
                self.gnss_position[0] if self.gnss_position else None),
            'gnss_measurement_y_m': (
                self.gnss_position[1] if self.gnss_position else None),
            'gnss_covariance_x_m2': self.gnss_covariance_x,
            'gnss_covariance_y_m2': self.gnss_covariance_y,
            'gnss_innovation_m': self.gnss_innovation_m,
            'gnss_measurement_count': self.gnss_measurement_count,
        })
        self._publish(evidence)

    def timeout_callback(self) -> None:
        now = self.get_clock().now().nanoseconds / 1.0e9
        if (self.last_receive_sec is not None and
                now - self.last_receive_sec <= self.stale_timeout_sec):
            return
        self._publish({
            'status': 'STALE',
            'finite': None,
            'freshness_sec': (
                None if self.last_receive_sec is None else
                max(0.0, now - self.last_receive_sec)),
            'message_count': self.message_count,
            'restart_count': self.restart_count,
            'stale': True,
            'output_topic': self.output_topic,
            'reference_topic': self.reference_topic,
            'publishes_tf': False,
        })

    def _publish(self, payload: dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(
            payload, separators=(',', ':'), sort_keys=True)
        self.publisher.publish(message)


def main(args: Optional[list[str]] = None) -> None:
    """Run the experimental-estimator health monitor."""
    rclpy.init(args=args)
    node = EstimatorHealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
