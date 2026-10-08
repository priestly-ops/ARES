#!/usr/bin/env python3
"""Bounded navigation benchmark with explicit acceptance and raw measurements."""
import argparse
import json
import math
import statistics
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SetInitialPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage


def stamp(t): return t.sec + t.nanosec * 1e-9

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def angle(x): return math.atan2(math.sin(x), math.cos(x))


class Monitor(Node):
    def __init__(self, config):
        super().__init__('ares_healthy_benchmark')
        self.c = config
        self.data = {}
        self.clock_samples = []
        self.commands = []
        self.amcl = None
        self.amcl_covariance = []
        self.path_length = 0.0
        self.last_xy = None
        self.max_plan_length = 0.0
        self.ekf_amcl_deltas = []
        self.tf_parents = {}
        self.tf_authorities = {}
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.subs = [self.create_subscription(TFMessage, '/tf', self.tf_cb, 100)]
        for topic, spec in self.c['sensors'].items():
            cls = {'scan': LaserScan, 'imu': Imu, 'odom': Odometry}[spec['type']]
            self.subs.append(self.create_subscription(cls, topic, lambda m, t=topic: self.sensor(t, m), qos_profile_sensor_data))
        self.subs += [self.create_subscription(Clock, '/clock', self.clock_cb, qos_profile_sensor_data),
                      self.create_subscription(TwistStamped, '/cmd_vel', self.command_cb, 10),
                      self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10),
                      self.create_subscription(NavPath, '/plan', self.plan_cb, 10)]
        self.lifecycle_clients = {name: self.create_client(GetState, '/' + name + '/get_state') for name in self.c['lifecycle_nodes']}
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def tf_cb(self, msg, info):
        for t in msg.transforms:
            self.tf_parents.setdefault(t.child_frame_id, set()).add(t.header.frame_id)
            self.tf_authorities.setdefault(t.child_frame_id, set()).add(bytes(info['publisher_gid']['data']).hex() if info['publisher_gid'] else 'unavailable')

    def clock_cb(self, msg): self.clock_samples.append((time.monotonic(), stamp(msg.clock)))
    def command_cb(self, msg): self.commands.append((abs(msg.twist.linear.x), abs(msg.twist.angular.z)))
    def amcl_cb(self, msg):
        self.amcl = msg
        self.amcl_covariance.append([msg.pose.covariance[i] for i in (0, 7, 35)])
        try:
            t = self.tf.lookup_transform(self.c['map_frame'], self.c['base_frame'], Time())
            self.ekf_amcl_deltas.append(math.hypot(t.transform.translation.x-msg.pose.pose.position.x, t.transform.translation.y-msg.pose.pose.position.y))
        except Exception: pass
    def plan_cb(self, msg):
        points = [p.pose.position for p in msg.poses]
        self.max_plan_length = max(self.max_plan_length, sum(math.hypot(a.x-b.x, a.y-b.y) for a, b in zip(points, points[1:])))

    def sensor(self, topic, msg):
        t = stamp(msg.header.stamp)
        d = self.data.setdefault(topic, {'count': 0, 'first': t, 'last': t, 'nonadvancing': 0, 'invalid': 0, 'max_gap_sim': 0.0})
        if d['count'] and t <= d['last']: d['nonadvancing'] += 1
        d['max_gap_sim'] = max(d['max_gap_sim'], t-d['last'])
        d.update(count=d['count']+1, last=t, frame=msg.header.frame_id, received_wall=time.monotonic())
        spec = self.c['sensors'][topic]
        valid = True
        if spec['type'] == 'scan':
            finite = [v for v in msg.ranges if math.isfinite(v)]
            valid = (len(msg.ranges) == 360 and abs(msg.angle_max-msg.angle_min-2*math.pi) < .02
                     and abs(msg.range_min-.12)<.01 and abs(msg.range_max-20)<.01
                     and bool(finite) and all(not math.isnan(v) and v != -math.inf for v in msg.ranges)
                     and all(msg.range_min <= v <= msg.range_max for v in finite))
            d.update(samples=len(msg.ranges), angle_min=msg.angle_min, angle_max=msg.angle_max,
                     range_min=msg.range_min, range_max=msg.range_max, finite_returns=len(finite))
        else:
            pose = msg.pose.pose if spec['type'] == 'odom' else None
            q = pose.orientation if pose else msg.orientation
            vectors = [pose.position, msg.twist.twist.linear, msg.twist.twist.angular] if pose else [msg.angular_velocity, msg.linear_acceleration]
            values = [v for vec in vectors for v in (vec.x, vec.y, vec.z)] + [q.x,q.y,q.z,q.w]
            valid = all(math.isfinite(v) for v in values) and abs(sum(v*v for v in (q.x,q.y,q.z,q.w))-1)<.05
            if pose:
                cov = list(msg.pose.covariance) + list(msg.twist.covariance)
                valid &= all(math.isfinite(v) for v in cov) and all(msg.pose.covariance[i]>=0 for i in (0,7,14,21,28,35))
                d.update(child=msg.child_frame_id, pose_covariance_diagonal=[msg.pose.covariance[i] for i in (0,7,35)],
                         x=pose.position.x, y=pose.position.y, vx=msg.twist.twist.linear.x, wz=msg.twist.twist.angular.z)
                if topic == '/odometry/filtered':
                    xy = (pose.position.x, pose.position.y)
                    if self.last_xy: self.path_length += math.dist(xy, self.last_xy)
                    self.last_xy = xy
            else:
                d.update(orientation=[q.x,q.y,q.z,q.w], angular_velocity=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z],
                         linear_acceleration=[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
        if not valid: d['invalid'] += 1

    def spin_for(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end: rclpy.spin_once(self, timeout_sec=.1)

    def wait(self, future, timeout):
        end = time.monotonic()+timeout
        while not future.done() and time.monotonic()<end: rclpy.spin_once(self, timeout_sec=.1)
        if not future.done(): raise TimeoutError('ROS request timed out')
        return future.result()

    def initialize(self):
        client = self.create_client(SetInitialPose, '/set_initial_pose')
        if not client.wait_for_service(timeout_sec=5): raise RuntimeError('AMCL initial-pose service unavailable')
        req = SetInitialPose.Request(); req.pose.header.frame_id = self.c['map_frame']
        req.pose.header.stamp = self.get_clock().now().to_msg()
        x,y,a = self.c['initial_pose']; req.pose.pose.pose.position.x=x;req.pose.pose.pose.position.y=y
        req.pose.pose.pose.orientation.z=math.sin(a/2);req.pose.pose.pose.orientation.w=math.cos(a/2)
        req.pose.pose.covariance[0]=.05;req.pose.pose.covariance[7]=.05;req.pose.pose.covariance[35]=.02
        self.wait(client.call_async(req), 10)

    def lifecycle(self):
        states = {}
        for name, client in self.lifecycle_clients.items():
            if not client.service_is_ready(): states[name] = False; continue
            try: states[name] = self.wait(client.call_async(GetState.Request()), 3).current_state.id == 3
            except Exception: states[name] = False
        return states

    def checks(self):
        checks = {}
        for topic, spec in self.c['sensors'].items():
            d = self.data.get(topic, {})
            duration = d.get('last',0)-d.get('first',0)
            rate = (d.get('count',0)-1)/duration if duration>0 else 0
            d['rate_sim_hz'] = rate
            checks[topic] = (d.get('count',0)>5 and self.count_publishers(topic)>0 and d.get('frame')==spec['frame']
                             and ('child' not in spec or d.get('child')==spec['child']) and not d.get('invalid',1)
                             and not d.get('nonadvancing',1) and time.monotonic()-d.get('received_wall',0)<3
                             and abs(rate/spec['rate']-1)<=self.c['rate_tolerance_fraction'])
        for frame in ['odom', self.c['base_frame'], 'chassis_link', 'imu_link', 'lidar_link', 'front_left_wheel_link', 'front_right_wheel_link', 'rear_left_wheel_link', 'rear_right_wheel_link']:
            checks['TF map -> '+frame] = self.tf.can_transform(self.c['map_frame'], frame, Time())
        checks.update({'active '+k:v for k,v in self.lifecycle().items()})
        checks['localization initialized'] = self.amcl is not None and all(math.isfinite(v) and v>=0 for v in self.amcl_covariance[-1])
        checks['single dynamic TF authority'] = all(len(v)==1 for v in self.tf_authorities.values()) and all(len(v)==1 for v in self.tf_parents.values())
        checks['clock advancing'] = len(self.clock_samples)>1 and self.clock_samples[-1][1]>self.clock_samples[0][1]
        checks['effective RTF'] = self.rtf() is not None and self.rtf()>=self.c['min_rtf']
        return checks

    def rtf(self):
        if len(self.clock_samples)<2: return None
        a,b = self.clock_samples[0],self.clock_samples[-1]
        return (b[1]-a[1])/(b[0]-a[0]) if b[0]>a[0] else None

    def run(self, health_only):
        deadline = time.monotonic()+self.c['readiness_timeout_wall']
        ready = False
        while time.monotonic()<deadline:
            self.spin_for(1)
            if len(self.data)==len(self.c['sensors']) and self.action.server_is_ready() and all(self.lifecycle().values()):
                ready=True;break
        if not ready: raise RuntimeError('Required sensors/action/lifecycle readiness timeout')
        if not health_only: self.initialize()
        self.spin_for(self.c['sample_seconds_wall'])
        pre = self.checks()
        for key,value in pre.items(): print(('PASS ' if value else 'FAIL ')+key, flush=True)
        if not all(pre.values()): return {'success':False, 'reason':'Preflight failed', 'checks':pre}
        if health_only: return {'success':True, 'checks':pre}
        start_wall=time.monotonic(); start_sim=self.get_clock().now().nanoseconds/1e9
        waypoints=[]
        for x,y,a in self.c['waypoints']:
            goal=NavigateToPose.Goal();goal.pose.header.frame_id=self.c['map_frame'];goal.pose.header.stamp=self.get_clock().now().to_msg()
            goal.pose.pose.position.x=x;goal.pose.pose.position.y=y
            goal.pose.pose.orientation.z=math.sin(a/2);goal.pose.pose.orientation.w=math.cos(a/2)
            handle=self.wait(self.action.send_goal_async(goal),10)
            if not handle.accepted: raise RuntimeError('Goal rejected')
            try: result=self.wait(handle.get_result_async(), self.c['goal_timeout_wall'])
            except TimeoutError:
                self.wait(handle.cancel_goal_async(),10);raise
            t=self.tf.lookup_transform(self.c['map_frame'],self.c['base_frame'],Time()).transform
            error=math.hypot(t.translation.x-x,t.translation.y-y); yaw_error=abs(angle(yaw(t.rotation)-a))
            ok=result.status==4 and error<=self.c['max_goal_error_m'] and yaw_error<=self.c['max_goal_yaw_error_rad']
            waypoints.append({'goal':[x,y,a],'action_status':result.status,'error_m':error,'yaw_error_rad':yaw_error,'success':ok})
            print(json.dumps(waypoints[-1]),flush=True)
            if not ok: break
        post=self.checks()
        return {'success':len(waypoints)==len(self.c['waypoints']) and all(w['success'] for w in waypoints) and all(post.values()),
                'duration_wall_s':time.monotonic()-start_wall,'duration_sim_s':self.get_clock().now().nanoseconds/1e9-start_sim,
                'waypoints':waypoints,'checks':post,'final_goal_error_m':waypoints[-1]['error_m'],
                'final_goal_yaw_error_rad':waypoints[-1]['yaw_error_rad']}

    def metrics(self):
        return {'sensors':self.data,'effective_rtf':self.rtf(),'filtered_path_length_m':self.path_length,
                'max_planned_path_length_m':self.max_plan_length,
                'mean_abs_linear_command_mps':statistics.mean(v[0] for v in self.commands) if self.commands else None,
                'max_abs_linear_command_mps':max((v[0] for v in self.commands),default=None),
                'mean_abs_angular_command_radps':statistics.mean(v[1] for v in self.commands) if self.commands else None,
                'max_abs_angular_command_radps':max((v[1] for v in self.commands),default=None),
                'amcl_covariance_xyz_yaw_samples':self.amcl_covariance,
                'amcl_tf_discrepancy_mean_m':statistics.mean(self.ekf_amcl_deltas) if self.ekf_amcl_deltas else None,
                'dynamic_tf_parents':{k:sorted(v) for k,v in self.tf_parents.items()},
                'dynamic_tf_authorities':{k:sorted(v) for k,v in self.tf_authorities.items()},
                'localization_metric_note':'AMCL covariance and asynchronous AMCL-to-TF consistency, not ground-truth accuracy.'}


def main(health_only=False):
    parser=argparse.ArgumentParser();parser.add_argument('--config',default=str(Path(get_package_share_directory('ares_benchmark'))/'config/mission.yaml'))
    parser.add_argument('--output',required=True);args,ros_args=parser.parse_known_args()
    output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
    if output.exists(): raise FileExistsError(output)
    rclpy.init(args=ros_args);node=Monitor(yaml.safe_load(Path(args.config).read_text()))
    result={'success':False}
    try: result=node.run(health_only)
    except Exception as exc: result.update(reason=str(exc))
    finally:
        result.update(node.metrics());result['configuration']=node.c
        output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        print(('PASS' if result['success'] else 'FAIL')+' result: '+str(output),flush=True)
        node.destroy_node();rclpy.shutdown()
    raise SystemExit(0 if result['success'] else 1)


if __name__=='__main__': main()
