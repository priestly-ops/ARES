#!/usr/bin/env python3
import argparse,json,os,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--domain',default='81');a=p.parse_args();os.environ['ROS_DOMAIN_ID']=a.domain;os.environ['ROS_LOG_DIR']='/tmp/ares_snapshot_logs'
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Imu,LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time
rclpy.init();n=Node('week1_snapshot');buf=Buffer();listener=TransformListener(buf,n);messages={};subs=[]
for topic,cls in [('/ares/scan',LaserScan),('/ares/imu',Imu),('/ares/odom',Odometry),('/odometry/filtered',Odometry)]:
 subs.append(n.create_subscription(cls,topic,lambda m,t=topic:messages.update({t:message_to_ordereddict(m)}),qos_profile_sensor_data))
end=time.monotonic()+10
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
result={'messages':messages,'nodes':n.get_node_names(),'endpoints':{t:[{'name':e.node_name,'type':e.topic_type} for e in n.get_publishers_info_by_topic(t)] for t in ['/ares/scan','/ares/imu','/ares/odom','/odometry/filtered','/tf']}}
result['transforms']={}
for parent,child in [('map','odom'),('odom','base_footprint'),('base_footprint','imu_link'),('base_footprint','lidar_link')]:
 try:result['transforms'][parent+' -> '+child]=message_to_ordereddict(buf.lookup_transform(parent,child,Time()))
 except Exception as exc:result['transforms'][parent+' -> '+child]=str(exc)
r=subprocess.run(['timeout','3','ros2','run','tf2_ros','tf2_echo','map','base_footprint','--ros-args','-p','use_sim_time:=true'],capture_output=True,text=True)
Path(a.output).with_suffix('.tf.txt').write_text(r.stdout+r.stderr)
Path(a.output).write_text(json.dumps(result,indent=2));print({t:m['header'] for t,m in messages.items()});n.destroy_node();rclpy.shutdown()
