#!/usr/bin/env python3
"""Fresh simulator and estimator per trial; never reset a running EKF in place."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import resource
import signal
import subprocess
import time
import yaml

p=argparse.ArgumentParser();p.add_argument('--runs',type=int,default=20);p.add_argument('--output',default='results/week1/healthy');p.add_argument('--health-only',action='store_true');p.add_argument('--config');p.add_argument('--software',action='store_true');p.add_argument('--core-dir');p.add_argument('--bridge-keep-rmw-loaded',action='store_true');a=p.parse_args()
if a.runs<1: p.error('--runs must be positive')
root=Path(__file__).resolve().parents[1];out=(root/a.output).resolve();out.mkdir(parents=True,exist_ok=False)
core_dir=Path(a.core_dir).resolve() if a.core_dir else None
if core_dir:
    core_dir.mkdir(parents=True,exist_ok=True)
    resource.setrlimit(resource.RLIMIT_CORE,(resource.RLIM_INFINITY,resource.RLIM_INFINITY))

def stop(proc):
    if proc is None:return
    try:
        if "launch" in proc.args:os.kill(proc.pid,signal.SIGINT)
        else:os.killpg(proc.pid,signal.SIGINT)
    except ProcessLookupError:return
    try:proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        try:os.killpg(proc.pid,signal.SIGTERM)
        except ProcessLookupError:pass
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    # A launch wrapper may exit before descendants; signal only its owned group.
    try:os.killpg(proc.pid,signal.SIGTERM)
    except ProcessLookupError:pass

def check_clean():
    # Refuse contaminated host experiments; do not kill unrelated processes.
    rows=subprocess.check_output(['ps','-eo','pid,args'],text=True).splitlines()
    conflicts=[r for r in rows if any(s in r for s in ('gz sim -','gz sim server','gz-sim-server','gz-sim-main','/ros_gz_bridge/parameter_bridge'))]
    if conflicts:raise RuntimeError('Existing simulator/bridge processes: '+str(conflicts))

def descendants(parent_pid):
    """Return live descendants without matching unrelated command lines."""
    parents={}
    for item in Path('/proc').iterdir():
        if not item.name.isdigit():continue
        try:
            fields=(item/'stat').read_text().split()
            parents[int(item.name)]=int(fields[3])
        except (OSError,PermissionError,ProcessLookupError,ValueError,IndexError):
            continue
    found=set();frontier={parent_pid}
    while frontier:
        children={pid for pid,ppid in parents.items() if ppid in frontier and pid not in found}
        found.update(children);frontier=children
    return found

def stop_owned_child(parent_pid,token,timeout=15):
    targets=[]
    for pid in sorted(descendants(parent_pid)):
        try:cmdline=Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
        except (OSError,PermissionError,ProcessLookupError):continue
        if token in cmdline:targets.append({'pid':pid,'cmdline':cmdline})
    for target in targets:
        try:os.kill(target['pid'],signal.SIGINT)
        except ProcessLookupError:pass
    deadline=time.monotonic()+timeout
    remaining={target['pid'] for target in targets}
    while remaining and time.monotonic()<deadline:
        remaining={pid for pid in remaining if Path(f'/proc/{pid}').exists()}
        if remaining:time.sleep(.1)
    return {'token':token,'targets':targets,'remaining_after_timeout':sorted(remaining),
            'clean':bool(targets) and not remaining}

summary=[]
for index in range(a.runs):
    check_clean()
    run=out/('run_'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(index+1).zfill(3));run.mkdir()
    domain_id=str(100+(os.getpid()+index)%100)
    env=os.environ.copy();env.update(ROS_DOMAIN_ID=domain_id,GZ_PARTITION='ares_trial_'+str(os.getpid())+'_'+str(index),ROS_LOG_DIR=str(run/'ros_logs'))
    if a.software:env['LIBGL_ALWAYS_SOFTWARE']='1'
    files=[f for name in ('ares_simulation','ares_localization','ares_jackal_description','ares_benchmark') for f in (root/'src'/name).rglob('*') if f.is_file() and '__pycache__' not in str(f)]
    (run/'manifest.json').write_text(json.dumps({str(f.relative_to(root)):hashlib.sha256(f.read_bytes()).hexdigest() for f in files},indent=2))
    launch=bag=mission=None
    exit_codes={'launch_before_shutdown':None, 'bag_before_shutdown':None,
                'mission':None, 'bag_shutdown':None, 'launch_shutdown':None,
                'lifecycle_shutdown':None, 'bridge_ordered_shutdown':None}
    logs=[]
    def start(command,name,cwd=None):
        f=open(run/name,'w');logs.append(f)
        return subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT,env=env,start_new_session=True,cwd=cwd or root)
    try:
        launch_command=['ros2','launch','ares_simulation','ares_baseline.launch.py','rviz:=false','camera:=false']
        if a.bridge_keep_rmw_loaded:launch_command.append('bridge_keep_rmw_loaded:=true')
        launch=start(launch_command,'launch.log',core_dir)
        time.sleep(2)
        topics=['/ares/odom','/ares/imu','/ares/scan','/odometry/filtered','/amcl_pose','/tf','/tf_static','/cmd_vel','/ares/cmd_vel','/plan','/clock','/navigate_to_pose/_action/status','/navigate_to_pose/_action/feedback','/diagnostics']
        bag=start(['ros2','bag','record','--use-sim-time','--include-hidden-topics','-o',str(run/'bag'),'--topics',*topics],'bag.log')
        command=['ros2','run','ares_benchmark','health_check' if a.health_only else 'healthy_mission','--output',str(run/'metrics.json')]
        if a.config:command+=['--config',str(Path(a.config).resolve())]
        command+=['--ros-args','-p','use_sim_time:=true']
        mission=start(command,'mission.log');deadline=time.monotonic()+1200
        while mission.poll() is None and launch.poll() is None and bag.poll() is None and time.monotonic()<deadline:time.sleep(.5)
        interrupted=mission.poll() is None
        if interrupted:stop(mission)
        exit_codes['mission']=mission.returncode
        exit_codes['launch_before_shutdown']=launch.poll()
        exit_codes['bag_before_shutdown']=bag.poll()
        result=json.loads((run/'metrics.json').read_text()) if (run/'metrics.json').exists() else {'success':False,'reason':'No metrics: inspect mission/launch/bag logs'}
        result['no_unexpected_process_exit']=launch.poll() is None and bag.poll() is None and not interrupted
        stop(bag);exit_codes['bag_shutdown']=bag.returncode;bag=None
        info=subprocess.run(['ros2','bag','info',str(run/'bag')],env=env,capture_output=True,text=True,timeout=20)
        (run/'bag_info.txt').write_text(info.stdout+info.stderr)
        result['bag_valid']=info.returncode==0 and (run/'bag/metadata.yaml').exists()
        if result['bag_valid']:
            metadata=yaml.safe_load((run/'bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
            counts={t['topic_metadata']['name']:t['message_count'] for t in metadata['topics_with_message_count']}
            required=['/ares/odom','/ares/imu','/ares/scan','/odometry/filtered','/amcl_pose','/tf','/tf_static','/clock']
            if not a.health_only:required+=['/cmd_vel','/plan']
            result['bag_topic_counts']=counts
            result['bag_valid']=all(counts.get(t,0)>0 for t in required)
        result['success']=result['success'] and result['no_unexpected_process_exit'] and result['bag_valid']
        result['bag_path']=str(run/'bag');result['log_path']=str(run/'launch.log');result['ros_domain_id']=domain_id
        (run/'metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        summary.append({key:result.get(key) for key in ('success','duration_wall_s','duration_sim_s','final_goal_error_m','effective_rtf','bag_path','log_path')})
    finally:
        stop(mission);stop(bag)
        if launch is not None and launch.poll() is None:
            try:
                shutdown=subprocess.run(['ros2','service','call','/lifecycle_manager_baseline/manage_nodes','nav2_msgs/srv/ManageLifecycleNodes','{command: 4}'],env=env,capture_output=True,text=True,timeout=15)
                (run/'lifecycle_shutdown.log').write_text(shutdown.stdout+shutdown.stderr)
                exit_codes['lifecycle_shutdown']=shutdown.returncode
            except subprocess.TimeoutExpired:
                (run/'lifecycle_shutdown.log').write_text('Lifecycle shutdown timed out; launch cleanup follows.\n')
        if launch is not None and launch.poll() is None:
            # Allow DDS endpoint removals from the mission, bag recorder, and
            # managed Nav2 nodes to settle before destroying the bridge's RMW
            # participant. The bridge core showed a DDS worker unload race.
            consumer_settle_s=5.0
            time.sleep(consumer_settle_s)
            bridge_shutdown=stop_owned_child(
                launch.pid,'/ros_gz_bridge/parameter_bridge')
            bridge_shutdown['consumer_settle_s']=consumer_settle_s
            (run/'bridge_shutdown.json').write_text(
                json.dumps(bridge_shutdown,indent=2)+'\n')
            exit_codes['bridge_ordered_shutdown']=0 if bridge_shutdown['clean'] else 1
        stop(launch)
        if launch is not None:exit_codes['launch_shutdown']=launch.returncode
        for f in logs:f.close()
        if (run/'metrics.json').exists():
            recorded=json.loads((run/'metrics.json').read_text())
            recorded['process_exit_codes']=exit_codes
            launch_text=(run/'launch.log').read_text(errors='replace')
            recorded['launch_child_exit_codes']=[
                {'process':match.group('process'), 'exit_code':int(match.group('code'))}
                for match in re.finditer(
                    r"\[ERROR\] \[(?P<process>[^]]+)\]: process has died \[pid \d+, exit code (?P<code>-?\d+),",
                    launch_text)
            ]
            recorded['launch_children_finished_cleanly']=[
                match.group('process')
                for match in re.finditer(
                    r"\[INFO\] \[(?P<process>[^]]+)\]: process has finished cleanly \[pid \d+\]",
                    launch_text)
            ]
            recorded['parameter_bridge_exit']=next((
                child['exit_code'] for child in recorded['launch_child_exit_codes']
                if child['process'].startswith('parameter_bridge-')
            ), 0 if any(name.startswith('parameter_bridge-') for name in recorded['launch_children_finished_cleanly']) else None)
            recorded['gazebo_exit']=next((
                child['exit_code'] for child in recorded['launch_child_exit_codes']
                if child['process'].startswith('gz-')
            ), 0 if any(name.startswith('gz-') for name in recorded['launch_children_finished_cleanly']) else None)
            recorded['bridge_keep_rmw_loaded']=a.bridge_keep_rmw_loaded
            recorded['clean_child_shutdown']=not recorded['launch_child_exit_codes']
            if not recorded['clean_child_shutdown']:
                recorded['success']=False
                recorded['reason']=recorded.get('reason') or 'Child process failed during shutdown'
            (run/'metrics.json').write_text(json.dumps(recorded,indent=2,allow_nan=False)+'\n')
            if summary:
                summary[-1].update({key:recorded.get(key) for key in summary[-1]})
    if (run/'metrics.json').exists():
        finalized=json.loads((run/'metrics.json').read_text())
        print(json.dumps({'run':str(run),'success':finalized['success'],'reason':finalized.get('reason'),'rtf':finalized.get('effective_rtf')}),flush=True)
    time.sleep(2);check_clean()
    with (out/'baseline_summary.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=summary[0].keys());writer.writeheader()
        for metrics in sorted(out.glob('run_*/metrics.json')):
            row=json.loads(metrics.read_text());writer.writerow({k:row.get(k) for k in summary[0]})
raise SystemExit(0 if all(r['success'] for r in summary) else 1)
