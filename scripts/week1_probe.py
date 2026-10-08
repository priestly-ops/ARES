#!/usr/bin/env python3
"""Fresh-process Gazebo measurement; owns and reaps only its process groups."""
import argparse, json, os, signal, subprocess, time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--name',required=True);p.add_argument('--world',default='src/ares_simulation/worlds/ares_test_world.sdf');p.add_argument('--bridge');p.add_argument('--seconds',type=float,default=30);p.add_argument('--software',action='store_true');p.add_argument('--stage');p.add_argument('--camera',action='store_true');p.add_argument('--gz-scan',action='store_true');p.add_argument('--shutdown-trace',action='store_true');a=p.parse_args()
out=Path('results/week1/performance')/a.name;out.mkdir(parents=True,exist_ok=False)
rows=subprocess.check_output(['ps','-eo','pid,args'],text=True).splitlines()
conflicts=[r for r in rows if any(s in r for s in ('gz sim -','gz sim server','gz-sim-server','gz-sim-main','/ros_gz_bridge/parameter_bridge'))]
if conflicts:raise RuntimeError('Contaminated process baseline: '+str(conflicts))
env=os.environ.copy();env['GZ_PARTITION']='ares_probe_'+str(os.getpid());env['ROS_DOMAIN_ID']='81';env['ROS_LOG_DIR']=str(out.resolve()/'ros_logs')
if a.software:env['LIBGL_ALWAYS_SOFTWARE']='1'
if a.shutdown_trace:env['LD_PRELOAD']='/tmp/ares_trace.so'
procs=[];handles=[]
def start(args,name):
 f=open(out/name,'w');handles.append(f);q=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,env=env,start_new_session=True);procs.append(q);return q
try:
 start(['ros2','launch','ares_simulation','ares_baseline.launch.py','stage:='+a.stage,'camera:='+str(a.camera).lower()], 'gazebo.log') if a.stage else start(['gz','sim','-s','-r','--seed','42',a.world],'gazebo.log')
 time.sleep(12)
 if a.bridge:start(['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args','-p','config_file:='+str(Path(a.bridge).resolve())],'bridge.log')
 if a.gz_scan:start(['gz','topic','-e','-t','/ares/scan','-d',str(a.seconds+8)],'scan.log')
 time.sleep(5)
 (out/'cpu.txt').write_text(subprocess.check_output(['ps','-eo','pid,ppid,pcpu,pmem,nlwp,etimes,comm','--sort=-pcpu'],text=True))
 t=time.monotonic();r=subprocess.run(['gz','topic','-e','-t','/world/ares_world/stats','--json-output','-d',str(a.seconds)],capture_output=True,text=True,env=env,timeout=a.seconds+10)
 (out/'stats.jsonl').write_text(r.stdout);(out/'stats.stderr').write_text(r.stderr)
 samples=[]
 for line in r.stdout.splitlines():
  try:samples.append(json.loads(line))
  except ValueError:pass
 def sec(v):return float(v.get('sec',0))+float(v.get('nsec',0))*1e-9
 result={'name':a.name,'samples':len(samples),'elapsed_monotonic':time.monotonic()-t,'process_exit_codes':[q.poll() for q in procs]}
 if len(samples)>1:
  first,last=samples[0],samples[-1]
  ds=sec(last['simTime'])-sec(first['simTime']);dr=sec(last['realTime'])-sec(first['realTime']);result.update(sim_delta=ds,real_delta=dr,effective_rtf=ds/dr)
 (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
finally:
 for q in reversed(procs):
  try:
   if a.stage:os.kill(q.pid,signal.SIGINT)
   else:os.killpg(q.pid,signal.SIGINT)
  except ProcessLookupError:pass
 for q in procs:
  try:q.wait(timeout=10)
  except subprocess.TimeoutExpired:
   os.killpg(q.pid,signal.SIGKILL);q.wait()
 for f in handles:f.close()
 if (out/'result.json').exists():
  result=json.loads((out/'result.json').read_text());result['shutdown_exit_codes']=[q.returncode for q in procs];(out/'result.json').write_text(json.dumps(result,indent=2))
