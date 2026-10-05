import json,sys
import numpy as np,pandas as pd
from pyulog import ULog
from rosidl_runtime_py.set_message import set_message_fields
from uav_usv_interfaces.msg import InterceptTrajectory
from uav_control.control.trajectory_tracker_node import trajectory_from_message
raw,vis,ulg=sys.argv[1:];r=[json.loads(l) for l in open(raw)];y=next(x['receipt'] for x in r if x['topic']=='driver' and x['message']['command']=='Y');v=pd.read_csv(vis);a=v[['image_clock_anchor_system_stamp','image_clock_anchor_sim_stamp']].dropna().drop_duplicates().sort_values('image_clock_anchor_system_stamp');ar,at=a.to_numpy().T
u=ULog(ulg,message_name_filter_list=['vehicle_local_position','trajectory_setpoint']);d={x.name:x.data for x in u.data_list};p=d['vehicle_local_position'];pt=p['timestamp_sample']*1e-6;pp=np.column_stack([p[k] for k in ('x','y','z')]);pv=np.column_stack([p[k] for k in ('vx','vy','vz')]);refs=d['trajectory_setpoint'];rt=refs['timestamp']*1e-6
plans={}
for x in r:
 if x['topic']=='trajectory':
  m=InterceptTrajectory();set_message_fields(m,x['message']);plans[m.plan_id]=trajectory_from_message(m)
out=[]
for x in r:
 m=x['message']
 if x['topic']!='controller' or m['status']!='PLAN_REJECTED' or not 0<x['receipt']-y<10:continue
 t=m['stamp']['sec']+m['stamp']['nanosec']*1e-9
 if not ar[0]<t<ar[-1]:continue
 ts=float(np.interp(t,ar,at));i=np.searchsorted(pt,ts)
 if not 0<i<len(pt) or pt[i]-pt[i-1]>.1:continue
 trajectory=plans[m['attempted_plan_id']];state=trajectory.sample_at_ros_time(t)
 pos=np.array([np.interp(ts,pt,pp[:,j]) for j in range(3)]);vel=np.array([np.interp(ts,pt,pv[:,j]) for j in range(3)])
 out.append({'y_elapsed':t-y,'attempt':m['attempted_plan_id'],'active':m['plan_id'],'reason':m['rejection_reason'],'native_position_minus_candidate':(pos-state.position).tolist(),'native_velocity_minus_candidate':(vel-state.velocity).tolist(),'native_position_error':float(np.linalg.norm(pos-state.position)),'native_velocity_error':float(np.linalg.norm(vel-state.velocity))})
print(json.dumps({'boundary':'NATIVE_PX4_ESTIMATE_AT_REJECT_TIME_NOT_TRUTH','rows':out},indent=2))
