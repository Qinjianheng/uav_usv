import json, math, sys
import numpy as np
import pandas as pd
from pyulog import ULog
from scipy.spatial.transform import Rotation, Slerp
from uav_control.perception.rgbd_target_localizer import local_ned_target_to_camera_flu

raw, vision, ulog = sys.argv[1:4]
r=[json.loads(l) for l in open(raw)]
y=next(x['receipt'] for x in r if x['topic']=='driver' and x['message']['command']=='Y')
s=lambda x:x['sec']+x['nanosec']*1e-9
truth=[(s(x['message']['stamp']),[x['message']['position'][k] for k in ('x','y','z')]) for x in r if x['topic']=='truth_evaluation_only']
tt=np.array([x[0] for x in truth]);tp=np.array([x[1] for x in truth])
v=pd.read_csv(vision);a=v[['image_clock_anchor_system_stamp','image_clock_anchor_sim_stamp']].dropna().drop_duplicates().sort_values('image_clock_anchor_system_stamp');ar,at=a.to_numpy().T
u=ULog(ulog,message_name_filter_list=['vehicle_local_position','vehicle_attitude'])
d={x.name:x.data for x in u.data_list};pos=d['vehicle_local_position'];pt=pos['timestamp_sample']*1e-6;pp=np.column_stack([pos[k] for k in ('x','y','z')]);att=d['vehicle_attitude'];qt=att['timestamp_sample']*1e-6
q=np.column_stack([att['q[1]'],att['q[2]'],att['q[3]'],att['q[0]']]);sl=Slerp(qt,Rotation.from_quat(q))
hh=.87;vh=math.atan(.75*math.tan(hh));out=[];reject=0

def bracket(t, times, gap):
 i=np.searchsorted(times,t,side='right')
 if i==0 or i>=len(times) or times[i]-times[i-1]>gap:raise ValueError('no bounded bracket')
 return i

for row in v.itertuples():
 t=row.measurement_stamp
 if not y-5<=t<=y+6:continue
 try:
  bracket(t,ar,.2);ts=float(np.interp(t,ar,at));bracket(ts,pt,.1);bracket(ts,qt,.1);bracket(t,tt,.15)
  p=np.array([np.interp(ts,pt,pp[:,j]) for j in range(3)]);target=np.array([np.interp(t,tt,tp[:,j]) for j in range(3)]);target[2]-=.42
  qs=sl(ts).as_quat();qw=[qs[3],*qs[:3]];result={'y_elapsed':t-y,'observed_valid':bool(row.valid),'rejection_reason':row.rejection_reason}
  for deg in (12,25):
   cam=local_ned_target_to_camera_flu(target,p,qw,(.35,0,.19),math.radians(deg))
   ha=math.atan2(cam[1],cam[0]);va=math.atan2(-cam[2],cam[0]);result[str(deg)]={'horizontal_angle':ha,'vertical_depression':va,'horizontal_margin':hh-abs(ha),'vertical_margin':vh-abs(va),'inside_fov':bool(cam[0]>.2 and abs(ha)<hh and abs(va)<vh)}
  out.append(result)
 except ValueError:reject+=1
print(json.dumps({'boundary':'OFFLINE_GEOMETRY_PX4_ESTIMATED_POSE_HELD_FIXED_TRUTH_TARGET_ONLY_EVALUATION','rejected_unbracketed':reject,'frames':out},indent=2))
