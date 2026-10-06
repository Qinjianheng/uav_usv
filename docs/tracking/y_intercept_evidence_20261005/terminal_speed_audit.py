import json,math,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,'/home/qin/data/uav_usv/src/uav_control')
from uav_control.control.trajectory_tracking import PolynomialSegmentData
root=Path('/home/qin/data/uav_usv');name=sys.argv[1]
r=[json.loads(x) for x in (root/f'data/experiments/current/y_intercept_20261006_{name}.jsonl').open()]
def sec(s):return s['sec']+s['nanosec']*1e-9
def vec(x):return np.array([x[a] for a in 'xyz'])
y=next(x['receipt'] for x in r if x['topic']=='driver' and x['message']['command']=='Y');results=[x for x in r if x['topic']=='result'];end=sec(results[-1]['message']['stamp']) if results else r[-1]['receipt'];plans={};active=None;nav=None;rows=[];phases=[]
for x in r:
 m=x['message'];topic=x['topic']
 if topic=='mission' and (not phases or phases[-1]['state']!=m['state_name']):phases.append({'state':m['state_name'],'t':x['receipt']-y})
 if topic=='trajectory':plans[m['plan_id']]=m
 if topic=='navigation':nav=m
 if topic=='controller' and m['status']=='PLAN_ACCEPTED' and m['trajectory_replaced']:active=plans.get(m['plan_id'])
 if topic=='controller' and m['status']=='TRACKING' and active and nav and y<=x['receipt']<=end:
  elapsed=x['receipt']-sec(active['source_stamp']);desired=None
  for seg in active['segments']:
   dur=sec(seg['duration']);desired=PolynomialSegmentData(dur,tuple(seg['coefficients'])).sample(elapsed)
   if elapsed<=dur:break
   elapsed-=dur
  cmd=vec(m['command_velocity']);v=vec(nav['velocity'])
  rows.append({'t':x['receipt']-y,'to_result':x['receipt']-end,'navigation_sample_t':sec(nav['stamp'])-y,'remaining':m['remaining_t_go'],'command_speed':float(np.linalg.norm(cmd[:2])),'actual_speed':float(np.linalg.norm(v[:2])),'minco_speed':math.hypot(*desired.velocity[:2]),'target_distance_estimate':m['target_distance'],'plan':active['plan_id']})
last=[x for x in rows if -.7<=x['to_result']<=0];stats={}
for k in ['command_speed','actual_speed','minco_speed']:
 v=np.array([x[k] for x in last]);stats[k]={'count':len(v),'first':float(v[0]),'last':float(v[-1]),'maximum':float(v.max()),'minimum':float(v.min()),'peak_minus_contact':float(v.max()-v[-1])} if len(v) else {}
report={'role':'ORIGINAL_SAMPLE_EPOCH_SPEED_AUDIT','results':[x['message'] for x in results],'phases':phases,'last_0p7_seconds':stats,'samples':rows};(root/f'docs/tracking/y_intercept_evidence_20261005/{name}_speed_audit.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2))
if rows:
 import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
 fig,ax=plt.subplots(figsize=(9,4));a=[x['to_result'] for x in rows];ax.plot(a,[x['minco_speed'] for x in rows],label='MINCO reference');ax.plot(a,[x['command_speed'] for x in rows],label='Command');ax.plot([x['navigation_sample_t']-(end-y) for x in rows],[x['actual_speed'] for x in rows],label='PX4 physical sample');ax.axhline(6.5,color='gray',ls=':',label='Command limit 6.5 m/s');ax.set(xlim=(-2.,0),xlabel='Seconds before result',ylabel='Horizontal speed (m/s)');ax.grid(alpha=.25);ax.legend();fig.tight_layout();fig.savefig(root/f'docs/tracking/y_intercept_evidence_20261005/{name}_speed_audit.png',dpi=160)
