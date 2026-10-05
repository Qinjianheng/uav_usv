import bisect
import json
import math
import sys
from collections import Counter

s=lambda v:v['sec']+v['nanosec']*1e-9
v=lambda p:tuple(p[k] for k in ('x','y','z'))
rows=[json.loads(x) for x in open(sys.argv[1])]
y=next(r['receipt'] for r in rows if r['topic']=='driver' and r['message']['command']=='Y')
truth=[(s(r['message']['stamp']),v(r['message']['position'])) for r in rows if r['topic']=='truth_evaluation_only']
times=[t for t,p in truth]
def actual(t):
 i=bisect.bisect_right(times,t)
 if i==0 or i==len(times):raise ValueError('outside history')
 a,b=truth[i-1],truth[i];f=(t-a[0])/(b[0]-a[0])
 return tuple(x+f*(z-x) for x,z in zip(a[1],b[1]))
plans=[r for r in rows if r['topic']=='trajectory']
for r in plans:
 m=r['message'];t=s(m['contact_stamp']);p=v(m['terminal_position'])
 pred=next(x['message'] for x in rows if x['topic']=='prediction' and x['message']['sequence_id']==m['prediction_sequence_id'])
 rel=t-s(pred['source_stamp']);pa=None
 for a,b in zip(pred['samples'],pred['samples'][1:]):
  ta,tb=s(a['relative_time']),s(b['relative_time'])
  if ta<=rel<=tb:
   f=(rel-ta)/(tb-ta);pa=tuple(x+f*(z-x) for x,z in zip(v(a['position']),v(b['position'])));break
 tr=actual(t)
 print('PLAN',m['plan_id'],'Y+',round(r['receipt']-y,4),'contactY+',round(t-y,4),'duration',m['selected_t_go'])
 print('terminal',p,'truth@contact',tr,'pred@contact',pa,'pred−truth',tuple(x-z for x,z in zip(pa,tr)),'horizontal',math.dist(pa[:2],tr[:2]),'3D',math.dist(pa,tr))
print(Counter(r['message']['rejection_stage'] for r in rows if r['topic']=='planner'))
