import json,sys,numpy as np
from pyulog import ULog
r=[json.loads(l) for l in open(sys.argv[1])];d=ULog(sys.argv[2],message_name_filter_list=['vehicle_local_position']).data_list[0].data;stamp=d['timestamp_sample'];p=np.column_stack([d[k] for k in ('x','y','z')]);err=[];dp=[]
for x in r:
 if x['topic']!='navigation' or not x['message']['valid']:continue
 m=x['message'];s=m['native_timestamp_sample'];i=np.searchsorted(stamp,s);c=[j for j in (i-1,i) if 0<=j<len(stamp)];j=min(c,key=lambda j:abs(int(stamp[j])-s));ep=np.linalg.norm(p[j]-np.array([m['position'][k] for k in 'xyz']));err.append((int(stamp[j])-s)*1e-6);dp.append(ep)
print(json.dumps({'count':len(err),'native_time_nearest_error_seconds_percentile':np.percentile(err,[0,50,95,100]).tolist(),'native_position_nearest_error_m_percentile':np.percentile(dp,[0,50,95,100]).tolist()},indent=2))
