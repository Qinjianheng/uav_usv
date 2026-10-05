import bisect
import json
import math
import sys
from collections import Counter
from uav_control.tracking.target_prediction import PredictionEngine, TargetKinematicState

def stamp(v):
    return v['sec'] + v['nanosec'] * 1e-9

def vec(v):
    return tuple(v[k] for k in ['x','y','z'])

def interpolate(samples, relative):
    for a, b in zip(samples, samples[1:]):
        ta,tb=stamp(a['relative_time']),stamp(b['relative_time'])
        if ta<=relative<=tb:
            f=(relative-ta)/(tb-ta)
            return tuple(x+f*(y-x) for x,y in zip(vec(a['position']),vec(b['position'])))
    raise ValueError(relative)

rows=[json.loads(x) for x in open(sys.argv[1])]
y=next(r['receipt'] for r in rows if r['topic']=='driver' and r['message']['command']=='Y')
kf=sorted((r for r in rows if r['topic']=='kf' and r['message']['valid']),key=lambda r:stamp(r['message']['stamp']))
engine=PredictionEngine()
index=0
new={}
old={}
for r in rows:
    if r['topic']!='prediction' or not r['message']['valid']:
        continue
    m=r['message']; epoch=stamp(m['source_stamp'])
    while index<len(kf) and stamp(kf[index]['message']['stamp']) <= epoch+1e-7:
        k=kf[index]['message'];index+=1
        state=TargetKinematicState(stamp=stamp(k['stamp']), observation_stamp=stamp(k['source_stamp']), position=vec(k['position']),velocity=vec(k['velocity']))
        engine.update(state, now=state.stamp)
    if engine.latest_state is None or abs(engine.latest_state.stamp-epoch)>1e-6:
        continue
    pred=engine.generate(stamp(m['generated_stamp']),m['mission_id'],m['sequence_id'])
    new[m['sequence_id']]=pred
    old[m['sequence_id']]=m

for seq in [1239,1240,1244,1245,1248,1249]:
    if seq not in new:continue
    a,b=old[seq],new[seq]
    print(seq, 'old', a['turn_rate'],a['turn_acceleration'], 'new',b.turn_rate,b.turn_acceleration)

shifts=[]
for seq,m in old.items():
    if seq+1 not in old or seq+1 not in new or not (0<=stamp(m['generated_stamp'])-y<3.7):continue
    other=old[seq+1]; t=stamp(m['source_stamp'])+3.0
    pa=interpolate(m['samples'],t-stamp(m['source_stamp']))
    pb=interpolate(other['samples'],t-stamp(other['source_stamp']))
    a,b=new[seq],new[seq+1]
    def npos(p,t):
        samples=p.samples;relative=t-p.source_stamp
        for sa,sb in zip(samples,samples[1:]):
            if sa.relative_time<=relative<=sb.relative_time:
                f=(relative-sa.relative_time)/(sb.relative_time-sa.relative_time)
                return tuple(x+f*(y-x) for x,y in zip(sa.position,sb.position))
        raise ValueError(relative)
    shifts.append((seq,math.dist(pa,pb),math.dist(npos(a,t),npos(b,t))))
print('matching snapshots',len(new),'replay 3s same absolute contact shifts',len(shifts))
for i in [1,2]:
    values=sorted(x[i] for x in shifts)
    print('old' if i==1 else 'deduplicated','median',values[len(values)//2],'max',max(values),'over.5',sum(x>.5 for x in values))
print('reject stages',Counter(r['message']['rejection_stage'] for r in rows if r['topic']=='planner'))
