import rosbag2_py, math
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r=rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri='/bags/202610101135',storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
types={t.name:t.type for t in r.get_all_topics_and_types()}
odo=[];poses=[];t0=None
while r.has_next():
    tp,data,t=r.read_next()
    if t0 is None:t0=t
    if tp=='/odometry/filtered':
        m=deserialize_message(data,get_message(types[tp])); p=m.pose.pose.position; q=m.pose.pose.orientation
        odo.append(((t-t0)/1e9,p.x,p.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z)))
    elif tp=='/pose':
        m=deserialize_message(data,get_message(types[tp])); p=m.pose.pose.position
        poses.append(((t-t0)/1e9,p.x,p.y))
L=0
for a,b in zip(odo,odo[1:]): L+=math.hypot(b[1]-a[1],b[2]-a[2])
print('odom path len %.2f m, duration %.1f, first move'%(L,odo[-1][0]-odo[0][0]))
print('odo start',odo[0],'end',odo[-1])
mv=[o for o in odo if math.hypot(o[1]-odo[0][1],o[2]-odo[0][2])>0.05]
print('first move t',mv[0][0] if mv else None)
print('/pose n',len(poses))
import bisect
ts=[o[0] for o in odo]
def od(t):
    i=min(bisect.bisect(ts,t),len(odo)-1);return odo[i]
# pose msgs distinct
prev=None; gaps=[]
for p in poses:
    o=od(p[0])
    if prev: gaps.append(math.hypot(o[1]-prev[1],o[2]-prev[2]))
    prev=o
gaps=sorted(gaps); print('odom dist between /pose msgs: median %.3f p10 %.3f p90 %.3f'%(gaps[len(gaps)//2],gaps[len(gaps)//10],gaps[9*len(gaps)//10]))
for p in poses[::20]: o=od(p[0]); print('t%.1f pose %.2f %.2f   odo %.2f %.2f'%(p[0],p[1],p[2],o[1],o[2]))
