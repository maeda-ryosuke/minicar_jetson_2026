import rosbag2_py, collections, math
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r=rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri='/bags/202610101135',storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
types={t.name:t.type for t in r.get_all_topics_and_types()}
seen=collections.Counter(); t0=None
maps=[]; tfs=collections.Counter(); mapodom=[]
while r.has_next():
    tp,data,t=r.read_next()
    if t0 is None: t0=t
    ts=(t-t0)/1e9
    if tp in ('/rosout',):
        m=deserialize_message(data,get_message(types[tp]))
        print(f'[{ts:6.1f}] rosout {m.name} L{m.level}: {m.msg[:200]}')
    elif tp in ('/scan','/scan_filtered') and seen[tp]<1:
        m=deserialize_message(data,get_message(types[tp]))
        n=len(m.ranges); v=[x for x in m.ranges if math.isfinite(x) and x>0]
        print(tp,'frame',m.header.frame_id,'amin',round(math.degrees(m.angle_min),1),'amax',round(math.degrees(m.angle_max),1),'inc',m.angle_increment,'n',n,'rmin',m.range_min,'rmax',m.range_max,'valid',len(v),'maxv',max(v) if v else None,'stamp',m.header.stamp.sec, m.header.stamp.nanosec, 'recv',t)
    elif tp=='/map':
        m=deserialize_message(data,get_message(types[tp]))
        i=m.info; occ=sum(1 for c in m.data if c>=65)
        maps.append((ts,i.width,i.height,i.resolution,i.origin.position.x,i.origin.position.y,occ))
    elif tp in ('/tf','/tf_static'):
        m=deserialize_message(data,get_message(types[tp]))
        for x in m.transforms:
            tfs[(tp,x.header.frame_id,x.child_frame_id)]+=1
            if x.child_frame_id=='odom' and x.header.frame_id=='map':
                q=x.transform.rotation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                mapodom.append((ts,x.transform.translation.x,x.transform.translation.y,yaw))
    seen[tp]+=1
for k,v in tfs.items(): print(k,v)
for m in maps[::7]+maps[-1:]: print('map',m)
print('map->odom n',len(mapodom))
for m in mapodom[::max(1,len(mapodom)//25)]: print('mo %.1f %.3f %.3f %.1f'%(m[0],m[1],m[2],math.degrees(m[3])))
