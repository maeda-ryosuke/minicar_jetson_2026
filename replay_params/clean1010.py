import rosbag2_py, csv, math
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
src='/bags/202610101135'; dst='/replay_params/sweep/bag_j1010_clean'
r=rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=src,storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
tt={t.name:t for t in r.get_all_topics_and_types()}
keep=['/scan','/tf','/tf_static','/odom','/imu','/odometry/filtered']
w=rosbag2_py.SequentialWriter()
w.open(rosbag2_py.StorageOptions(uri=dst,storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
for k in keep:
    t=tt[k]; w.create_topic(rosbag2_py.TopicMetadata(name=k,type=t.type,serialization_format='cdr',offered_qos_profiles=t.offered_qos_profiles))
TF=get_message('tf2_msgs/msg/TFMessage')
f=open('/replay_params/sweep/j1010_actual_tf.csv','w'); f.write('t,mo_x,mo_y,mo_yaw\n')
removed=0
while r.has_next():
    tp,data,t=r.read_next()
    if tp not in keep: continue
    if tp=='/tf':
        m=deserialize_message(data,TF); ts=[]
        for x in m.transforms:
            if x.header.frame_id=='map':
                q=x.transform.rotation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                st=x.header.stamp.sec+x.header.stamp.nanosec*1e-9
                f.write(f'{st},{x.transform.translation.x},{x.transform.translation.y},{yaw}\n'); removed+=1
            else: ts.append(x)
        if not ts: continue
        m.transforms=ts; data=serialize_message(m)
    w.write(tp,data,t)
print('removed map tf',removed)
