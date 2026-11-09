import json, math, time
from pathlib import Path
import rclpy
from action_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
run='20261108T142740Z-packages'
out=Path.home()/'ros2_course_com260_ch16_ch17_ws/runs'/run/'trees';out.mkdir(parents=True,exist_ok=True)
rclpy.init();node=rclpy.create_node('post_arrival_observer')
clock=0.;events=[];samples=[]
def on_clock(m):
 global clock
 clock=m.clock.sec+m.clock.nanosec/1e9
def on_status(kind,msg):
 for s in msg.status_list:
  key=bytes(s.goal_info.goal_id.uuid).hex()
  if not any(e['kind']==kind and e['id']==key and e['status']==s.status for e in events):
   event={'kind':kind,'id':key,'status':s.status,'sim_time':clock,'wall_time':time.time()};events.append(event);print(json.dumps(event),flush=True)
def on_odom(m):
 q=m.pose.pose.orientation
 yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
 samples.append([m.header.stamp.sec+m.header.stamp.nanosec/1e9,yaw,m.twist.twist.linear.x,m.twist.twist.angular.z])
node.create_subscription(Clock,'/clock',on_clock,10)
node.create_subscription(Odometry,'/odom',on_odom,qos_profile_sensor_data)
qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)
for kind in ['wait','spin']:
 node.create_subscription(GoalStatusArray,f'/{kind}/_action/status',lambda m,k=kind:on_status(k,m),qos)
print('RUN_ID='+run,flush=True)
def completed(kind):
 pairs=[]
 for finish in events:
  if finish['kind']!=kind or finish['status']!=4:continue
  start=next((e for e in events if e['kind']==kind and e['id']==finish['id'] and e['status']==2),None)
  if start:pairs.append((start,finish))
 return pairs
end=time.monotonic()+450
wait_pair=spin_pair=None
while time.monotonic()<end:
 rclpy.spin_once(node,timeout_sec=.1)
 wait_pair=next((p for p in completed('wait') if p[1]['sim_time']-p[0]['sim_time']>=1.9),None)
 if wait_pair:
  spin_pair=next((p for p in completed('spin') if p[0]['sim_time']>=wait_pair[1]['sim_time']-.1),None)
  if spin_pair:break
result={'run_id':run,'events':events,'odom_samples':len(samples)}
if wait_pair and spin_pair:
 result['wait_duration_sim_s']=wait_pair[1]['sim_time']-wait_pair[0]['sim_time']
 phase=[m for m in samples if spin_pair[0]['sim_time']<=m[0]<=spin_pair[1]['sim_time']]
 result['spin_duration_sim_s']=spin_pair[1]['sim_time']-spin_pair[0]['sim_time']
 result['spin_odom_yaw_rad']=sum(math.atan2(math.sin(b[1]-a[1]),math.cos(b[1]-a[1])) for a,b in zip(phase,phase[1:]))
result['passed']=result.get('wait_duration_sim_s',0)>=1.9 and 6.0<=result.get('spin_odom_yaw_rad',0)<=6.6
(out/'post-arrival.json').write_text(json.dumps(result,indent=2));(out/'post-arrival-odom.csv').write_text('sim_time,yaw,vx,wz\n'+''.join(','.join(map(str,m))+'\n' for m in samples))
print(json.dumps(result),flush=True)
node.destroy_node();rclpy.shutdown()
assert result['passed']
