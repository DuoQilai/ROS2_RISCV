import json,time
from pathlib import Path
import rclpy
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
from nav2_msgs.srv import GetCostmap
rclpy.init();node=rclpy.create_node('costmap_exercise_check')
result={};saved={}
def call(service,kind,request):
 client=node.create_client(kind,service)
 assert client.wait_for_service(timeout_sec=5),service
 future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=8)
 assert future.done(),service
 reply=future.result();node.destroy_client(client);return reply
def get(prefix,names):
 return call(prefix+'/get_parameters',GetParameters,GetParameters.Request(names=names)).values
def set_values(prefix,names,values):
 response=call(prefix+'/set_parameters',SetParameters,SetParameters.Request(parameters=[Parameter(name=n,value=v) for n,v in zip(names,values)]))
 assert all(r.successful for r in response.results),str(response)
try:
 for prefix in ['/global_costmap/global_costmap','/local_costmap/local_costmap']:
  names=['robot_radius','inflation_layer.inflation_radius']
  saved[prefix]=(names,get(prefix,names))
  set_values(prefix,names,[ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=v) for v in [0.3,0.6]])
  actual=get(prefix,names+['resolution','update_frequency']+(['width','height'] if prefix.startswith('/local') else []))
  values=[v.double_value if v.type==ParameterType.PARAMETER_DOUBLE else v.integer_value for v in actual]
  assert values[0:3]==[0.3,0.6,0.05],values
  if prefix.startswith('/local'):assert values[3:]==[5.0,3,3],values
  result[prefix]=dict(zip(names+['resolution','update_frequency','width','height'],values))
 time.sleep(3)
 for prefix in ['/global_costmap','/local_costmap']:
  m=call(prefix+'/get_costmap',GetCostmap,GetCostmap.Request()).map
  result[prefix+'/geometry']=[m.metadata.size_x,m.metadata.size_y,m.metadata.resolution]
  assert abs(m.metadata.resolution-0.05)<1e-6
  if prefix=='/local_costmap':assert [m.metadata.size_x,m.metadata.size_y]==[60,60]
 result['passed']=True
finally:
 for prefix,(names,values) in saved.items():set_values(prefix,names,values)
 node.destroy_node();rclpy.shutdown()
 result['original_parameters_restored']=True
 (Path.home()/'ros2_course_com260_ch16_ch17_ws/runs/20261108T142740Z-packages/exercise-config.json').write_text(json.dumps(result,indent=2))
 print(json.dumps(result),flush=True)
