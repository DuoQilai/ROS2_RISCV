import json
from pathlib import Path
import rclpy
from rcl_interfaces.srv import GetParameters
run="20261108T142740Z-packages"
rclpy.init();node=rclpy.create_node("final_parameter_check")
expected={"/planner_server":{"GridBased.tolerance":0.15},"/controller_server":{"general_goal_checker.xy_goal_tolerance":0.1,"FollowPath.xy_goal_tolerance":0.1,"progress_checker.required_movement_radius":0.1,"progress_checker.movement_time_allowance":10.0}}
for prefix in ["/global_costmap/global_costmap","/local_costmap/local_costmap"]:expected[prefix]={"robot_radius":0.22,"inflation_layer.inflation_radius":0.55,"inflation_layer.cost_scaling_factor":3.0}
actual={}
for prefix,values in expected.items():
 client=node.create_client(GetParameters,prefix+"/get_parameters");assert client.wait_for_service(timeout_sec=8)
 future=client.call_async(GetParameters.Request(names=list(values)));rclpy.spin_until_future_complete(node,future,timeout_sec=8)
 assert future.done();actual[prefix]=dict(zip(values,[v.double_value for v in future.result().values]));assert actual[prefix]==values,(prefix,actual[prefix])
result={"run_id":run,"actual":actual,"passed":True}
(Path.home()/"ros2_course_com260_ch16_ch17_ws/runs"/run/"parameters-final.json").write_text(json.dumps(result,indent=2))
print(json.dumps(result));node.destroy_node();rclpy.shutdown()
