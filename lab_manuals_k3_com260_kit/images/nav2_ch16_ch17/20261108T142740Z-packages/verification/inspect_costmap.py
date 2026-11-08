import json
import math
from pathlib import Path
import sys
import rclpy
from nav2_msgs.srv import GetCostmap

print('RUN_ID=20261108T142740Z-packages', flush=True)
rclpy.init()
node = rclpy.create_node('costmap_acceptance')
client = node.create_client(GetCostmap, '/global_costmap/get_costmap')
assert client.wait_for_service(timeout_sec=15)
future = client.call_async(GetCostmap.Request())
rclpy.spin_until_future_complete(node, future, timeout_sec=15)
assert future.done()
grid = future.result().map
meta = grid.metadata
region = []
for y in [i / 20 for i in range(-6, 7)]:
    for x in [i / 20 for i in range(10, 23)]:
        gx = math.floor((x-meta.origin.position.x)/meta.resolution)
        gy = math.floor((y-meta.origin.position.y)/meta.resolution)
        region.append(grid.data[gy*meta.size_x+gx])
data = list(grid.data)
result = {'stage': sys.argv[1], 'size': [meta.size_x, meta.size_y], 'resolution': meta.resolution,
          'free': data.count(0), 'lethal': data.count(254), 'inscribed': data.count(253),
          'unknown': data.count(255), 'inflated': sum(1 for v in data if 0 < v < 253),
          'inflated_sum': sum(v for v in data if 0 < v < 253),
          'box_region_max': max(region), 'box_region_lethal': region.count(254)}
out = Path.home() / 'ros2_course_com260_ch16_ch17_ws/runs/20261108T142740Z-packages/costmap'
out.mkdir(parents=True, exist_ok=True)
(out/(sys.argv[1]+'.json')).write_text(json.dumps(result, indent=2))
print(json.dumps(result), flush=True)
node.destroy_node()
rclpy.shutdown()
