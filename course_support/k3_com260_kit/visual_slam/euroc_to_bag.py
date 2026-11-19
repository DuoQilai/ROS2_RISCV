from pathlib import Path
import json
import sys
import zipfile
import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, Imu

archive, destination = Path(sys.argv[1]), Path(sys.argv[2])
writer = rosbag2_py.SequentialWriter()
writer.open(rosbag2_py.StorageOptions(uri=str(destination), storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
for name, kind in [('/camera/image_raw', 'sensor_msgs/msg/Image'), ('/imu', 'sensor_msgs/msg/Imu')]:
    writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=kind, serialization_format='cdr'))
counts = {'image': 0, 'imu': 0}
with zipfile.ZipFile(archive) as data:
    def rows(name):
        return [line.split(',') for line in data.read(name).decode().splitlines() if line and not line.startswith('#')]
    images = rows('mav0/cam0/data.csv')
    imu = rows('mav0/imu0/data.csv')
    events = [(int(row[0]), 'image', row) for row in images]
    events += [(int(row[0]), 'imu', row) for row in imu]
    events.sort(key=lambda event: (event[0], event[1] == 'image'))
    for timestamp, kind, row in events:
        if kind == 'image':
            pixels = cv2.imdecode(np.frombuffer(data.read('mav0/cam0/data/' + row[1]), np.uint8), cv2.IMREAD_GRAYSCALE)
            assert pixels is not None and pixels.shape == (480, 752)
            message = Image(height=480, width=752, encoding='mono8', is_bigendian=0, step=752, data=pixels.tobytes())
            message.header.frame_id = 'camera'
            topic = '/camera/image_raw'
        else:
            values = [float(x) for x in row[1:]]
            message = Imu()
            message.header.frame_id = 'imu'
            message.orientation_covariance[0] = -1.0
            message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = values[:3]
            message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = values[3:6]
            topic = '/imu'
        message.header.stamp.sec = timestamp // 10**9
        message.header.stamp.nanosec = timestamp % 10**9
        writer.write(topic, serialize_message(message), timestamp)
        counts[kind] += 1
    with (destination.parent / 'reference.tum').open('w') as output:
        for row in rows('mav0/state_groundtruth_estimate0/data.csv'):
            timestamp = int(row[0])
            output.write(f'{timestamp // 10**9}.{timestamp % 10**9:09d} ' + ' '.join(row[1:4] + row[5:8] + row[4:5]) + '\n')
print(json.dumps({'counts': counts, 'source': archive.name, 'first_ns': events[0][0], 'last_ns': events[-1][0]}))
