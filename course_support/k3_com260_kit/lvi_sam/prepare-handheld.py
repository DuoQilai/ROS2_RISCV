"""Convert the first complete chunks of the official handheld bag to a Humble bag."""
import hashlib
import sys
from collections import Counter
from io import BytesIO
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
from PIL import Image
from rosbags.rosbag1.reader import Header, RecordType, read_uint32
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

DIGEST = '8bc81930f249cf5e5d3559b418504f260ebec65e704e08f72aada34550f52578'
TOPICS = {
    '/points_raw': 'sensor_msgs/msg/PointCloud2',
    '/imu_raw': 'sensor_msgs/msg/Imu',
    '/imu_correct': 'sensor_msgs/msg/Imu',
    '/camera/image_raw/compressed': 'sensor_msgs/msg/CompressedImage',
}


def records(stream, end):
    while stream.tell() < end:
        header = Header.read(stream)
        yield header, stream.read(read_uint32(stream))


def convert(source, destination):
    with source.open('rb') as stream:
        digest = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
        if digest.hexdigest() != DIGEST:
            raise SystemExit('Unexpected handheld input. Check the byte range and download again.')
        stream.seek(0)
        stream.readline()  # ROS bag version line
        store = get_typestore(Stores.ROS2_HUMBLE)
        connections = {}
        counts = Counter()
        with Writer(destination, version=8) as bag:
            for header, data in records(stream, source.stat().st_size):
                if header.get_uint8('op') != RecordType.CHUNK:
                    continue
                if header.get_string('compression') != 'none':
                    raise ValueError('Expected uncompressed handheld bag chunks.')
                for entry, raw in records(BytesIO(data), len(data)):
                    op = entry.get_uint8('op')
                    cid = entry.get_uint32('conn')
                    if op == RecordType.CONNECTION:
                        topic = entry.get_string('topic')
                        if topic in TOPICS and cid not in connections:
                            name = topic.removesuffix('/compressed')
                            msgtype = TOPICS[topic] if name == topic else 'sensor_msgs/msg/Image'
                            connections[cid] = (TOPICS[topic], bag.add_connection(name, msgtype, typestore=store))
                    elif op == RecordType.MSGDATA and cid in connections:
                        original, output = connections[cid]
                        cdr = store.ros1_to_cdr(raw, original)
                        if original != output.msgtype:
                            image = store.deserialize_cdr(cdr, original)
                            gray = np.asarray(Image.open(BytesIO(image.data.tobytes())).convert('L'))
                            height, width = gray.shape
                            msg = store.types[output.msgtype](image.header, height, width, 'mono8', 0, width, gray.ravel())
                            cdr = store.serialize_cdr(msg, output.msgtype)
                        bag.write(output, entry.get_time('time'), cdr)
                        counts[output.topic] += 1
    expected = {'/points_raw': 596, '/imu_raw': 30039, '/imu_correct': 30040, '/camera/image_raw': 1802}
    if counts != expected:
        raise ValueError(f'Incomplete handheld recording: {dict(counts)}')
    print(dict(counts))


if __name__ == '__main__':
    if len(sys.argv) != 3:
        raise SystemExit('Usage: prepare-handheld.py handheld-first60.part OUTPUT_DIRECTORY')
    source, target = map(Path, sys.argv[1:])
    if target.exists():
        raise SystemExit(f'Output already exists: {target}. Inspect it with ros2 bag info.')
    target.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(dir=target.parent) as tmp:
        bag = Path(tmp) / target.name
        convert(source, bag)
        bag.rename(target)
