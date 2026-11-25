#!/usr/bin/env python3
"""Generate the enlarged COM260 project scene and its reachable-floor mask."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from PIL import Image, ImageDraw, ImageFilter

root = Path(__file__).resolve().parents[3]
source = root / 'src_k3_pico_itx/robot_sim_demo'
out = Path(__file__).resolve().parents[1] / 'config'
scale, resolution, height = 1.5, 0.05, 0.18
c, s = math.cos(-1.57), math.sin(-1.57)

def world(x, y):
    return scale * (c*x-s*y), scale * (s*x+c*y)

mesh = ET.parse(source / 'models/ISCAS_Museum/meshes/ISCAS_museum.dae').getroot()
ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
by_id = {e.get('id'): e for e in mesh.iter() if e.get('id')}
segments = []
for node in mesh.findall('.//c:visual_scene/c:node', ns):
    instance = node.find('c:instance_geometry', ns)
    if instance is None:
        continue
    matrix = list(map(float, node.find('c:matrix', ns).text.split()))
    geometry = by_id[instance.get('url')[1:]]
    vertex_source = geometry.find('.//c:vertices/c:input', ns).get('source')[1:]
    values = list(map(float, by_id[vertex_source].find('c:float_array', ns).text.split()))
    points = []
    for i in range(0, len(values), 3):
        v = values[i:i+3]+[1]
        p = [sum(matrix[j*4+k]*v[k] for k in range(4)) for j in range(3)]
        points.append((*world(p[0], p[1]), p[2]))
    for triangles in geometry.findall('.//c:triangles', ns):
        inputs = triangles.findall('c:input', ns)
        stride = max(int(x.get('offset')) for x in inputs)+1
        offset = next(int(x.get('offset')) for x in inputs if x.get('semantic') == 'VERTEX')
        ids = list(map(int, triangles.find('c:p', ns).text.split()))[offset::stride]
        for i in range(0, len(ids), 3):
            triangle = [points[j] for j in ids[i:i+3]]
            cuts = []
            for a, b in zip(triangle, triangle[1:]+triangle[:1]):
                if (a[2] <= height < b[2]) or (b[2] <= height < a[2]):
                    t = (height-a[2])/(b[2]-a[2])
                    cuts.append((a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])))
            if len(cuts) == 2:
                segments.append(cuts)

model = ET.parse(source / 'models/ISCAS_Museum/model.sdf').getroot().find('model')
model.set('name', 'COM260_Project_Museum')
ET.SubElement(model, 'pose').text = '0 0 0 0 0 -1.57'
for mesh_geometry in model.findall('.//mesh'):
    ET.SubElement(mesh_geometry, 'scale').text = f'{scale} {scale} 1'
for link in model.findall('link'):
    box = link.find('.//collision/geometry/box/size')
    if box is None:
        continue
    pose = list(map(float, link.find('pose').text.split()))
    size = list(map(float, box.text.split()))
    corners = [world(pose[0]+dx*size[0]/2, pose[1]+dy*size[1]/2)
               for dx, dy in [(-1,-1),(1,-1),(1,1),(-1,1)]]
    segments.extend(zip(corners, corners[1:]+corners[:1]))
    pose[0] *= scale; pose[1] *= scale
    link.find('pose').text = ' '.join(map(str, pose))
    for size_element in link.findall('.//box/size'):
        size_element.text = f'{size[0]*scale} {size[1]*scale} {size[2]}'

xmin = math.floor(min(p[0] for line in segments for p in line))-1
ymin = math.floor(min(p[1] for line in segments for p in line))-1
xmax = math.ceil(max(p[0] for line in segments for p in line))+1
ymax = math.ceil(max(p[1] for line in segments for p in line))+1
width, rows = round((xmax-xmin)/resolution), round((ymax-ymin)/resolution)
image = Image.new('L', (width, rows), 0)
draw = ImageDraw.Draw(image)
def pixel(p):
    return round((p[0]-xmin)/resolution), rows-1-round((p[1]-ymin)/resolution)
for segment in segments:
    draw.line([pixel(p) for p in segment], fill=255, width=1)
# A 0.22 m robot radius: exclude walls, furniture and unreachable components.
image = image.filter(ImageFilter.MaxFilter(2*math.ceil(0.22/resolution)+1))
ImageDraw.floodfill(image, pixel((0,0)), 128)
assert not any(image.getpixel(p) == 128 for p in [(0,0),(width-1,0),(0,rows-1),(width-1,rows-1)]), 'Floor boundary is open'
mask = image.point(lambda p: 255 if p == 128 else 0)
area = mask.histogram()[255] * resolution**2
assert area >= 500, f'Explorable floor is only {area:.2f} m2'
out.mkdir(exist_ok=True)
mask.save(out / 'exploration_area.pgm')
(out / 'exploration_area.yaml').write_text(
    f'image: exploration_area.pgm\nresolution: {resolution}\norigin: [{xmin}, {ymin}, 0]\n'
    f'area_m2: {area:.6f}\nrobot_radius: 0.22\nscale_xy: {scale}\n')
scene = ET.parse(source / 'worlds/museum.sdf')
world_element = scene.getroot().find('world')
for include in world_element.findall('include'):
    if include.find('uri').text == 'model://ISCAS_Museum':
        world_element.remove(include)
world_element.append(model)
ET.indent(scene, space='  ')
scene.write(out / 'project_museum.sdf', encoding='utf-8', xml_declaration=True)
print(f'Explorable floor: {area:.3f} m2, {width}x{rows}, resolution {resolution} m')
