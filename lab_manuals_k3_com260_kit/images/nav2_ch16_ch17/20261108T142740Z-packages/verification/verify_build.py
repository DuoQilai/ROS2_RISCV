import json,subprocess,hashlib,sys
from pathlib import Path
root=Path(sys.argv[1]).expanduser()
packages=['custom_nav_component', 'navigation_client_cpp', 'global_planner_client', 'costmap_demo', 'map_loader', 'obstacle_layer_processor', 'costmap_debugger']
commands=[];files=[]
for package in packages:
 commands+=json.loads((root/'build'/package/'compile_commands.json').read_text())
 files+=sorted((root/'install'/package/'lib'/package).iterdir())
assert len(commands)==len(files)==9
macros=subprocess.check_output(['c++','-dM','-E','-x','c++','/dev/null'],text=True)
cpp=next(line for line in macros.splitlines() if line.startswith('#define __cplusplus '))
assert '201703L' in cpp
assert all('-g' in c['command'].split() for c in commands)
result={'packages':packages,'compile_units':len(commands),'default_cpp':cpp,'debug':True,'binaries':[{'name':f.name,'package':f.parent.name,'sha256':hashlib.sha256(f.read_bytes()).hexdigest(),'elf':next(line.strip() for line in subprocess.check_output(['readelf','-h',str(f)],text=True).splitlines() if 'Machine:' in line)} for f in files],'passed':True}
print(json.dumps(result,indent=2))
