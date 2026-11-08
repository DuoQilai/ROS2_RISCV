import json,re,subprocess,time
from pathlib import Path
run='20261108T142740Z-packages';root=Path.home()/'ros2_course_com260_ch16_ch17_ws';out=root/'runs'/run;out.mkdir(parents=True,exist_ok=True)
print('RUN_ID='+run,flush=True)
command=[str(root/'install/global_planner_client/lib/global_planner_client/global_planner_client'),'--ros-args','-p','use_sim_time:=true']
start=time.monotonic();p=subprocess.run(command,capture_output=True,text=True,timeout=50);text=p.stdout+p.stderr
print(text,flush=True);(out/'planner.log').write_text(text)
match=re.search(r'路径规划成功: (\d+) 个路径点',text)
result={'run_id':run,'start':[0,0],'goal':[3,2],'planner':'GridBased','exit':p.returncode,'duration_s':time.monotonic()-start,'path_points':int(match[1]) if match else 0,'passed':p.returncode==0 and bool(match)}
(out/'planner.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True);assert result['passed']
