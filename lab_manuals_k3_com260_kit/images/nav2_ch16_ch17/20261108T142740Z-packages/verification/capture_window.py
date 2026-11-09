import json,os,subprocess,sys
from datetime import datetime,timezone
from pathlib import Path
run="20261108T142740Z-packages"
window,seconds,name=sys.argv[1:]
out=Path.home()/"ros2_course_com260_ch16_ch17_humble_ws/runs"/run
out.mkdir(parents=True,exist_ok=True)
env=dict(os.environ,DISPLAY=":1",XAUTHORITY="/run/user/1000/gdm/Xauthority")
command=["ffmpeg","-hide_banner","-loglevel","error","-f","x11grab","-framerate","12","-window_id",window,"-i",":1.0","-t",seconds,"-c:v","libx264","-preset","ultrafast","-crf","23","-pix_fmt","yuv420p","-y",str(out/(name+".mp4"))]
result={"run_id":run,"start_utc":datetime.now(timezone.utc).isoformat(),"window_id":window,"command":command}
result["exit"]=subprocess.run(command,env=env).returncode
result["end_utc"]=datetime.now(timezone.utc).isoformat()
(out/(name+"-capture.json")).write_text(json.dumps(result,indent=2))
print(json.dumps(result))
sys.exit(result["exit"])
