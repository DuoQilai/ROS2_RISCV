import json,subprocess,sys
run="20261108T142740Z-packages"
name="chapter17_box"
if sys.argv[1]=="add":
 sdf='<sdf version="1.9"><model name="'+name+'"><static>true</static><pose>0.8 0 0.25 0 0 0</pose><link name="body"><collision name="collision"><geometry><box><size>0.3 0.3 0.5</size></box></geometry></collision><visual name="visual"><geometry><box><size>0.3 0.3 0.5</size></box></geometry><material><ambient>0.8 0.1 0.1 1</ambient><diffuse>0.8 0.1 0.1 1</diffuse></material></visual></link></model></sdf>'
 service,kind,request="create","gz.msgs.EntityFactory","sdf: "+json.dumps(sdf)
else:service,kind,request="remove","gz.msgs.Entity",'name: "'+name+'" type: MODEL'
command=["podman","exec","-e","GZ_PARTITION="+run,"ros2-com260-"+run,"gz","service","-s","/world/default/"+service,"--reqtype",kind,"--reptype","gz.msgs.Boolean","--timeout","5000","--req",request]
result=subprocess.run(command,capture_output=True,text=True);print(result.stdout+result.stderr);assert result.returncode==0 and "data: true" in result.stdout
