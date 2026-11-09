import json
from pathlib import Path
import subprocess
import threading
import time
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose

root=Path.home()/'ros2_course_com260_ch16_ch17_ws'
out=root/'runs/20261108T142740Z-packages/action-failures';out.mkdir(parents=True, exist_ok=True)
print('RUN_ID=20261108T142740Z-packages', flush=True)
rclpy.init()
node=rclpy.create_node('synthetic_action_failure_server')
executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
thread=threading.Thread(target=executor.spin);thread.start()
results=[]
try:
    for mode in ['reject','abort','cancel-refused']:
        finished=threading.Event()
        def execute(handle):
            if mode=='cancel-refused': time.sleep(14)
            handle.abort();finished.set();return NavigateToPose.Result()
        server=ActionServer(node,NavigateToPose,'navigate_to_pose',execute,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=lambda _:GoalResponse.REJECT if mode=='reject' else GoalResponse.ACCEPT,
            cancel_callback=lambda _:CancelResponse.REJECT)
        start=time.monotonic()
        args=[str(root/'install/navigation_client_cpp/lib/navigation_client_cpp/bt_navigator_client')]
        if mode=='cancel-refused':args+=['--ros-args','-p','cancel_after:=1.0']
        result=subprocess.run(args,capture_output=True,text=True,timeout=25)
        elapsed=time.monotonic()-start
        text=result.stdout+result.stderr
        (out/(mode+'.log')).write_text(text)
        print('SYNTHETIC CASE',mode,'elapsed',round(elapsed,2),flush=True)
        print(text,flush=True)
        assert result.returncode==1,mode
        if mode=='cancel-refused':
            assert 10<=elapsed<16 and '取消后未收到结果' in text
        elif mode=='reject':assert '目标未接受' in text
        else:assert 'ABORTED' in text
        results.append({'case':mode,'exit':result.returncode,'elapsed_s':elapsed,'passed':True})
        if mode!='reject':assert finished.wait(5)
        server.destroy()
    (out/'results.json').write_text(json.dumps(results,indent=2))
    print('SYNTHETIC_FAILURE_CASES_PASSED',len(results),flush=True)
finally:
    executor.shutdown();thread.join();node.destroy_node();rclpy.shutdown()
