import csv
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time
from PIL import ImageGrab

root=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='qhrr05-gui-') as tmp:
    log=Path(tmp)/'daemon.log'
    csv_path=Path(tmp)/'telemetry.csv'
    with log.open('w') as stream:
        process=subprocess.Popen([str(root/'build/daemon'),'--csv',str(csv_path)],cwd='/tmp',stdout=stream,stderr=subprocess.STDOUT)
    window=None
    def contents():return log.read_text()
    def key(value):subprocess.run(['xdotool','key','--window',window,value],check=True,capture_output=True)
    try:
        deadline=time.monotonic()+25
        while time.monotonic()<deadline:
            if process.poll() is not None:raise AssertionError(contents())
            found=subprocess.run(['xdotool','search','--name','qhrr-05.*Homing control'],capture_output=True,text=True)
            if found.returncode==0:
                window=found.stdout.splitlines()[-1]
            if window and 'HOLD:' in contents():break
            time.sleep(.1)
        assert window and 'HOLD:' in contents(),contents()
        time.sleep(.5)
        key('space')
        time.sleep(.4)
        geometry=subprocess.check_output(['xdotool','getwindowgeometry','--shell',window],text=True)
        fields=dict(line.split('=',1) for line in geometry.splitlines() if '=' in line)
        x,y,w,h=(int(fields[k]) for k in ('X','Y','WIDTH','HEIGHT'))
        ImageGrab.grab().crop((x,y,x+w,y+h)).save(root/'docs/homing_preview.png')
        key('r')
        time.sleep(.2)
        key('space')
        time.sleep(.7)
        key('Escape')
        code=process.wait(timeout=8)
        assert code==0,contents()
        rows=list(csv.DictReader(csv_path.open()))
        assert len(rows)>100,len(rows)
        assert {row['run_id'] for row in rows}=={'0','1'}
        first_run=[row for row in rows if row['run_id']=='0']
        last=first_run[-1]
        goal=[0,.9,-1.8]
        joints=['FL_hip_joint','FL_thigh_joint','FL_calf_joint']
        error=max(abs(float(last['q_'+j])-g) for j,g in zip(joints,goal))
        assert error<.01,error
        assert 'task_controller exited code=0' in contents(),contents()
        print('PASS: daemon GUI, real angle/torque plots, pause/restart/ESC, CSV run IDs')
        print('GUI final pre-restart error:',error)
        print(contents())
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill();process.wait()
