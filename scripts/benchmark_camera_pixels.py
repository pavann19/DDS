"""Offline A/B detector timing on eight saved actual camera PNGs.

No driving success, frame-rate or hard real-time claim follows from this test.
"""
import argparse
from dataclasses import asdict
import importlib.util
import json
from pathlib import Path
import statistics
import subprocess
import sys
import time
import types

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from PIL import Image
from app.services import control_camera as candidate
from app.services.camera_rig import CAMERA_CALIBRATIONS, CAMERA_IDS


def benchmark(directory, rounds, output):
    if not 1<=rounds<=1000:raise ValueError('Rounds must be 1..1000')
    reference=types.ModuleType('dds_camera_reference')
    sys.modules[reference.__name__]=reference
    source=subprocess.check_output(['git','show','ef518b21ce1f6ccaa2790a7ce6855177c5091fd3:app/services/control_camera.py'],text=True,encoding='utf-8')
    exec(compile(source,'baseline/control_camera.py','exec'),reference.__dict__)
    frames=[]
    for name in CAMERA_IDS:
        calibration=CAMERA_CALIBRATIONS[name]
        with Image.open(directory/(name+'.png')) as image:
            if image.size!=(calibration.width,calibration.height):raise ValueError('Camera dimensions differ from calibration')
            frames.append(candidate.CameraFrame(image.convert('RGB').tobytes(),1.,calibration))
    for frame in frames:
        if [asdict(x) for x in candidate.detect_controls(frame)] != [asdict(x) for x in reference.detect_controls(frame)]:
            raise AssertionError('Detector observations differ')
    timings={'reference_ms':[],'optimized_ms':[]}
    for index in range(rounds):
        order=[('reference_ms',reference.detect_controls),('optimized_ms',candidate.detect_controls)]
        if index%2:order.reverse()
        for name,detect in order:
            start=time.perf_counter()
            for frame in frames:detect(frame)
            timings[name].append((time.perf_counter()-start)*1000)
    summary={name:{'median':statistics.median(values),'min':min(values),'max':max(values)} for name,values in timings.items()}
    result={'reference_commit':'ef518b21ce1f6ccaa2790a7ce6855177c5091fd3','frames':8,'rounds':rounds,
        'serial_bundle':True,'observations_identical':True,'timings':timings,'summary':summary}
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2))
    print(json.dumps(summary))


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--frames',type=Path,required=True)
    parser.add_argument('--rounds',type=int,default=30)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();benchmark(args.frames,args.rounds,args.output)

