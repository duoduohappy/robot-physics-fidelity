"""Render recorded target simulation states using licensed source visual meshes."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import numpy as np
import mujoco


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--receipt',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    receipt=json.loads(Path(a.receipt).read_text(encoding='utf-8'))
    if receipt['status']!='COMPLETED' or not receipt.get('frames_qualified') or receipt['steps_executed']!=250:
        raise ValueError('Only completed qualified trajectories may be rendered as this successful run')
    out=Path(a.output).resolve();out.mkdir(parents=True,exist_ok=False)
    model=mujoco.MjModel.from_xml_path(str(Path(a.source).resolve()))
    model.vis.global_.offwidth=720;model.vis.global_.offheight=480
    renderer=mujoco.Renderer(model,height=480,width=720)
    data=mujoco.MjData(model);camera=mujoco.MjvCamera()
    camera.lookat[:]=[0,0,.23];camera.distance=1.25;camera.azimuth=130;camera.elevation=-18
    ffmpeg=shutil.which('ffmpeg')
    if not ffmpeg:raise RuntimeError('Optional visualization requires ffmpeg on PATH')
    video=out/'go2-loaded-run.mp4';preview=out/'go2-loaded-preview.png'
    command=[ffmpeg,'-hide_banner','-loglevel','error','-n','-f','rawvideo','-pixel_format','rgb24','-video_size','720x480','-framerate','25','-i','pipe:0','-c:v','libx264','-preset','medium','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(video)]
    process=subprocess.Popen(command,stdin=subprocess.PIPE)
    try:
        for i,frame in enumerate(receipt['frames'][1::2]):
            data.qpos[:]=frame['source_visual_qpos'];mujoco.mj_forward(model,data)
            renderer.update_scene(data,camera=camera);pixels=renderer.render()
            if not np.std(pixels)>2:raise RuntimeError('Empty or uniform rendered frame')
            process.stdin.write(pixels.tobytes())
            if i==90:
                subprocess.run([ffmpeg,'-hide_banner','-loglevel','error','-n','-f','rawvideo','-pixel_format','rgb24','-video_size','720x480','-i','pipe:0','-frames:v','1',str(preview)],input=pixels.tobytes(),check=True)
    finally:
        process.stdin.close();renderer.close()
    if process.wait()!=0:raise RuntimeError('Video encoding failed')
    metadata={'status':'RENDERED_RECORDED_TARGET_STATES','frames':125,'fps':25,'playback_seconds':5,'simulated_seconds':.5,'slowdown':10,
              'physics':'Newton-derived native MuJoCo CPU target model admitted by the downstream gate',
              'visualization':'Original source meshes positioned by recorded target qpos mapping; source renderer does not advance a separate source trajectory',
              'model_license':'BSD-3-Clause; original Unitree Go2 attribution required','profile':receipt['profile']}
    (out/'render-provenance.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
    print(video)


if __name__=='__main__':
    os.environ.pop('MUJOCO_GL',None)
    main()
