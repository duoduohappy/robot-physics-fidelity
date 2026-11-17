"""Run the unmodified source on CPU; this is not transfer qualification."""
import argparse
import json
import math
from pathlib import Path
import mujoco
import numpy as np


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--report',required=True);a=p.parse_args()
    report=Path(a.report)
    if report.exists():p.error('Report must not exist')
    model=mujoco.MjModel.from_xml_path(str(Path(a.source).resolve()))
    key=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_KEY,'home')
    if key<0:raise ValueError('Expected original home keyframe')
    data=mujoco.MjData(model);mujoco.mj_resetDataKeyframe(model,data,key)
    trace=[]
    for step in range(math.ceil(.5/model.opt.timestep)):
        mujoco.mj_step(model,data);mujoco.mj_forward(model,data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():raise ValueError('Nonfinite state')
        trace.append({'step':step+1,'time_s':float(data.time),'contacts':data.ncon,'root_z_m':float(data.qpos[2])})
    result={'status':'FINITE_SOURCE_ROLLOUT','nbody':model.nbody,'ngeom':model.ngeom,'nu':model.nu,
            'explicit_pairs':model.npair,'peak_contacts':max(r['contacts'] for r in trace),
            'warnings':data.warning.number.tolist(),'trajectory':trace,
            'meaning':'Original fixed home controls for 0.5 seconds; not locomotion, sim-to-real or converted-asset qualification'}
    report.parent.mkdir(parents=True,exist_ok=True)
    with report.open('x',encoding='utf-8') as f:json.dump(result,f,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k!='trajectory'},indent=2))


if __name__=='__main__':main()
