"""Actual load/step integration with controlled bad-export and revocation cases."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
from unittest.mock import patch
import mujoco
from pxr import Usd,UsdPhysics
from robot_physics_fidelity import auto_transactions as at,downstream,engine,transactions as tx


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    source=Path(a.source).resolve();out=Path(a.output).resolve();out.mkdir(parents=True,exist_ok=False)
    os.environ.setdefault('WARP_CACHE_PATH',str(out/'kernel-cache'))
    original={str(f.relative_to(source.parent)):hashlib.sha256(f.read_bytes()).hexdigest() for f in source.parent.rglob('*') if f.is_file()}
    receipts={};workspace=out/'normal-workspace'
    at.prepare(source,workspace);verified=at.verify(workspace)
    assert verified['status']=='PASS'
    tx.write_json(out/'normal-verification.json',verified)
    normal=downstream.run(workspace,out/'normal-job',profile='raised');receipts['normal']=normal
    assert normal['status']=='COMPLETED' and normal['steps_executed']==250 and normal['model_created']
    print('normal: actual model created and advanced 250 steps',flush=True)

    badroot=out/'controlled-bad-export'
    shutil.copytree(workspace/'original',badroot)
    rel=Path(tx.read(workspace/'auto-plan.json')['original_asset']).relative_to('original');asset=badroot/rel
    stage=Usd.Stage.Open(str(asset));base=next(p for p in stage.Traverse() if p.GetName()=='base' and p.HasAPI(UsdPhysics.MassAPI))
    mass=UsdPhysics.MassAPI(base).GetMassAttr();mass.Set(mass.Get()*3);stage.GetRootLayer().Save()
    def bad_export(src,folder):shutil.copytree(badroot,folder);return Path(folder)/rel
    failed=out/'failed-workspace'
    with patch.object(engine,'convert',side_effect=bad_export):at.prepare(source,failed)
    negative=at.verify(failed);tx.write_json(out/'fault-verification.json',negative);assert negative['status']=='FAIL'
    fault=downstream.run(failed,out/'fault-job');receipts['fault']=fault
    assert fault['status']=='BLOCKED' and not fault['loader_called'] and fault['steps_executed']==0
    print('fault: verification failed; downstream loader was never called',flush=True)

    at.rollback(workspace)
    revoked=downstream.run(workspace,out/'revoked-job');receipts['revoked']=revoked
    assert revoked['status']=='BLOCKED' and not revoked['loader_called'] and revoked['steps_executed']==0
    print('revoked: loader was never called',flush=True)

    at.verify(workspace)
    with (workspace/'original'/rel).open('a',encoding='utf-8') as f:f.write('\n# controlled artifact-integrity change\n')
    tampered=downstream.run(workspace,out/'tampered-job');receipts['tampered']=tampered
    assert tampered['status']=='BLOCKED' and not tampered['loader_called'] and tampered['steps_executed']==0
    print('tampered: loader was never called',flush=True)

    # Reuse the normal job's unmodified frozen export as an injected converter result.
    clean=out/'running-revocation-workspace'
    def original_export(src,folder):shutil.copytree(out/'normal-job/frozen-asset',folder);return Path(folder)/rel
    with patch.object(engine,'convert',side_effect=original_export):at.prepare(source,clean)
    at.verify(clean)
    step=mujoco.mj_step;count=0
    def revoke_after_three(model,data):
        nonlocal count
        step(model,data);count+=1
        if count==3:at.rollback(clean)
    with patch.object(mujoco,'mj_step',side_effect=revoke_after_three):
        interrupted=downstream.run(clean,out/'running-revocation-job')
    assert interrupted['status']=='BLOCKED' and interrupted['steps_executed']==3 and interrupted['block_code']=='AUTHORIZATION_CHANGED'
    tx.write_json(out/'running-revocation.json',interrupted)
    print('running revocation: stopped before step 4; previous steps were not undone',flush=True)
    after={str(f.relative_to(source.parent)):hashlib.sha256(f.read_bytes()).hexdigest() for f in source.parent.rglob('*') if f.is_file()}
    assert original==after
    summary={'status':'ACCEPTED_GATED_LOAD_AND_STEP','cases':{name:{k:v for k,v in r.items() if k not in ['frames','source_bundle','runtime_configuration']} for name,r in receipts.items()},
             'source_files_unchanged':True,'running_revocation_stopped_after_steps':3,'source_visualization':'Source visual meshes render target native-model states; no source trajectory substitutes',
             'controlled_fault':'Artificial 3x base mass before plan hash recording; not a claimed real converter defect',
             'human_or_hardware_trial':False}
    tx.write_json(out/'demo-summary.json',summary)
    return summary


if __name__=='__main__':main()
