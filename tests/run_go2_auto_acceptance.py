from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess
import sys
import argparse
from unittest.mock import patch
from pxr import Usd,UsdGeom,UsdPhysics,Sdf
from robot_physics_fidelity import automatic as auto,auto_transactions as at,engine,transactions as tx

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);parser.add_argument('--source',required=True);args=parser.parse_args()
out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=False)
source=Path(args.source).resolve();before={str(p.relative_to(source.parent)):hashlib.sha256(p.read_bytes()).hexdigest() for p in source.parent.rglob('*') if p.is_file()}
os.environ['WARP_CACHE_PATH']=str(root/'work/go2-assessment-cache')
workspace=out/'positive';at.prepare(source,workspace)
result=at.verify(workspace);tx.write_json(out/'positive.json',result)
assert result['status']=='PASS' and result['qualified'] and result['repair_required'] is False
active=at.active_path(workspace);tx.write_json(out/'active.json',active);assert active['qualified']
receipt=at.rollback(workspace);tx.write_json(out/'rollback.json',receipt);assert not at.active_path(workspace)['qualified']
assert result['rules']['pairs_checked']==276 and result['rules']['source_eligible_count']==233
print('positive: 276 pairs / four profiles / qualification / revocation verified',flush=True)

identity=result['identity'];first=next(r for r in identity if r['body']=='base');second=next(r for r in identity if r['body']=='base' and r!=first)
faults=[]
for fault in ['mask','missing','radius','ambiguous','mass']:
    directory=out/('fault-'+fault);asset_dir=directory/'upstream-output'
    shutil.copytree(workspace/'original',asset_dir)
    original_rel=tx.read(workspace/'auto-plan.json')['original_asset']
    asset=asset_dir/Path(original_rel).relative_to('original')
    stage=Usd.Stage.Open(str(asset));prim=stage.GetPrimAtPath(first['usd_path'])
    if fault=='mask':
        prim.CreateAttribute('mjc:contype',Sdf.ValueTypeNames.Int).Set(0)
        prim.CreateAttribute('mjc:conaffinity',Sdf.ValueTypeNames.Int).Set(0)
    elif fault=='missing':UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
    elif fault=='radius':prim.GetAttribute('radius').Set(float(prim.GetAttribute('radius').Get())*1.3)
    elif fault=='ambiguous':
        other=stage.GetPrimAtPath(second['usd_path']);other.GetAttribute('radius').Set(prim.GetAttribute('radius').Get())
        transform=UsdGeom.Xformable(prim).GetLocalTransformation();xform=UsdGeom.Xformable(other)
        xform.ClearXformOpOrder();xform.AddTransformOp(opSuffix='intakeFault').Set(transform)
    elif fault=='mass':
        body=stage.GetPrimAtPath(prim.GetPath().GetParentPath());mass=UsdPhysics.MassAPI(body).GetMassAttr()
        assert mass and mass.Get()>0;mass.Set(mass.Get()*3)
    stage.GetRootLayer().Save()
    badworkspace=directory/'workspace'
    def broken_export(src,folder):
        shutil.copytree(asset_dir,folder);return Path(folder)/asset.relative_to(asset_dir)
    # Simulate an upstream faulty export BEFORE its immutable plan hash is recorded.
    # This is injected evidence, not a claim that the real converter emitted it.
    with patch.object(engine,'convert',side_effect=broken_export):at.prepare(source,badworkspace)
    try:
        bad=at.verify(badworkspace);status=bad['status'];tx.write_json(directory/'verification.json',bad)
    except engine.Unsupported as exc:
        status='UNSUPPORTED';tx.write_json(directory/'verification.json',{'status':status,'reason':str(exc)})
    assert status in ['FAIL','UNSUPPORTED'],(fault,status)
    assert not at.active_path(badworkspace)['qualified']
    faults.append({'fault':fault,'status':status,'qualified':False,'type':'controlled USD fault injection'})
    print(fault+': blocked ('+status+')',flush=True)

# Verify a changed imported artifact cannot reuse earlier passing evidence.
at.verify(workspace)
artifact=workspace/'original'/Path(tx.read(workspace/'auto-plan.json')['original_asset']).relative_to('original')
with artifact.open('a',encoding='utf-8') as f:f.write('\n# controlled integrity fault\n')
try:at.active_path(workspace);raise AssertionError('Tampered asset was accepted')
except ValueError:pass
after={str(p.relative_to(source.parent)):hashlib.sha256(p.read_bytes()).hexdigest() for p in source.parent.rglob('*') if p.is_file()}
assert before==after
summary={'status':'ACCEPTED_BOUNDED_GO2_AUTOMATIC_LANE','source_files_unchanged':True,'original_files_checked':len(before),
         'positive_profiles':[(p['profile'],p['status']) for p in result['profiles']],'pairs_checked':276,'eligible_pairs':233,
         'faults':faults,'tampered_asset_blocked':True,'repair_performed':False,'real_hardware_validation':False,
         'scope':'Pinned original Go2, recorded CPU configuration and four fixed profiles; not general automatic-collision qualification'}
tx.write_json(out/'acceptance.json',summary);print(json.dumps(summary,indent=2))
