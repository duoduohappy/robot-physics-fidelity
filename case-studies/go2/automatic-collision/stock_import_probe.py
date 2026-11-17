"""Read-only upstream feasibility probe; never activates a candidate."""
from pathlib import Path
import hashlib
import json
import os
import argparse
import numpy as np
import mujoco
import newton
import warp as wp
from pxr import Usd,UsdPhysics
from robot_physics_fidelity import engine

parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);parser.add_argument('--source',required=True);parser.add_argument('--usd',required=True);parser.add_argument('--cache')
args=parser.parse_args();out=Path(args.output).resolve()
out.mkdir(exist_ok=False)
source=Path(args.source).resolve();asset=Path(args.usd).resolve()
before={str(p.relative_to(source.parent)):hashlib.sha256(p.read_bytes()).hexdigest() for p in source.parent.rglob('*') if p.is_file()}
os.environ['WARP_CACHE_PATH']=str(Path(args.cache).resolve() if args.cache else out/'kernel-cache')
report={'status':'PROBE_STARTED','production_gate_changed':False,'candidate_activated':False}
try:
 a=mujoco.MjModel.from_xml_path(str(source))
 stage=Usd.Stage.Open(str(asset))
 colliders=[p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
 report['usd_colliders']=[{'path':str(p.GetPath()),'type':p.GetTypeName(),
                         'contype_authored':bool(p.GetAttribute('mjc:contype').HasAuthoredValue()),
                         'conaffinity_authored':bool(p.GetAttribute('mjc:conaffinity').HasAuthoredValue())} for p in colliders]
 with wp.ScopedDevice('cpu'):
  builder=newton.ModelBuilder();newton.solvers.SolverMuJoCo.register_custom_attributes(builder)
  imported=builder.add_usd(stage,enable_self_collisions=True)
  solver=newton.solvers.SolverMuJoCo(builder.finalize(device='cpu'),use_mujoco_cpu=True,use_mujoco_contacts=True)
 b=solver.mj_model;maps=engine.alignment(a,b)
 report['counts']={'source_geoms':a.ngeom,'target_geoms':b.ngeom,'source_bodies':a.nbody,'target_bodies':b.nbody}
 for field in engine.SETTINGS:setattr(b.opt,field,getattr(a.opt,field))
 da,db=engine.initial(a,b,maps,'home');mujoco.mj_forward(a,da);mujoco.mj_forward(b,db)
 active=[i for i in range(a.ngeom) if a.geom_contype[i] or a.geom_conaffinity[i]]
 report['source_collision_geometry']=[{'index':i,'body':int(a.geom_bodyid[i]),'type':int(a.geom_type[i]),'size':a.geom_size[i].tolist(),'world_position':da.geom_xpos[i].tolist()} for i in active]
 report['target_geometry']=[{'index':j,'body':int(b.geom_bodyid[j]),'type':int(b.geom_type[j]),'size':b.geom_size[j].tolist(),'world_position':db.geom_xpos[j].tolist()} for j in range(b.ngeom)]
 candidates={}
 for i in active:
  owner=maps['body'][int(a.geom_bodyid[i])]
  found=[]
  for j in range(b.ngeom):
   if b.geom_bodyid[j]!=owner or b.geom_type[j]!=a.geom_type[i]:continue
   if not np.allclose(da.geom_xpos[i],db.geom_xpos[j],rtol=1e-6,atol=1e-6):continue
   dimensions={int(mujoco.mjtGeom.mjGEOM_PLANE):0,int(mujoco.mjtGeom.mjGEOM_SPHERE):1}.get(int(a.geom_type[i]))
   if dimensions is None:raise ValueError('Probe supports only Go2 sphere/plane collision signature')
   if dimensions and not np.allclose(a.geom_size[i,:dimensions],b.geom_size[j,:dimensions],rtol=1e-5,atol=1e-6):continue
   found.append(j)
  candidates[i]=found
 report['identity_candidates']=[{'source_geom_index':i,'target_candidates':v} for i,v in candidates.items()]
 if any(len(v)!=1 for v in candidates.values()) or len({v[0] for v in candidates.values()})!=len(active):
  report.update(status='IDENTITY_MAPPING_NOT_ESTABLISHED',interpretation='Stop before trajectory comparison; no guessing of anonymous collision identities')
 else:
  geom_map={i:v[0] for i,v in candidates.items()};inv={v:k for k,v in geom_map.items()}
  report['mapped_collision_geoms']=len(geom_map)
  report['mask_rows']=[{'source_index':i,'target_index':j,'source':[int(a.geom_contype[i]),int(a.geom_conaffinity[i])],
                       'target':[int(b.geom_contype[j]),int(b.geom_conaffinity[j])]} for i,j in geom_map.items()]
  # This observes dynamic contacts and motion; it is not a complete static-eligibility proof.
  trace=[];pairs_a=set();pairs_b=set()
  for step in range(int(round(.5/a.opt.timestep))):
   mujoco.mj_step(a,da);mujoco.mj_step(b,db);mujoco.mj_forward(a,da);mujoco.mj_forward(b,db)
   if not all(np.isfinite(v).all() for v in [da.qpos,db.qpos,da.qvel,db.qvel]):raise ValueError('Nonfinite state')
   pe=max(float(np.max(np.abs(da.xpos[i]-db.xpos[j]))) for i,j in maps['body'].items())
   ve=max(float(abs(da.qvel[i]-db.qvel[j])) for i,j in maps['dof'].items())
   for c in da.contact:pairs_a.add(tuple(sorted((int(c.geom1),int(c.geom2)))))
   for c in db.contact:pairs_b.add(tuple(sorted((inv.get(int(c.geom1),-1),inv.get(int(c.geom2),-1)))))
   trace.append({'step':step+1,'position_error_m':pe,'qvel_error':ve,'source_contacts':da.ncon,'target_contacts':db.ncon})
  report.update(status='MEASURED_NOT_QUALIFIED',trace=trace,max_position_error_m=max(r['position_error_m'] for r in trace),
                max_qvel_error=max(r['qvel_error'] for r in trace),source_contact_pairs=sorted(pairs_a),target_contact_pairs=sorted(pairs_b),
                observed_contact_pair_sets_equal=pairs_a==pairs_b,
                matches_existing_motion_thresholds=max(r['position_error_m'] for r in trace)<=.001 and max(r['qvel_error'] for r in trace)<=.01,
                qualification='Probe only: full collision-rule equivalence, multi-state identity, contract coverage and repair are not established')
except Exception as exc:report.update(status='PROBE_ERROR',error=str(exc))
after={str(p.relative_to(source.parent)):hashlib.sha256(p.read_bytes()).hexdigest() for p in source.parent.rglob('*') if p.is_file()}
report['source_files_unchanged']=before==after
report['versions']={'mujoco':mujoco.__version__,'newton':newton.__version__,'warp':wp.__version__}
(out/'stock-import.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k not in ['trace','identity_candidates','mask_rows','usd_colliders']},indent=2))
