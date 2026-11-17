"""Separate, fixed-Go2 CPU acceptance lane; never rewrites physics assets."""
import hashlib
from importlib.resources import files
import itertools
import json
from pathlib import Path
import numpy as np
import mujoco
import newton
import warp as wp
from pxr import Usd, UsdUtils
from . import engine, semantics

PROFILES=('home','raised','joint-offset','control-offset')
CONTRACT='go2-auto-sphere-plane-v1'
RULE_SOURCE='https://github.com/google-deepmind/mujoco/blob/3.17.0/src/engine/engine_collision_driver.c'


def source_bundle(source):
    source=Path(source).resolve()
    lock=json.loads(files('robot_physics_fidelity').joinpath('auto-go2-lock.json').read_text(encoding='utf-8'))
    if source.name!=lock['scene']:
        raise engine.Unsupported('Auto lane requires the pinned original Go2 scene_mjx.xml bundle')
    hashes={}
    for row in lock['files']:
        relative=Path(*Path(row['path']).parts[1:]);path=source.parent/relative
        if any(p.is_symlink() for p in [path,*path.parents]):raise engine.Unsupported('Symlink source bundles unsupported')
        if not path.is_file():raise engine.Unsupported('Pinned Go2 file missing: '+str(relative))
        value=path.read_bytes()
        git=hashlib.sha1(b'blob '+str(len(value)).encode()+b'\0'+value).hexdigest()
        if len(value)!=row['bytes'] or git!=row['git_blob_sha1']:
            raise engine.Unsupported('Source differs from pinned Go2: '+str(relative))
        hashes[relative.as_posix()]=hashlib.sha256(value).hexdigest()
    a=mujoco.MjModel.from_xml_path(str(source))
    if a.npair or a.nexclude or a.neq or a.ntendon or a.nmocap or a.nflex or a.nplugin or np.any(a.actuator_dyntype):
        raise engine.Unsupported('Fixed Go2 automatic lane excludes pairs, excludes, constraints, flex, plugins and stateful actuators')
    active=[i for i in range(a.ngeom) if int(a.geom_contype[i]) or int(a.geom_conaffinity[i])]
    if len(active)!=24 or any(int(a.geom_type[i]) not in (0,2) or int(a.geom_contype[i])!=1 or int(a.geom_conaffinity[i])!=1 for i in active):
        raise engine.Unsupported('Fixed Go2 uniform-mask sphere/plane contract mismatch')
    return a,{'commit':lock['commit'],'files_sha256':hashes,'source_signature':semantics.fingerprint(a)['sha256']}


def load_usd(path):
    path=Path(path).resolve();root=path.parent
    layers,assets,unresolved=UsdUtils.ComputeAllDependencies(str(path))
    if unresolved:raise engine.Unsupported('Unresolved USD dependencies')
    for entry in [*[l.realPath for l in layers],*assets]:
        dependency=Path(entry).resolve()
        if dependency!=root and root not in dependency.parents:
            raise engine.Unsupported('USD dependency escapes immutable artifact tree')
    stage=Usd.Stage.Open(str(path))
    if not stage:raise ValueError('Could not open USD')
    with wp.ScopedDevice('cpu'):
        builder=newton.ModelBuilder()
        newton.solvers.SolverMuJoCo.register_custom_attributes(builder)
        imported=builder.add_usd(stage,enable_self_collisions=True)
        solver=newton.solvers.SolverMuJoCo(builder.finalize(device='cpu'),use_mujoco_cpu=True,use_mujoco_contacts=True)
    shape_paths={}
    for path,shape in imported['path_shape_map'].items():shape_paths.setdefault(int(shape),[]).append(str(path))
    shape_ids=solver.mjc_geom_to_newton_shape.numpy()[0]
    paths={}
    for geom,shape in enumerate(shape_ids):
        choices=shape_paths.get(int(shape),[])
        if len(choices)!=1:raise engine.Unsupported('Target geom has no unique USD path')
        paths[geom]=choices[0]
        prim=stage.GetPrimAtPath(paths[geom])
        for name in ['mjc:contype','mjc:conaffinity']:
            attr=prim.GetAttribute(name)
            if attr and attr.HasAuthoredValue() and attr.Get()!=1:
                raise engine.Unsupported('Authored USD mask contradicts the uniform-mask Go2 source')
    return solver,paths


def inputs(a,b,maps,profile):
    if profile not in PROFILES:raise ValueError('Unknown fixed profile')
    da,db=engine.initial(a,b,maps,'home')
    free=next(i for i in range(a.njnt) if int(a.jnt_type[i])==0)
    adr=int(a.jnt_qposadr[free])
    if profile=='raised':da.qpos[adr+2]+=.06
    if profile=='joint-offset':
        for i in range(a.njnt):
            if int(a.jnt_type[i])!=0:da.qpos[int(a.jnt_qposadr[i])]+=.02*(-1 if i%2 else 1)
    if profile=='control-offset':
        da.ctrl[:]+=.015*np.array([1 if i%2 else -1 for i in range(a.nu)])
    for i,j in maps['qpos'].items():db.qpos[j]=da.qpos[i]
    for i,j in maps['dof'].items():db.qvel[j]=da.qvel[i]
    for i,j in maps['actuator'].items():db.ctrl[j]=da.ctrl[i]
    mujoco.mj_forward(a,da);mujoco.mj_forward(b,db)
    return da,db


def identities(a,b,maps,usd_paths):
    active=[i for i in range(a.ngeom) if int(a.geom_contype[i]) or int(a.geom_conaffinity[i])]
    if len(active)!=b.ngeom:raise engine.Unsupported('Collision geom count changed or unaccounted target geom')
    states=[inputs(a,b,maps,p) for p in ('home','raised','joint-offset')]
    matches={};rows=[]
    for i in active:
        options=[];t=int(a.geom_type[i])
        if t not in (0,2):raise engine.Unsupported('Only sphere/plane identity is qualified')
        for j in range(b.ngeom):
            if int(b.geom_bodyid[j])!=maps['body'][int(a.geom_bodyid[i])] or int(b.geom_type[j])!=t:continue
            if t==2 and not np.isclose(a.geom_size[i,0],b.geom_size[j,0],rtol=1e-6,atol=1e-8):continue
            if any(not np.allclose(da.geom_xpos[i],db.geom_xpos[j],rtol=0,atol=1e-6) for da,db in states):continue
            if t==0 and any(not np.allclose(da.geom_xmat[i].reshape(3,3)[:,2],db.geom_xmat[j].reshape(3,3)[:,2],rtol=0,atol=1e-6) for da,db in states):continue
            options.append(j)
        if len(options)!=1:raise engine.Unsupported('Missing/ambiguous collision identity for source index '+str(i))
        j=options[0];matches[i]=j
        rows.append({'source_geom_index':i,'source_name':mujoco.mj_id2name(a,mujoco.mjtObj.mjOBJ_GEOM,i),
                     'body':mujoco.mj_id2name(a,mujoco.mjtObj.mjOBJ_BODY,int(a.geom_bodyid[i])),
                     'target_geom_index':j,'usd_path':usd_paths[j],'geom_type':t,
                     'effective_radius_m':float(a.geom_size[i,0]) if t==2 else None})
    if len(set(matches.values()))!=len(active):raise engine.Unsupported('Non-bijective collision identity')
    return matches,rows


def eligible(m,i,j):
    """Configuration-independent rigid pair eligibility, before spatial culling."""
    if m.nflex or m.nmocap or m.npair:raise engine.Unsupported('Static rule checker excludes flex/mocap/explicit pairs')
    if int(m.opt.enableflags)&int(mujoco.mjtEnableBit.mjENBL_SLEEP):raise engine.Unsupported('Sleeping collision rules not qualified')
    if int(m.opt.disableflags)&(int(mujoco.mjtDisableBit.mjDSBL_CONTACT)|int(mujoco.mjtDisableBit.mjDSBL_CONSTRAINT)):
        return False
    if sorted([int(m.geom_type[i]),int(m.geom_type[j])]) not in ([0,2],[2,2]):return False
    x,y=int(m.geom_bodyid[i]),int(m.geom_bodyid[j])
    w1,w2=int(m.body_weldid[x]),int(m.body_weldid[y])
    if w1==w2 or (m.body_dofnum[w1]==0 and m.body_dofnum[w2]==0):return False
    p1=int(m.body_weldid[int(m.body_parentid[w1])]);p2=int(m.body_weldid[int(m.body_parentid[w2])])
    if not (int(m.opt.disableflags)&int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT)) and w1!=0 and w2!=0 and (w1==p2 or w2==p1):return False
    signature=(min(x,y)<<16)+max(x,y)
    if signature in m.exclude_signature:return False
    ct1,ca1,ct2,ca2=[int(v)&0xffffffff for v in [m.geom_contype[i],m.geom_conaffinity[i],m.geom_contype[j],m.geom_conaffinity[j]]]
    return bool((ct1&ca2) or (ct2&ca1))


def rule_table(a,b,geoms):
    rows=[]
    for i,k in itertools.combinations(sorted(geoms),2):
        left=eligible(a,i,k);right=eligible(b,geoms[i],geoms[k])
        rows.append({'source_pair':[i,k],'source_eligible':left,'target_eligible':right,'match':left==right})
    differences=[r for r in rows if not r['match']]
    return {'status':'PASS' if not differences else 'FAIL','pairs_checked':len(rows),
            'source_eligible_count':sum(r['source_eligible'] for r in rows),'differences':differences,'rows':rows,
            'scope':'Rigid sphere/plane configuration-independent eligibility before bounding volumes, positions and narrowphase; sleep/callbacks/flex/explicit pairs excluded',
            'algorithm_source':RULE_SOURCE}


def evaluate_models(a,b,usd_paths):
    if mujoco.get_mjcb_contactfilter() is not None or mujoco.get_mjcb_control() is not None:
        raise engine.Unsupported('Custom contact/control callbacks unsupported')
    if b.npair or b.neq>1 or np.any(b.eq_active0) or b.ntendon or b.nmocap or b.nflex or b.nplugin or np.any(b.actuator_dyntype):
        raise engine.Unsupported('Target introduces unsupported constraints or stateful physics')
    maps=engine.alignment(a,b)
    config={}
    for field in engine.SETTINGS:
        value=getattr(a.opt,field);setattr(b.opt,field,value)
        config[field]=np.asarray(value).tolist()
    geoms,identity=identities(a,b,maps,usd_paths)
    rules=rule_table(a,b,geoms)
    if rules['status']!='PASS':
        return {'status':'FAIL','stage':'collision_rules','rules':rules,'identity':identity,'repair_required':False}
    inverse={v:k for k,v in geoms.items()};profiles=[]
    for profile in PROFILES:
        da,db=inputs(a,b,maps,profile);peakp=peakv=0.;trace=[];contact_bad=[];seen=set();has_contact=False
        for step in range(int(round(engine.DURATION/a.opt.timestep))):
            mujoco.mj_step(a,da);mujoco.mj_step(b,db);mujoco.mj_forward(a,da);mujoco.mj_forward(b,db)
            if np.any(db.eq_active):raise engine.Unsupported('Target equality placeholder became active')
            if not all(np.isfinite(v).all() for v in [da.qpos,db.qpos,da.qvel,db.qvel]):raise ValueError('Nonfinite state')
            pe=max(float(np.max(np.abs(da.xpos[i]-db.xpos[j]))) for i,j in maps['body'].items())
            ve=max(float(abs(da.qvel[i]-db.qvel[j])) for i,j in maps['dof'].items())
            peakp=max(peakp,pe);peakv=max(peakv,ve)
            ca=sorted(tuple(sorted((int(c.geom1),int(c.geom2)))) for c in da.contact)
            cb=sorted(tuple(sorted((inverse.get(int(c.geom1),-1),inverse.get(int(c.geom2),-1)))) for c in db.contact)
            has_contact|=bool(ca and cb);seen.update(ca)
            if ca!=cb:contact_bad.append(step+1)
            trace.append({'step':step+1,'time_s':float(da.time),'position_error_m':pe,'qvel_error':ve,'source_contacts':da.ncon,'target_contacts':db.ncon})
        warnings={'source':da.warning.number.tolist(),'target':db.warning.number.tolist()}
        warning_present=any(warnings['source']) or any(warnings['target'])
        status='FAIL' if peakp>engine.POSITION_TOL or peakv>engine.VELOCITY_TOL or contact_bad or warning_present else 'PASS' if has_contact else 'INSUFFICIENT'
        profiles.append({'profile':profile,'status':status,'max_position_error_m':peakp,'max_qvel_error':peakv,
                         'contact_mismatch_steps':contact_bad,'warnings':warnings,'observed_pairs':sorted(seen),'trace':trace})
    status='FAIL' if any(p['status']=='FAIL' for p in profiles) else 'INSUFFICIENT' if any(p['status']=='INSUFFICIENT' for p in profiles) else 'PASS'
    return {'status':status,'contract':CONTRACT,'rules':rules,'identity':identity,'profiles':profiles,
            'repair_required':False,'asset_changes':[],
            'interpretation':'No asset repair performed. PASS qualifies only this pinned Go2 export under the recorded CPU runtime configuration and four fixed profiles.',
            'thresholds':{'duration_s':engine.DURATION,'position_m':engine.POSITION_TOL,'qvel_native_units':engine.VELOCITY_TOL},
            'runtime_configuration':config,'requires_recorded_runtime_configuration':True,
            'target_inactive_equality_placeholders':int(b.neq),'required_default_equality_active':b.eq_active0.tolist()}


def validate(source,usd):
    a,bundle=source_bundle(source)
    solver,paths=load_usd(usd)
    result=evaluate_models(a,solver.mj_model,paths)
    _,after=source_bundle(source)
    if bundle!=after:raise ValueError('Source changed during verification')
    result.update(source_bundle=bundle,candidate_activated=False)
    return result
