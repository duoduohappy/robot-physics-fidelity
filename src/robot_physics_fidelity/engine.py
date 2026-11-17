"""Qualified CPU lane and evidence generation. No workspace-specific paths."""
from pathlib import Path
import hashlib
import math
import numpy as np
import mujoco
import newton
import warp as wp
import mujoco_usd_converter
from pxr import Usd, UsdPhysics
from . import contract

DURATION = .5
POSITION_TOL = .001
VELOCITY_TOL = .01
SETTINGS = ['timestep','integrator','iterations','ls_iterations','solver','cone','jacobian','tolerance','ls_tolerance','noslip_iterations','impratio','gravity','disableflags','enableflags']

class Unsupported(ValueError):
    pass

def digest_model(m):
    from .semantics import fingerprint
    return fingerprint(m)['sha256']


def legacy_selected_array_digest(m):
    h = hashlib.sha256()
    for field in ['names','body_pos','body_quat','body_mass','body_inertia','body_iquat','body_ipos','geom_type','geom_size','geom_pos','geom_quat','geom_contype','geom_conaffinity','mesh_vert','mesh_face','jnt_type','jnt_axis','jnt_range','jnt_solref','jnt_solimp','dof_damping','dof_armature','dof_frictionloss','pair_geom1','pair_geom2','pair_friction','pair_solref','pair_solimp','pair_margin','pair_gap','actuator_gainprm','actuator_biasprm','actuator_ctrlrange','key_qpos','key_ctrl']:
        value = getattr(m,field)
        h.update(field.encode())
        h.update(value if isinstance(value,bytes) else np.asarray(value).tobytes())
    for field in SETTINGS:
        h.update(field.encode()); h.update(np.asarray(getattr(m.opt,field)).tobytes())
    return h.hexdigest()

def preflight(source, keyframe):
    m = mujoco.MjModel.from_xml_path(str(Path(source).resolve()))
    reasons=[]
    if m.ngeom>256 or m.nbody>256: reasons.append('At most 256 geoms and bodies')
    if m.neq or m.ntendon or m.nmocap: reasons.append('Equalities, tendons and mocap are not qualified')
    if sum(m.jnt_type==0)!=1 or any(t not in (0,3) for t in m.jnt_type): reasons.append('Exactly one free root and named hinge joints required')
    if any(m.geom_bodyid[i]!=0 and (m.geom_contype[i] or m.geom_conaffinity[i]) for i in range(m.ngeom)): reasons.append('Moving-body automatic masks must be zero')
    if not m.npair: reasons.append('At least one explicit pair required')
    ends=set(m.pair_geom1)|set(m.pair_geom2)
    if any(m.geom_bodyid[i]==0 and (m.geom_contype[i] or m.geom_conaffinity[i]) and i not in ends for i in range(m.ngeom)): reasons.append('Static automatic collider not represented by pair contract')
    primitives={int(mujoco.mjtGeom.mjGEOM_PLANE),int(mujoco.mjtGeom.mjGEOM_SPHERE),int(mujoco.mjtGeom.mjGEOM_BOX),int(mujoco.mjtGeom.mjGEOM_CAPSULE),int(mujoco.mjtGeom.mjGEOM_CYLINDER)}
    if any(int(m.geom_type[i]) not in primitives or not mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,int(i)) for i in ends): reasons.append('Pair endpoints must be named primitive geoms')
    if any(int(t)!=int(mujoco.mjtTrn.mjTRN_JOINT) for t in m.actuator_trntype): reasons.append('Only joint-transmission actuators supported')
    if np.any(m.actuator_dyntype): reasons.append('Stateful actuator dynamics are not qualified')
    key=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_KEY,keyframe)
    if key<0: reasons.append('Named source keyframe not found')
    if not 0<m.opt.timestep<=.01: reasons.append('Source timestep outside (0,.01]')
    if reasons: raise Unsupported('; '.join(reasons))
    return m

def primitive(stage,name):
    matches=[p for p in stage.Traverse() if p.GetName()==name and p.GetTypeName() in ('Cube','Sphere','Capsule','Cylinder','Plane')]
    if len(matches)!=1: raise Unsupported('Ambiguous geom identity: '+str(name))
    return matches[0]

def hinge(stage,name):
    matches=[p for p in stage.Traverse() if p.GetName()==name and p.GetTypeName()=='PhysicsRevoluteJoint']
    if len(matches)!=1: raise Unsupported('Ambiguous hinge identity: '+str(name))
    return matches[0]

def operations(m,stage):
    result=[]
    for p in stage.Traverse():
        if p.IsInstance():
            if p.GetTypeName() not in ('Mesh','Material') or p.HasAPI(UsdPhysics.CollisionAPI): raise Unsupported('Physical/unknown native instances unsupported')
            result.append({'kind':'prepare_nonphysical_instance','path':str(p.GetPath()),'type':p.GetTypeName()})
    for i in sorted(set(m.pair_geom1)|set(m.pair_geom2)):
        name=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,int(i)); p=primitive(stage,name)
        result.append({'kind':'retain_pair_endpoint','source_geom':name,'path':str(p.GetPath()),'had_collision_api':p.HasAPI(UsdPhysics.CollisionAPI)})
    for i in range(m.npair):
        result.append({'kind':'preserve_explicit_pair','pair_index':i,'endpoints':[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,int(getattr(m,'pair_geom'+str(e))[i])) for e in (1,2)],'parameters':{f:(getattr(m,a)[i].tolist() if n else int(getattr(m,a)[i]) if f=='condim' else float(getattr(m,a)[i])) for f,(a,_,n) in contract.FIELDS.items()}})
    for i in range(m.njnt):
        if m.jnt_type[i]==0: continue
        name=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_JOINT,i); p=hinge(stage,name); a=p.GetAttribute('mjc:solreflimit')
        result.append({'kind':'author_raw_joint_limit','joint':name,'path':str(p.GetPath()),'had_authored_value':a.HasAuthoredValue(),'source':m.jnt_solref[i].tolist()})
    return result

def convert(source,folder):
    return Path(mujoco_usd_converter.Converter().convert(str(Path(source).resolve()),str(folder)).path)

def apply(m,original,target):
    stage=Usd.Stage.Open(Usd.Stage.Open(str(original)).Flatten())
    while True:
        instances=[p for p in stage.Traverse() if p.IsInstance()]
        if not instances: break
        for p in instances:
            if p.GetTypeName() not in ('Mesh','Material') or p.HasAPI(UsdPhysics.CollisionAPI): raise Unsupported('Physical/unknown native instances unsupported')
            p.SetInstanceable(False)
    root=stage.GetDefaultPrim()
    contract.attr(root,'formatVersion',contract.Sdf.ValueTypeNames.Int,1)
    paths={}
    for i in set(m.pair_geom1)|set(m.pair_geom2):
        p=primitive(stage,mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,int(i)))
        paths[int(i)]=p.GetPath()
        UsdPhysics.CollisionAPI.Apply(p).CreateCollisionEnabledAttr(True)
        contract.attr(p,'contype',contract.Sdf.ValueTypeNames.Int64,int(m.geom_contype[i]))
        contract.attr(p,'conaffinity',contract.Sdf.ValueTypeNames.Int64,int(m.geom_conaffinity[i]))
    for i in range(m.npair):
        p=stage.DefinePrim(root.GetPath().AppendChild('ContactBridgePairs').AppendChild('pair_'+str(i)),'Scope')
        contract.attr(p,'version',contract.Sdf.ValueTypeNames.Int,1)
        for e in (1,2): p.CreateRelationship(contract.PREFIX+'geom'+str(e)).SetTargets([paths[int(getattr(m,'pair_geom'+str(e))[i])]])
        for f,(a,t,n) in contract.FIELDS.items():
            v=getattr(m,a)[i]; contract.attr(p,f,t,v.tolist() if n else int(v) if f=='condim' else float(v))
    for i in range(m.njnt):
        if m.jnt_type[i]!=0: hinge(stage,mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_JOINT,i)).GetAttribute('mjc:solreflimit').Set(m.jnt_solref[i].tolist())
    stage.GetRootLayer().Export(str(target))

def load_target(path):
    with wp.ScopedDevice('cpu'):
        b=newton.ModelBuilder(); contract.import_asset(b,path)
        solver=newton.solvers.SolverMuJoCo(b.finalize(device='cpu'),use_mujoco_cpu=True,use_mujoco_contacts=True)
    return solver

def alignment(a,b):
    result={}
    for label,kind,count in [('joint',mujoco.mjtObj.mjOBJ_JOINT,'njnt'),('body',mujoco.mjtObj.mjOBJ_BODY,'nbody')]:
        mapping={}
        if getattr(a,count)!=getattr(b,count): raise Unsupported('Entity count changed: '+label)
        for i in range(getattr(a,count)):
            name=mujoco.mj_id2name(a,kind,i)
            if label=='joint' and a.jnt_type[i]==0: found=[j for j in range(b.njnt) if b.jnt_type[j]==0]
            else: found=[j for j in range(getattr(b,count)) if mujoco.mj_id2name(b,kind,j)==name or (name and (mujoco.mj_id2name(b,kind,j) or '').endswith('_'+name))]
            if len(found)!=1: raise Unsupported('Ambiguous '+label+' identity: '+str(name))
            mapping[i]=found[0]
        if len(set(mapping.values()))!=len(mapping): raise Unsupported('Non-bijective identity mapping')
        result[label]=mapping
    acts={}
    for i in range(a.nu):
        found=[j for j in range(b.nu) if int(b.actuator_trnid[j,0])==result['joint'][int(a.actuator_trnid[i,0])]]
        if len(found)!=1: raise Unsupported('Ambiguous actuator target')
        acts[i]=found[0]
    if a.nu!=b.nu or len(set(acts.values()))!=a.nu: raise Unsupported('Actuator mapping changed')
    result['actuator']=acts; result['qpos']={}; result['dof']={}
    for i,j in result['joint'].items():
        if a.jnt_type[i]!=b.jnt_type[j]: raise Unsupported('Joint type changed')
        free=a.jnt_type[i]==0
        result['qpos'].update({int(a.jnt_qposadr[i]+k):int(b.jnt_qposadr[j]+k) for k in range(7 if free else 1)})
        result['dof'].update({int(a.jnt_dofadr[i]+k):int(b.jnt_dofadr[j]+k) for k in range(6 if free else 1)})
    result['geom']={}
    for i in set(a.pair_geom1)|set(a.pair_geom2):
        name=mujoco.mj_id2name(a,mujoco.mjtObj.mjOBJ_GEOM,int(i))
        exact=[j for j in range(b.ngeom) if mujoco.mj_id2name(b,mujoco.mjtObj.mjOBJ_GEOM,j)==name]
        found=exact or [j for j in range(b.ngeom) if (mujoco.mj_id2name(b,mujoco.mjtObj.mjOBJ_GEOM,j) or '').rsplit('/',1)[-1].rsplit('_',1)[0]==name]
        if len(found)!=1: raise Unsupported('Required endpoint was lost: '+name)
        result['geom'][int(i)]=found[0]
    return result

def initial(a,b,maps,key):
    da=mujoco.MjData(a); db=mujoco.MjData(b)
    mujoco.mj_resetDataKeyframe(a,da,mujoco.mj_name2id(a,mujoco.mjtObj.mjOBJ_KEY,key))
    for i,j in maps['qpos'].items(): db.qpos[j]=da.qpos[i]
    for i,j in maps['dof'].items(): db.qvel[j]=da.qvel[i]
    for i,j in maps['actuator'].items(): db.ctrl[j]=da.ctrl[i]
    return da,db

def contact_rows(m,d,geom_reverse=None,source_model=None):
    rows=[]
    for k in range(d.ncon):
        c=d.contact[k]; pair=[int(c.geom1),int(c.geom2)]
        if geom_reverse is not None: pair=[geom_reverse.get(i,-1) for i in pair]
        force=np.empty(6); mujoco.mj_contactForce(m,d,k,force)
        names=source_model if source_model is not None else m
        rows.append({'geom_ids':pair,'geom_names':[mujoco.mj_id2name(names,mujoco.mjtObj.mjOBJ_GEOM,i) if i>=0 else 'UNKNOWN' for i in pair],'position_m':c.pos.tolist(),'normal':c.frame[:3].tolist(),'distance_m':float(c.dist),'dimension':int(c.dim),'force_local':force.tolist()})
    return rows

def match_contacts(source,target):
    remaining=list(range(len(target))); matches=[]
    for left in source:
        eligible=[j for j in remaining if sorted(target[j]['geom_ids'])==sorted(left['geom_ids'])]
        if not eligible: return {'matched':False,'reason':'contact pair or contact count differs'}
        distances=sorted((float(np.linalg.norm(np.array(left['position_m'])-target[j]['position_m'])),j) for j in eligible)
        distance,j=distances[0]; right=target[j]; remaining.remove(j)
        n1=np.array(left['normal'])*(1 if left['geom_ids'][0]<left['geom_ids'][1] else -1)
        n2=np.array(right['normal'])*(1 if right['geom_ids'][0]<right['geom_ids'][1] else -1)
        matches.append({'pair':left['geom_names'],'point_distance_m':distance,'normal_angle_rad':float(np.arccos(np.clip(np.dot(n1,n2),-1,1))),'distance_error_m':abs(left['distance_m']-right['distance_m']),'normal_force_difference':abs(left['force_local'][0]-right['force_local'][0]),'ambiguous_nearest_match':len(distances)>1 and abs(distances[0][0]-distances[1][0])<1e-9})
    return {'matched':not remaining,'matching_method':'nearest point within each geom pair; not a formal contact identity proof','matches':matches}

def evaluate(a,b,key,details=True):
    maps=alignment(a,b)
    for f in SETTINGS: setattr(b.opt,f,getattr(a.opt,f))
    da,db=initial(a,b,maps,key)
    allowed={frozenset((int(a.pair_geom1[i]),int(a.pair_geom2[i]))) for i in range(a.npair)}
    pair_check=[]
    limit_check=all(np.allclose(a.jnt_solref[i],b.jnt_solref[j],rtol=1e-6,atol=1e-8) for i,j in maps['joint'].items() if a.jnt_type[i]!=0)
    for i in range(a.npair):
        expected={maps['geom'][int(a.pair_geom1[i])],maps['geom'][int(a.pair_geom2[i])]}
        found=[j for j in range(b.npair) if {int(b.pair_geom1[j]),int(b.pair_geom2[j])}==expected]
        ok=len(found)==1 and all(np.allclose(getattr(a,field)[i],getattr(b,field)[found[0]],rtol=1e-6,atol=1e-8) for field,_,_ in contract.FIELDS.values())
        pair_check.append(bool(ok))
    inv={v:k for k,v in maps['geom'].items()}
    first=None; trace=[]; snapshots=[]; peak_p=peak_v=0.; extra=set(); observed=set(); contacts_a=contacts_b=0
    dof_names={int(a.jnt_dofadr[i]+k):mujoco.mj_id2name(a,mujoco.mjtObj.mjOBJ_JOINT,i) or 'free_root' for i in range(a.njnt) for k in range(6 if a.jnt_type[i]==0 else 1)}
    for step in range(math.ceil(DURATION/a.opt.timestep)):
        mujoco.mj_step(a,da); mujoco.mj_step(b,db)
        mujoco.mj_forward(a,da); mujoco.mj_forward(b,db)
        if not all(np.isfinite(v).all() for v in [da.qpos,db.qpos,da.qvel,db.qvel]): raise ValueError('Non-finite physics state')
        worst_body=max(maps['body'],key=lambda i:np.max(abs(da.xpos[i]-db.xpos[maps['body'][i]])))
        worst_dof=max(maps['dof'],key=lambda i:abs(da.qvel[i]-db.qvel[maps['dof'][i]]))
        pe=float(np.max(abs(da.xpos[worst_body]-db.xpos[maps['body'][worst_body]])))
        ve=float(abs(da.qvel[worst_dof]-db.qvel[maps['dof'][worst_dof]]))
        peak_p=max(peak_p,pe); peak_v=max(peak_v,ve); contacts_a=max(contacts_a,da.ncon); contacts_b=max(contacts_b,db.ncon)
        for k in range(db.ncon):
            p=frozenset(inv.get(int(v),-1) for v in [db.contact[k].geom1,db.contact[k].geom2])
            observed.add(tuple(sorted(p)))
            if p not in allowed: extra.add(tuple(sorted(p)))
        row={'step':step+1,'time_s':float(da.time),'position_error_m':pe,'qvel_error':ve,'worst_body':mujoco.mj_id2name(a,mujoco.mjtObj.mjOBJ_BODY,worst_body),'worst_joint':dof_names[worst_dof],'source_contacts':da.ncon,'target_contacts':db.ncon}
        exceeded=pe>POSITION_TOL or ve>VELOCITY_TOL
        if first is None and exceeded:
            first=dict(row)
            if details:
                same=mujoco.MjData(b)
                for i,j in maps['qpos'].items(): same.qpos[j]=da.qpos[i]
                for i,j in maps['dof'].items(): same.qvel[j]=da.qvel[i]
                for i,j in maps['dof'].items(): same.qacc_warmstart[j]=da.qacc_warmstart[i]
                for i,j in maps['actuator'].items(): same.ctrl[j]=da.ctrl[i]
                same.time=da.time
                mujoco.mj_forward(b,same)
                first['same_state_max_qacc_error']=max(float(abs(da.qacc[i]-same.qacc[j])) for i,j in maps['dof'].items())
                first['same_state_fields_matched']=['qpos','qvel','ctrl','qacc_warmstart','time']
                left=contact_rows(a,da); same_rows=contact_rows(b,same,inv,a)
                snapshots.append({'event':'first_threshold_exceedance','source':left,'target':contact_rows(b,db,inv,a),'same_state_target':same_rows,'same_state_contact_comparison':match_contacts(left,same_rows)})
        if details: trace.append(row)
    status='PASS' if first is None and not extra and all(pair_check) and a.npair==b.npair and limit_check else 'FAIL'
    if status=='PASS' and (not contacts_a or not contacts_b): status='INSUFFICIENT'
    return {'status':status,'thresholds':{'position_m':POSITION_TOL,'qvel_native_units':VELOCITY_TOL,'duration_s':DURATION},'max_body_position_error_m':peak_p,'max_qvel_error':peak_v,'first_divergence':first,'pair_parameters_pass':all(pair_check) and a.npair==b.npair,'joint_limit_parameters_pass':limit_check,'configured_pairs':int(a.npair),'observed_pair_count':len(observed),'extra_contact_pairs':[list(p) for p in sorted(extra)],'source_peak_contacts':contacts_a,'target_peak_contacts':contacts_b,'trace':trace,'contact_snapshots':snapshots,'settings_explicitly_matched':SETTINGS,'scope':'fixed source/keyframe/controls; reconstructed native CPU model; not general qualification'}

def diagnose(a,b,key):
    base=evaluate(a,b,key)
    maps=alignment(a,b); differences=[]; probes=[]
    for field in ['jnt_solref','jnt_solimp','jnt_actfrcrange']:
        rows=[]
        for i,j in maps['joint'].items():
            if a.jnt_type[i]==0: continue
            if not np.allclose(getattr(a,field)[i],getattr(b,field)[j],rtol=1e-5,atol=1e-7):
                rows.append({'joint':mujoco.mj_id2name(a,mujoco.mjtObj.mjOBJ_JOINT,i),'source':getattr(a,field)[i].tolist(),'target':getattr(b,field)[j].tolist()})
        if not rows: continue
        differences.append({'field':field,'differences':rows})
        saved=getattr(b,field).copy()
        try:
            for i,j in maps['joint'].items(): getattr(b,field)[j]=getattr(a,field)[i]
            probe=evaluate(a,b,key,details=False)
            probes.append({'field':field,'status':probe['status'],'max_body_position_error_m':probe['max_body_position_error_m'],'max_qvel_error':probe['max_qvel_error'],'intervention':'temporary native-array diagnostic only; not an asset repair'})
        finally: getattr(b,field)[:]=saved
    confirmed=[p['field'] for p in probes if base['status']=='FAIL' and p['status']=='PASS']
    base['diagnosis']={'certainty':'CONFIRMED_WITHIN_TEST' if confirmed else 'NO_FAILURE' if base['status']=='PASS' else 'UNRESOLVED','confirmed_fields':confirmed,'parameter_candidates':differences,'controlled_probes':probes,'interpretation':'Different arrays alone are not a cause. Only a bounded single-field intervention that changes FAIL to PASS is confirmed within this test. No claim of a unique global root cause.'}
    return base
