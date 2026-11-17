"""Two fixed real-asset qualification profiles; v0.1 public API guards stay unchanged."""
import json, pathlib, time, traceback, sys, argparse, math
import numpy as np
import mujoco, newton, warp as wp
import mujoco_usd_converter
ROOT=pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'outputs/usd-contact-pilot'))
import contact_bridge as bridge
from pxr import Usd, UsdPhysics, Sdf

def run():
    parser=argparse.ArgumentParser()
    parser.add_argument('--asset',choices=['apollo','g1'],required=True)
    parser.add_argument('--keyframe',required=True)
    parser.add_argument('--explicit-joint-limits',action='store_true')
    args=parser.parse_args()
    source_dir=ROOT/'work'/('apollo-original' if args.asset=='apollo' else 'g1-original')
    scene=source_dir/('scene.xml' if args.asset=='apollo' else 'scene_mjx.xml')
    out=ROOT/'outputs/cross-asset-results'/(args.asset+'-'+args.keyframe+'-'+str(time.time_ns()))
    out.mkdir(parents=True)
    native=mujoco.MjModel.from_xml_path(str(scene))
    steps=math.ceil(.5/native.opt.timestep)
    explicit_limits=args.explicit_joint_limits
    result={'scope':'Qualified pair-only moving bodies; static world masks retained; native CPU reconstruction', 'asset':args.asset, 'keyframe':args.keyframe, 'duration_s':steps*native.opt.timestep,
            'explicit_joint_limit_fix':explicit_limits,
            'thresholds':{'body_position_m':.001,'qvel':.01,'pair_rtol':1e-6,'steps':steps},'passed':False}
    try:
        assert all(native.geom_bodyid[i]==0 or (native.geom_contype[i]==0 and native.geom_conaffinity[i]==0) for i in range(native.ngeom)), 'Moving-body automatic masks are not qualified'
        asset=mujoco_usd_converter.Converter().convert(str(scene),str(out/'official'))
        stage=Usd.Stage.Open(Usd.Stage.Open(asset.path).Flatten())
        deinstanced=[]
        while True:
            instances=[p for p in stage.Traverse() if p.IsInstance()]
            if not instances: break
            for p in instances:
                if p.GetTypeName() not in ('Mesh','Material'): raise ValueError('Physical or unknown instance is outside Apollo lane')
                deinstanced.append(str(p.GetPath())); p.SetInstanceable(False)
        result['nonphysical_instances_expanded']=deinstanced
        root=stage.GetDefaultPrim()
        if explicit_limits:
            authored=[]
            for i in range(native.njnt):
                if native.jnt_type[i]==mujoco.mjtJoint.mjJNT_FREE: continue
                name=mujoco.mj_id2name(native,mujoco.mjtObj.mjOBJ_JOINT,i)
                matches=[p for p in stage.Traverse() if p.GetName()==name and p.GetTypeName()=='PhysicsRevoluteJoint']
                if len(matches)!=1: raise ValueError('Ambiguous joint limit source '+name)
                p=matches[0]; a=p.GetAttribute('mjc:solreflimit')
                authored.append({'joint':name,'had_authored_value':a.HasAuthoredValue(),'fallback':list(a.Get()),'source':native.jnt_solref[i].tolist()})
                a.Set(native.jnt_solref[i].tolist())
            result['explicit_limit_authoring']=authored
        bridge.attr(root,'formatVersion',Sdf.ValueTypeNames.Int,1)
        endpoints=set(native.pair_geom1)|set(native.pair_geom2)
        mapping={}
        for i in endpoints:
            name=mujoco.mj_id2name(native,mujoco.mjtObj.mjOBJ_GEOM,int(i))
            matches=[p for p in stage.Traverse() if p.GetName()==name and p.GetTypeName() in ('Cube','Sphere','Capsule','Cylinder','Plane')]
            if len(matches)!=1: raise ValueError('Ambiguous pair endpoint '+str(name))
            p=matches[0]; mapping[int(i)]=p.GetPath()
            UsdPhysics.CollisionAPI.Apply(p).CreateCollisionEnabledAttr(True)
            bridge.attr(p,'contype',Sdf.ValueTypeNames.Int64,int(native.geom_contype[i]))
            bridge.attr(p,'conaffinity',Sdf.ValueTypeNames.Int64,int(native.geom_conaffinity[i]))
        for i in range(native.npair):
            p=stage.DefinePrim(root.GetPath().AppendChild('ContactBridgePairs').AppendChild('pair_'+str(i)),'Scope')
            bridge.attr(p,'version',Sdf.ValueTypeNames.Int,1)
            for end in [1,2]:
                p.CreateRelationship(bridge.PREFIX+'geom'+str(end)).SetTargets([mapping[int(getattr(native,'pair_geom'+str(end))[i])]])
            for field,(array,typ,length) in bridge.FIELDS.items():
                v=getattr(native,array)[i]
                bridge.attr(p,field,typ,v.tolist() if length else int(v) if field=='condim' else float(v))
        path=out/'apollo-contact.usda'; stage.GetRootLayer().Export(str(path))
        with wp.ScopedDevice('cpu'):
            b=newton.ModelBuilder(); usdmap=bridge.import_asset(b,path)
            solver=newton.solvers.SolverMuJoCo(b.finalize(device='cpu'),use_mujoco_cpu=True,use_mujoco_contacts=True)
            rebuilt=solver.mj_model
            if explicit_limits:
                # The repair must survive USD import, not a post-load mjModel edit.
                for i in range(native.njnt):
                    if native.jnt_type[i]==0: continue
                    name=mujoco.mj_id2name(native,mujoco.mjtObj.mjOBJ_JOINT,i)
                    found=[j for j in range(rebuilt.njnt) if (mujoco.mj_id2name(rebuilt,mujoco.mjtObj.mjOBJ_JOINT,j) or '').endswith('_'+name)]
                    assert len(found)==1
                    # Newton custom attributes store this vector in float32.
                    np.testing.assert_allclose(native.jnt_solref[i].astype(np.float32),rebuilt.jnt_solref[found[0]],rtol=0,atol=1e-10)
            assert rebuilt.npair==native.npair
            def align(kind,n):
                m={}
                for i in range(getattr(native,n)):
                    name=mujoco.mj_id2name(native,kind,i) or '__unnamed_free_joint__'
                    found=[j for j in range(getattr(rebuilt,n)) if (mujoco.mj_id2name(rebuilt,kind,j) or '').endswith('_'+name) or mujoco.mj_id2name(rebuilt,kind,j)==name]
                    if kind==mujoco.mjtObj.mjOBJ_JOINT and native.jnt_type[i]==0:
                        found=[j for j in range(rebuilt.njnt) if rebuilt.jnt_type[j]==0]
                    if len(found)!=1: raise ValueError('Identity '+str(name))
                    m[i]=found[0]
                assert len(m)==len(set(m.values()))
                return m
            joints=align(mujoco.mjtObj.mjOBJ_JOINT,'njnt'); bodies=align(mujoco.mjtObj.mjOBJ_BODY,'nbody')
            geom_map={}
            for i in endpoints:
                shape=usdmap['path_shape_map'][str(mapping[int(i)])]
                # Pinned CPU solver labels geoms with the USD shape label and builder shape id.
                expected=str(mapping[int(i)])+'_'+str(shape)
                geom=mujoco.mj_name2id(rebuilt,mujoco.mjtObj.mjOBJ_GEOM,expected)
                assert 0<=geom<rebuilt.ngeom; geom_map[int(i)]=geom
            pair_map={}
            for i in range(native.npair):
                want={geom_map[int(native.pair_geom1[i])],geom_map[int(native.pair_geom2[i])]}
                found=[j for j in range(rebuilt.npair) if {int(rebuilt.pair_geom1[j]),int(rebuilt.pair_geom2[j])}==want]
                assert len(found)==1; pair_map[i]=found[0]
                for _,(array,_,_) in bridge.FIELDS.items():
                    np.testing.assert_allclose(getattr(native,array)[i],getattr(rebuilt,array)[found[0]],rtol=1e-6,atol=1e-8)
            # Explicitly match solver settings; pair portability is the intervention.
            settings=['timestep','integrator','iterations','ls_iterations','solver','cone','jacobian','tolerance','ls_tolerance','noslip_iterations','impratio','gravity','disableflags','enableflags']
            for setting in settings: setattr(rebuilt.opt,setting,getattr(native.opt,setting))
            manual_b=newton.ModelBuilder()
            newton.solvers.SolverMuJoCo.register_custom_attributes(manual_b)
            manual_paths=manual_b.add_usd(stage)['path_shape_map']
            for i,p in mapping.items(): manual_b.shape_collision_group[manual_paths[str(p)]]=0
            for i in range(native.npair):
                values={f:(getattr(native,a)[i].tolist() if length else int(getattr(native,a)[i]) if f=='condim' else float(getattr(native,a)[i])) for f,(a,_,length) in bridge.FIELDS.items()}
                newton.solvers.mujoco.add_contact_pair(manual_b,manual_paths[str(mapping[int(native.pair_geom1[i])])],manual_paths[str(mapping[int(native.pair_geom2[i])])],**values)
            manual_solver=newton.solvers.SolverMuJoCo(manual_b.finalize(device='cpu'),use_mujoco_cpu=True)
            manual=manual_solver.mj_model
            for setting in settings: setattr(manual.opt,setting,getattr(native.opt,setting))
            for f in ['body_mass','body_inertia','body_pos','body_quat','body_iquat','geom_size','geom_pos','geom_quat','pair_friction','pair_solref','pair_solimp','actuator_gainprm']:
                np.testing.assert_allclose(getattr(manual,f),getattr(rebuilt,f),rtol=1e-8,atol=1e-10)
            da=mujoco.MjData(native); db=mujoco.MjData(rebuilt)
            key=mujoco.mj_name2id(native,mujoco.mjtObj.mjOBJ_KEY,args.keyframe); assert key>=0; mujoco.mj_resetDataKeyframe(native,da,key)
            qvelmap={}
            for i,j in joints.items():
                nq=7 if native.jnt_type[i]==0 else 1; nv=6 if nq==7 else 1
                db.qpos[rebuilt.jnt_qposadr[j]:rebuilt.jnt_qposadr[j]+nq]=da.qpos[native.jnt_qposadr[i]:native.jnt_qposadr[i]+nq]
                qvelmap.update({int(native.jnt_dofadr[i]+k):int(rebuilt.jnt_dofadr[j]+k) for k in range(nv)})
            for i in range(native.nu):
                target=joints[int(native.actuator_trnid[i,0])]
                found=[j for j in range(rebuilt.nu) if rebuilt.actuator_trnid[j,0]==target]
                assert len(found)==1; db.ctrl[found[0]]=da.ctrl[i]
            dm=mujoco.MjData(manual); dm.qpos[:]=db.qpos; dm.ctrl[:]=db.ctrl
            manual_vs_bridge=0.
            max_pos=max_vel=0.; nc_a=nc_b=0; contact_pairs=set(); trajectories=[]
            reverse_geom={v:k for k,v in geom_map.items()}
            allowed={frozenset((int(native.pair_geom1[i]),int(native.pair_geom2[i]))) for i in range(native.npair)}
            for step in range(steps):
                mujoco.mj_step(native,da); mujoco.mj_step(rebuilt,db); mujoco.mj_step(manual,dm)
                manual_vs_bridge=max(manual_vs_bridge,float(np.max(abs(dm.qpos-db.qpos))))
                mujoco.mj_forward(native,da); mujoco.mj_forward(rebuilt,db)
                pe=max(float(np.max(abs(da.xpos[i]-db.xpos[j]))) for i,j in bodies.items())
                ve=max(float(abs(da.qvel[i]-db.qvel[j])) for i,j in qvelmap.items())
                max_pos=max(max_pos,pe); max_vel=max(max_vel,ve); nc_a=max(nc_a,da.ncon); nc_b=max(nc_b,db.ncon)
                for k in range(db.ncon):
                    pair=frozenset((reverse_geom[int(db.contact[k].geom1)],reverse_geom[int(db.contact[k].geom2)]))
                    assert pair in allowed; contact_pairs.add(tuple(sorted(pair)))
                trajectories.append({'step':step,'max_body_position_error_m':pe,'max_qvel_error':ve,'native_contacts':da.ncon,'rebuilt_contacts':db.ncon})
            result.update(native_pairs=native.npair,rebuilt_pairs=rebuilt.npair,restored_endpoints=len(endpoints),
                max_body_position_error_m=max_pos,max_qvel_error=max_vel,native_peak_contacts=nc_a,rebuilt_peak_contacts=nc_b,
                extra_contact_pairs=0,observed_pair_count=len(contact_pairs),settings_matched=settings)
            result['manual_correct_configuration']={'compiled_parameters_match':True,'max_qpos_difference_vs_bridge':manual_vs_bridge}
            result['observed_pairs']=[[mujoco.mj_id2name(native,mujoco.mjtObj.mjOBJ_GEOM,i) for i in pair] for pair in sorted(contact_pairs)]
            result['native_final_root_z']=float(da.qpos[0+2])
            result['rebuilt_final_root_z']=float(db.qpos[int(rebuilt.jnt_qposadr[joints[0]])+2])
            (out/'trajectory.json').write_text(json.dumps(trajectories,indent=2),encoding='utf-8')
            assert max_pos<=.001 and max_vel<=.01 and nc_a>0 and nc_b>0
            result['passed']=True
    except Exception: result['error']=traceback.format_exc()
    (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2)); print('OUTPUT='+str(out))
    return 0 if result['passed'] else 2

if __name__=='__main__': raise SystemExit(run())
