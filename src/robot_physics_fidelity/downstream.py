"""Gated, real CPU load/step adapter. No simulator object before admission."""
import hashlib
from pathlib import Path
import shutil
import numpy as np
import mujoco
from . import auto_transactions as at,automatic as auto,engine,transactions as tx


class Blocked(ValueError):
    def __init__(self,code,message):self.code=code;super().__init__(message)


def lease(workspace):
    workspace=Path(workspace)
    selection=tx.read(workspace/'auto-active.json')
    names=['auto-plan.json','auto-active.json']
    if selection.get('selection')=='verified_original':
        names.append(str(tx.under(workspace,selection['verification']).relative_to(workspace)))
    return {name:hashlib.sha256((workspace/name).read_bytes()).hexdigest() for name in names}


def ensure_lease(workspace,expected):
    if lease(workspace)!=expected:
        raise Blocked('AUTHORIZATION_CHANGED','认证已撤销或回执发生变化，停止继续推进仿真。')


def run(workspace,output,profile='home'):
    if profile not in auto.PROFILES:raise ValueError('Only certified fixed profiles are supported')
    workspace=Path(workspace).resolve();output=Path(output).resolve()
    # Output evidence may never be nested in a managed immutable workspace.
    if workspace==output or workspace in output.parents or output in workspace.parents:
        raise ValueError('Job output and managed workspace must be separate directory trees')
    output.mkdir(parents=True,exist_ok=False)
    receipt={'status':'BLOCKED','profile':profile,'loader_called':False,'model_created':False,'steps_executed':0,
             'gate_checked':False,'events':[],'frames':[],
             'scope':'Fixed Go2 native CPU load and stepping; not a robot controller, hardware trial or generic simulator plugin'}
    def event(name):receipt['events'].append({'event':name,'steps_executed':receipt['steps_executed']})
    try:
        authorization=lease(workspace)
        selected=at.active_path(workspace);receipt['gate_checked']=True;event('qualification_checked')
        if not selected['qualified']:
            raise Blocked('NOT_QUALIFIED','当前资产未认证或认证已撤销；下游加载器未调用。')
        ensure_lease(workspace,authorization)
        if mujoco.get_mjcb_contactfilter() is not None or mujoco.get_mjcb_control() is not None:
            raise Blocked('UNSUPPORTED_CALLBACK','固定验收配置不允许自定义接触或控制回调。')
        plan=tx.read(workspace/'auto-plan.json')
        snapshot=output/'frozen-asset'
        shutil.copytree(workspace/'original',snapshot)
        digest=tx.tree_hash(snapshot)
        if digest!=plan['original_tree_sha256']:raise Blocked('SNAPSHOT_MISMATCH','加载快照与已认证资产不一致。')
        receipt.update(snapshot_tree_sha256=digest,source_bundle=plan['source_bundle'],versions=plan['versions'],
                       verification=selected['qualification'],requires_recorded_runtime_configuration=True)
        # Recheck after copying, before the real model loader is invoked.
        if not at.active_path(workspace)['qualified']:raise Blocked('NOT_QUALIFIED','复制快照时认证被撤销。')
        ensure_lease(workspace,authorization)
        asset=snapshot/Path(plan['original_asset']).relative_to('original')
        receipt['loader_called']=True;event('downstream_loader_called')
        solver,_=auto.load_usd(asset);model=solver.mj_model
        receipt['model_created']=True;receipt['loaded_asset']=str(asset);event('model_created_from_frozen_asset')
        for field,value in selected['runtime_configuration'].items():setattr(model.opt,field,value)
        if model.eq_active0.tolist()!=selected['required_default_equality_active']:
            raise Blocked('RUNTIME_MISMATCH','目标默认约束状态不符合认证配置。')
        a,bundle=auto.source_bundle(plan['source'])
        if bundle!=plan['source_bundle']:raise Blocked('SOURCE_CHANGED','源模型与已认证来源不一致。')
        maps=engine.alignment(a,model);_,data=auto.inputs(a,model,maps,profile)
        if not at.active_path(workspace)['qualified']:raise Blocked('NOT_QUALIFIED','加载期间认证被撤销，尚未执行仿真步。')
        ensure_lease(workspace,authorization);event('authorization_rechecked_before_first_step')
        for step in range(int(round(engine.DURATION/model.opt.timestep))):
            ensure_lease(workspace,authorization)
            if mujoco.get_mjcb_contactfilter() is not None or mujoco.get_mjcb_control() is not None:
                raise Blocked('UNSUPPORTED_CALLBACK','运行中出现自定义回调，停止继续推进仿真。')
            mujoco.mj_step(model,data);receipt['steps_executed']+=1
            mujoco.mj_forward(model,data)
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or np.any(data.warning.number) or np.any(data.eq_active):
                raise Blocked('SIMULATION_ERROR','仿真出现警告、无效状态或意外约束，已停止。')
            visual=np.zeros(a.nq)
            for i,j in maps['qpos'].items():visual[i]=data.qpos[j]
            receipt['frames'].append({'step':step+1,'time_s':float(data.time),'target_qpos':data.qpos.tolist(),
                                      'source_visual_qpos':visual.tolist(),'contacts':int(data.ncon)})
        ensure_lease(workspace,authorization)
        if not at.active_path(workspace)['qualified']:
            raise Blocked('NOT_QUALIFIED','结束时认证已撤销，本次不交出成功结果。')
        ensure_lease(workspace,authorization)
        if tx.tree_hash(snapshot)!=digest:raise Blocked('SNAPSHOT_CHANGED','运行快照被改变，不接受本次结果。')
        receipt.update(status='COMPLETED',simulated_duration_s=float(data.time),runtime_configuration=selected['runtime_configuration'],
                       frames_qualified=True,interpretation='Actual native model loaded through Newton standard USD import and advanced under recorded CPU settings; source meshes may be used to visualize these target states.')
        event('job_completed')
    except Blocked as exc:receipt.update(block_code=exc.code,reason=str(exc),frames_qualified=False)
    except Exception as exc:
        receipt.update(block_code='INTEGRITY_OR_INPUT_ERROR',reason=str(exc),frames_qualified=False)
    tx.write_json(output/'load-result.json',receipt)
    return receipt
