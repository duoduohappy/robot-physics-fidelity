"""Immutable asset copies, verified activation and pointer-only rollback."""
from pathlib import Path
import hashlib
import json
import os
import uuid
from . import engine
from pxr import Usd

def write_json(path,value):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True); temporary=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    with temporary.open('x',encoding='utf-8') as f:
        json.dump(value,f,ensure_ascii=False,indent=2,allow_nan=False); f.flush(); os.fsync(f.fileno())
    os.replace(temporary,path)

def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))

def under(root,relative):
    root=Path(root).resolve(); path=(root/relative).resolve()
    if path==root or root not in path.parents: raise ValueError('Artifact path escapes workspace')
    return path

def tree_hash(folder):
    folder=Path(folder).resolve(); h=hashlib.sha256()
    for p in sorted(folder.rglob('*')):
        if p.is_symlink(): raise ValueError('Symlink artifact not accepted')
        if p.is_file():
            h.update(p.relative_to(folder).as_posix().encode()); h.update(p.read_bytes())
    return h.hexdigest()

def original_ok(workspace,manifest):
    directory=under(workspace,'original')
    if tree_hash(directory)!=manifest['original_tree_sha256']: raise ValueError('Original artifact integrity mismatch')
    return under(workspace,manifest['original_asset'])

def prepare(source,keyframe,workspace):
    source=Path(source).resolve(); workspace=Path(workspace).resolve()
    # Unsupported inputs leave no half-built managed workspace.
    model=engine.preflight(source,keyframe)
    if workspace.exists(): raise ValueError('Workspace must not exist')
    workspace.mkdir(parents=True)
    try:
        asset=engine.convert(source,workspace/'original')
        ops=engine.operations(model,Usd.Stage.Open(str(asset)))
        manifest={'format':1,'source':str(source),'source_signature':engine.digest_model(model),'keyframe':keyframe,'original_asset':asset.relative_to(workspace).as_posix(),'original_tree_sha256':tree_hash(workspace/'original'),'operations':ops,'repairs_allowed':['retain_pair_endpoint','preserve_explicit_pair','author_raw_joint_limit']}
        write_json(workspace/'plan.json',manifest)
        write_json(workspace/'active.json',{'selection':'original','asset':manifest['original_asset'],'tree_sha256':manifest['original_tree_sha256'],'qualification':'UNQUALIFIED_ORIGINAL'})
        return {'status':'PLANNED','workspace':str(workspace),'operation_count':len(ops),'active':'original','plan':str(workspace/'plan.json')}
    except Exception as exc:
        write_json(workspace/'preparation-error.json',{'status':'ERROR','error':str(exc),'candidate_activated':False})
        raise

def repair(workspace):
    workspace=Path(workspace).resolve(); manifest=read(workspace/'plan.json')
    if manifest.get('format')!=1: raise ValueError('Unknown plan format')
    original=original_ok(workspace,manifest)
    source=engine.preflight(manifest['source'],manifest['keyframe'])
    if engine.digest_model(source)!=manifest['source_signature']: raise ValueError('Source model changed after planning')
    ops=engine.operations(source,Usd.Stage.Open(str(original)))
    if ops!=manifest['operations'] or manifest['repairs_allowed']!=['retain_pair_endpoint','preserve_explicit_pair','author_raw_joint_limit']:
        raise ValueError('Plan changed or contains unsupported repair operations')
    directory=workspace/('candidate-'+uuid.uuid4().hex)
    directory.mkdir()
    candidate=directory/'asset.usda'
    try:
        engine.apply(source,original,candidate)
        solver=engine.load_target(candidate)
        result=engine.evaluate(source,solver.mj_model,manifest['keyframe'])
        digest=tree_hash(directory)
        result.update(candidate=str(candidate),candidate_tree_sha256=digest,candidate_activated=False,source_signature=manifest['source_signature'],keyframe=manifest['keyframe'])
        # Write the result outside the immutable candidate tree.
        report=workspace/(directory.name+'-verification.json')
        write_json(report,result)
        if result['status']=='PASS':
            original_ok(workspace,manifest)
            if tree_hash(directory)!=digest: raise ValueError('Candidate changed during verification')
            write_json(workspace/'active.json',{'selection':'candidate','asset':candidate.relative_to(workspace).as_posix(),'tree_sha256':digest,'verification':report.name,'qualification':'PASS_FIXED_TEST_ONLY'})
            result['candidate_activated']=True
            write_json(report,result)
        write_json(workspace/'last-result.json',{'status':result['status'],'verification':report.name,'candidate_activated':result['candidate_activated']})
        return result
    except Exception as exc:
        write_json(workspace/(directory.name+'-error.json'),{'status':'ERROR','error':str(exc),'candidate_activated':False})
        raise

def active_path(workspace):
    workspace=Path(workspace).resolve(); manifest=read(workspace/'plan.json'); original_ok(workspace,manifest)
    selection=read(workspace/'active.json'); path=under(workspace,selection['asset'])
    if selection['selection']=='original':
        if selection['asset']!=manifest['original_asset'] or selection['tree_sha256']!=manifest['original_tree_sha256']: raise ValueError('Invalid original selection')
    elif selection['selection']=='candidate':
        if tree_hash(path.parent)!=selection['tree_sha256']: raise ValueError('Active candidate integrity mismatch')
        report=read(under(workspace,selection['verification']))
        if report['status']!='PASS' or report['candidate_tree_sha256']!=selection['tree_sha256']: raise ValueError('Active candidate has no matching passing verification')
        if report.get('source_signature')!=manifest['source_signature'] or report.get('keyframe')!=manifest['keyframe']: raise ValueError('Verification does not match planned source and keyframe')
    else: raise ValueError('Unknown active selection')
    return {**selection,'status':'SELECTED','asset':str(path)}

def rollback(workspace):
    workspace=Path(workspace).resolve(); manifest=read(workspace/'plan.json'); original_ok(workspace,manifest)
    old=read(workspace/'active.json')
    restored={'selection':'original','asset':manifest['original_asset'],'tree_sha256':manifest['original_tree_sha256'],'qualification':'UNQUALIFIED_ORIGINAL'}
    write_json(workspace/'active.json',restored)
    receipt={'status':'ROLLED_BACK','previous_selection':old['selection'],'restored_asset':manifest['original_asset'],'source_files_modified':False,'original_integrity_verified':True,'runtime_state_restored':False}
    write_json(workspace/('rollback-'+uuid.uuid4().hex+'.json'),receipt)
    return receipt
