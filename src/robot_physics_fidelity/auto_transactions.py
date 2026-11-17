"""Read-only export qualification, explicit revocation and bound evidence."""
import hashlib
import importlib.metadata
from pathlib import Path
import shutil
import uuid
from . import automatic, engine, transactions as tx

FORMAT='go2-automatic-workspace-v1'


def complete_pass(report,manifest):
    return (report.get('status')=='PASS' and report.get('contract')==automatic.CONTRACT
            and report.get('source_bundle')==manifest['source_bundle'] and report.get('versions')==manifest['versions']
            and report.get('original_tree_sha256')==manifest['original_tree_sha256']
            and report.get('profiles_required')==list(automatic.PROFILES)
            and [p['profile'] for p in report.get('profiles',[])]==list(automatic.PROFILES)
            and all(p['status']=='PASS' for p in report.get('profiles',[]))
            and report.get('rules',{}).get('status')=='PASS' and report['rules'].get('pairs_checked')==276
            and len(report['rules'].get('rows',[]))==276 and all(r['match'] for r in report['rules']['rows'])
            and len(report.get('identity',[]))==24
            and set(report.get('runtime_configuration',{}))==set(engine.SETTINGS))


def versions():
    return {n:importlib.metadata.version(n) for n in ['robot-physics-fidelity','mujoco','newton','mujoco-usd-converter','warp-lang']}


def prepare(source,workspace):
    source=Path(source).resolve();workspace=Path(workspace).resolve()
    _,bundle=automatic.source_bundle(source)
    if workspace.exists():raise ValueError('Auto workspace must not exist')
    workspace.mkdir(parents=True)
    try:
        asset=engine.convert(source,workspace/'original')
        shutil.copy2(source.parent/'LICENSE',workspace/'original/UPSTREAM-LICENSE.txt')
        manifest={'format':FORMAT,'contract':automatic.CONTRACT,'source':str(source),'source_bundle':bundle,
                  'profiles':list(automatic.PROFILES),'versions':versions(),'original_asset':asset.relative_to(workspace).as_posix(),
                  'original_tree_sha256':tx.tree_hash(workspace/'original'),'asset_repairs_allowed':[]}
        tx.write_json(workspace/'auto-plan.json',manifest)
        tx.write_json(workspace/'auto-active.json',{'selection':'unqualified_original','qualification':'UNQUALIFIED_ORIGINAL'})
        return {'status':'PLANNED','lane':automatic.CONTRACT,'workspace':str(workspace),'asset_modified':False,
                'repair_required':'not determined until verification','qualified':False}
    except Exception as exc:
        tx.write_json(workspace/'auto-preparation-error.json',{'status':'ERROR','error':str(exc),'qualified':False})
        raise


def checked(workspace):
    workspace=Path(workspace).resolve();manifest=tx.read(workspace/'auto-plan.json')
    if manifest['format']!=FORMAT or manifest['contract']!=automatic.CONTRACT or manifest['profiles']!=list(automatic.PROFILES) or manifest['asset_repairs_allowed']!=[]:
        raise ValueError('Auto plan contract or profiles changed')
    if manifest['versions']!=versions():raise ValueError('Runtime versions changed after planning')
    asset=tx.original_ok(workspace,manifest)
    _,bundle=automatic.source_bundle(manifest['source'])
    if bundle!=manifest['source_bundle']:raise ValueError('Source differs from planned bundle')
    return manifest,asset


def verify(workspace):
    workspace=Path(workspace).resolve();manifest,asset=checked(workspace)
    result=automatic.validate(manifest['source'],asset)
    checked(workspace)
    result.update(original_tree_sha256=manifest['original_tree_sha256'],profiles_required=list(automatic.PROFILES),
                  versions=manifest['versions'],qualified=False,asset_modified=False)
    report=workspace/('auto-verification-'+uuid.uuid4().hex+'.json')
    if result['status']=='PASS' and not complete_pass(result,manifest):raise ValueError('Incomplete passing automatic evidence')
    if result['status']=='PASS':result['qualified']=True
    tx.write_json(report,result)
    if result['status']=='PASS':
        tx.write_json(workspace/'auto-active.json',{'selection':'verified_original','qualification':'PASS_GO2_AUTO_PROFILE',
                      'verification':report.name,'verification_sha256':hashlib.sha256(report.read_bytes()).hexdigest(),
                      'asset':manifest['original_asset'],'tree_sha256':manifest['original_tree_sha256']})
    tx.write_json(workspace/'auto-last-result.json',{'status':result['status'],'qualified':result['qualified'],'report':report.name})
    return result


def active_path(workspace):
    workspace=Path(workspace).resolve();manifest,asset=checked(workspace)
    selection=tx.read(workspace/'auto-active.json')
    if selection['selection']=='unqualified_original':
        return {'status':'SELECTED','selection':'unqualified_original','asset':str(asset),'qualified':False,'qualification':'UNQUALIFIED_ORIGINAL'}
    if selection['selection']!='verified_original' or selection.get('qualification')!='PASS_GO2_AUTO_PROFILE':
        raise ValueError('Unknown automatic selection')
    if selection.get('asset')!=manifest['original_asset'] or selection.get('tree_sha256')!=manifest['original_tree_sha256']:
        raise ValueError('Active artifact does not match plan')
    report_path=tx.under(workspace,selection['verification'])
    if hashlib.sha256(report_path.read_bytes()).hexdigest()!=selection.get('verification_sha256'):
        raise ValueError('Verification report integrity mismatch')
    report=tx.read(report_path)
    if report.get('qualified') is not True or not complete_pass(report,manifest):
        raise ValueError('No matching complete passing automatic verification')
    return {'status':'SELECTED','selection':'verified_original','qualified':True,'qualification':'PASS_GO2_AUTO_PROFILE',
            'asset':str(asset),'repair_required':False,'requires_recorded_runtime_configuration':True,
            'runtime_configuration':report['runtime_configuration'],'profiles':list(automatic.PROFILES),
            'required_default_equality_active':report['required_default_equality_active']}


def rollback(workspace):
    workspace=Path(workspace).resolve();_,asset=checked(workspace)
    old=tx.read(workspace/'auto-active.json')
    tx.write_json(workspace/'auto-active.json',{'selection':'unqualified_original','qualification':'UNQUALIFIED_ORIGINAL'})
    receipt={'status':'ROLLED_BACK','previous_selection':old['selection'],'selection':'unqualified_original',
             'asset':str(asset),'qualified':False,'runtime_state_restored':False,'asset_modified':False,
             'meaning':'Revoked qualification of the same original export; no candidate file switch or running-state rollback'}
    tx.write_json(workspace/('auto-rollback-'+uuid.uuid4().hex+'.json'),receipt)
    return receipt
