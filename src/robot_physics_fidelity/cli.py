"""rpf CLI: inspect, plan, repair, diagnose, validate, active-path and rollback."""
import argparse
import importlib.metadata
import json
import os
from pathlib import Path

EXIT={'PASS':0,'PLANNED':0,'SUPPORTED':0,'SELECTED':0,'ROLLED_BACK':0,'READY':0,'FETCHED':0,'FAIL':4,'INSUFFICIENT':5,'UNSUPPORTED':3,'ERROR':2}

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    sub.add_parser('doctor')
    fetch=sub.add_parser('fetch-demo'); fetch.add_argument('--asset',choices=['apollo','g1'],required=True); fetch.add_argument('--directory',required=True)
    for name in ['inspect','plan','diagnose','validate']:
        p=sub.add_parser(name); p.add_argument('--source',required=True); p.add_argument('--keyframe',required=True)
        if name=='plan': p.add_argument('--workspace',required=True)
        if name in ['diagnose','validate']: p.add_argument('--candidate',required=True)
        p.add_argument('--report')
    for name in ['repair','rollback','active-path']:
        p=sub.add_parser(name); p.add_argument('--workspace',required=True); p.add_argument('--report')
    args=parser.parse_args(argv)
    # Local cache belongs to this invocation environment, never a compiled-in user's directory.
    if 'WARP_CACHE_PATH' not in os.environ:
        os.environ['WARP_CACHE_PATH']=str(Path.cwd()/'.rpf-cache')
    try:
        expected={'mujoco':'3.17.0','mujoco-warp':'3.17.0','newton':'1.8.0','mujoco-usd-converter':'0.9.0','usd-exchange':'3.0.1','warp-lang':'1.19.0','numpy':'2.4.6','newton-usd-schemas':'0.7.0'}
        versions={k:importlib.metadata.version(k) for k in expected}
        if args.command=='fetch-demo':
            from .assets import fetch
            result=fetch(args.asset,args.directory)
        elif args.command=='doctor' or versions!=expected:
            result={'status':'READY' if versions==expected else 'UNSUPPORTED','versions':versions,'expected':expected,'network_access_required':False}
        else:
            from . import engine, transactions
            if args.command in ['inspect','plan','diagnose','validate']:
                source=engine.preflight(args.source,args.keyframe)
            if args.command=='inspect':
                result={'status':'SUPPORTED','bodies':source.nbody,'joints':source.njnt,'pairs':source.npair,'source_signature':engine.digest_model(source),'scope':'supported for attempting this fixed CPU test; not yet qualified'}
            elif args.command=='plan': result=transactions.prepare(args.source,args.keyframe,args.workspace)
            elif args.command=='repair': result=transactions.repair(args.workspace)
            elif args.command=='rollback': result=transactions.rollback(args.workspace)
            elif args.command=='active-path': result=transactions.active_path(args.workspace)
            else:
                solver=engine.load_target(args.candidate)
                result=(engine.diagnose if args.command=='diagnose' else engine.evaluate)(source,solver.mj_model,args.keyframe)
                result.update(source_signature=engine.digest_model(source),keyframe=args.keyframe)
    except Exception as exc:
        unsupported=type(exc).__name__=='Unsupported' or isinstance(exc,importlib.metadata.PackageNotFoundError)
        result={'status':'UNSUPPORTED' if unsupported else 'ERROR','error':str(exc),'activation_state':'inspect active-path for an existing workspace; no success is implied'}
    if getattr(args,'report',None):
        from .transactions import write_json
        write_json(args.report,result)
    summary={k:v for k,v in result.items() if k not in ['trace','contact_snapshots']}
    print(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False))
    return EXIT.get(result['status'],2)
