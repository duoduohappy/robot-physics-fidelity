"""Manifest-driven CI adapter with immutable baseline input."""
import importlib.metadata
import json
from pathlib import Path
import platform
import time
import xml.etree.ElementTree as ET
from . import engine, transactions, semantics


def compare(previous, current):
    changes = []
    if previous.get('format') != 1:
        raise ValueError('Unknown baseline format')
    if previous['environment'] != current['environment']:
        changes.append({'kind':'environment_changed','action':'review and explicitly approve new baseline'})
    old = {r['id']:r for r in previous['cases']}
    new = {r['id']:r for r in current['cases']}
    for name in sorted(set(old) | set(new)):
        if name not in old or name not in new:
            changes.append({'case':name,'kind':'case_set_changed'}); continue
        x,y = old[name],new[name]
        for field in ['input_fingerprint','keyframe','status','expected_status','semantic_summary']:
            if x.get(field) != y.get(field):
                changes.append({'case':name,'kind':field+'_changed'})
        for metric in ['max_body_position_error_m','max_qvel_error']:
            before, after = x.get(metric), y.get(metric)
            if before is not None and after is not None and after > before + max(1e-8,abs(before)*.1):
                changes.append({'case':name,'kind':'metric_regression','metric':metric,'previous':before,'current':after})
    return changes


def run(manifest_path, output, baseline=None):
    manifest_path=Path(manifest_path).resolve(); config=transactions.read(manifest_path)
    if config.get('format')!=1 or not config.get('cases'):
        raise ValueError('Manifest requires format=1 and nonempty cases')
    ids=[c['id'] for c in config['cases']]
    if len(ids)!=len(set(ids)) or any(not n.replace('-','').replace('_','').isalnum() for n in ids):
        raise ValueError('Case ids must be unique simple names')
    for c in config['cases']:
        if c['expected_status'] not in ['PASS','FAIL','INSUFFICIENT','UNSUPPORTED']:
            raise ValueError('Invalid expected_status')
        if c['expected_status']!='PASS' and not c.get('reason'):
            raise ValueError('Expected non-pass requires a written reason')
    previous=transactions.read(baseline) if baseline else None
    output=Path(output).resolve(); output.mkdir(parents=True,exist_ok=False)
    report={'format':1,'environment':{'python':platform.python_version(),'platform':platform.system(),
             'versions':{n:importlib.metadata.version(n) for n in ['robot-physics-fidelity','mujoco','newton','warp-lang','mujoco-usd-converter']}},
            'cases':[],'external_human_trial':False,'baseline_compared':bool(previous)}
    for c in config['cases']:
        started=time.monotonic(); source=(manifest_path.parent/c['source']).resolve()
        case={'id':c['id'],'keyframe':c['keyframe'],'expected_status':c['expected_status'],'reason':c.get('reason')}
        folder=output/c['id']; folder.mkdir()
        try:
            model=engine.preflight(source,c['keyframe']); case['input_fingerprint']=semantics.fingerprint(model)
            transactions.prepare(source,c['keyframe'],folder/'workspace')
            result=transactions.repair(folder/'workspace')
            transactions.write_json(folder/'physics.json',result)
            target=engine.load_target(result['candidate']).mj_model
            semantic=semantics.audit(model,target)
            transactions.write_json(folder/'semantics.json',semantic)
            case.update(status=result['status'],candidate_activated=result['candidate_activated'],
                        semantic_summary={'status':semantic['status'],'difference_count':semantic['difference_count'],
                                          'fields':sorted(set(r['field'] for r in semantic['differences']))},
                        max_body_position_error_m=result['max_body_position_error_m'],max_qvel_error=result['max_qvel_error'])
            if result['status']!='PASS' and result['candidate_activated']:
                raise RuntimeError('Nonpassing candidate activated')
        except engine.Unsupported as exc:
            case.update(status='UNSUPPORTED',error=str(exc))
        except Exception as exc:
            case.update(status='ERROR',error=str(exc))
        case['expected_result_observed']=case['status']==case['expected_status']
        case['seconds']=round(time.monotonic()-started,3);report['cases'].append(case)
        transactions.write_json(output/'progress.json',report)
        print(c['id']+': '+case['status'],flush=True)
    report['regressions']=compare(previous,report) if previous else []
    unexpected=[c for c in report['cases'] if not c['expected_result_observed']]
    report['status']='UNEXPECTED_RESULT' if unexpected else 'REVIEW_REQUIRED' if report['regressions'] else 'EXPECTED_RESULTS_VERIFIED'
    report['qualification']='Expected failures remain failures; this aggregate is a workflow/CI result, not blanket asset qualification.'
    transactions.write_json(output/'suite.json',report)
    junit=ET.Element('testsuite',name='robot-physics-fidelity',tests=str(len(ids)),failures=str(len(unexpected)+bool(report['regressions'])))
    for c in report['cases']:
        node=ET.SubElement(junit,'testcase',name=c['id'],time=str(c['seconds']))
        if not c['expected_result_observed']:
            ET.SubElement(node,'failure',message='Unexpected result').text=json.dumps(c,ensure_ascii=False)
        elif c['status']!='PASS':
            ET.SubElement(node,'skipped',message='Known '+c['status']+': '+c['reason'])
    if report['regressions']:
        junit.set('tests',str(len(ids)+1)); n=ET.SubElement(junit,'testcase',name='baseline-regression')
        ET.SubElement(n,'failure',message='Baseline review required').text=json.dumps(report['regressions'])
    ET.ElementTree(junit).write(output/'junit.xml',encoding='utf-8',xml_declaration=True)
    return report
