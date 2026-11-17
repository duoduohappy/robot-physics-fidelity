"""Reproducible synthetic faults plus real conversion failure; no downloads."""
import argparse
import copy
import json
from pathlib import Path
import statistics
import random
import time
import mujoco
from robot_physics_fidelity import engine, research, transactions


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--g1-source');p.add_argument('--g1-candidate');p.add_argument('--repetitions',type=int,default=5);p.add_argument('--seed',type=int,default=20261117);args=p.parse_args()
    if not 1<=args.repetitions<=20:p.error('repetitions must be 1..20')
    rng=random.Random(args.seed)
    output=Path(args.output).resolve();output.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parents[1];rows=[]
    scenarios=[]
    for key in ['hold','limit','excited']:
        for fault in ['gain','damping','combined']:
            scenarios.append(('rig-'+key+'-'+fault,root/'examples/hinge_rig.xml',key,fault,'development' if key=='hold' else 'held_out_configuration'))
    scenarios.append(('box-slide-damping',root/'examples/box_slide.xml','slide','box_damping','held_out_topology'))
    start=time.monotonic()
    for name,source,key,fault,split in scenarios:
        a=engine.preflight(source,key);b=copy.copy(a);truth=[]
        # Deliberate distractors: small damping and unused fixed-gain/affine slots.
        b.dof_damping[:]+=1e-6
        if a.nu:
            b.actuator_gainprm[:,8]+=.1;b.actuator_biasprm[:,8]+=.1
            b.jnt_actfrcrange[1:]=[-5,5]  # limits not enabled in this fixture
        b.geom_friction[:]*=.75  # explicit pairs override these geom values
        if fault in ['gain','combined']:
            b.actuator_gainprm[0,0]=0.;truth.append('actuator_gainprm:0')
        if fault in ['damping','combined']:
            j=mujoco.mj_name2id(a,mujoco.mjtObj.mjOBJ_JOINT,'elbow' if fault=='combined' else 'shoulder')
            i=int(a.jnt_dofadr[j]);b.dof_damping[i]=4.;truth.append('dof_damping:'+str(i))
        if fault=='box_damping':
            b.dof_damping[0]=20.;truth.append('dof_damping:0')
        baseline=engine.evaluate(a,copy.copy(b),key,details=False)
        if baseline['status']!='FAIL':
            rows.append({'case':name,'split':split,'status':'INVALID_INJECTION','observed':baseline['status']});continue
        for repetition in range(args.repetitions):
            methods=['singletons','ddmin','unranked','ranked'];rng.shuffle(methods)
            for order,method in enumerate(methods):
                result=research.localize(a,b,key,method,64)
                selected={c['id'] for c in result['selected_changes']}
                row={'case':name,'split':split,'repetition':repetition,'execution_order':order,'truth':truth,'selected':sorted(selected),
                     'exact_injected_set':selected==set(truth),'false_selected_count':len(selected-set(truth)),**result}
                transactions.write_json(output/(name+'-'+method+'-'+str(repetition)+'.json'),row)
                rows.append({k:v for k,v in row.items() if k not in ['trials','selected_changes','ranking_scores']})
        print(name+': benchmarked all methods',flush=True)
    if bool(args.g1_source)!=bool(args.g1_candidate):
        raise ValueError('G1 requires both source and candidate')
    if args.g1_source:
        a=engine.preflight(args.g1_source,'knees_bent');b=engine.load_target(args.g1_candidate).mj_model
        for method in ['ddmin','ranked']:
            try:
                result=research.localize(a,b,'knees_bent',method,64)
            except engine.Unsupported as exc:
                result={'status':'UNSUPPORTED','method':method,'reason':str(exc)}
            transactions.write_json(output/('real-g1-'+method+'.json'),result)
            rows.append({'case':'real-g1','split':'real_conversion_failure','truth':None,**{k:v for k,v in result.items() if k not in ['trials','selected_changes','ranking_scores']}})
    synthetic=[r for r in rows if r.get('truth') is not None and 'method' in r]
    methods={}
    for method in ['singletons','ddmin','unranked','ranked']:
        subset=[r for r in synthetic if r['method']==method]
        methods[method]={'unique_cases':len(set(r['case'] for r in subset)),'runs':len(subset),'exact_injected_set_runs':sum(r['exact_injected_set'] for r in subset),
                         'confirmed':sum(r['status']=='CONFIRMED_WITHIN_TEST' for r in subset),
                         'false_selected_count':sum(r['false_selected_count'] for r in subset),
                         'full_rollouts':sum(r['full_rollouts'] for r in subset),
                         'same_state_forward_calls':sum(r['same_state_forward_calls'] for r in subset),
                         'source_sampling_steps':sum(r['source_sampling_steps'] for r in subset),
                         'source_sampling_forward_calls':sum(r['source_sampling_forward_calls'] for r in subset),
                         'seconds':sum(r['seconds'] for r in subset),
                         'median_seconds_per_run':statistics.median(r['seconds'] for r in subset)}
    report={'status':'COMPLETED_EXPERIMENTS','methods':methods,'cases':rows,'total_seconds':time.monotonic()-start,
            'repetitions':args.repetitions,'seed':args.seed,'method_order':'seeded random order within every case/repetition',
            'predeclared_primary_metrics':['exact injected-set recovery','false selected rows','full rollouts','forward calls','wall time including ranking'],
            'external_validation':False,'novel_algorithm_claim':False,
            'limitations':['Synthetic known faults and inactive/tiny distractors; not a representative industry dataset',
                           'Held-out means predefined fixture configurations/topology, not a separate organization or robot',
                           'Repeated same-machine timing; wall times are descriptive, no cross-machine or significance claim',
                           'Single-row interventions may fail for combinations; 1-minimal sufficient repair is not a unique root cause',
                           'G1 real failure may remain unresolved; all candidate repairs are native diagnostic copies']}
    transactions.write_json(output/'benchmark.json',report)
    print(json.dumps(methods,indent=2))


if __name__=='__main__':main()
