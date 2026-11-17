"""Bounded experimental repair-set localization; no persistent repair."""
import copy
import itertools
import math
import time
import numpy as np
import mujoco
from . import engine, semantics


FIELDS = {'dof':['dof_damping','dof_frictionloss'],
          'joint':['jnt_solref','jnt_solimp','jnt_actfrcrange'],
          'actuator':['actuator_gainprm','actuator_biasprm'],
          'geom':['geom_friction']}


def candidates(a,b,maps):
    result=[]
    for mapping, fields in FIELDS.items():
        for field in fields:
            for i,j in maps[mapping].items():
                if not np.allclose(getattr(a,field)[i],getattr(b,field)[j],rtol=1e-6,atol=1e-8):
                    result.append({'id':f'{field}:{i}','field':field,'source_index':i,'target_index':j,
                                   'source':np.asarray(getattr(a,field)[i]).tolist(),
                                   'target':np.asarray(getattr(b,field)[j]).tolist()})
    return result


class BudgetExceeded(RuntimeError): pass


class Oracle:
    def __init__(self,a,b,key,changes,budget):
        self.a,self.b,self.key,self.changes,self.budget=a,b,key,changes,budget
        self.cache={}; self.rollouts=0; self.forward_calls=0
        self.source_sampling_steps=0; self.source_sampling_forwards=0
        self.maps=engine.alignment(a,b)

    def model(self,subset):
        model=copy.copy(self.b)
        for k in subset:
            c=self.changes[k]
            getattr(model,c['field'])[c['target_index']]=c['source']
        return model

    def test(self,subset):
        subset=tuple(sorted(subset))
        if subset in self.cache: return self.cache[subset]['status']=='PASS'
        if self.rollouts>=self.budget: raise BudgetExceeded('full-rollout budget exhausted')
        self.rollouts+=1
        self.cache[subset]=engine.evaluate(self.a,self.model(subset),self.key,details=False)
        return self.cache[subset]['status']=='PASS'

    def rank(self):
        # Cheap score uses source states at t=0, .05 and .1 seconds, never injection labels.
        da,_=engine.initial(self.a,self.b,self.maps,self.key)
        states=[]
        for step in range(max(1,math.ceil(.1/self.a.opt.timestep))+1):
            if step in {0,math.ceil(.05/self.a.opt.timestep),math.ceil(.1/self.a.opt.timestep)}:
                mujoco.mj_forward(self.a,da)
                self.source_sampling_forwards+=1
                states.append((da.qpos.copy(),da.qvel.copy(),da.ctrl.copy(),da.qacc_warmstart.copy(),float(da.time),da.qacc.copy()))
            mujoco.mj_step(self.a,da)
            self.source_sampling_steps+=1

        def residual(model):
            for f in engine.SETTINGS: setattr(model.opt,f,getattr(self.a.opt,f))
            total=0.
            for qp,qv,ctrl,warm,t,acc in states:
                data=mujoco.MjData(model)
                for i,j in self.maps['qpos'].items(): data.qpos[j]=qp[i]
                for i,j in self.maps['dof'].items(): data.qvel[j]=qv[i];data.qacc_warmstart[j]=warm[i]
                for i,j in self.maps['actuator'].items(): data.ctrl[j]=ctrl[i]
                data.time=t; mujoco.mj_forward(model,data); self.forward_calls+=1
                # Relative squared residual weights each dof by source acceleration scale.
                total+=sum(((acc[i]-data.qacc[j])/(1+abs(acc[i])))**2 for i,j in self.maps['dof'].items())
            return float(total)
        baseline=residual(self.model([]))
        scores=[baseline-residual(self.model([i])) for i in range(len(self.changes))]
        return sorted(range(len(scores)),key=lambda i:(-scores[i],i)),scores


def minimize(oracle,selected):
    selected=list(selected)
    # Finish only after a complete sweep without successful deletion.
    changed=True
    while changed:
        changed=False
        for item in list(selected):
            subset=[i for i in selected if i!=item]
            if oracle.test(subset): selected=subset; changed=True
    return selected


def search(oracle,method):
    all_items=list(range(len(oracle.changes))); scores=None
    if oracle.test([]): return [],True,'NO_FAILURE',scores
    if not oracle.test(all_items): return [],False,'UNRESOLVED',scores
    if method=='singletons':
        for i in all_items:
            if oracle.test([i]): return [i],True,'CONFIRMED_WITHIN_TEST',scores
        return [],False,'UNRESOLVED_COMBINATION',scores
    if method in ['ranked','unranked']:
        order,scores=oracle.rank() if method=='ranked' else (all_items,None)
        selected=[]
        for i in order:
            selected.append(i)
            if oracle.test(selected): break
        return minimize(oracle,selected),True,'CONFIRMED_WITHIN_TEST',scores
    if method!='ddmin': raise ValueError('Unknown method')
    current=all_items; granularity=2
    while len(current)>=2:
        size=math.ceil(len(current)/granularity)
        parts=[current[i:i+size] for i in range(0,len(current),size)]
        reduced=False
        # Standard subset + complement ddmin, with PASS as preservation predicate.
        for subset in parts+[[i for i in current if i not in p] for p in parts]:
            if oracle.test(subset):
                current=subset;granularity=max(2,granularity-1);reduced=True;break
        if not reduced:
            if granularity>=len(current): break
            granularity=min(len(current),granularity*2)
    return minimize(oracle,current),True,'CONFIRMED_WITHIN_TEST',scores


def localize(a,b,key,method='ranked',budget=64):
    if budget<2 or budget>512: raise ValueError('Budget must be between 2 and 512')
    start=time.monotonic(); changes=candidates(a,b,engine.alignment(a,b))
    if len(changes)>128: raise engine.Unsupported('Research module capped at 128 changed rows')
    oracle=Oracle(a,b,key,changes,budget); subset=[]; certified=False;scores=None
    source_before=semantics.fingerprint(a);target_before=semantics.fingerprint(b)
    try:
        subset,certified,status,scores=search(oracle,method)
    except BudgetExceeded:
        status='BUDGET_EXHAUSTED'
    finally:
        if source_before!=semantics.fingerprint(a) or target_before!=semantics.fingerprint(b):
            raise RuntimeError('Diagnostic search mutated caller model')
    return {'status':status,'method':method,'budget':budget,'candidate_count':len(changes),
            'selected_changes':[changes[i] for i in subset],
            'one_minimal_verified':certified and status=='CONFIRMED_WITHIN_TEST',
            'full_rollouts':oracle.rollouts,'same_state_forward_calls':oracle.forward_calls,
            'source_sampling_steps':oracle.source_sampling_steps,'source_sampling_forward_calls':oracle.source_sampling_forwards,
            'seconds':time.monotonic()-start,'ranking_scores':scores,
            'trials':[{'restored':[changes[i]['id'] for i in ids], 'status':r['status'],
                      'max_qvel_error':r['max_qvel_error'],'max_body_position_error_m':r['max_body_position_error_m']}
                      for ids,r in oracle.cache.items()],
            'caller_models_unchanged':True,'asset_modified':False,
            'meaning':'A 1-minimal sufficient native-array repair set within this test and candidate whitelist; not a unique cause or a persistent USD repair.'}
