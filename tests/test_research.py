import unittest
import copy
from pathlib import Path
import mujoco
from robot_physics_fidelity import research


class PredicateOracle:
    def __init__(self,truth,n=8,limit=100):
        self.truth=set(truth);self.changes=list(range(n));self.calls=0;self.limit=limit;self.cache={}
    def test(self,items):
        key=tuple(sorted(items))
        if key not in self.cache:
            self.calls+=1
            if self.calls>self.limit:raise research.BudgetExceeded()
            self.cache[key]=self.truth<=set(items)
        return self.cache[key]
    def rank(self):return sorted(self.changes,key=lambda i:(i not in self.truth,i)),None


class ResearchTests(unittest.TestCase):
    def test_combination_and_one_minimality(self):
        for method in ['ddmin','ranked','unranked']:
            oracle=PredicateOracle([2,7]);selected,minimal,status,_=research.search(oracle,method)
            self.assertEqual(set(selected),{2,7});self.assertTrue(minimal)
            self.assertEqual(status,'CONFIRMED_WITHIN_TEST')
    def test_singletons_do_not_falsely_resolve_combination(self):
        self.assertEqual(research.search(PredicateOracle([2,7]),'singletons')[2],'UNRESOLVED_COMBINATION')
    def test_budget_not_silently_called_confirmation(self):
        with self.assertRaises(research.BudgetExceeded):research.search(PredicateOracle([2,7],limit=2),'ranked')
    def test_no_failure_is_not_cause_confirmation(self):
        self.assertEqual(research.search(PredicateOracle([]),'ddmin')[2],'NO_FAILURE')
    def test_unavailable_candidate_set_remains_unresolved(self):
        self.assertEqual(research.search(PredicateOracle([100]),'ranked')[2],'UNRESOLVED')

    def test_physics_intervention_preserves_callers_and_finds_gain(self):
        a=mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parents[1]/'examples/hinge_rig.xml'))
        b=copy.copy(a);b.actuator_gainprm[0,0]=0
        result=research.localize(a,b,'hold','ranked',64)
        self.assertEqual(result['status'],'CONFIRMED_WITHIN_TEST')
        self.assertTrue(result['caller_models_unchanged'])
        self.assertEqual(b.actuator_gainprm[0,0],0)
        self.assertEqual(result['selected_changes'][0]['id'],'actuator_gainprm:0')

    def test_unlisted_physical_defect_not_falsely_resolved(self):
        a=mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parents[1]/'examples/hinge_rig.xml'))
        b=copy.copy(a);b.actuator_gear[0,0]=10
        self.assertEqual(research.localize(a,b,'excited','ranked',64)['status'],'UNRESOLVED')


if __name__=='__main__':unittest.main()
