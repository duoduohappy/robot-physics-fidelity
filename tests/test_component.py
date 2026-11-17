import copy
import unittest
from pathlib import Path
import mujoco
from robot_physics_fidelity import semantics, suite


class ComponentTests(unittest.TestCase):
    def setUp(self):
        self.a=mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parents[1]/'examples/hinge_rig.xml'))

    def test_fingerprint_tracks_previously_omitted_force_limit(self):
        b=copy.copy(self.a); b.jnt_actfrcrange[1]=[-2,2]
        self.assertNotEqual(semantics.fingerprint(self.a),semantics.fingerprint(b))

    def test_audit_flags_mass_and_gain_changes_without_calling_them_causes(self):
        b=copy.copy(self.a); b.body_mass[1]*=2; b.actuator_gainprm[0,0]*=2
        report=semantics.audit(self.a,b)
        fields={r['field'] for r in report['differences']}
        self.assertTrue({'body_mass','actuator_gainprm'}<=fields)

    def test_equal_native_model_audits_equal(self):
        self.assertEqual(semantics.audit(self.a,copy.copy(self.a))['difference_count'],0)

    def test_dependency_upgrade_requests_review(self):
        a={'format':1,'environment':{'newton':'old'},'cases':[]}
        b={'format':1,'environment':{'newton':'new'},'cases':[]}
        self.assertEqual(suite.compare(a,b)[0]['kind'],'environment_changed')

    def test_regression_known_failure_becomes_unexpected_or_metric_drift(self):
        a={'format':1,'environment':{},'cases':[{'id':'x','status':'FAIL','max_qvel_error':.1}]}
        b=copy.deepcopy(a); b['cases'][0]['max_qvel_error']=.5
        self.assertTrue(any(r['kind']=='metric_regression' for r in suite.compare(a,b)))

    def test_fingerprint_deterministic_independent_copy(self):
        self.assertEqual(semantics.fingerprint(self.a),semantics.fingerprint(copy.copy(self.a)))


if __name__=='__main__': unittest.main()
