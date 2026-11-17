import copy
import json
from pathlib import Path
import shutil
import unittest
import uuid
from unittest.mock import patch
import mujoco
from robot_physics_fidelity import automatic as auto,auto_transactions as at,engine,transactions as tx


class RuleTests(unittest.TestCase):
    def test_mask_rules_agree_with_native_contacts(self):
        for mask1,mask2 in [((1,1),(1,1)),((2,2),(1,1)),((2,0),(0,2)),((0,0),(1,1))]:
            xml=f'<mujoco><worldbody><body name="a" pos="0 0 .4"><freejoint/><geom type="sphere" size=".1" contype="{mask1[0]}" conaffinity="{mask1[1]}"/></body><body name="b" pos=".1 0 .4"><freejoint/><geom type="sphere" size=".1" contype="{mask2[0]}" conaffinity="{mask2[1]}"/></body></worldbody></mujoco>'
            m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
            self.assertEqual(auto.eligible(m,0,1),bool(d.ncon),(mask1,mask2))

    def test_parent_filter_agrees_with_native(self):
        xml='<mujoco><worldbody><body name="a"><freejoint/><geom type="sphere" size=".1"/><body name="b" pos=".1 0 0"><joint type="hinge"/><geom type="sphere" size=".1"/></body></body></worldbody></mujoco>'
        m=mujoco.MjModel.from_xml_string(xml)
        for disabled in [False,True]:
            m.opt.disableflags=int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT) if disabled else 0
            d=mujoco.MjData(m);mujoco.mj_forward(m,d)
            self.assertEqual(auto.eligible(m,0,1),bool(d.ncon));self.assertEqual(bool(d.ncon),disabled)

    def test_weld_and_exclude_agree_with_native(self):
        cases=[
            '<mujoco><worldbody><body><freejoint/><geom type="sphere" size=".1"/><body pos=".1 0 0"><geom type="sphere" size=".1"/></body></body></worldbody></mujoco>',
            '<mujoco><worldbody><body name="a"><freejoint/><geom type="sphere" size=".1"/></body><body name="b" pos=".1 0 0"><freejoint/><geom type="sphere" size=".1"/></body></worldbody><contact><exclude body1="a" body2="b"/></contact></mujoco>']
        for xml in cases:
            m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
            self.assertFalse(auto.eligible(m,0,1));self.assertEqual(d.ncon,0)

    def test_changed_mask_caught_without_waiting_for_contact(self):
        m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body pos="0 0 10"><freejoint/><geom type="sphere" size=".1"/></body></worldbody></mujoco>')
        b=copy.copy(m);b.geom_contype[1]=0;b.geom_conaffinity[1]=0
        self.assertEqual(auto.rule_table(m,b,{0:0,1:1})['status'],'FAIL')

    def test_equivalent_different_raw_masks_pass(self):
        m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type="plane" size="1 1 .1"/><body pos="0 0 1"><freejoint/><geom type="sphere" size=".1"/></body></worldbody></mujoco>')
        b=copy.copy(m);b.geom_contype[:]=2;b.geom_conaffinity[:]=2
        self.assertEqual(auto.rule_table(m,b,{0:0,1:1})['status'],'PASS')


class AutoTransactionTests(unittest.TestCase):
    def setUp(self):
        self.root=(Path.cwd()/'.rpf-test-work'/('auto-'+uuid.uuid4().hex)).resolve();self.root.mkdir(parents=True)
        self.source=self.root/'scene_mjx.xml';self.source.write_text('fixture');(self.root/'LICENSE').write_text('fixture license')
        self.workspace=self.root/'workspace';self.bundle={'source_signature':'fixed-test','files_sha256':{'fixture':'fixed-test'}}
        self.source_patch=patch.object(auto,'source_bundle',return_value=(None,self.bundle));self.source_patch.start();self.addCleanup(self.source_patch.stop)
        def convert(source,folder):
            folder=Path(folder);folder.mkdir();p=folder/'asset.usda';p.write_text('#usda 1.0\ndef Xform "Fixture" {}');return p
        with patch.object(engine,'convert',side_effect=convert):at.prepare(self.source,self.workspace)

    def tearDown(self):
        parent=(Path.cwd()/'.rpf-test-work').resolve()
        if self.root.parent!=parent:raise RuntimeError('Unsafe test cleanup')
        shutil.rmtree(self.root)

    def passing(self):
        return {'status':'PASS','contract':auto.CONTRACT,'source_bundle':self.bundle,
                'rules':{'status':'PASS','pairs_checked':276,'rows':[{'match':True} for _ in range(276)]},
                'identity':[{} for _ in range(24)],'runtime_configuration':{f:0 for f in engine.SETTINGS},
                'required_default_equality_active':[],
                'profiles':[{'profile':p,'status':'PASS'} for p in auto.PROFILES]}

    def test_success_no_repair_then_revoke_same_asset(self):
        original=(self.workspace/'original/asset.usda').read_bytes()
        with patch.object(auto,'validate',return_value=self.passing()):self.assertTrue(at.verify(self.workspace)['qualified'])
        self.assertTrue(at.active_path(self.workspace)['qualified'])
        result=at.rollback(self.workspace)
        self.assertFalse(result['qualified']);self.assertFalse(result['runtime_state_restored'])
        self.assertEqual((self.workspace/'original/asset.usda').read_bytes(),original)
        self.assertFalse(at.active_path(self.workspace)['qualified'])

    def test_fail_does_not_qualify(self):
        with patch.object(auto,'validate',return_value={'status':'FAIL','source_bundle':self.bundle}):
            self.assertFalse(at.verify(self.workspace)['qualified'])
        self.assertFalse(at.active_path(self.workspace)['qualified'])

    def test_incomplete_pass_never_qualifies(self):
        result=self.passing();result['profiles']=result['profiles'][:1]
        with patch.object(auto,'validate',return_value=result),self.assertRaises(ValueError):at.verify(self.workspace)
        self.assertFalse(at.active_path(self.workspace)['qualified'])

    def test_asset_tampering_blocks_active(self):
        with patch.object(auto,'validate',return_value=self.passing()):at.verify(self.workspace)
        (self.workspace/'original/asset.usda').write_text('changed')
        with self.assertRaises(ValueError):at.active_path(self.workspace)

    def test_report_tampering_blocks_active(self):
        with patch.object(auto,'validate',return_value=self.passing()):at.verify(self.workspace)
        selected=tx.read(self.workspace/'auto-active.json')
        tx.write_json(self.workspace/selected['verification'],{'status':'PASS'})
        with self.assertRaises(ValueError):at.active_path(self.workspace)

    def test_source_change_blocks_active(self):
        with patch.object(auto,'validate',return_value=self.passing()):at.verify(self.workspace)
        with patch.object(auto,'source_bundle',return_value=(None,{'source_signature':'changed'})),self.assertRaises(ValueError):at.active_path(self.workspace)


class PreflightTests(unittest.TestCase):
    def test_unknown_source_rejected_before_creating_workspace(self):
        workspace=Path.cwd()/'.rpf-test-work'/('reject-'+uuid.uuid4().hex)
        with self.assertRaises(engine.Unsupported):at.prepare('unknown.xml',workspace)
        self.assertFalse(workspace.exists())


if __name__=='__main__':unittest.main()
