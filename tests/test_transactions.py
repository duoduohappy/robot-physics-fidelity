"""Fault-injection checks for preservation and activation safety, not a physics substitute."""
import json
from pathlib import Path
import tempfile
import shutil
import os
import unittest
from unittest.mock import patch
from robot_physics_fidelity import transactions as t
from robot_physics_fidelity import engine

WORK=Path(os.environ.get('RPF_TEST_WORK',Path.cwd()/'.rpf-test-work')).resolve()

class TransactionTests(unittest.TestCase):
    def setUp(self):
        WORK.mkdir(exist_ok=True)
        self.root=Path(tempfile.mkdtemp(prefix='rpf-test-',dir=WORK)).resolve()
        (self.root/'original').mkdir(); (self.root/'original/asset.usda').write_text('immutable original')
        self.plan={'format':1,'source':'fixture.xml','source_signature':'sig','keyframe':'stand','original_asset':'original/asset.usda','original_tree_sha256':t.tree_hash(self.root/'original'),'operations':[],'repairs_allowed':['retain_pair_endpoint','preserve_explicit_pair','author_raw_joint_limit']}
        t.write_json(self.root/'plan.json',self.plan)
        t.write_json(self.root/'active.json',{'selection':'original','asset':'original/asset.usda','tree_sha256':self.plan['original_tree_sha256'],'qualification':'UNQUALIFIED_ORIGINAL'})
    def tearDown(self):
        assert self.root.parent==WORK.resolve() and self.root.name.startswith('rpf-test-')
        shutil.rmtree(self.root)
    def test_tampered_original_refuses_rollback(self):
        before=(self.root/'active.json').read_bytes()
        (self.root/'original/asset.usda').write_text('tampered')
        with self.assertRaisesRegex(ValueError,'integrity'): t.rollback(self.root)
        self.assertEqual(before,(self.root/'active.json').read_bytes())
    def test_workspace_escape_is_rejected(self):
        state=t.read(self.root/'active.json'); state.update(selection='candidate',asset='../outside.usda')
        t.write_json(self.root/'active.json',state)
        with self.assertRaisesRegex(ValueError,'escapes'): t.active_path(self.root)
    def test_verification_for_other_keyframe_is_rejected(self):
        p=self.root/'candidate-other-key'; p.mkdir(); (p/'asset.usda').write_text('verified elsewhere')
        digest=t.tree_hash(p)
        t.write_json(self.root/'verification.json',{'status':'PASS','candidate_tree_sha256':digest,'source_signature':'sig','keyframe':'other'})
        t.write_json(self.root/'active.json',{'selection':'candidate','asset':'candidate-other-key/asset.usda','tree_sha256':digest,'verification':'verification.json'})
        with self.assertRaisesRegex(ValueError,'keyframe'): t.active_path(self.root)
    def test_failed_verification_does_not_activate(self):
        before=(self.root/'active.json').read_bytes()
        def apply(source,original,target): target.write_text('rejected candidate')
        with patch.object(engine,'preflight',return_value=object()),patch.object(engine,'digest_model',return_value='sig'),patch.object(engine,'operations',return_value=[]),patch.object(t.Usd.Stage,'Open',return_value=object()),patch.object(engine,'apply',side_effect=apply),patch.object(engine,'load_target') as load,patch.object(engine,'evaluate',return_value={'status':'FAIL'}):
            load.return_value.mj_model=object()
            result=t.repair(self.root)
        self.assertFalse(result['candidate_activated'])
        self.assertEqual(before,(self.root/'active.json').read_bytes())
    def test_changed_source_rejected_before_candidate_creation(self):
        with patch.object(engine,'preflight',return_value=object()),patch.object(engine,'digest_model',return_value='changed'):
            with self.assertRaisesRegex(ValueError,'Source model changed'): t.repair(self.root)
        self.assertEqual([],list(self.root.glob('candidate-*')))
    def test_success_then_rollback_preserves_original_bytes(self):
        original=(self.root/'original/asset.usda').read_bytes()
        def apply(source,baseline,target): target.write_text('verified candidate')
        with patch.object(engine,'preflight',return_value=object()),patch.object(engine,'digest_model',return_value='sig'),patch.object(engine,'operations',return_value=[]),patch.object(t.Usd.Stage,'Open',return_value=object()),patch.object(engine,'apply',side_effect=apply),patch.object(engine,'load_target') as load,patch.object(engine,'evaluate',return_value={'status':'PASS'}):
            load.return_value.mj_model=object(); result=t.repair(self.root)
        self.assertTrue(result['candidate_activated']); self.assertEqual('candidate',t.active_path(self.root)['selection'])
        receipt=t.rollback(self.root)
        self.assertTrue(receipt['original_integrity_verified']); self.assertFalse(receipt['runtime_state_restored'])
        self.assertEqual('original',t.active_path(self.root)['selection'])
        self.assertEqual(original,(self.root/'original/asset.usda').read_bytes())
    def test_tampered_active_candidate_is_not_accepted(self):
        p=self.root/'candidate-example'; p.mkdir(); (p/'asset.usda').write_text('verified')
        digest=t.tree_hash(p)
        t.write_json(self.root/'verification.json',{'status':'PASS','candidate_tree_sha256':digest})
        t.write_json(self.root/'active.json',{'selection':'candidate','asset':'candidate-example/asset.usda','tree_sha256':digest,'verification':'verification.json'})
        (p/'asset.usda').write_text('changed')
        with self.assertRaisesRegex(ValueError,'integrity'): t.active_path(self.root)
        # A bad candidate does not prevent returning to a verified original.
        self.assertEqual('ROLLED_BACK',t.rollback(self.root)['status'])
    def test_unsupported_input_creates_no_workspace(self):
        target=self.root/'never-created'
        with patch.object(engine,'preflight',side_effect=engine.Unsupported('unsupported')):
            with self.assertRaises(engine.Unsupported): t.prepare('source.xml','stand',target)
        self.assertFalse(target.exists())

if __name__=='__main__': unittest.main()
