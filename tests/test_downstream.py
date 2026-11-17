from pathlib import Path
import shutil
import unittest
import uuid
from unittest.mock import patch
from robot_physics_fidelity import downstream as ds


class DownstreamGateTests(unittest.TestCase):
    def setUp(self):
        self.root=(Path.cwd()/'.rpf-test-work'/('loader-'+uuid.uuid4().hex)).resolve();self.root.mkdir(parents=True)
        self.workspace=self.root/'workspace';self.workspace.mkdir()
        (self.workspace/'auto-plan.json').write_text('{}');(self.workspace/'auto-active.json').write_text('{"selection":"unqualified_original"}')
    def tearDown(self):
        if self.root.parent!=(Path.cwd()/'.rpf-test-work').resolve():raise RuntimeError('Unsafe cleanup')
        shutil.rmtree(self.root)
    def test_unqualified_never_calls_model_loader(self):
        with patch.object(ds.at,'active_path',return_value={'qualified':False}),patch.object(ds.auto,'load_usd') as load:
            result=ds.run(self.workspace,self.root/'job')
        load.assert_not_called();self.assertEqual(result['status'],'BLOCKED');self.assertEqual(result['steps_executed'],0)
    def test_integrity_error_never_calls_model_loader(self):
        with patch.object(ds.at,'active_path',side_effect=ValueError('modified')),patch.object(ds.auto,'load_usd') as load:
            result=ds.run(self.workspace,self.root/'job')
        load.assert_not_called();self.assertFalse(result['model_created'])
    def test_job_cannot_write_inside_managed_asset(self):
        with self.assertRaises(ValueError):ds.run(self.workspace,self.workspace/'original/job')
    def test_lease_detects_revocation(self):
        before=ds.lease(self.workspace)
        (self.workspace/'auto-active.json').write_text('{"selection":"revoked"}')
        with self.assertRaises(ds.Blocked):ds.ensure_lease(self.workspace,before)


if __name__=='__main__':unittest.main()
