"""Run installed RPF CLI without importing its source or parsing console logs."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', required=True, help='Extracted v1 package/repository directory')
    parser.add_argument('--output', required=True, help='New evidence directory; must not exist')
    parser.add_argument('--g1-source', help='Optional previously fetched G1 scene_mjx.xml')
    args = parser.parse_args()
    package = Path(args.package).resolve()
    source = package / 'examples' / 'pair_only.xml'
    if not source.is_file():
        parser.error('Package must contain examples/pair_only.xml')
    g1 = Path(args.g1_source).resolve() if args.g1_source else None
    if g1 and not g1.is_file():
        parser.error('G1 source does not exist; fetch it using rpf fetch-demo first')
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env.pop('PYTHONPATH', None)
    env['PYTHONIOENCODING'] = 'utf-8'
    steps = []
    summary = {'kind': 'automated_reproduction_only', 'external_human_trial': False,
               'python': sys.executable, 'steps': steps, 'status': 'RUNNING'}

    def run(name, arguments, expected_status=None, expected_code=0):
        report = output / (name + '.json') if expected_status else None
        command = [sys.executable, '-m', 'robot_physics_fidelity', *arguments]
        if report:
            command += ['--report', str(report)]
        start = time.monotonic()
        with (output / (name + '.log')).open('wb') as log:
            result = subprocess.run(command, cwd=output, env=env, stdout=log,
                                    stderr=subprocess.STDOUT, timeout=300)
        payload = json.loads(report.read_text(encoding='utf-8')) if report and report.exists() else None
        record = {'name': name, 'exit_code': result.returncode,
                  'expected_exit_code': expected_code, 'seconds': round(time.monotonic()-start, 3),
                  'status': payload.get('status') if payload else None,
                  'expected_status': expected_status}
        steps.append(record)
        if result.returncode != expected_code or (expected_status and record['status'] != expected_status):
            raise RuntimeError(f'{name}: unexpected result; see {name}.log and JSON')
        print(name + ': expected result verified', flush=True)
        return payload

    try:
        run('doctor', ['doctor'])
        workspace = output / 'offline-workspace'
        run('offline-plan', ['plan', '--source', str(source), '--keyframe', 'home',
                            '--workspace', str(workspace)], 'PLANNED')
        repaired = run('offline-repair', ['repair', '--workspace', str(workspace)], 'PASS')
        if repaired.get('candidate_activated') is not True:
            raise RuntimeError('Passing offline candidate was not activated')
        selected = run('offline-active', ['active-path', '--workspace', str(workspace)], 'SELECTED')
        if selected.get('selection') != 'candidate':
            raise RuntimeError('Expected candidate selection')
        receipt = run('offline-rollback', ['rollback', '--workspace', str(workspace)], 'ROLLED_BACK')
        if receipt.get('runtime_state_restored') is not False:
            raise RuntimeError('Rollback receipt must state runtime state was not restored')
        restored = run('offline-restored', ['active-path', '--workspace', str(workspace)], 'SELECTED')
        if restored.get('selection') != 'original':
            raise RuntimeError('Expected original selection after rollback')
        if g1:
            workspace = output / 'g1-workspace'
            run('g1-plan', ['plan', '--source', str(g1), '--keyframe', 'knees_bent',
                            '--workspace', str(workspace)], 'PLANNED')
            failed = run('g1-repair', ['repair', '--workspace', str(workspace)], 'FAIL', 4)
            if failed.get('candidate_activated') is not False:
                raise RuntimeError('Failed G1 candidate must not activate')
            current = run('g1-active', ['active-path', '--workspace', str(workspace)], 'SELECTED')
            if current.get('selection') != 'original':
                raise RuntimeError('Failed G1 case must retain original selection')
            diagnosis = run('g1-diagnosis', ['diagnose', '--source', str(g1), '--keyframe',
                            'knees_bent', '--candidate', failed['candidate']], 'FAIL', 4)
        summary['status'] = 'EXPECTED_RESULTS_VERIFIED'
    except Exception as exc:
        summary.update(status='REPRODUCTION_BLOCKED', error=str(exc))
    finally:
        (output / 'trial-summary.json').write_text(
            json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    print(summary['status'])
    return 0 if summary['status'] == 'EXPECTED_RESULTS_VERIFIED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
