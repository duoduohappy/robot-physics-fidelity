"""Example downstream gate: hand off only a verified candidate, never fallback."""
import argparse
import json
from robot_physics_fidelity.transactions import active_path


def main():
    p=argparse.ArgumentParser();p.add_argument('--workspace',required=True);a=p.parse_args()
    result=active_path(a.workspace)
    if result['selection']!='candidate' or result['qualification']!='PASS_FIXED_TEST_ONLY':
        print(json.dumps({'status':'BLOCKED','reason':'No passing candidate is selected'}))
        return 1
    print(json.dumps({'status':'READY_FOR_FIXED_TEST_HANDOFF','asset':result['asset'],
                      'scope':'Use in your pipeline only after evaluating the same task assumptions'}))
    return 0


if __name__=='__main__':raise SystemExit(main())
