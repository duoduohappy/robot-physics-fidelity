"""Owner-guided Go2 verification; records automation separately from interaction."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid


def main():
    if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
    p=argparse.ArgumentParser();p.add_argument('--source',required=True);a=p.parse_args()
    source=Path(a.source).resolve();out=Path.cwd()/'runs'/('go2-auto-owner-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6])
    out.mkdir(parents=True,exist_ok=False);workspace=out/'workspace'
    receipt={'status':'IN_PROGRESS','steps':[],'external_user_trial':False,
             'mode':'interactive_owner_guided' if sys.stdin.isatty() else 'scripted_replay'}
    env=os.environ.copy();env.pop('PYTHONPATH',None);env['PYTHONIOENCODING']='utf-8'
    def run(name,args):
        print('\n'+{'doctor':'1. 检查环境：不改变模型。','auto-plan':'2. 准备官方导出及验收计划：不修改物理参数，也不认证。',
                    'auto-verify':'3. 检查 276 个几何对和四组运动条件：全部通过才认证；此例无需修复。',
                    'auto-active-path':'4. 查询当前认证及所需运行配置。原始官方导出文件本身没有被修复。',
                    'auto-rollback':'5. 撤销认证选择：同一文件保持不变，不恢复正在运行的仿真。'}[name])
        if input('按 Enter 执行，输入 q 停止：').strip().lower()=='q':raise KeyboardInterrupt
        report=out/(name+'-'+str(len(receipt['steps']))+'.json')
        command=[sys.executable,'-m','robot_physics_fidelity',name,*args]
        if name!='doctor':command+=['--report',str(report)]
        with (out/(name+'-'+str(len(receipt['steps']))+'.log')).open('wb') as log:
            r=subprocess.run(command,cwd=Path.cwd(),env=env,stdout=log,stderr=subprocess.STDOUT,timeout=300)
        data=json.loads(report.read_text(encoding='utf-8')) if report.exists() else {}
        receipt['steps'].append({'command':name,'exit_code':r.returncode,'status':data.get('status'),'qualified':data.get('qualified')})
        if r.returncode!=0:raise RuntimeError('结果未达到要求，保留日志后发给我；不会把失败改成通过。')
        print('结果：'+data.get('status','环境检查成功'))
        if name=='auto-verify':
            print('无需修复资产；已按固定 Go2 契约完成验收。')
            for profile in data['profiles']:print('  '+profile['profile']+'：'+profile['status'])
        if 'qualified' in data:print('当前有资格交出资产：'+str(data['qualified']))
        if 'asset' in data:print('资产：'+data['asset'])
        if data.get('requires_recorded_runtime_configuration'):print('使用条件：必须同时采用报告记录的 CPU 运行配置；不是任意仿真配置都适用。')
        return data
    try:
        run('doctor',[])
        run('auto-plan',['--source',str(source),'--workspace',str(workspace)])
        run('auto-verify',['--workspace',str(workspace)])
        selected=run('auto-active-path',['--workspace',str(workspace)])
        if not selected.get('qualified'):raise RuntimeError('尚未认证，停止交出资产')
        run('auto-rollback',['--workspace',str(workspace)])
        final=run('auto-active-path',['--workspace',str(workspace)])
        if final.get('qualified'):raise RuntimeError('撤销后仍认证，请保留记录')
        receipt['status']='EXPECTED_WORKFLOW_COMPLETED'
        print('\n本次撤销的是认证选择，没有制造修复文件，也没有改变原始 Go2。')
    except (KeyboardInterrupt,EOFError):receipt['status']='STOPPED'
    except Exception as exc:receipt.update(status='BLOCKED',error=str(exc));print(str(exc))
    finally:(out/'my-trial.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2),encoding='utf-8')
    print('记录：'+str(out/'my-trial.json'))
    return 0 if receipt['status']=='EXPECTED_WORKFLOW_COMPLETED' else 1


if __name__=='__main__':raise SystemExit(main())
