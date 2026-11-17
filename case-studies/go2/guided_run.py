"""Guided owner self-trial; never represented as an independent user study."""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid


def main():
    if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package',help='v2 source/extracted directory')
    parser.add_argument('--asset-directory',help='Previously fetched original Go2 directory')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[2]
    package=Path(args.package).resolve() if args.package else root/'outputs/robot-physics-fidelity-v2'
    if not args.package and not (package/'examples/pair_only.xml').is_file():package=root
    if not (package/'examples/pair_only.xml').is_file():parser.error('Specify --package containing examples/pair_only.xml')
    asset=Path(args.asset_directory).resolve() if args.asset_directory else root/'work/go2-original'
    if not args.asset_directory and not asset.exists():asset=root/'data/go2'
    output=root/'runs'/('owner-trial-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6])
    output.mkdir(parents=True,exist_ok=False)
    receipt={'status':'IN_PROGRESS','guide_version':'1.1','execution_mode':'interactive_owner_self_trial' if sys.stdin.isatty() else 'scripted_replay',
             'external_user_trial':False,'operator_identity_verified':False,'steps':[],'feedback':{},'output':str(output)}
    env=os.environ.copy();env.pop('PYTHONPATH',None);env['PYTHONIOENCODING']='utf-8'

    def save():
        (output/'my-trial.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2),encoding='utf-8')
    def pause(text):
        print('\n'+text)
        answer=input('按 Enter 执行；输入 q 结束并保留记录：').strip()
        if answer.lower()=='q':raise KeyboardInterrupt
    def run(name,args,status=None,code=0):
        report=output/(name+'.json') if status else None
        command=[sys.executable,'-m','robot_physics_fidelity',*args]
        if report:command+=['--report',str(report)]
        print('执行：rpf '+' '.join(args),flush=True)
        with (output/(name+'.log')).open('wb') as file:
            result=subprocess.run(command,cwd=root,env=env,stdout=file,stderr=subprocess.STDOUT,timeout=300)
        payload=json.loads(report.read_text(encoding='utf-8')) if report and report.exists() else {}
        receipt['steps'].append({'step':name,'exit_code':result.returncode,'expected_exit_code':code,'status':payload.get('status'),'expected_status':status})
        save()
        if result.returncode!=code or (status and payload.get('status')!=status):
            raise RuntimeError(name+' 结果不符合预期，查看 '+str(output/(name+'.log')))
        print('结果：'+str(payload.get('status','命令成功')))
        explanations={
            'PLANNED':'已列出准备做的修改。此时没有生成通过验收的修复候选。',
            'PASS':'候选已经接受固定测试并达到门槛；仅对这个初态、控制和时长有效。',
            'ROLLED_BACK':'已把资产选择恢复到原始文件。这不是重新验收，也不恢复运行中的仿真状态。',
            'UNSUPPORTED':'工具检查后停止：输入超出支持范围。尚未做行为验收，不是在判断机器人模型物理不合格。'}
        if payload.get('status') in explanations:print('这意味着：'+explanations[payload['status']])
        if name=='installation':print('这意味着：工具已安装且依赖版本符合当前要求；还没有检查任何机器人资产。')
        if payload.get('selection')=='candidate':print('当前使用：修复候选文件，已通过本次固定测试。后续程序仍需主动读取这个路径。')
        elif payload.get('selection')=='original':print('当前使用：原始文件；UNQUALIFIED_ORIGINAL 表示它尚未被认证，不代表它一定错误。')
        if payload.get('status')=='UNSUPPORTED':
            print('Go2 的具体原因：它用自动碰撞规则，而 v2 仅接受命名的显式接触对。不要把掩码清零或只补几个脚底接触来凑通过。')
            print('本案接入结果：尚未成功；安全停止是正确行为，但没有解决 Go2 的转换保真问题。')
        for key in ['candidate_activated','selection','qualification','asset','runtime_state_restored','error']:
            if key in payload:print(str(key)+' = '+str(payload[key]))
        return payload

    print('机器人仿真资产工具：本人操作练习')
    print('先用已支持的离线示例练习完整闭环，再检查原版 Go2。Go2 预期被拒绝，不会被改成支持模型。')
    print('脚本不安装依赖、不下载网格、不上传反馈。每个步骤先说明，再由你按 Enter 执行。')
    print('记录目录：'+str(output))
    print('\n先理解三个不同概念：')
    print('  通过 PASS：已经测试，达到指定门槛。')
    print('  拒绝 UNSUPPORTED：工具不支持，先停止，尚未验收。')
    print('  回退 ROLLED_BACK：改回使用原始文件，和是否通过测试是两回事。')
    print('\n碰撞规则：自动碰撞由引擎按几何掩码等规则筛选；显式接触对是直接指定某两个几何可以接触。')
    print('两种机制可以同时存在。Go2 使用自动碰撞；本次不会修改它的规则。')
    try:
        if importlib.metadata.version('robot-physics-fidelity')!='2.0.0':raise RuntimeError('请使用安装了 v2.0.0 的 Python 环境')
        pause('第 1 步：检查安装环境。成功后可打开 installation.log 查看 READY 和依赖版本。')
        run('installation',['doctor'])
        workspace=output/'offline-workspace'
        pause('第 2 步：生成修复计划。此时还没有完成修复或通过验收。')
        run('plan',['plan','--source',str(package/'examples/pair_only.xml'),'--keyframe','home','--workspace',str(workspace)],'PLANNED')
        pause('第 3 步：生成候选并验收。预期 PASS，candidate_activated 为 true。')
        repaired=run('repair',['repair','--workspace',str(workspace)],'PASS')
        if repaired.get('candidate_activated') is not True:raise RuntimeError('候选没有启用')
        pause('第 4 步：查询当前资产。预期 selection=candidate；路径指向已验证候选。')
        selected=run('active',['active-path','--workspace',str(workspace)],'SELECTED')
        if selected.get('selection')!='candidate':raise RuntimeError('当前不是候选资产')
        pause('第 5 步：回退并再次查询。预期恢复 original，但原始资产不是已经认证的资产；运行中的仿真状态也不会恢复。')
        run('rollback',['rollback','--workspace',str(workspace)],'ROLLED_BACK')
        selected=run('restored',['active-path','--workspace',str(workspace)],'SELECTED')
        if selected.get('selection')!='original':raise RuntimeError('回退没有恢复原始资产选择')
        pause('第 6 步：检查未改动的 Go2。预期 UNSUPPORTED（退出码 3），说明自动碰撞超出 v2 支持范围。不是安装失败，也不是 Go2 模型损坏。')
        lock=json.loads(Path(__file__).with_name('asset-lock.json').read_text(encoding='utf-8'))
        hashes={}
        for row in lock['files']:
            f=asset/Path(*Path(row['path']).parts[1:])
            if not f.is_file():raise RuntimeError('Go2 文件缺失；先按说明显式下载：'+str(f))
            data=f.read_bytes();git=hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
            if len(data)!=row['bytes'] or git!=row['git_blob_sha1']:raise RuntimeError('Go2 文件与固定原版不一致：'+str(f))
            hashes[str(f)]=hashlib.sha256(data).hexdigest()
        run('go2-inspect',['inspect','--source',str(asset/'scene_mjx.xml'),'--keyframe','home'],'UNSUPPORTED',3)
        rejected=output/'go2-workspace'
        run('go2-plan',['plan','--source',str(asset/'scene_mjx.xml'),'--keyframe','home','--workspace',str(rejected)],'UNSUPPORTED',3)
        if rejected.exists():raise RuntimeError('不支持输入创建了工作区，请检查')
        unchanged=all(hashlib.sha256(Path(f).read_bytes()).hexdigest()==h for f,h in hashes.items())
        if not unchanged:raise RuntimeError('原始 Go2 文件发生变化')
        receipt['source_files_unchanged']=True
        print('\n命令流程已完成；Go2 仍未成功接入。离线小模型通过、Go2 被拒绝、回退选择原始文件，三者含义不同。')
        print('下面三项请按实际感受填写，可直接按 Enter 留空；不会自动填写成功评价。')
        for key,prompt in [('unclear_step','哪一步没看懂？'),('help_needed','过程中需要了什么帮助？'),('evidence_clarity','通过、拒绝和回退的区别是否清楚？')]:
            receipt['feedback'][key]=input(prompt+' ')
        receipt['status']='EXPECTED_WORKFLOW_COMPLETED'
    except (KeyboardInterrupt,EOFError):
        receipt['status']='STOPPED_BY_OPERATOR_OR_NO_INPUT'
    except Exception as exc:
        receipt.update(status='BLOCKED',error=str(exc));print(str(exc))
    finally:save()
    print('\n结果：'+receipt['status'])
    print('请把下面这个文件的内容或路径发回给我：\n'+str(output/'my-trial.json'))
    return 0 if receipt['status']=='EXPECTED_WORKFLOW_COMPLETED' else 1


if __name__=='__main__':raise SystemExit(main())
