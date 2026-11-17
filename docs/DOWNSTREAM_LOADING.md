# 2.2：认证控制实际加载与推进

本版解决一个具体接入问题：上游报告通过，不应意味着下游还能随意加载未经认证、已撤销或已变更的模型。`rpf auto-run` 是真正创建并推进目标模型的受限 CPU 入口，不只是打印认证状态。

## 复现

安装根目录 requirements-lock.txt 和 2.2.0 wheel，使用 Python 3.11。输出目录须不存在。

```powershell
rpf fetch-demo --asset go2 --directory demo-data
rpf auto-plan --source demo-data/go2/scene_mjx.xml --workspace runs/go2
rpf auto-verify --workspace runs/go2 --report runs/go2-verification.json
rpf auto-run --workspace runs/go2 --output runs/loaded --profile raised
rpf auto-rollback --workspace runs/go2
rpf auto-run --workspace runs/go2 --output runs/revoked
```

第一次 auto-run 预期 COMPLETED、250 步、0.5 秒；撤销后的命令预期 BLOCKED、退出码 10、未调用加载器且 0 步。每次输出目录的 load-result.json 是实际回执。auto-run 不接受任意运行配置，只允许 home、raised、joint-offset、control-offset 四种已认证条件。

完整对照（含人工注入的三倍基座质量故障、撤销、字节变更、运行中撤销）：

```powershell
python tests/run_downstream_demo.py --source demo-data/go2/scene_mjx.xml --output runs/gated-demo
python examples/render_gated_run.py --source demo-data/go2/scene_mjx.xml --receipt runs/gated-demo/normal-job/load-result.json --output runs/gated-video
```

视频渲染另外需要可用的 ffmpeg。运行验收无需 ffmpeg 或 GPU。渲染器仅用原模型外观网格显示实际目标模型的状态，不使用源模型运动轨迹替代目标轨迹。

## 加载约束

入口核对认证选择、报告、原始资产指纹和锁定环境。资产复制到作业独立快照后，逐项比对快照哈希，再调用 Newton 标准 USD 导入和 SolverMuJoCo 生成目标原生 MuJoCo CPU 模型；使用报告记录的运行配置。运行前、每步和结束时检查授权，保留真实步数和事件；运行中撤销会停止后续推进，不能撤销已经执行的步骤。结束时再次核对完整性。

这是协作式本地文件授权检查，不是操作系统安全隔离：不防御有能力篡改程序、同时恶意改写整个工作区的攻击者；快照也不是系统级不可写挂载。调用者直接绕开入口自行启动引擎时，不受本入口约束。

## 验收解释

- 正常：上游 PASS，下游确实创建目标模型并推进 250 步。
- 人工故障：在计划指纹生成前注入三倍基座质量；上游验收失败，下游未调用加载器。
- 撤销：文件不变，认证选择撤销，下游未调用加载器。
- 变更：通过后追加 USD 注释，字节指纹不符，下游未调用加载器，即使注释本身不改变物理。
- 运行中撤销：控制测试在第 3 步后撤销；保留已执行 3 步，拒绝第 4 步，不交出成功结果。

边界仍是固定 Go2、四组短时条件、指定 CPU 配置。不是通用资产认证、Newton 外层状态同步验证、机器人控制器、真机或 sim-to-real 证明。人工故障不代表已发现真实转换器缺陷，自动化对照也不代表独立用户或企业业务采用。

[公开加载对照演示](https://robot-physics-fidelity-demo.savory-fox-4497.chatgpt.site/go2-gate/index.html) 展示历史执行回执和视频；标签只切换证据，不在浏览器执行仿真，也不查询当前本地认证。
