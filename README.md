# Robot Physics Fidelity 2.0

面向机器人仿真资产的受限物理语义审计、批量验收、可回退修复和诊断研究组件。第二档是可复用工程组件，第三档是附实验的诊断研究模块；不是新物理求解器，也没有真实机器人或 sim-to-real 验证。

[公开轨迹演示](https://robot-physics-fidelity-demo.savory-fox-4497.chatgpt.site) · [开发边界](docs/V2_V3_SCOPE.md) · [查重与方法归属](docs/RELATED_WORK.md)

## 安装与快速运行

Windows、Python 3.11。先确认 `python --version` 为 3.11；也可用已安装 Python 3.11 的绝对路径替代 `python`。依赖安装需要网络，离线场景不需要机器人或额外网格。

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-lock.txt
.\.venv\Scripts\python.exe -m pip install --no-deps releases/robot_physics_fidelity-2.0.0-py3-none-any.whl
.\.venv\Scripts\rpf.exe doctor
.\.venv\Scripts\rpf.exe suite --manifest examples/suite.json --output runs/first
.\.venv\Scripts\rpf.exe suite --manifest examples/suite.json --output runs/repeat --baseline runs/first/suite.json
```

ZIP 中 wheel 位于 `dist/`，将安装命令的 `releases/` 改成 `dist/`。也可从源码 `pip install --no-deps .`。每次输出目录必须不存在。首先检查 `runs/first/suite.json`、`junit.xml` 和每个案例的 `physics.json`、`semantics.json`。

固定验收：0.5 秒 CPU 原生 MuJoCo 对照；身体位置分量最大差 ≤1 mm，qvel 分量最大差 ≤0.01（各自原生单位）；pair/限位参数保持，无额外接触，并要求实际接触证据。没有修改原门槛。

## 第二档：可复用验收组件

- 按身体、关节、自由度、执行器和 pair 端点身份对齐，检查惯量张量、坐标系、关节限位/动力学、执行器增益/限制、碰撞/pair 和求解配置；差异是审核候选，不等于已证实原因。
- 源指纹覆盖锁定 MuJoCo 所公开的数值数组、标量及字节字段，记录字段数量；不是对所有外部文件及任意 USD 的语义等价证明。
- 批量清单、逐案例证据、JUnit 和 Windows CPU CI 示例。已知失败显示为失败证据，并在 JUnit 中 skipped；不算保真通过。
- 基线比较检查环境、输入、初态、状态、语义摘要和误差退化。变更需人工审核，不自动覆盖基线。实际多版本依赖兼容性尚未验证。
- `examples/consume_verified_asset.py` 展示下游只消费已经通过且完整性有效的候选；原始回退资产不被当成已认证资产。

[团队接入流程](docs/TEAM_INTEGRATION.md) · [验收结论](docs/ACCEPTANCE_V2_V3.md)

```powershell
.\.venv\Scripts\rpf.exe plan --source examples/hinge_rig.xml --keyframe hold --workspace runs/rig
.\.venv\Scripts\rpf.exe repair --workspace runs/rig --report runs/rig-result.json
.\.venv\Scripts\rpf.exe active-path --workspace runs/rig
.\.venv\Scripts\rpf.exe rollback --workspace runs/rig
```

修复继续只保留端点、显式 pair 参数及原始限位 solref。先计划、独立候选验证、PASS 才原子启用；失败/证据不足不启用。回退恢复磁盘资产选择，不恢复正在运行的仿真状态；原始资产标为 UNQUALIFIED_ORIGINAL。

## 第三档：有基线和消融的诊断研究

实验比较逐行干预、既有 ddmin、未排序累计恢复和三状态加速度残差排序。先验证全部候选恢复是否足够，再按预算搜索并核查 1-minimal 修复集合。所有干预都只在原生模型副本中，不修改资产。

```powershell
.\.venv\Scripts\python.exe research/benchmark.py --output runs/research --repetitions 5
```

需要先准备候选 USD 再诊断实际资产：

```powershell
$report = Get-Content runs/rig-result.json -Raw | ConvertFrom-Json
.\.venv\Scripts\rpf.exe localize --source examples/hinge_rig.xml --keyframe hold --candidate $report.candidate --method ranked --budget 64 --report runs/localization.json
```

无故障返回 NO_FAILURE；白名单不足返回 UNRESOLVED；预算耗尽返回 BUDGET_EXHAUSTED。CONFIRMED_WITHIN_TEST 只说明找到了当前测试内足以恢复通过、经单项删除验证的集合，不保证唯一真实根因或全局最小基数。

[研究方法](docs/RESEARCH_METHOD.md) · [实验结果](docs/RESEARCH_RESULTS.md)

## 真实公开资产

```powershell
.\.venv\Scripts\rpf.exe fetch-demo --asset apollo --directory data
.\.venv\Scripts\rpf.exe fetch-demo --asset g1 --directory data
```

Apollo 使用 `data/apollo/scene.xml`、`stand`；G1 使用 `data/g1/scene_mjx.xml`、`home` 或 `knees_bent`。将路径相对清单目录写入自己的 suite 清单即可。只下载固定提交清单所需文件，校验 Git blob SHA1，保留资产原许可。无需下载整个数据集。

## 支持边界与退出码

仍限制一个自由根、命名 hinge、显式 primitive pair、零移动体自动碰撞掩码、静态自动碰撞受 pair 契约覆盖、无状态 joint-transmission 执行器，最多 256 body/geom。未支持 equality、tendon、mocap、其他关节类型、mesh 碰撞、未知物理 USD 实例、其他后端和硬件。

退出码：0 对应命令成功或预期流程完成；2 ERROR；3 UNSUPPORTED；4 物理 FAIL；5 INSUFFICIENT；6 语义差异或基线需审核；7 批量意外结果；8 研究 UNRESOLVED；9 研究预算耗尽。批量返回 0 不代表其中已知失败资产通过。上游日志可能混入 stdout，自动化读取 JSON 报告。

当前没有外部团队采用、真实使用者节时或硬件效果证据。试用清单见 `docs/TRIAL_GUIDE.md`。历史 v1 结果与安装包仍保留；第二/三档结论以本版验收与研究报告为准。
