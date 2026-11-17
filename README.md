# Robot Physics Fidelity · 第一档工具 v1.0

面向机器人仿真资产的**受限诊断、修复与验收命令行工具**。本次升级只完成三类工程能力：定位显著分歧、验证后启用且可回退的资产修复、独立安装和复现。

[公开轨迹演示](https://robot-physics-fidelity-demo.savory-fox-4497.chatgpt.site) · [项目仓库](https://github.com/duoduohappy/robot-physics-fidelity)

## 当前证据

在全新 Python 3.11 venv 中安装 wheel，禁用系统 site-packages，清空 `PYTHONPATH`，从固定源网格重新生成 USD：

| 案例 | 验收 | 候选被启用 | 身体位置最大差 |
|---|---|---|---:|
| Apollo / stand | PASS | 是 | 0.000287 mm |
| G1 / home | PASS | 是 | 0.000621 mm |
| G1 / knees_bent | FAIL | 否 | 0.195096 mm，速度超限 |
| 随包离线基本几何示例 | PASS | 是 | 小于 0.000001 mm |
| 高空无接触示例 | INSUFFICIENT | 否 | 缺少接触证据，不能判通过 |

固定标准不变：0.5 秒；身体位置分量最大差 ≤1 mm；qvel 分量最大差 ≤0.01（各分量原生单位）。此外必须保留支持的 pair/限位参数、不得增加额外接触，且源/目标均有实际接触。配置 pair 全部保留不等于每条都动态触发。

19 次 CLI 命令得到预期退出状态；8 项故障注入测试覆盖失败不启用、完整性检查、越界路径、源变化、初态与验收证据绑定及回退。它们不代替物理测试。

G1 的自动诊断定位到首次超限为第 17 步、0.068 秒，最大速度分歧对应 `left_shoulder_yaw_joint`；此时最大位置分歧身体为 `right_ankle_pitch_link`。输出相同 qpos/qvel/ctrl/加速度 warmstart/time 下的接触位置、法向、距离与力，并执行有限参数对照。**根因仍为 UNRESOLVED**；不会根据数组不同直接宣称找到了原因。

## 安装与离线示例（Windows / Python 3.11）

从仓库或交付包取得源码与 `releases/robot_physics_fidelity-1.0.0-py3-none-any.whl`。本地交付目录中的 wheel 在 `dist/`。

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-lock.txt
.\.venv\Scripts\python.exe -m pip install --no-deps releases/robot_physics_fidelity-1.0.0-py3-none-any.whl
.\.venv\Scripts\rpf.exe doctor
.\.venv\Scripts\rpf.exe plan --source examples/pair_only.xml --keyframe home --workspace runs/example
.\.venv\Scripts\rpf.exe repair --workspace runs/example --report runs/example-result.json
.\.venv\Scripts\rpf.exe active-path --workspace runs/example
.\.venv\Scripts\rpf.exe rollback --workspace runs/example
```

若使用本地 ZIP，第三行改为 `dist/robot_physics_fidelity-1.0.0-py3-none-any.whl`。依赖安装需要网络；示例本身不需要联网或下载机器人。无需激活 PowerShell 环境脚本，也不依赖原开发工作区目录。

本次干净环境测试使用新安装的约 628 MiB 运行依赖，未使用旧依赖目录或旧内核缓存；真实模型复用了已校验的**只读原始网格缓存**。这不是另一个人的试用，也不宣称 Linux/macOS 已验收。`doctor` 检查版本，不代替后续真实运行。

## 真实资产流程

```powershell
.\.venv\Scripts\rpf.exe fetch-demo --asset apollo --directory data
.\.venv\Scripts\rpf.exe fetch-demo --asset g1 --directory data
.\.venv\Scripts\rpf.exe inspect --source data/g1/scene_mjx.xml --keyframe knees_bent
.\.venv\Scripts\rpf.exe plan --source data/g1/scene_mjx.xml --keyframe knees_bent --workspace runs/g1-knees
.\.venv\Scripts\rpf.exe repair --workspace runs/g1-knees --report runs/g1-knees-result.json
.\.venv\Scripts\rpf.exe diagnose --source data/g1/scene_mjx.xml --keyframe knees_bent --candidate runs/g1-knees/candidate-<生成的标识>/asset.usda --report runs/g1-diagnosis.json
```

`repair` 的报告给出候选路径，不要手写标识。G1 屈膝预期退出 4，当前选择仍为原始资产。Apollo 使用 `data/apollo/scene.xml` 和 `stand`；G1 home 使用 `home`。下载仅包含固定提交所需网格、XML 和原许可，并逐文件验证 Git blob SHA1；不自动下载全仓库。

`plan` 工作区必须不存在；转换或检查中断时保留证据，重试使用新工作区。计划记录接触端点、pair 参数、明确限位参数与非物理实例准备操作，未执行候选修复。执行前再次核对选定编译数组的源签名、操作清单和原始 USD 树哈希；记录的签名或内容不一致时拒绝执行。该签名不是任意模型全部语义的完整证明。通过证据另与源签名和指定初态绑定。

## 修复、启用和回退的真实含义

- 仅支持保留显式 pair 所需端点、pair 有效参数、明确源限位 `solref`；不做自动调参。
- 生成独立候选 USD，重新导入和验收；只有 PASS 才原子更新 `active.json`。FAIL、INSUFFICIENT 或错误不作为成功候选启用。
- `active-path` 验证哈希及通过证据后返回当前选定文件；它不控制外部仿真器，消费者需要使用返回路径。
- `rollback` 校验原始副本后恢复**磁盘资产选择指针**；不恢复正在运行的机器人/仿真器状态。原始资产可能本就不保真，所以标记 `UNQUALIFIED_ORIGINAL`，回退不是对原始资产的认证。
- 不覆盖源 XML 或已有原始 USD。候选、失败报告及回退回执保留。文件哈希用于本工具的完整性检查，不是对恶意用户篡改整个工作区的签名防护。

## 状态和退出码

| 状态 | 退出码 | 意义 |
|---|---:|---|
| PASS | 0 | 当前固定验收通过 |
| PLANNED / SUPPORTED / SELECTED / READY | 0 | 对应命令成功，不等于模型保真验收通过 |
| ERROR | 2 | 输入、文件、运行或完整性错误；检查原始报告 |
| UNSUPPORTED | 3 | 超出范围，拒绝尝试该修复 |
| FAIL | 4 | 已执行验收但未通过，候选不启用 |
| INSUFFICIENT | 5 | 数值符合但缺少接触证据，候选不启用 |

标准输出可能含上游诊断日志。自动化应读取 `--report` 生成的 JSON，不依赖截取控制台文本。

## 支持范围

一个自由根、命名铰链、基本几何 pair 端点、移动体自动碰撞掩码为零、静态 world 碰撞均由显式 pair 覆盖、单一关节传动执行器且无状态动态；最多 256 个 body/geom。可保留真实机器人视觉网格；不把 mesh 碰撞或任意接触模型宣称为已支持。

不支持 tendon、实际 equality、mocap、多个自由根、状态执行器、其他关节类型、含物理内容的原生 USD instance 等。歧义身份与未标记碰撞端点拒绝导入。部分检查必须在官方转换后才能完成；中断会留下未启用的准备错误记录。

诊断输入是本实验契约的候选 USD；任意未知 USD 不会自动猜测其含义。对未修复官方转换缺口，先看 `plan` 的差异清单。源初态、控制和积分配置显式按身份对齐；并不声称全部配置已自动往返。

未验证真实硬件、长期任务、其他物理后端、完整传感器往返、Newton state/control 同步，也未测得节省人工时间。不是新物理算法，无全球原创性证明。

## 证据与真实试用

`evidence/clean/acceptance.json`、`environment.json`、物理验收与回退报告是原始本机证据。`docs/ACCEPTANCE.md` 为交付结论，`docs/TRIAL_GUIDE.md` 为交给另一位工程师的试用任务。**真实外部使用者试用尚未开展**，没有伪造反馈或代替其签收。

依赖和公开资产遵循各自许可；模型原许可见随包文件。新 wheel 不包含大型网格或二进制依赖。
