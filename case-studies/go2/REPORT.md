# 接入报告 · Unitree Go2 · 2026-11-17

结论：**原版 Go2 能运行、官方 converter 能导出，但 RPF 2.0 的接入被支持范围检查拒绝。目标资产尚未纳入受支持的修复/验收流程。**这是一份包含失败的工程接入记录，不是业务采用证明。

## 选题与来源

应用场景是将已有 Go2 MJCF 描述纳入 USD/Newton 资产管线。它是本项目自选的公开需求场景，没有外部客户委托。Go2 是未参与前述组件开发测试的四足资产；和 Apollo/G1 同来自 Menagerie，因此独立性限于资产，不等于独立来源或用户。

- 仓库：google-deepmind/mujoco_menagerie
- 固定提交：71f066ad0be9cd271f7ed58c030243ef157af9f4
- 入口：unitree_go2/scene_mjx.xml；初态 home。
- 来源与变体说明：https://github.com/google-deepmind/mujoco_menagerie/blob/71f066ad0be9cd271f7ed58c030243ef157af9f4/unitree_go2/README.md
- 模型许可：BSD-3-Clause，保留上游 Unitree 声明。不是对新版本整仓库许可的推断。

选定前查看过 Go2、H1、Berkeley Humanoid、Booster T1、Fourier N1、Adam Lite 的固定源描述，均未声明显式 contact pair。没有为了凑通过资产不断下载训练或网格数据；只下载了选定 Go2 的必要文件。

## 实际记录

| 步骤 | 结果 | 证据 |
|---|---|---|
| 安装核验 | doctor 退出 0、版本就绪；复用先前已验收的隔离 venv，未声称本次又做了全新安装 | installation-check.log |
| 获取资产 | 21 文件、28,427,945 字节；大小、Git blob SHA1 核对，另存 SHA256 | download.json、asset-lock.json |
| 原生编译 | 14 body、57 geom、13 joint、18 速度自由度、12 执行器 | source-snapshot.json |
| 原模型运行 | home 控制保持 0.5 秒，状态有限，接触峰值 4 | source-rollout.json |
| 本工具 inspect | UNSUPPORTED，退出 3 | inspect.json |
| 本工具 plan | UNSUPPORTED，退出 3；没有生成受管理工作区 | plan.json、intake.json |
| 官方独立转换 | 导出成功，退出 0；未认证、未启用，不是修复 | official-conversion.json |
| 源文件保护 | 21 文件前后 SHA256 全部相同 | source-integrity.json |

本工具没有执行该资产的语义审计、修复和源/目标行为验收，因为支持检查已拒绝。没有候选被启用。原模型的有限状态只证明这次 CPU 回放未出现非有限数，不证明稳定步态、控制效果或真实物理准确性。

## 为什么拒绝

原模型有 23 个移动体自动碰撞几何，显式 pair 数为 0；52 个几何没有名称。v2 当前契约要求零移动体自动掩码、有命名的显式 pair 端点，并要求静态自动碰撞体有 pair 契约覆盖。实际拒绝信息为：

```
Moving-body automatic masks must be zero;
At least one explicit pair required;
Static automatic collider not represented by pair contract
```

不能直接清零掩码或随手补几个脚底 pair 来“接入成功”，因为那会改变原模型的身体/腿部接触集合。没有私自重命名碰撞几何、删碰撞或改物理参数。

## 对项目的启示

本案验证了拒绝行为和源保护，但也暴露当前覆盖不足。不能用“安全拒绝”替代“解决 Go2 转换问题”。若后续决定支持自动碰撞，应单独定义掩码、父子/焊接体过滤、排除列表、几何身份及动态接触证据的保留契约，再做变更前后回归；本次没有扩大开发范围。

未测人工排查基线、节省时间、外部用户体验或真实机器人效果。命令执行秒数只是本机测量，不是人工节时。公开资产案例不等于商业落地。

## 后续标准导入探测

本报告保留了 v2 首次接入被拒绝的历史。后续独立探测发现，复用官方 converter 和 Newton 标准导入，固定 Go2 home 的 0.5 秒 CPU 数值符合既有运动门槛，观察接触对集合一致。该结果没有使用本工具 contactBridge 修复，说明应优先复用上游能力、扩展本项目的验收契约；不能把 v2 的拒绝说成 Newton 缺少自动碰撞能力。

完整规则等价性、多初态身份与生产事务尚未验证，因此 v2 仍不正式支持此资产。详见 [后续开发边界与证据](automatic-collision/DEVELOPMENT_BOUNDARY.md)。
