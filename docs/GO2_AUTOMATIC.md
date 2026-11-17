# 固定 Go2 自动碰撞验收通道 · 2.1

这个新通道复用官方 converter、Newton 标准 USD 导入和原生 CPU 求解模型。没有写新碰撞检测算法，不改源文件、不做自动调参。旧的显式 pair 通道和旧的拒绝边界保持不变。

## 使用

先安装主 README 的锁定依赖和 2.1.0 wheel，再显式获取固定 Go2 资产（约 27.1 MiB）：

```powershell
rpf fetch-demo --asset go2 --directory data
rpf auto-plan --source data/go2/scene_mjx.xml --workspace runs/go2
rpf auto-verify --workspace runs/go2 --report runs/go2-result.json
rpf auto-active-path --workspace runs/go2 --report runs/go2-active.json
rpf auto-rollback --workspace runs/go2 --report runs/go2-rollback.json
```

`auto-plan` 生成官方导出及不可变副本、记录源文件哈希和版本，不认证。`auto-verify` 只有完整验收通过才认证，输出 `repair_required: false` 和 `asset_modified: false`。此例无需修复资产。

`auto-active-path` 返回资产路径、资格和**必须采用的运行配置**。不能只取路径然后宣称任意仿真器/配置已通过。这里验证的是重建的原生 CPU 模型，不含 Newton 外层 state/control 同步。

`auto-rollback` 撤销认证选择，同一个原始官方导出文件保持不变；不会虚构候选文件切换，也不恢复运行中机器人状态。成功返回退出码 0，物理 FAIL 为 4，不支持为 3，证据不足为 5，输入/完整性错误为 2。

只读比对已有导出可用 `auto-validate --source ... --candidate ... --report ...`，不会认证或激活工作区。

## 支持边界

- 只接受固定 Menagerie 提交的 Go2 MJX 原始 21 文件，核对大小/Git blob SHA1，存储 SHA256。不是任意自动碰撞机器人模型支持。
- 一个自由根、hinge、原始统一 1/1 掩码、23 球体＋1 平面、无源显式 pair/exclude/约束/flex/plugin/状态执行器；改过的源文件先拒绝。
- 目标必须完整映射 24 个有效碰撞几何，来源绑定源指纹/索引、身体身份、USD 路径、有效物理尺寸和三个同状态位置。球体只比较半径，不把未使用的 size 分量误判成形状变化；歧义/缺失拒绝，不用最近邻猜测。
- 276 个几何对全部检查配置无关的碰撞资格：类型、mask、同体/焊接体、父子过滤、双静态和排除列表。比较有效允许关系，不要求重新编码后的整数掩码逐项相等。空间包围体/窄相不属于这张静态表，由几何验证和动态测试覆盖。
- sleep、显式 pair、柔性体、mocap 和自定义接触/控制回调不在规则检查范围。Newton 可能生成一个**未启用的 equality 占位项**；允许它默认且持续未启用，拒绝已启用约束。报告保留 required_default_equality_active。

## 四组固定条件

都从原版 home 派生，不改原 keyframe 文件：home；自由根提高 0.06m；hinge 位置交替偏移 0.02rad；控制交替偏移 0.015（原生单位）。每组均 0.5 秒，位置分量 ≤1mm、qvel 分量 ≤0.01；源/目标都必须实际接触，逐步接触对多重集合一致，且数值有限。

这是一组明确的固定验收条件，不是长时间运动、策略训练或硬件验证。不把源配置与目标默认配置自然相等作为前提：验收显式绑定记录的源运行配置。

## 故障与完整性

受控 USD 故障包括矛盾掩码、禁用球体、错误半径、同体重复形状造成身份歧义、质量变化。故障发生在计划记录之前，模拟错误上游导出，不宣称这些都是当前转换器真实缺陷。规则单元测试还覆盖未发生接触时的掩码错误，并对照原生 MuJoCo 接触验证过滤公式。

失败和证据不足不认证；不完整的 PASS 报告也不认证。源变化、资产树变化、回执篡改、运行版本变化或路径越界会阻止再次交出资产。哈希用于完整性检查，不是对有权改写整个工作区的恶意用户的数字签名。

## 方法来源

资格公式按 [MuJoCo 3.17.0 collision driver](https://github.com/google-deepmind/mujoco/blob/3.17.0/src/engine/engine_collision_driver.c) 的 filterBodyPair、filterBitmask 和 exclude 逻辑限定实现，并用原生接触作独立检查；不是原创算法。上游导入能力见 [Newton 1.8.0](https://newton-physics.github.io/newton/1.8.0/solvers/mujoco.html)。
