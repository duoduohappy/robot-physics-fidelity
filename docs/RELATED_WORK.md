# 查重与方法归属 · 2026-11-17

本次检索针对 Newton USD 保真审计、MuJoCo 参数差异诊断、差分仿真验收、Delta Debugging 及加权最小化。搜索未发现完全相同交付组合不构成全球原创性证明。

1. Newton 官方已有 USD schema resolver、关节/驱动属性映射及导入能力。本项目复用这些实现，不把导入或参数映射本身当作贡献。https://github.com/newton-physics/newton/blob/main/docs/concepts/usd_parsing.rst
2. Newton 官方已有惯量验证测试和执行器库。审计质量与回归流程属于工程集成，不宣称该类别从未存在。https://github.com/newton-physics/newton-actuators
3. Zeller 的 Delta Debugging 已系统研究最小化失败诱因。本研究以 PASS 为保留谓词，寻找足以恢复验收的集合，ddmin 是既有基线。https://arxiv.org/abs/cs/0012009
4. WDD（ICSE 2025）已有加权 Delta Debugging。不能把“给候选排序/加权”直接宣称为原创。本文实验使用不同的领域分数：三个相同状态下的相对加速度残差下降；不是 WDD 的输入片段大小权重，也不声称整个思想新颖。https://arxiv.org/abs/2411.19410
5. MuJoCo 官方说明模型修改的安全边界。本研究只修改候选白名单中的动力学参数、深拷贝原生模型、重建数据，不改拓扑或质量/惯量等派生常量，不覆盖源文件。https://mujoco.readthedocs.io/en/stable/programming/simulation.html

目前可以主张的贡献范围：绑定固定机器人转换链路与门槛的验收组件；保留失败证据的修复事务；可复现实验中比较领域残差排序与既有最小化方法的效果。不能主张新物理求解器、通用真实因果发现、已解决 sim-to-real 或已证明全球唯一。
