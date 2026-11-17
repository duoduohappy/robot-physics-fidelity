# 团队接入与基线审批

这是可运行的接入适配器和 CI 示例，尚无外部团队采纳证据。

1. 用 `examples/suite.json` 描述资产、命名初态和预期状态。路径相对清单文件。预期 FAIL/INSUFFICIENT/UNSUPPORTED 必须写原因，不可把所有失败配置成预期来伪造保真通过。
2. `rpf suite --manifest examples/suite.json --output runs/first` 生成各案例的 USD 候选、物理报告、语义报告、汇总 JSON 和 JUnit。输出目录必须不存在，避免覆盖历史证据。
3. 审核输入指纹、初态、结果和语义差异后，将这份 `suite.json` 作为批准基线保存在版本控制中。工具不会自动批准基线。
4. 升级组件或依赖后运行 `rpf suite --manifest examples/suite.json --output runs/next --baseline runs/first/suite.json`。环境、案例集合、源指纹、初态、状态、语义摘要变化，或误差明显增加时返回 REVIEW_REQUIRED（退出码 6）；意外状态返回 7。没有基线时只能说明得到预期结果，不能说明没有版本回归。
5. 依赖仍严格锁定。本交付实际验证同版本重复回放及注入的版本元数据变化检测；未实际安装和测试多个 Newton/MuJoCo 版本。修改锁文件前需建立兼容性实验，不可绕过 doctor 后把结果算作受支持。
6. `examples/consume_verified_asset.py` 演示下游消费：重新校验资产与通过证据，只交出选定且通过固定测试的候选路径；原始回退资产会阻断。它不是机器人控制器或正在运行的仿真器插件。

`.github/workflows/physics-acceptance.yml` 为 Windows CPU 离线场景工作流。依赖安装需网络，测试场景不下载模型，证据通过 artifact 保存。已知非通过案例在 JUnit 中标记 skipped 并保留原因；不能把 skipped 计作模型通过。

审计在评价器显式同步求解配置之前记录差异；几何默认接触属性与显式 pair 参数不同，坐标系分解不同、未启用的限制区间等不一定构成物理故障。报告分类帮助审核，但不自动推断因果。当前自动修复白名单沿用第一档，不会把所有审计差异强行抹平。
