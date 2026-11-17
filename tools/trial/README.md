# 第一档试用辅助包

这是独立于 v1 wheel 的命令行辅助脚本，不修改物理算法或验收门槛。使用 Python 标准库调用已经安装的工具。不会自动安装依赖、下载网格或上传数据。

先按 v1 README 建立 Python 3.11 环境并安装锁定依赖和 wheel。从包含 `.venv` 的目录运行以下命令，替换尖括号路径；输出目录必须尚不存在：

```powershell
.\.venv\Scripts\python.exe <辅助包路径>/run_trial.py --package <v1源码或解压目录> --output trial-results
```

脚本运行 doctor、离线示例计划、修复、当前选择查询、回退及再次查询。每一步保留日志和可用的 JSON 报告，最终查看 `trial-results/trial-summary.json`。单步超时为 300 秒；遇到非预期结果即停止，保留证据，不删除或覆盖已有工作区。

若已通过 `rpf fetch-demo --asset g1 --directory data` 取得固定 G1 数据，可追加 `--g1-source data/g1/scene_mjx.xml`。这会再运行屈膝负例，直接从修复报告读取候选路径，核对失败候选没有被启用，并保存诊断。预期的 G1 FAIL 是负例复现成功，不是 G1 物理验收通过。

`EXPECTED_RESULTS_VERIFIED` 只说明所选流程得到预期结果，不代表真实用户认可、全新安装成功或任意资产保真。环境安装仍按主 README 执行。试用者请另按 `docs/TRIAL_GUIDE.md` 记录操作障碍与反馈；自动报告明确标记 `external_human_trial: false`。
