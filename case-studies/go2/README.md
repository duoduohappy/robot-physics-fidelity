# Go2 独立公开资产接入案例

**接入目标尚未达成：RPF 2.0 不支持此原版模型的自动碰撞。**已完成资产选择、许可核实、原模型运行、官方导出实验和本工具拒绝流程的记录。没有把模型修改成支持范围，也没有实际客户采用或第三方工程师评价。

选择固定 MuJoCo Menagerie 提交中的 Unitree Go2 MJX 描述，区别于开发时使用的 Apollo/G1。它仍来自同一公开资产集合，不是独立机构的数据。选取 MJX 描述不表示本测试使用 JAX/MJX：实际使用 MuJoCo CPU。

## 你现在亲自操作

在本机打开本目录，双击 **开始亲自操作.cmd**。

1. 黑色窗口会先说明任务，按 Enter 执行当前步骤；输入 q 可随时停止。
2. 先练习离线小模型：检查环境 → 生成计划 → 修复并验收 → 查询当前资产 → 回退。
3. 再检查原版 Go2：应显示 UNSUPPORTED 和自动碰撞拒绝原因，不生成修复工作区。
4. 最后写三条简短反馈；可以留空，脚本不会代填成功评价。
5. 窗口会显示 `runs/owner-trial-日期-标识/my-trial.json` 的绝对路径，把该文件内容或路径发回，我再根据你实际遇到的问题指导。

这是在向导帮助下的项目所有者自测，不算独立工程师试用。报告不会自动声称你已经理解所有原理。脚本不安装依赖、不下载模型、不上传反馈、不改原版模型。

如双击无法运行，在 PowerShell 中执行（当前已准备好的本机路径）：

```powershell
cd <你的本机项目根目录>
.\work\rpf-v2-final-env\Scripts\python.exe .\outputs\go2-independent-intake\guided_run.py
```

本机入口使用已验收的 Python 环境，不代表你独立完成了新环境安装。遇到红字先保留窗口和日志，不要自行修改 XML 或放宽门槛。

## 换电脑或从仓库复现

先按仓库主 README 安装 Python 3.11、锁定依赖与 2.0.0 wheel。以下路径以仓库根目录为例：

```powershell
python case-studies/go2/fetch_asset.py --directory data/go2 --report runs/go2-download.json
python case-studies/go2/source_check.py --source data/go2/scene_mjx.xml --report runs/go2-source.json
rpf inspect --source data/go2/scene_mjx.xml --keyframe home --report runs/go2-inspect.json
rpf plan --source data/go2/scene_mjx.xml --keyframe home --workspace runs/go2-workspace --report runs/go2-plan.json
```

后两条的预期退出码是 **3**。在 PowerShell 中可用 `$LASTEXITCODE` 查看；不要把它改为 0 或误称通过。`runs/go2-workspace` 应不存在。

下载为 21 个固定文件，共 28,427,945 字节（约 27.1 MiB），上限 32 MiB。逐文件校验大小与 Git blob SHA1，源记录补充 SHA256；不是下载整个仓库。保留上游 LICENSE，模型采用 BSD-3-Clause；许可文本见 UPSTREAM-LICENSE.txt，原始声明以下载的 LICENSE 为准。

想运行相同操作向导：

```powershell
python case-studies/go2/guided_run.py --package . --asset-directory data/go2
```

## 官方转换器对照

官方 converter 的独立导出命令如下；使用尚不存在的输出目录：

```powershell
python -c "from robot_physics_fidelity import engine; print(engine.convert('data/go2/scene_mjx.xml','runs/go2-official-export'))"
```

本次该导出成功。但“生成 USD”不等于通过行为对照，也不表示这个 USD 已被 RPF 修复或启用。本工具的受管理接入在预检时已经停止，未继续审计、修复或做源/目标保真验收。

详见 REPORT.md 和 evidence 中的报告。原始网格、依赖和转换产物不打入案例包。
