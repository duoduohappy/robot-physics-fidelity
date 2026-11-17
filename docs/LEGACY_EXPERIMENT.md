# Robot Physics Fidelity

机器人仿真资产物理语义保真验证与修复工程实验。

**公开演示：[Physics Fidelity Lab](https://robot-physics-fidelity-demo.savory-fox-4497.chatgpt.site)**

沿 MuJoCo → USD → Newton → MuJoCo CPU 链路，保留显式接触对并诊断关节限位参数的消费语义变化。提供真实轨迹演示、原始证据和固定版本复现入口。

## 验证结果

三个固定初态均测试 0.5 秒，位置门槛 1 mm，qvel 分量门槛 0.01（各分量原生单位）。

| 资产 / 初态 | 接触对保留 | 身体位置最大差 | 结果 |
|---|---:|---:|---|
| Apollo / stand | 6/6 | 0.000287 mm | 通过 |
| Unitree G1 / home | 49/49 | 0.000621 mm | 通过 |
| Unitree G1 / knees_bent | 49/49 | 0.195096 mm | 速度超限，整体不通过 |

配置配对保留不代表全部配对均触发过接触；Apollo 本次触发 3 条，G1 本次触发 6 条。保留失败案例，不将单场景通过推广为通用保真认证。

## 解决的问题

- 自动碰撞掩码为零的几何仍可能由显式 pair 选中；转换时应保留必要端点及其关系。
- schema 默认值省略与消费者依据 authored opinion 选择 RAW 模式的逻辑叠加，会改变限位响应。使用单字段对照定位问题，将源有效值明确写入实际 USD 后重新验证。

## 复现

Windows / Python 3.11 为已执行环境。解压或克隆时保留 `outputs/` 布局，并在根目录创建 `work/`。

```powershell
python -m pip install --target work/usd-pilot-deps -r outputs/cross-asset-validation/requirements-lock.txt
python outputs/cross-asset-validation/fetch_apollo.py
python outputs/cross-asset-validation/fetch_g1.py
./outputs/cross-asset-validation/run.ps1 -Asset apollo -Keyframe stand
./outputs/cross-asset-validation/run.ps1 -Asset g1 -Keyframe home
./outputs/cross-asset-validation/run.ps1 -Asset g1 -Keyframe knees_bent
```

前两项预期退出 0，第三项预期退出 2。不得将非零结果改成通过。二进制依赖和原网格不纳入仓库，固定输入按清单下载并校验哈希。不要另行混入独立 `usd-core`；`usd-exchange` 提供匹配的 OpenUSD。

## 范围

这是两个固定公开模型的短时 CPU 工程实证，不是新物理算法、真实机器人验证、长期站立或任意资产转换保证。物理测试执行 Newton 重建 mjModel 的原生 MuJoCo CPU 步进，未验收 Newton 状态同步接口。传感器、相机、灯光和 keyframe 的完整往返不在验证范围内。

## 来源与许可

源资产来自 [MuJoCo Menagerie 固定提交](https://github.com/google-deepmind/mujoco_menagerie/tree/71f066ad0be9cd271f7ed58c030243ef157af9f4)。资产原许可保留于复现目录。接触模型及 Newton API 是上游已有能力；本项目不宣称首次提出相关物理算法。
