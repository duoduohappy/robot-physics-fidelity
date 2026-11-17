# 自动碰撞接入评估（不是生产支持）

先读 DEVELOPMENT_BOUNDARY.md。stock-import-measured.json 是修正有效尺寸比对后的标准导入探测；stock-import.json 是首次未建立身份映射时的停止记录。

本次源码和原始模型未改写，不启用任何生产候选。固定 Go2 home 的数值符合现有运动门槛，但尚未证明所有有效碰撞规则、多初态或其他掩码组合等价。

在安装了 RPF 2.0.0 的环境，准备固定原始 Go2 和官方导出目录后，可用以下形式复现（输出目录必须不存在）：

```powershell
python stock_import_probe.py --source <原始Go2目录>/scene_mjx.xml --usd "<官方导出目录>/go2 scene.usda" --output runs/go2-stock-probe
```

该脚本临时将源初态、控制和指定求解配置映射到重建的原生 CPU 模型。不是 Newton 外层 state/control 同步的整机验证，也不是任意 USD 的正式验收器。输出 status 保留 MEASURED_NOT_QUALIFIED，不能把它换写成正式 PASS。
