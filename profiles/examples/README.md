# profiles/examples — 参数转换样例

本目录存放 **Motor Pilot 格式的样例导出**，用途是让参数转换流水线可以在没有硬件、
没有实物辨识结果的情况下被端到端验证。

## 为什么需要样例

`tools/motor_profiler_convert.py` 把 ST Motor Pilot 的电机参数导出转成 FluxRT 的
参数候选。真实导出来自实物辨识，而实物辨识需要能连上 Profiler 固件——当前那条路
是断的（见 `docs/MotorProfiler串口协议逆向与实测差异.md`）。

样例让工具链**不必等待硬件**就能被验证与回归。

## 文件

| 文件 | 说明 |
|---|---|
| `motor_pilot_export_reference.json` | 字段名与官方 `GUI/profiler.qml` 的 `jsonExportData` 一致；数值取 **ST Workbench 数据库参考值** |

> **重要**：样例里的数值是**数据库参考值，不是实物辨识结果**，因此**不能**当作
> 可用参数。它的唯一作用是验证工具链与复现量纲口径。

## 用法

```powershell
python tools/motor_profiler_convert.py convert `
    profiles/examples/motor_pilot_export_reference.json `
    --baseline profiles/candidates/rev1-gbm2804h-unapproved.json `
    --flux-convention ke-phph-rms `
    --revision 2 `
    --name "示例候选" `
    --output profiles/candidates/rev2-example.json
```

`--flux-convention` 是**必填**：`Ke` 的单位口径存在 √3 倍歧义，工具不会替你选。
详见 [参数候选与审批流程](../../docs/parameters/参数候选与审批流程.md)。

## 样例会复现什么

跑上面这条命令会复现本次调查的核心量纲结论：

```text
磁链        基线 0.005529026 Wb
            本次 0.038704149 Wb   (Ke = 4.964 Vrms/kRPM, 按官方单位换算)
            偏差 7.0002 倍
```

即**工程现存磁链比 `Ke` 的物理含义小约 7 倍**。观测器的反电势幅值与滑模增益都按
磁链标度，因此这个偏差会让无感观测器整体失准——**在用它整定之前，必须先用一次
实物反电势测量判定口径**。

这条结论由 `tests/profile/test_motor_profiler_convert.py` 的 `WorkedExampleTests`
钉住，样例数值若有改动测试会失败。
