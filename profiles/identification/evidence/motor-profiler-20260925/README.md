# Motor Profiler 参数识别实测结果（2026-09-25）

本目录保存通过脚本化客户端从设备**直接读回**的识别结果，以及由它生成的 FluxRT
参数候选。这是本工程第一次拿到**实物辨识**的电机参数（此前所有电气参数都抄自
ST Workbench 数据库）。

## 读取方式

用 `tools/motor_profiler_client.py` 的 MCP 读寄存器路径，经 COM6（ST-Link VCP，
1843200 baud）读回 `SC_*` 寄存器。**前置条件是先修掉 Profiler 固件的两处缺陷**，
见 [../../../../docs/patches/README.md](../../../../docs/patches/README.md)。

## 实测结果

| 参数 | 运行 1 | 运行 2 | **运行 3** | 数据库参考 | 运行 3 偏差 |
|---|---:|---:|---:|---:|---:|
| `SC_PP`（极对数） | 7 | 7 | **7** | 7 | — |
| `SC_CURRENT`（识别电流） | 0.800 A | 0.800 A | **0.640 A** | 0.8 A | profiler 自行扫描 |
| `SC_RS`（定子电阻） | 4.693 Ω | 4.715 Ω | **5.1569 Ω** | 5.292 Ω | **−2.5%** |
| `SC_LS`（定子电感） | 0.954 mH | 0.935 mH | **0.8595 mH** | 1.058 mH | −18.8% |
| `SC_KE`（反电势常数） | 2.7146 | 3.9825 | **4.3372** | 4.964 | **−12.6%** |
| `SC_VBUS`（辨识时母线） | 11.8336 V | 11.8558 V | 11.9138 V | — | — |
| `SC_STATE` | 1 | 4 | **1** | 0 | 见下 |
| `SC_COMPLETED` | 0 | 0 | **0** | — | 未完成 |
| `SC_J` / `SC_F` | 0 | 0 | **0** | — | 未测 |

**关键趋势：`SC_RS` 与 `SC_KE` 单调收敛。**

```text
SC_RS:  4.693 -> 4.715 -> 5.157 ohm     (数据库 5.292, 最终偏差 -2.5%)
SC_KE:  2.715 -> 3.982 -> 4.337         (数据库 4.964, 最终偏差 -12.6%)
```

运行 3 中 `SC_CURRENT` 由 0.8 A 自行变为 **0.64 A**，说明 **Profiler 正在做电流扫描**，
这是它在主动推进识别、而不是停住。

## 状态机推进证据

`SC_STATE` 的取值来自固件枚举 `SCC_State_t`：

```text
0=SCC_IDLE  1=SCC_DUTY_DETECTING_PHASE  2=SCC_ALIGN_PHASE
3=SCC_RS_DETECTING_PHASE_RAMP  4=SCC_RS_DETECTING_PHASE
5=SCC_LS_DETECTING_PHASE  6=SCC_WAIT_RESTART  7=SCC_RESTART_SCC
8=SCC_KE_DETECTING_PHASE  9=SCC_PHASE_STOP  10=SCC_CALIBRATION_END
```

**运行 3 完整推进链**：发 `STOP(cmd=0)` → `START(cmd=1)` 后，`SC_STATE` 依次为
**1 → 3（Rs 斜坡）→ 4（Rs 检测）→ 1（回到占空比检测，开始新一轮）**。

## 证据边界（必须保留）

- **运行 1/2 曾停在 `SC_STATE=4` 达 38 秒不动**，那是**旧状态残留**；本轮在目标自由运行、
  状态干净的前提下，识别自主循环推进，未再卡死。
- **`SC_COMPLETED` 仍为 0**：识别尚未走到 `SCC_CALIBRATION_END`，`SC_J`/`SC_F`/`SC_MEAS_NOMINALSPEED` 仍为 0。
- 三次运行都是**进行中的快照**，不是最终辨识结果。`Rs` 已收敛到 −2.5%，但 `Ls` 偏差 −18.8%、`Ke` 偏差 −12.6%，且三次之间仍在变化。
- **未做 0.4 / 0.6 / 0.8 A 多电流档重复性统计**（目标要求）。三次运行的识别电流是
  profiler 自行选定的，不是我设定的档位，**不构成** EXP-09 要求的重复性门。
- 实机无编码器/测速仪，转速类量都不是独立真值。

## 由它生成的候选

`../../../../candidates/rev2-gbm2804h-profiled-20260925.json`，用

```powershell
python tools/motor_profiler_convert.py convert `
    profiles/identification/evidence/motor-profiler-20260925/motor_pilot_export.json `
    --baseline profiles/candidates/rev1-gbm2804h-unapproved.json `
    --flux-convention ke-phph-rms --revision 2 `
    --output profiles/candidates/rev2-gbm2804h-profiled-20260925.json
```

生成。候选保持了 `approvals = false`，CRC 已清空待重算。

**磁链换算仍带警告**：按官方单位（线电压 RMS）`Ke = 2.714620` 应得
`0.021165805 Wb`，而工程基线是 `0.005529026 Wb`，相差 **3.83 倍**。`Ke` 的单位口径
（线电压还是相电压）仍需一次**实物反电势测量**判定，见
[../../../../docs/MotorProfiler串口协议逆向与实测差异.md](../../../../docs/MotorProfiler串口协议逆向与实测差异.md)。

## 下一步

1. 把识别**完整跑完**（需要电机上电、空载可自由旋转），直到 `SC_COMPLETED = 1`。
2. 按 `EXP-09` 用 0.4 / 0.6 / 0.8 A 各重复三次，做重复性统计。
3. 用一次实物反电势测量判定 `Ke` 的单位口径。
4. 四项都满足后，方可把候选升格并重算 CRC。
