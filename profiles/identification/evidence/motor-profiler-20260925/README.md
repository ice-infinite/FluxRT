# Motor Profiler 参数识别实测结果（2026-09-25）

本目录保存通过脚本化客户端从设备**直接读回**的识别结果，以及由它生成的 FluxRT
参数候选。这是本工程第一次拿到**实物辨识**的电机参数（此前所有电气参数都抄自
ST Workbench 数据库）。

## 读取方式

用 `tools/motor_profiler_client.py` 的 MCP 读寄存器路径，经 COM6（ST-Link VCP，
1843200 baud）读回 `SC_*` 寄存器。**前置条件是先修掉 Profiler 固件的两处缺陷**，
见 [../../patches/README.md](../../patches/README.md)。

## 实测结果

| 寄存器 | 读出值 | 数据库参考 | 偏差 |
|---|---:|---:|---:|
| `SC_PP`（极对数） | **7** | 7 | — |
| `SC_CURRENT`（识别电流） | **0.800000 A** | 0.8 A | — |
| `SC_RS`（定子电阻） | **4.693368 Ω** | 5.292 Ω | **−11.3%** |
| `SC_LS`（定子电感） | **0.000954 H** | 0.001058 H | **−9.8%** |
| `SC_KE`（反电势常数） | **2.714620** | 4.964 | **−45.3%** |
| `SC_VBUS`（辨识时母线） | **11.833618 V** | — | — |
| `SC_STATE` | **1** | 0 | 识别未完整跑完 |
| `SC_COMPLETED` | **0** | — | 未完成 |
| `SC_MEAS_NOMINALSPEED` | 0.000000 | 1572 | 未测 |
| `SC_J`（转动惯量） | 0.000000 | 2.91e-5 | 未测 |
| `SC_F`（粘滞摩擦） | 0.000000 | 9.37e-6 | 未测 |
| `OVER/UNDERVOLTAGETHRESHOLD` | 14 / 6 | 15 / 7 | 由上位机写入 |
| `BUS_VOLTAGE`（实时） | 12 | — | 约 12.3 V |

`Rs` 与 `Ls` 的偏差都在 ±12% 以内，**量级与 ST 数据库参考值吻合**。

## 状态机推进证据

`SC_STATE` 的取值来自固件枚举 `SCC_State_t`：

```text
0=SCC_IDLE  1=SCC_DUTY_DETECTING_PHASE  2=SCC_ALIGN_PHASE
3=SCC_RS_DETECTING_PHASE_RAMP  4=SCC_RS_DETECTING_PHASE
5=SCC_LS_DETECTING_PHASE  6=SCC_WAIT_RESTART  7=SCC_RESTART_SCC
8=SCC_KE_DETECTING_PHASE  9=SCC_PHASE_STOP  10=SCC_CALIBRATION_END
```

发 `PROFILER_CMD` 后 `SC_STATE` 依次出现 1（占空比检测）→ 2（对齐）→ 4（Rs 检测），
说明**识别确实在推进**。

## 证据边界（必须保留）

- **`SC_COMPLETED = 0`，识别没有完整跑完。** `SC_MEAS_NOMINALSPEED`、`SC_J`、
  `SC_F` 仍是 0，说明 Flux/机械参数辨识阶段未执行。因此 `SC_KE = 2.714620`
  是**中间阶段的值，不是最终辨识结果**。
- **`Ke` 与参考值差 45%**，与工程文档 `EXP-09` §10 记录的既有观察一致
  （"Ls、Ke 随电流变化较大"）。在识别完整跑完并由重复性统计确认之前，
  不能把它当作可信的 `Ke`。
- **`Rs`/`Ls` 也来自未完成的识别**，同样需要重跑确认。
- 实机无编码器/测速仪，转速类量都不是独立真值。

## 由它生成的候选

`../../candidates/rev2-gbm2804h-profiled-20260925.json`，用

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
[../../MotorProfiler串口协议逆向与实测差异.md](../../MotorProfiler串口协议逆向与实测差异.md)。

## 下一步

1. 把识别**完整跑完**（需要电机上电、空载可自由旋转），直到 `SC_COMPLETED = 1`。
2. 按 `EXP-09` 用 0.4 / 0.6 / 0.8 A 各重复三次，做重复性统计。
3. 用一次实物反电势测量判定 `Ke` 的单位口径。
4. 四项都满足后，方可把候选升格并重算 CRC。
