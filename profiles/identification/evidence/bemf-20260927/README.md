# BEMF 实测证据（2026-09-27）

## 结论

- CM4.4 Diagnostic 已完成三次有效的 582 rpm 停机 BEMF 窗；三次
  `measurement_validity` 均为 `ok`，`ψ_SI` 为 5.418/5.447/5.263 mWb。
- 三次均值 **`ψ_SI=5.376 mWb`**，极差/均值 **3.41%**、样本变异系数 1.84%，
  通过 `<10%` 重复性门；比工程基线 `5.529 mWb` 低 2.77%，在名义分压精度内一致。
- `Ke_line` 三次均值 4.826 Vll_rms/kRPM；但相对未完成且离散达 43.2% 的
  Profiler `SC_KE=3.75248`，三窗仍均为 `sc_ke_matches=ambiguous`。因此不得用
  `SC_KE` 执行 convert；rev7 直接采用实测 SI 磁链，且保持未审批、未集成。
- 两个 1000 rpm 窗都因实际转速严重偏低或波形不平衡被拒绝，只保留为失败证据。

## 条件与固件

- NUCLEO-G431RB + X-NUCLEO-IHM16M1 + GBM2804H-100T，空载；
- 母线约 12.30 V，外部电源限流 2 A；
- `cmake-build/fluxrt.hex` SHA-256：
  `9292869F76861B18B93F5ACC1FABE435EF5213A60DCBDC2BDBA1016719104AD9`；
- profile rev6，config CRC `EDF4F6CA`，闭环关闭；
- 名义相电压比例 4469 uV/count，尚未逐板标定。

## 结果索引

| 前缀 | 请求/实际 rpm | ψ_SI | 判定 |
|---|---:|---:|---|
| `bemf_1000rpm_run1` | 1000 / 249.77 | 7.452 mWb | tracking anomaly，拒绝 |
| `bemf_1000rpm_run2` | 1000 / 149.83 | 4.940 mWb | frequency/amplitude imbalance，拒绝 |
| `bemf_0582rpm_run3` | 582 / 537.26 | 5.418 mWb | 测量门通过，SC_KE 口径 ambiguous |
| `bemf_0582rpm_run4` | 582 / 564.17 | 5.447 mWb | 测量门通过，SC_KE 口径 ambiguous |
| `bemf_0582rpm_run5` | 582 / 547.71 | 5.263 mWb | 测量门通过，SC_KE 口径 ambiguous |
| **三次汇总** | 均值 549.71 | **5.376 mWb** | **极差/均值 3.41%，重复性 PASS** |

每个前缀包含：

- `.csv`：256 点原始固定窗；
- `.log`：完整串口会话；
- `.json`：采集时序、固件运行信息和 CSV/log 哈希；
- `.analysis.json`：v5 平衡三线联合拟合结果。
- `repeatability_summary.json`：三次有效窗的哈希、聚合统计、通过门和限制条件。
- `rev6_model_on_measured_plant.csv` / `rev7_model_on_measured_plant.csv`：固定实测 plant
  后的 15 s 正转闭环 A/B trace。
- `rev7_simulation_ab_summary.json`：正/反转、轻载、补偿和拆分试验的汇总判定。

原始 CSV 与 log 的 SHA-256 已按同名 metadata 复核；run3/run4/run5 的 analysis
SHA-256 固化在 `repeatability_summary.json`。

## 后续门

1. rev7 PC 功能门通过，但性能晋级门为 `inconclusive`；不得直接修改或烧录固件；
2. A19 逐板相电压增益标定后，应按新比例重算磁链绝对值；
3. 记录绕组测量温度，决定 DC `Rs=4.9667 Ω` 是否归一化到 20/25℃；
4. `SC_KE` 口径仍需独立解释，但不再阻塞直接测得的 SI 磁链进入未审批候选；
5. 优先补 `Ls(I)`：当前 1.114 mH 是小信号值，不能据仿真直接替代 0.8 A 工作点值。
