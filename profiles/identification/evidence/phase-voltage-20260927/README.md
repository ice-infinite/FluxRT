# Phase-voltage software evidence — 2026-09-27

本目录保存相电压实测/Hybrid 支线的 PC 证据，不包含硬件授权。

| 文件 | 内容 | 边界 |
|---|---|---|
| `a24_fault_scenario_matrix.json` | 13个确定性正常/非理想/故障场景及CommandModel、Measured、Hybrid选择结果 | Host-only；阈值未获生产批准；没有串口、烧录或实机 |
| `a24_matlab_fault_scenario_matrix.json` | MATLAB R2025b以独立公式重建同一13场景并逐项核对状态、原因和三种来源选择 | PC-only；13/13一致不代表板级参数或实机正确 |

复现：

```powershell
python simulation\analyze_phase_voltage_fault_scenarios.py `
  --output profiles\identification\evidence\phase-voltage-20260927\a24_fault_scenario_matrix.json
python tests\simulation\test_phase_voltage_fault_scenarios.py
.\simulation\run_matlab_phase_voltage_fault_scenarios.ps1
python tests\simulation\test_phase_voltage_matlab_contract.py
```

SHA-256：

```text
D2EB93E49C9AC8C2689B95400EFCADB6E2F00B35E0129191AAF40D34B8671898  a24_fault_scenario_matrix.json
1AB40B717531B1C2D10919611AD6073E7AD8EF0C07C5FA54875B0BC289ED351B  a24_matlab_fault_scenario_matrix.json
```

MATLAB JSON 已连续生成两次并得到相同 SHA-256；当前结果为
`MATLAB_PHASE_VOLTAGE_PASS scenarios=13 mismatches=0`。
