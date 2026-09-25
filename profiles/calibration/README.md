# Calibration 标定记录目录

本目录只保存经过 `fluxrt-phase-voltage-calibration-v1` 契约校验的板卡标定记录。A18
只建立格式和拒绝门，没有伪造任何测量值；第一份真实记录要等 A19 使用稳定直流源和可信
万用表后再创建。

A19 首次 Calibration 板端 S4 安全门和停机原始码位于
`evidence/a19-s4-20260924/`。该目录不是校准记录：它没有万用表参考值、拟合或审批，
不得用来生成物理量参数。详细边界见
[`docs/performance/2026-09-24-A19-Calibration板端安全门与静态基线.md`](../../docs/performance/2026-09-24-A19-Calibration板端安全门与静态基线.md)。

显式 divider 模式的新固件板端证据位于 `evidence/a19-divider-baseline-20260924/`：off/on
各三窗，带 CSV/日志哈希和 `baseline_comparison.json`。它验证了模式切换与重复性，但没有
受控电压真值，因此同样不能作为增益拟合点或 approved 标定记录。

## 采集模式

新 Calibration 固件把 PC9 分压网络分成两个显式模式，且两种模式都只能在电机未 arm、
功率输出关闭时采集：

- `--divider off`：PC9 全窗保持高电平，只记录分压网络断开基线；
- `--divider on`：PC9 在 256 拍窗口内拉低，窗口完成、停止或异常路径立即恢复高电平；
- Shell 旧命令 `foc_phase_capture start` 为兼容仍等价于 `start on`，新证据必须显式写
  `on` 或 `off`。

```powershell
python simulation/capture_phase_voltage_window.py --port COM6 --divider off `
  --output profiles/calibration/evidence/<session>/u-zero-off.csv
python simulation/capture_phase_voltage_window.py --port COM6 --divider on `
  --output profiles/calibration/evidence/<session>/u-zero-on.csv
```

CSV 每行含 `divider_mode`；同名 JSON 保存模式、样本率、序号范围、三相统计和 CSV/日志
SHA-256。A19.1 起 JSON 还保存固件输出的 `phase_voltage_model`：官方名义模型必须显示
`quality=nominal-not-calibrated`、`observer=disabled`。旧 A19 CSV 没有模式字段，只保留为
历史 S4 证据，不能自动导入标定记录。

`3.3 V / 12 bit / 10 kΩ / 2.2 kΩ` 对应理论满量程 18.3 V、4469 uV/count，只允许
诊断显示和初步对拍。它不是已知电压点、不能填入 `fit`、不能设置 `approved`，也不能让
相电压进入观察器。板端 S4 证据见 `evidence/a19-official-nominal-20260924/`。

## 标定记录工作流

先用实际板卡身份和**本次将烧录的** Calibration BIN 建 draft：

```powershell
python tools/foc_calibration_tool.py init-draft `
  profiles/calibration/<record>.json `
  --record-id <record-id> --serial <board-serial> `
  --hardware-revision <ihm16m1-revision> `
  --firmware-bin cmake-build-calibration/fluxrt.bin `
  --reference-voltage-v 3.33
```

每获得一个万用表确认的电压点，就从 CSV 自动计算该相的均值、总体标准差、样本数、模式
和证据哈希，不手抄 ADC 码：

```powershell
python tools/foc_calibration_tool.py add-point profiles/calibration/<record>.json `
  --channel u --applied-voltage-v 6.000 --temperature-c 25.0 `
  --capture-csv profiles/calibration/evidence/<session>/u-6v-on.csv
```

三相各有至少三个不同的 `divider_mode=on` 电压点后拟合并复核：

```powershell
python tools/foc_calibration_tool.py fit profiles/calibration/<record>.json
python tools/foc_calibration_tool.py validate profiles/calibration/<record>.json
```

`off` 点只用于开关/零点诊断，不进入增益拟合，也不能凑足审批所需的三个电压点。工具会
计算线性拟合和验收指标，但不会把记录改成 `approved`；仪表证据、独立 review 和审批仍需
人工确认。

兼容的简写校验命令仍可使用：

```powershell
python tools/foc_calibration_tool.py profiles/calibration/<record>.json
```

规则：

- 固件必须是独立 `Calibration` 构建档，并记录实际 `fluxrt.bin` SHA-256；
- U/V/W 映射固定为 `PC0/ADC12_IN6`、`PC3/ADC12_IN9`、`PC1/ADC12_IN7`；
- `PC9` 是低有效分压使能；
- `draft` 可以没有测量点，但不能带审批；
- `approved` 每相至少三个不同已知电压点、合格拟合、三类本地哈希证据和独立审批；
- 工具可以创建 draft、从模式化 CSV 追加点和计算拟合，但不修改固件、不会自动批准。

Schema 位于 `profiles/schema/fluxrt-phase-voltage-calibration-v1.schema.json`。
