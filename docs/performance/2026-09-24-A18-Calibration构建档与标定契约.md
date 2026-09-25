# A18：Calibration 构建档与相电压标定契约

## 1. 结论

A18 已完成：FluxRT 现在有互相隔离的 `Diagnostic / Calibration / Production` 三档。
Calibration 只保留 A17 的 256 拍相电压原始码固定窗，裁掉在线调参、实时 trace 和数学
诊断；`foc_start` 在应用层明确拒绝，`foc_platform_control_start()` 在平台层再次硬拒绝。

本阶段只完成 Host/工具测试和 STM32G431 交叉构建，没有烧录、没有打开串口、没有使能
功率级、没有运行电机，也没有生成任何虚构标定值。板上仍是 A17 Diagnostic 安全停机
固件。

## 2. 构建隔离

`build.ps1` 和 `custom.cmake` 为三档分别使用：

| 档位 | CMake 目录 | Rust target 目录 | 主宏 |
|---|---|---|---|
| Diagnostic | `cmake-build` | `build/rust-target-diagnostic-s` | `FLUXRT_DIAGNOSTIC_BUILD` |
| Calibration | `cmake-build-calibration` | `build/rust-target-calibration-s` | `FLUXRT_CALIBRATION_BUILD` |
| Production | `cmake-build-production` | `build/rust-target-production-s` | `FLUXRT_PRODUCTION_BUILD` |

`foc_build_profile.h` 检查主宏只能三选一，再派生运行期调参、trace、相电压固定窗和数学
诊断能力。裸 SCons/Host 未传档位时默认 Diagnostic，以保持原开发流程；CMake 总是显式
传入档位。档位不写入全局 `.config`，避免三套构建共享一份 Kconfig 时互相污染；HAL、
硬件和算法候选仍由 Env/SCons/Kconfig 生成。

三份 Rust 归档当前 SHA-256 都是
`8098806709CB242DB38F504F115CA9D83F6AA544B73BFAC465648AC0F91704C9`，原因是本次三档
没有改变 Rust feature；独立目录仍然防止以后 Diagnostic feature 或优化等级变化时串库。

## 3. 能力矩阵与安全门

| 能力 | Diagnostic | Calibration | Production |
|---|---:|---:|---:|
| `foc_cfg` | 是 | 否 | 否 |
| `foc_trace` | 是 | 否 | 否 |
| `foc_phase_capture` / 4 KiB 固定窗 | 是 | 是 | 否 |
| 停机数学诊断 | 按 Kconfig | 否 | 否 |
| `foc_start` | 正常安全门 | 始终拒绝 | 正常安全门 |
| 硬关断、Break、过流、deadline、输出复核 | 保留 | 保留 | 保留 |

Calibration 保留一个 `foc_start` Shell 拒绝桩，目的是让误操作得到明确原因；它不会解析
目标转速或进入控制启动。即使未来出现绕过 Shell 的调用，平台层也先
`foc_platform_emergency_stop()` 再返回 `FOC_STATUS_DISABLED`。

## 4. 相电压标定记录 v1

新增文件：

- `profiles/schema/fluxrt-phase-voltage-calibration-v1.schema.json`：机器可读格式；
- `tools/foc_calibration_tool.py`：标准库离线拒绝门；
- `profiles/calibration/README.md`：真实记录落盘规则；
- `tests/profile/test_foc_calibration_tool.py`：草稿、映射、审批、哈希和篡改回归。

契约固定 U/V/W 为 `PC0/ADC12_IN6`、`PC3/ADC12_IN9`、`PC1/ADC12_IN7`，并固定
`PC9` 为低有效分压使能。每条记录绑定板卡身份、Calibration BIN SHA-256、ADC Vref/位数、
每点样本数、分压阻值、验收阈值、各相测量点/拟合、证据哈希和审批人。

`draft` 可以没有测量点，但不能携带审批；`approved` 每相至少需要三个不同已知电压点、
正斜率拟合、残差/R²达标、`raw-measurements / meter-reference / review` 三类项目内文件和
匹配的 SHA-256，以及明确 UTC 审批时间。工具只校验，不拟造数据、不生成固件、不设置
审批位。

## 5. S1 测试结果

执行：

```powershell
.\test.ps1
```

结果：

- Python 配置/证据测试 19/19 PASS，其中标定工具 5/5；
- Rust 单元测试 69 + 16 + 11 + 15 全部 PASS；
- PC 闭环仿真 PASS，`final_rpm=534.52`、目标 524 rpm；
- Rust fmt、Clippy、CPU/CORDIC Cortex-M4F 静态库 PASS；
- C Host 4/4 PASS，包含三档能力编译契约。

这些结果证明数据契约和编译能力边界，没有证明目标板行为或测量精度。

## 6. S2 三档目标构建

执行三次 `build.ps1 -Regenerate/-BuildOnly -Profile <档位> -RustOptLevel s`，最终产物：

| 档位 | text | data | bss | ROM | RAM | Flash 余量 | BIN SHA-256 |
|---|---:|---:|---:|---:|---:|---:|---|
| Diagnostic | 126,836 | 1,764 | 16,508 | 128,600 B | 18,272 B | 2,472 B | `B7BF3C8B843F9EBD7ADF23C9BC4EB46DC7C597C057CA0A09E2B1DAF38988E193` |
| Calibration | 95,972 | 1,156 | 11,852 | 97,128 B | 13,008 B | 33,944 B | `4E36F55F03BB5F14C9E4161E4A46DDA0A141C4726369ECE74F0FD20D5B6894D7` |
| Production | 94,552 | 1,156 | 7,740 | 95,708 B | 8,896 B | 35,364 B | `ABC1930D1A4D83F470007C6C682E24643C063E6985AA701A310FD0CC7D0B03ED` |

`arm-none-eabi-nm` 还确认：

- Diagnostic 有 `__fsym_foc_cfg`、`__fsym_foc_trace`、`__fsym_foc_phase_capture`；
- Calibration 无 `foc_cfg/foc_trace` Shell 符号，有 `foc_phase_capture` 和 start 拒绝提示；
- Production 无 `foc_cfg/foc_trace/foc_phase_capture` Shell 符号。

链接器仍报告工具链 `crtn.o` 缺 `.note.GNU-stack` 的既有警告；三档均链接成功，没有新增
C 编译警告。

## 7. 未完成与下一门

- A18 没有进行 S4/S5，不能声称 Calibration 已在目标板验证；
- 标定目录没有真实 JSON 记录，因为尚未记录板卡序列号、稳定直流源和万用表数据；
- ADC Vref、实际 10 kΩ/2.2 kΩ、电路偏置、二极管/源阻抗影响仍未知；
- A17 增加三次 ADC 转换后的运行态 WCET 仍必须在 A21 重新实测；
- 下一阶段 A19 只做停机静态多点标定。必须记录仪表、温度、固件哈希和原始证据；没有
  仪器时只能准备采集流程，不能把 A19 标成完成。
