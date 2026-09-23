# FluxRT 构建档与优化等级

## 1. 当前选择

日常开发默认使用：

```text
Profile = Diagnostic
RustOptLevel = s
```

原因不是单看体积：STM32G431 实机 5 秒开环对比中，Rust `s` 相对 `3` 回收约 5 KiB
Flash，完整 ISR WCET 只增加约 2%；`z` 只比 `s` 再节省 800 B，却使 WCET 增加约 7%。

`Production` 是可继续验收的生产候选构建档，不代表已经量产通过。它仍保持
`closed_loop_enable=0`，并保留立即关断、输出复核、过流、母线、驱动故障和 deadline
保护。

## 2. 两种构建档

| 能力 | Diagnostic | Production |
|---|---|---|
| `foc_start` / `foc_stop` / `foc_status` | 保留 | 保留 |
| 在线 `foc_cfg` 浮点调参 | 保留 | 裁剪 |
| `foc_trace` 与 64 项环形缓冲 | 保留 | 裁剪，API 返回未配置 |
| 同拍 WCET 与 deadline 保护 | 保留 | 保留 |
| 默认闭环权限 | 关闭 | 关闭 |
| 默认 Rust 优化 | `s` | `s` |
| 输出目录 | `cmake-build` | `cmake-build-production` |

Production 裁剪在线调参的原因是固定参数候选不应在运行现场被 Shell 任意修改；同时
`strtof` 会把 newlib 的 `strtod`、断言和格式化依赖链拉入镜像。裁剪 trace 则回收
3,072 B 环形缓冲及其 ISR/输出代码。

## 3. 常用命令

日常诊断构建：

```powershell
.\build.ps1 -Regenerate
.\build.ps1 -BuildOnly
```

Production 候选构建：

```powershell
.\build.ps1 -Regenerate -Profile Production -RustOptLevel s
.\build.ps1 -BuildOnly -Profile Production -RustOptLevel s
```

重新比较 Rust 优化等级：

```powershell
.\build.ps1 -Regenerate -Profile Diagnostic -RustOptLevel 3
.\build.ps1 -Regenerate -Profile Diagnostic -RustOptLevel s
.\build.ps1 -Regenerate -Profile Diagnostic -RustOptLevel z
```

`-BuildOnly` 会检查所请求的 Profile/优化等级是否与 CMake cache 一致，不一致时拒绝
构建，避免把一个配置的固件误标成另一个配置。各优化等级的 Rust 产物放在独立的
`build/rust-target-3|s|z`，CMake 会显式链接对应目录，不能复用旧档位的静态库。

直接使用 SCons 时仍是 Diagnostic 语义，并使用 `rust/Cargo.toml` 的默认 `s`；日常 VS
Code 和 Production 构建统一走 `build.ps1`。

## 4. 实机采集

Diagnostic 支持 trace 关闭、10 Hz、50 Hz：

```powershell
python .\simulation\capture_timing_baseline.py `
  --profile diagnostic --port COM6 --rates 0,10,50 `
  --duration 5 --target-rpm 582 --allow-motor-run `
  --output .\simulation\results\timing-diagnostic.json
```

Production 已裁掉 trace，只能采集关闭状态：

```powershell
python .\simulation\capture_timing_baseline.py `
  --profile production --port COM6 --rates 0 `
  --duration 5 --target-rpm 582 --allow-motor-run `
  --output .\simulation\results\timing-production.json
```

脚本没有 `--allow-motor-run` 时拒绝启动；结束路径始终发送 `foc_stop`。闭环仍需另外
显式添加 `--closed-loop`，本构建档对比没有启用闭环。

## 5. 选择与发布规则

- 日常调参与波形采集使用 Diagnostic + `s`；
- `3` 只用于性能回归基准，不再作为默认，因为 Flash 余量不足；
- `z` 只作为尺寸下界，除非后续 Flash 压力超过 `s` 且重新通过 WCET；
- 发布候选使用 Production + `s`，但还必须通过独立速度/角度真值、故障注入、长测和
  多板多电机一致性；
- 切换 Profile 或优化等级后必须重新记录 BIN SHA-256、`text/data/bss` 和板端 WCET；
- 不得仅凭“构建成功”或“电机能转”标记为生产可用。

本轮具体数据见
[`performance/2026-09-23-a1-build-profile-matrix.md`](performance/2026-09-23-a1-build-profile-matrix.md)。
