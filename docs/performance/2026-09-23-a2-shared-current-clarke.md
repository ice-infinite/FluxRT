# 阶段 1.2 共享电流 Clarke（2026-09-23）

## 1. 结论

实时桥接路径现在对每个 ADC 电流快照只执行一次电流 Clarke，结果同时供观察器、无感
接管 `Iq` 初始化和电流环使用。普通控制器/观察器 API 仍保留自行 Clarke 的兼容入口，
因此 PC 测试和独立调用者不需要预先计算中间量。

这次改动完成了数据流去重和后续死区模型的 αβ 复用入口，但**没有测得 WCET 改善**：
旧基线为 8,257 cycles，新固件连续三次 5 秒开环运行均为 8,267 cycles。固定的 10 cycles
差异只占 12 kHz 物理周期约 0.07%，没有 deadline miss 或控制故障，但也不能宣称性能
提升。因此路线阶段 1.2 的“WCET 降低”验收项仍未完成，下一步必须从 CORDIC 事务和
换算开销取得可测收益。

## 2. 数据流变化

修改前：

```text
ADC abc ─┬─ observer 内 Clarke
         ├─ handoff 时再次 Clarke
         └─ current loop 内再次 Clarke
```

修改后：

```text
ADC abc ─→ realtime bridge Clarke 一次 ─┬─ observer
                                         ├─ handoff Iq
                                         └─ current loop
```

观察器由 PWM 重构电压时使用的是另一组电压 Clarke，不属于同一个中间量，未在本批强行
合并。电流环内部原有的一组 sin/cos 继续同时服务 Park 和逆 Park，本批没有修改。

## 3. 代码边界

- `foc-control::CurrentLoop::update_from_alpha_beta_with_math` 接收预计算 αβ；
- `RotorEstimator::update_from_alpha_beta_with_math` 让具体观察器复用同一电流向量；
- `foc-rt-bridge::foc_rust_realtime_step` 是唯一实时 Clarke 的组合位置；
- 原 `update` / `update_with_math` 入口仍可独立使用，并在内部计算 Clarke；
- C 平台 ADC、PWM、立即关断和输出复核没有改变。

## 4. 验证结果

共同实机条件：Diagnostic + Rust `s`、12 kHz PWM/控制环、默认开环、582 rpm、板端
约 12.24～12.28 V、空载、trace 关闭、每次 5 秒。

| 项目 | 旧基线 | 新 run 1 | 新 run 2 | 新 run 3 |
|---|---:|---:|---:|---:|
| 完整 ISR WCET | 8,257 | 8,267 | 8,267 | 8,267 |
| control 段 | 未单列用于本对比 | 7,409 | 7,409 | 7,409 |
| invalid / miss / error / fault | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 |
| 结果 | PASS | PASS | PASS | PASS |

本机 ignored 原始记录：

- `simulation/results/timing_diagnostic_s_shared_clarke_openloop_582rpm_12v_20260923.json/.log`；
- `simulation/results/timing_diagnostic_s_shared_clarke_run2_openloop_582rpm_12v_20260923.json/.log`；
- `simulation/results/timing_diagnostic_s_shared_clarke_run3_openloop_582rpm_12v_20260923.json/.log`。

软件和构建证据：

- S1：`test.ps1` PASS；Rust 95 项（新增 1 项等价测试）、格式、Clippy、CPU/CORDIC
  交叉库、C Host 1/1 和 PMSM 仿真通过；
- S2：Diagnostic + Rust `s` 构建通过，BIN 122,788 B，比上一版小 24 B，RAM 不变；
- S4：STM32CubeProgrammer 下载、verify 和 reset 通过；
- S5：三组低压限流空载短测通过；
- S6：示波器、编码器真值、故障注入和长测未执行。

## 5. 最终固件

| 文件 | 大小 | SHA-256 |
|---|---:|---|
| `cmake-build/fluxrt.bin` | 122,788 B | `DD91427930F050A584F85E2BD75DFE960B9F12D3E2436CFBC27189CFBDFE6B6B` |
| `cmake-build/fluxrt.hex` | 345,445 B | `31D8F79544E28E8CC3CF9AE9AC2A02578263C2EB5648415B54E64C7FBC1E30C6` |
| `cmake-build/rtthread.elf` | 2,965,760 B | `CC5D358D699EC1576218D7C78247893AE3CEB3DFE4962D7B39982EE20AA6FF37` |

结束时已执行 `foc_stop`，回读 `errors=0`、`misses=0`、`fault=0`、duty
`500/500/500` 为停机后的最后算法快照；TIM1 输出和门极使能由 C 平台明确关闭。

## 6. 下一步

在不删除 CORDIC 超时保护的前提下，单独建立数学后端基准并优化事务：

1. 量化 sin/cos、magnitude、atan2 各自平均和最坏 cycles；
2. 为未饱和电压向量增加可证明等价的快速判定，避免不必要的 magnitude 事务；
3. 对比当前 Q31/Q15 换算与打包路径的误差、Flash 和 WCET；
4. 任何候选实现都重复相同 Host、目标构建和三组实机短测。
