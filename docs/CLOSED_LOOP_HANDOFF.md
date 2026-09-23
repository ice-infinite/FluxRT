# 开环到无感闭环接管

## 当前实现

控制状态按下面顺序运行：

```text
alignment → open-loop-ramp → observer-transition → closed-loop
                         └→ open-loop-hold → 超时故障
closed-loop + 连续失锁 → 故障停机
```

- 观察器达到速度、反电势、方差和连续样本门后，才开始接管；不会因为固定计时结束而跳角。
- 接管可在开环加速阶段提前开始，行为与 ST 参考工程从 Phase 2 检查观察器一致。
- 25 ms 内沿最短角差融合强制角和观察角，同时把 Iq 从启动值渐变到观察坐标系实测值。
- 闭环入口先把速度参考同步到当前观察速度，再按配置斜率走向用户目标。
- 当前参考工程定义 `PID_SPEED_INTEGRAL_INIT_DIV=0`，因此速度 PI 默认零预装；应用 Iq 以限速器平滑跟随 PI 输出。
- 等待观察器超时使用 `FOC_RUST_FAULT_OBSERVER_STARTUP`；闭环连续失锁使用 `FOC_RUST_FAULT_OBSERVER_LOST`。两者都返回硬件故障，由 C 平台关断 PWM。

## 可调参数

所有参数只能在 `foc_stop` 后修改：

| Shell 命令 | 含义 | 默认值 |
|---|---|---:|
| `foc_cfg minspeed 524` | 观察器最低可靠机械转速 | 524 rpm |
| `foc_cfg minbemf 250` | 最低反电势幅值 | 250 mV |
| `foc_cfg variance 10` | 速度归一化方差上限 | 10/1000 |
| `foc_cfg confirm 2` | 连续可靠窗口数，窗口频率 1 kHz | 2 |
| `foc_cfg acquire 500` | 开环 ramp 结束后的收敛等待上限 | 500 ms |
| `foc_cfg loss 50` | transition/closed-loop 连续失锁上限 | 50 ms |
| `foc_cfg accel 500` | 闭环速度给定斜率 | 500 rpm/s |
| `foc_cfg preload 0` | 速度 PI 对切换 Iq 的预装比例 | 0/1000 |
| `foc_cfg islew 32000` | 闭环应用 Iq 的最大变化率 | 32000 mA/s |

`preload=0` 对应本次 ST 参考工程；如果以后更换参考工程且其速度 PI 使用积分预装，才应
在仿真和限流实机验证后提高此值。

## PC 同构验证

```powershell
cd E:\File\RT-Thread\projects\FluxRT
& "$env:USERPROFILE\.cargo\bin\cargo.exe" run `
  --manifest-path .\rust\Cargo.toml -p foc-sim `
  --bin foc-bringup-sim --release -- `
  --closed-loop --duration 10 --target-rpm 582 `
  --bus-voltage 12.3 --sample-every 12 `
  --csv .\simulation\results\rust_closedloop_ideal_12khz_10s.csv
```

当前 12 kHz 基线结果：最终真值 582.11 rpm、观察速度 579.61 rpm，状态为 `7`；
4～5 s 目标速度 RMSE 为 3.189 rpm，8～10 s Iq/Id RMSE 分别为
0.000671/0.000139 A。`test.ps1` 还覆盖未收敛超时、闭环失锁和零输出故障路径。

## 实机顺序

1. 电机空载可自由旋转，电源限流 2 A，先确认 `foc_cfg show` 和 `foc_status` 正常。
2. 保持默认 `closedloop 0` 复跑一次短时开环，确认相序、母线、偏置和观察器可靠。
3. 停机后执行 `foc_cfg closedloop 1`，首次只做短时 trace；异常立即 `foc_stop`。
4. 必须看到 `observer-transition → closed-loop`，且无 Rust fault、过流、deadline miss，才延长时间。
5. 反复冷启动并取得编码器、霍尔或外部测速真值后，才能考虑把上电默认值改为 1。

2026-09-23 已烧录 12 kHz 镜像并完成两轮 5 秒接管试验：两轮都在约 2.10 s 进入
transition、约 2.12 s 进入 closed-loop，状态 7 保持到结束；控制错误和 deadline miss
均为 0，ISR 最坏分别为 10,065/12,500 和 10,083/12,500 cycles。每轮结束后都恢复
`closedloop 0` 并关断输出。由于还没有独立轴端速度/角度真值，这属于短时接管证据，
不是参数准确性、负载能力或量产安全证明。
