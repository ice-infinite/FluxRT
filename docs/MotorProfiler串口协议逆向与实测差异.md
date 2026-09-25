# ST Motor Profiler 串口协议：逆向结论与实测差异

本文记录从官方源码逆向出的 ASPEP/MCP 协议，以及**实测固件与之不一致的地方**。
写这份文档的原因是：按源码推导的行为在真实设备上多次被证伪，后续任何人接手都
必须先看实测结论，而不是直接读 `Src/aspep.c`。

## 1. 范围与依据

| 项 | 内容 |
|---|---|
| 目标固件 | ST Motor Profiler（MCSDK 6.4.1），板上实测 |
| 硬件 | NUCLEO-G431RB + X-NUCLEO-IHM16M1 + GBM2804H-100T |
| 串口 | ST-Link VCP（COM6），**1843200 baud**，8 位、1 停止位、无校验、无流控 |
| 源码依据 | `Src/aspep.c`、`Inc/aspep.h`、`Src/mcp.c`、`Inc/mcp.h`、`Inc/register_interface.h`、`Inc/parameters_conversion.h`、`Src/mcp_config.c` |
| 实机依据 | 2026-09-25 的原始抓包，已固化为 `tests/profile/test_motor_profiler_client.py` 与 `test_motor_profiler_probe.py` 里的断言常量 |

**重要前提**：Profiler 的识别内核 `SCC_CMD` 是**预编译静态库**（`libmp-G4-*.a`），
源码不在工程内。因此"启动识别"的命令编码**无法从源码得到**，只能实测。

## 2. ASPEP 帧格式

### 2.1 头部（4 字节，小端）

```text
位域 / bit field
  bits  0..3    包类型 packet type
  bits  4..16   负载长度 payload length（13 位）
  bits 17..27   保留 / 包序号 reserved / packet number
  bits 28..31   CRC-4，覆盖低 28 位 CRC-4 over the low 28 bits
```

包类型 / packet types：`BEACON=0x5`、`PING=0x6`、`DATA_PACKET=0x9`、`ACK=0xA`、
`NACK=0xF`。

### 2.2 CRC-4

生成多项式 `x^4+x+1`（CCITT-G704）。固件用两张查表：

- `CRC4_Lookup8[256]`：按字节喂 3 次
- `CRC4_Lookup4[16]`：最后一个 nibble 用

关键细节：**字节表是由 nibble 表按"低 nibble 先入"复合而成的**，即
`byte_table[b] = N[N[0 ^ 低nibble] ^ 高nibble]`。反过来（先高后低）只能匹配
32/256，实测确认。

校验方式：把整个 32 位头部（含 CRC 自身）再算一遍，**结果为 0 才有效**。

> 本工程的 `tools/motor_profiler_client.py` 把两张查表的**整表 SHA-256**
> 钉在测试里，防止抄表出错。

### 2.3 握手状态机（按源码）

```text
performer 侧（固件）:
  IDLE        收到 BEACON -> 校验能力, 回 BEACON, 进入 CONFIGURED
              收到 PING   -> 回 PING(RESET), 留在 IDLE
  CONFIGURED  收到 BEACON -> 再校验, 留在 CONFIGURED(or 回 IDLE), 回 BEACON
              收到 PING   -> 回 PING(CFG), 进入 CONNECTED
  CONNECTED   收到 DATA_PACKET -> 交给 MCP 处理, 回同步响应
```

**控制端必须先发 BEACON。** 固件 `ASPEP_start` 只做
`fASPEP_cfg_recept(rxHeader, 4)`（只配置接收），整个固件没有任何主动发送 BEACON
的路径；`mc_tasks.c` 调用 `ASPEP_start(&aspepOverUartA)` 之后就一直在等。
所以"等对方先发包"会永远超时。

### 2.4 能力协商

`ASPEP_CheckBeacon` 的判定**不是取小后比较**，而是：

| 字段 | 判定 |
|---|---|
| `version` | 严格相等 |
| `DATA_CRC` | 严格相等（且取小后才比，见下） |
| `RX_maxSize` | 控制端**可以更大**（`> performer` 才失败） |
| `TXS_maxSize` | 严格相等 |
| `TXA_maxSize` | 严格相等 |

固件自己的初始能力（`Src/mcp_config.c` 的 `aspepOverUartA`）：

```text
DATA_CRC = 0        RX_maxSize = (256>>5)-1 = 7
TXS_maxSize = 7     TXA_maxSize = 2048>>6 = 32     version = 0
```

**`DATA_CRC` 必须是 0。** 官方在使能时的数据段 CRC 是**未实现的桩**，源码里就是
`/* TODO : Compute real CRC*/`，写死 `0xCA 0xFE`。协商成 1 会让每个数据包校验失败。

### 2.5 MCP 命令层

同步负载 = 2 字节命令头 + 参数。命令码（`Src/mcp.h`）：

| 命令 | 值 |
|---|---|
| `GET_MCP_VERSION` | 0x00 |
| `SET_DATA_ELEMENT` | 0x08 |
| `GET_DATA_ELEMENT` | 0x10 |
| `START_MOTOR` | 0x18 |
| `STOP_MOTOR` | 0x20 |
| `STOP_RAMP` | 0x28 |
| `START_STOP` | 0x30 |
| `PROFILER_CMD` | 0x68 |
| `SW_RESET` | 0x78 |

响应码：`MCP_CMD_OK=0x00`、`MCP_CMD_NOK=0x01`、`MCP_ERROR_BAD_DATA_TYPE=0x07`、
`MCP_ERROR_BAD_RAW_FORMAT=0x0A`。

### 2.6 寄存器寻址

16 位 element ID：

```text
  bits  3..5   类型编码 type code
  bits  6..15  寄存器号 register index
  bits  0..2   电机号 motor id（固件做 (id-1) & 0x7，所以从 1 开始）
```

类型编码是 `N << 3`：`SEG_END=0`、`8BIT=1`、`16BIT=2`、`32BIT=3`、`STRING=4`、
`RAW=5`、`FLAG=6`、`SEG_BEG=7`。

读寄存器：`GET_DATA_ELEMENT` + element ID；写：`SET_DATA_ELEMENT` + element ID + 数据。

### 2.7 Profiler 寄存器（来自官方 Motor Pilot 清单）

结果寄存器（全部 `F32`，可直接映到 FluxRT 的电机参数）：

| 寄存器 | ID | 含义 |
|---|---|---|
| `SC_RS` | 91 | 定子电阻 `[ohm]` |
| `SC_LS` | 92 | 定子电感 `[H]`（只测 `Ld`，不单独测 `Lq`） |
| `SC_KE` | 93 | 反电势常数 `[Vrms/kRPM 线电压]` |
| `SC_VBUS` | 94 | 母线电压 `[V]` |
| `SC_MEAS_NOMINALSPEED` | 95 | 实测最高转速 `[rpm]` |
| `SC_J` | 101 | 转动惯量 |
| `SC_F` | 102 | 粘滞摩擦 |
| `SC_MAX_CURRENT` | 103 | 电流 |

输入/状态寄存器：`SC_PP`(18, U8, 极对数)、`SC_NOMINAL_SPEED`(99, **16 位**)、
`SC_CURRENT`(96, F32)、`SC_STATE`(16, U8)、`SC_STEPS`(17, U8)、
`SC_COMPLETED`(20, U8)。

> **官方两处定义冲突**：`SC_NOMINAL_SPEED` 在固件 `register_interface.h` 里是
> `TYPE_DATA_16BIT`，而 Motor Pilot 的 `RegListProfiler_MCPV2.json` 标成 `S32`。
> 实测按 16 位读回正常值，按 32 位只拿到一个响应码。**以固件为准。**

## 3. 实测固件与源码不一致的地方

以下每条都有原始抓包，并已固化为测试断言。**这是本文最重要的部分。**

### 3.1 控制帧没有负载

官方 `ASPEP_sendBeacon` / `ASPEP_sendPing` / `ASPEP_sendNack` 都构造 4 字节负载
并把 `ASPEP_CTRL_SIZE` 传给发送函数。但实测设备发的是**只有 4 字节头部**的帧：

```text
BEACON 回应: 05 00 00 50
NACK:        0f 04 04 c0
PING 回应:   06 00 00 60
```

### 3.2 控制帧的长度字段不可信

实测 NACK 字节 `0f 04 04 c0` 解出的**长度字段是 64**，但设备只发 4 字节就结束。
真实固件既没按源码填 4，也没清零，而是把错误码塞进了头部的高位。

**结论：对控制帧必须"尽力读取"，按长度字段读永远读不齐。**

### 3.3 NACK 是"滞后报告"，不是对当前帧的判决

同一会话内连发 6 次 BEACON，响应呈固定三拍循环：

```text
发#1 -> BEACON | NACK
发#2 -> NACK
发#3 -> NACK | NACK
发#4 -> BEACON | NACK      <- 循环回到 #1 的形态
发#5 -> NACK
发#6 -> NACK | NACK
```

设备**基本每帧都接受**；NACK 报的是此前积累的错误状态。源码注释也印证：
`ASPEP_RXframeProcess` 发完一次 NACK 就调 `fASPEP_HWSync` 重新同步，并明确写着
"下一个包会丢失"。

### 3.4 发送速率影响可靠性

Windows 侧一次写入 8 字节实测耗时约 **170–210 µs**，远高于 8 字节 @1843200 的
理论值 43 µs，说明写入经过 OS/VCP 缓冲。逐字节加间隔可提高稳定性；本工程 CLI
默认 **1 ms/字节**。

### 3.5 错误码可区分

设备对**故意构造**的坏帧能给出正确且可区分的错误码，这一条与源码一致：

| 我发的 | 错误码 | 含义 |
|---|---|---|
| 坏 CRC 的头部 | 4 | `BAD_CRC_HEADER` |
| 长度 8191 的 DATA_PACKET | 2 | `BAD_PACKET_SIZE` |
| 未定义包类型 | 4 | `BAD_CRC_HEADER` |

## 4. 当前进度与未解问题

### 4.1 已打通

| 项 | 状态 | 证据 |
|---|---|---|
| CRC-4 实现 | ✅ | 对设备自己发的头部与本地构造的头部都自洽 |
| BEACON 交换 | ✅ 稳定 | 多次运行每次都能拿到 BEACON 回应 |
| 能力协商 | ✅ 不是阻塞点 | 约 10 组不同能力值（含官方 7/7/32）都能拿到回应 |
| PING 帧格式 | ✅ 正确 | 一次性测试中拿到过 PING 回应 |

### 4.2 未打通

| 项 | 现象 |
|---|---|
| PING 在 BEACON 之后 | 被 NACK；但单独发送时曾成功过一次，差异未定位 |
| 寄存器读写 | 未成功，`DATA_PACKET` 被 NACK |
| Profiler 启动命令 | 编码在预编译库里，**源码不可得** |

### 4.3 已系统排除的因素

- **不是 CRC 算法问题**：同一实现能验证设备自己发的帧。
- **不是 UART 参数问题**：与固件 `huart2` 配置逐项一致；
  `USART2 CR1=0x4D`、`CR3=0xC1`、时钟与 NVIC 均已确认。
- **不是发送速率问题**：空闲时间 0.05–2.0 s、字节间延时 0–5 ms 都试过，接受率
  无显著变化。
- **不是能力协商问题**：多种能力值都能拿到回应。

## 5. 建议的下一步

**抓一次真实上位机的完整对话。** 这是唯一能同时确定"可靠帧格式"与"启动命令
编码"的方法，而且不需要再猜：

```powershell
# 1. 启动 Motor Pilot 并 Connect
# 2. 同时用 OpenOCD + GDB 旁读设备收到的字节（断点或监视 rxHeader）
# 3. 把抓到的字节与 tools/motor_profiler_probe.py 的输出逐字节对比
```

`tools/motor_profiler_probe.py --dump` 可在不发任何帧的情况下监听设备输出。

### 一个必须记住的操作教训

**OpenOCD 连接时会挂起 CPU。** 排查过程中多次出现"设备无响应"，实际是调试会话
把目标停在 `main.c:122` 的 `while(1)` 里。用 `-c 'reset run'` 启动，并让目标在
断开后自由运行。

## 6. 相关文件

| 文件 | 作用 |
|---|---|
| `tools/motor_profiler_client.py` | ASPEP/MCP 客户端：帧编解码、握手、寄存器读写 |
| `tools/motor_profiler_probe.py` | 原始对话诊断：不依赖成帧，按固定模式统计接受/拒绝 |
| `tools/motor_profiler_convert.py` | Motor Pilot 导出 → FluxRT 参数候选 |
| `tests/profile/test_motor_profiler_client.py` | 协议测试，含 CRC 表指纹与实测抓包断言 |
| `tests/profile/test_motor_profiler_probe.py` | 诊断工具测试，断言用实测字节序列 |
| `tests/profile/test_motor_profiler_convert.py` | 转换工具测试，含磁链量纲差异的钉板测试 |

另见 [工程操作日志](工程操作日志.md) 的 `LOG-20260925-005`～`007`，
以及 [参数辨识采集与筛查流程](parameters/参数辨识采集与筛查流程.md)。
