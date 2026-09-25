# Motor_Profiler 固件 ASPEP 握手修复

本目录记录对**另一个工程**的修改，用于解除 FluxRT 全自动参数识别线路上的阻塞。

| 项 | 内容 |
|---|---|
| 目标工程 | `~/.st_workbench/Projects2/Motor_Profiler`（STM32CubeMX + MCSDK 6.4.1） |
| 改动文件 | `Src/aspep.c` |
| 补丁 | [`MotorProfiler-aspep-修复.patch`](MotorProfiler-aspep-修复.patch) |
| 原文件备份 | 目标工程内 `Src/aspep.c.orig-backup`（勿删） |
| 授权 | 用户明确同意后修改（2026-09-25） |
| 构建 | `cmake --build build/Debug`（GNU Tools for STM32 13.3.1） |
| 结果 | RAM 10072 B / 30.74%，FLASH 86980 B / 66.36% |

## 为什么必须改固件

FluxRT 的脚本化参数识别需要读 Profiler 的寄存器（`SC_RS`/`SC_LS`/`SC_KE` 等）。
这条线路被固件的两个缺陷完全堵死，**且同样导致官方 Motor Pilot 无法连接**
（实测 `syncPacketCount=0`）。控制端无论怎么发包都不可能成功，所以只能改固件。

## 改动一：控制帧（BEACON/PING）没有武装负载接收

`ASPEP_HWDataReceivedIT` 里只有 `DATA_PACKET` 分支会重新武装 RX DMA 去收负载；
`BEACON`/`PING` 分支只置 `NewPacketAvailable`。随后 `ASPEP_RXframeProcess` 末尾
立即执行 `fASPEP_cfg_recept(..., rxHeader, ASPEP_HEADER_SIZE)`，把 DMA 重新指向
下一条包头，**丢弃仍留在 USART 接收寄存器里的 4 字节控制帧负载**。

后果：`rxBuffer` 永远全 0，`ASPEP_CheckBeacon` 拿不到对端能力值。

**修法**：让 `BEACON`/`PING` 分支像 `DATA_PACKET` 一样武装负载接收，状态转入
`WAITING_PAYLOAD`，由既有的 `WAITING_PAYLOAD` 分支在负载到达后置
`NewPacketAvailable`。

```c
case BEACON:
case PING:
{
  pHandle->rxLengthASPEP = (uint16_t)ASPEP_CTRL_SIZE;
  pHandle->fASPEP_cfg_recept(pHandle->ASPEPIp, pHandle->rxBuffer, ASPEP_CTRL_SIZE);
  pHandle->ASPEP_TL_State = WAITING_PAYLOAD;
  break;
}
```

## 改动二：`ASPEP_CheckBeacon` 从错误的位置读能力值

原代码从 **`rxHeader`** 解出对端能力：

```c
uint32_t packetHeader = *((uint32_t *)pHandle->rxHeader);
MasterCapabilities.version    = (packetHeader & 0x70U) >> 4;      /*头部 bit 4..6*/
MasterCapabilities.RX_maxSize = pHandle->rxHeader[1] & 0x3FU;     /*头部 bit 8..13*/
MasterCapabilities.TXS_maxSize= (packetHeader & 0x01FC000U) >> 14; /*头部 bit 14..20*/
MasterCapabilities.TXA_maxSize= (packetHeader & 0xFE00000U) >> 21; /*头部 bit 21..27*/
```

但头部的 **bit 4..16 是 13 位 `length` 字段**，与这些位置**完全重叠**。结果是
`length=4` 的合法 BEACON 被解成 `version=4`，而设备要求 `version=0`，判定恒失败。
实测（临时调试探针读出 `ASPEP_CheckBeacon` 的实际输入）：

```text
packetHeader = 0xB0000045
master: ver=4  DATA_CRC=0 RX=0 TXS=0 TXA=0
device: ver=0  DATA_CRC=0 RX=0 TXS=0 TXA=0
version: 4 != 0  -> FAIL      ← 唯一失败的项
```

**这是一个死结**：任何 packet type 非零的帧，其 `length` 字段都会污染 `version`。
而 `length=0` 又没有负载可带。所以原固件对**任何**合规控制端都不可用——这也解释了
为什么 Motor Pilot 连不上。

`ASPEP_sendBeacon` 自己就是把能力值构造进**控制缓冲区（负载）**的，所以正确位置
本来就是负载。修法即改为从 `rxBuffer` 按同一布局解码。

## 修复后的实测效果

```text
1) 发 BEACON -> 收到 05 c7 01 14     (设备回它的能力值: ver=0 RX=7 TXS=7 TXA=32)
2) 发 PING   -> 收到 f6 00 00 c0     (回包序号)
3) 读 SC_VBUS-> 收到 5a 00 00 c0 00 00 00 00 00   (首次拿到数据帧, 不再是 NACK)
```

**握手已完全打通。** 修复前设备对每个带负载的帧都回
`0f 04 04 c0`（NACK，`BAD_CRC_HEADER`）；现在返回真实数据。

## MCP 寄存器寻址：实测 motorID 必须是 1

修复后扫描 `motorID`（element ID 的低 3 位）得到明确结论：

| motorID | 读 SC_VBUS 的响应 | 含义 |
|---|---|---|
| 0 | `1a 00 00 20 \| 05` | 参数被接受（`MCP_CMD_OK`），但没有返回值 |
| **1** | **`5a 00 00 c0 \| 00 00 00 00 00`** | **唯一的"数据帧"响应** |
| 2–5 | `1a 00 00 20 \| 01` | 参数被拒 |

`motorID = 1` 是唯一能取回数据的取值。`2`–`5` 被拒说明设备校验了取值上界。

## 寄存器读取已完全打通（实测证据）

修复后在实机上逐寄存器核对，结果与工程已知参数**完全吻合**：

| 寄存器 | 类型 | 实测载荷 | 解出 | 核对 |
|---|---|---|---|---|
| `SC_PP` (18) | 8BIT | `07 00` | **7** | ✅ 与 GBM2804H 极对数一致 |
| `SC_CURRENT` (96) | 32BIT | `cd cc 4c 3f 00` | **0.800000 A** | ✅ **正是 EXP-09 要求的识别电流** |
| `SC_STATE` (16) | 8BIT | `00 00` | 0 | ✅ IDLE（未识别） |
| `SC_STEPS` (17) | 8BIT | `0f 00` | 15 | ✅ 识别共 15 步 |
| `SC_COMPLETED` (20) | 8BIT | `00 00` | 0 | ✅ 未完成 |
| `SC_CHECK` (15) | 8BIT | `01 00` | 1 | ✅ |
| `SC_RS` (91) | 32BIT | `00 00 00 00 00` | 0.000000 | ✅ 未识别 |
| `BUS_VOLTAGE` (22) | 16BIT | `0c 00 00` | 12 | 实时母线 |

**`SC_PP = 7` 与 `SC_CURRENT = 0.8 A` 是读取路径正确的铁证**：这两个值不可能来自
巧合，且 `0.8` 的浮点字节 `cd cc 4c 3f` 完全精确。

**结论：Profiler 结果寄存器（`SC_RS`/`SC_LS`/`SC_KE`）读出 0 是正确的**——设备
从未完成过参数识别。设备已按 EXP-09 配好极对数与识别电流，只等启动识别。

### 响应格式（实测）

```text
响应 = 4 字节 ASPEP 头 + 载荷
载荷长度 = 头的 length 字段
载荷内容 = [寄存器值(bytes)] + [00 状态填充]
```

按 `register_interface.h` 的类型定义选择宽度即可正确解码（8BIT=1 字节、
16BIT=2 字节、32BIT=4 字节浮点）。

### Profiler 启动命令：已定出命令字（仍在收尾）

`PROFILER_CMD`（命令码 `0x68`）的处理函数 `MC_ProfilerCommand` 只是 `SCC_CMD` 的
弱符号包装；`SCC_CMD` 的实现在预编译静态库
`MCSDK_v6.4.1-Full/MotorControl/libMP/libmp-G4-*.a` 里。**通过反汇编该库解出了
负载格式**（`ar x` 解包后 `arm-none-eabi-objdump -d mp_self_com_ctrl.o`）：

```asm
00000000 <SCC_CMD>:
   0: push  {r4, r5, r6, lr}
   2: ldrb  r1, [r2, #0]        ; r2 = rxBuffer，取第 1 字节
   6: cmp   r1, #6
   8: bhi   0x88                ; > 6 直接返回错误码 2
   a: mov   r4, r0
   c: tbb   [pc, r1]            ; 按第 1 字节做跳转表（0..6）
```

即**负载第 1 字节是命令选择码，合法范围 0–6**，与官方 `GUI/profiler.qml` 的
`bCMD_*` 常量一一对应。

实测命令字映射（每次重新握手、发 `68 00 <cmd> 00`、读 `SC_STATE`）：

| cmd | 设备响应 | `SC_STATE` | 结论 |
|---|---|---|---|
| **0** | `00` = `MCP_CMD_OK` | 1 | **接受**（`bCMD_SC_STOP`） |
| **1** | `00` = `MCP_CMD_OK` | 2 | **接受**（`bCMD_SC_START`） |
| 2–5 | `02` = `MCP_CMD_UNKNOWN` | 4 | 被拒（本库不支持 HT_*） |
| **6** | `00` = `MCP_CMD_OK` | 1 | **接受**（`bCMD_PPD_START`） |

`SC_STATE` 随命令变化，说明状态机**确实在响应**。

**仍未完成**：`SC_COMPLETED` 始终为 0，说明识别没有真正跑完。可能还需要：
- 命令负载里的第 2 字节（TBB 之后的字段）的取值；
- 或者 `SCC_Start` 的前置条件（反汇编显示它检查 `[r4,#63] == 4` 与 `[r4,#52] == 0`）。

## 回退

```powershell
Copy-Item Src\aspep.c.orig-backup Src\aspep.c -Force
cmake --build build/Debug
# 然后重新烧录
```

备份文件 SHA256 可通过与补丁里的 `index 3598f83..` 对照确认。
