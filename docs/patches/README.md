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

## 仍未完成

寄存器读取的**响应负载**还没解对：四种寄存器（`SC_VBUS`/`SC_RS`/`SC_LS`/`SC_KE`）
返回的都是同一串 `5a 00 00 c0 00 00 00 00 00`，说明解析方式或请求编码仍需调整。
下一步应在控制端侧继续对照，而不是再改固件。

## 回退

```powershell
Copy-Item Src\aspep.c.orig-backup Src\aspep.c -Force
cmake --build build/Debug
# 然后重新烧录
```

备份文件 SHA256 可通过与补丁里的 `index 3598f83..` 对照确认。
