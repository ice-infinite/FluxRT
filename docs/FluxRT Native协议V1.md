# FluxRT Native 协议 V1

## 1. 当前冻结范围

V1 已冻结**目标中立帧层、首批固定载荷和参数事务线格式**：魔数、版本、固定头、
节点/Axis寻址、消息类型、CRC-32C、字节流重同步，以及Identity、Capabilities、Status、
CommandResult、ProductCommand和参数get/set/transaction逐字段编码。唯一实现位于Rust
`no_std` crate `rust/crates/foc-protocol/`，C/RT-Thread只能通过`foc_native_bridge.h`调用。

当前仍没有UART/USB/CAN驱动，也没有把命令提交给CommandService/MotorService或真实配置
执行器。Fake driver只把已有64 B有界packet队列接到Rust字节流和只读Fake service；
ProductCommand及参数写/事务都返回`READ_ONLY`。没有参数持久化、CAN分片或上位机。
解析成功只表示帧/载荷合法，不表示命令获准执行。

机器可读帧契约/向量为`schema-v1.json`、`golden-v1.json`；payload契约/向量为
`payload-schema-v1.json`、`payload-golden-v1.json`，均位于`protocol/native/`。

## 2. 线格式

所有多字节整数使用 little-endian。帧长度为 `20 + payload_length + 4`，最大280 B。

| offset | 大小 | 字段 | V1规则 |
|---:|---:|---|---|
| 0 | 2 | magic | ASCII `FR`，hex `46 52` |
| 2 | 1 | wire version | `1` |
| 3 | 1 | header size | `20` |
| 4 | 2 | flags | request/response/error/ack-required；未知bit拒绝 |
| 6 | 2 | message type | 只接受已冻结编号 |
| 8 | 2 | source node | `1..65534`，不能广播 |
| 10 | 2 | destination node | `1..65535`；`65535`为广播，响应禁止广播 |
| 12 | 2 | Axis | `0..255`，`65535`表示设备级 |
| 14 | 2 | payload length | `0..256` |
| 16 | 4 | sequence | 发送方32位序号；重放/乱序策略由上层owner处理 |
| 20 | N | payload | 按消息schema逐字段编码，不允许复制Rust/C内存布局 |
| 20+N | 4 | CRC | CRC-32C/Castagnoli，覆盖固定头和payload |

CRC参数：reflected polynomial `0x82F63B78`，initial/final xor均为`0xFFFFFFFF`。

## 3. 标志与消息编号

| flag | 值 | 约束 |
|---|---:|---|
| Request | `0x01` | request消息必须置位，不能同时有Response/Error |
| Response | `0x02` | response消息必须置位，不能同时有Request/AckRequired |
| Error | `0x04` | 只允许随Response |
| AckRequired | `0x08` | 只允许随Request |

| 消息 | 请求/响应编号 | 当前payload状态 |
|---|---:|---|
| Discover | `0x0001 / 0x0002` | 请求0 B；响应Identity 48 B |
| Capabilities | `0x0011 / 0x0012` | 请求0 B；响应Capabilities 64 B |
| Status | `0x0021 / 0x0022` | 请求0 B；响应Status 64 B |
| ProductCommand / Result | `0x0031 / 0x0032` | command 104 B逐字段编码；result 32 B |
| Parameter Get | `0x0041 / 0x0042` | 请求32 B；响应256 B，单块数据最大224 B |
| Parameter Set | `0x0043 / 0x0044` | 请求256 B；响应32 B |
| Parameter Commit | `0x0045 / 0x0046` | 请求/响应32 B；承载BEGIN/VALIDATE/APPLY/COMMIT/ROLLBACK |

ProductCommand虽然与当前C结构同为104 B，编码器仍逐字段写入13个`u32`和13个IEEE-754
`binary32`，解码后再调用产品契约语义校验；禁止`memcpy(foc_product_command_t)`。

## 4. 首批payload

| payload | 大小 | 关键语义 |
|---|---:|---|
| Identity | 48 B | 产品/固件/配置ABI、board/motor/inverter、profile/build、axis数、node ID |
| Capabilities | 64 B | Axis/控制/输入/反馈/命令mask，compiled/board transport与input，最大载荷和最小状态周期 |
| Status | 64 B | device flags、uptime/config revision、Axis/模式/source/fault、四个带valid位的物理量 |
| CommandResult | 32 B | 请求关联序号、source/Axis、结果/detail、仅成功时可非零的accepted sequence |
| ProductCommand | 104 B | 公共产品命令V1的显式little-endian字段映射，完整语义仍由产品契约验证 |
| ParameterGetRequest | 32 B | active/pending作用域、事务token、组mask、offset/length；active禁止携带token |
| ParameterGetResponse | 256 B | result、配置revision、总长度、offset、224 B有界chunk；失败响应不得夹带数据 |
| ParameterSetRequest | 256 B | request/token/group/offset、final-chunk、224 B有界chunk；boolean必须canonical |
| ParameterTransaction | 32 B | BEGIN/VALIDATE/APPLY/COMMIT/ROLLBACK、组mask、token和revision的事务请求/结果 |

未知mask/枚举、非有限浮点、非零reserved、capability子集冲突和“无valid位但残留非零值”均
拒绝。Capabilities明确区分compiled与board能力；Fake向量中的能力只用于协议测试，不是
当前G431板的能力声明。

## 5. 分层和安全边界

```text
UART/USB/CAN driver (C, bytes/frames only)
    -> bounded transport queue
    -> foc-protocol (Rust: frame/CRC/resync)
    -> message payload adapter (Rust: schema/value validation)
    -> CommandService / ConfigService (C application owner)
    -> Rust CommandArbiter / transaction core
    -> MotorService safety gate
```

- codec无动态分配、锁、RTOS、HAL和全局可变状态；
- stream decoder使用固定316 B上下文，最大帧280 B；
- 坏magic/version/header/flag/type/address/length/CRC均fail-closed；
- 字节流遇到噪声后只在`FR`魔数重新同步；
- C ABI尺寸固定：message 292 B、stream decoder 316 B；
- payload C ABI固定为48/64/64/32/32/256/256/32 B，Fake service上下文196 B；
- 默认Kconfig关闭；选中codec也不会声明任何board transport、启动驱动或arm；
- Fake service支持完整frame decode→只读处理→encode内存链路；Fake driver在管理线程中按
  byte budget消费64 B packet，保留半帧状态，把最大280 B响应重新分片，并在TX背压时保留
  待发帧；禁用、链路不健康或恢复时复位decoder，绝不自动arm；
- UART/USB可直接消费字节流；CAN Classic/CAN-FD的分片、总线ID和重组尚未冻结，禁止把
  280 B逻辑帧直接塞入CAN帧。

## 6. Golden

广播Discover请求：source=42、destination/device-axis=`0xFFFF`、sequence=`0x78563412`。

```text
46 52 01 14 01 00 01 00 2a 00 ff ff ff ff 00 00
12 34 56 78 e2 d6 3a 11
```

Rust单测和独立Python合同测试都锁定该向量、CRC-32C已知向量`123456789 -> 0xE3069283`，
并检查该帧的全部192个单bit变异不能同时通过头与CRC。

P2.6B另冻结Identity、Capabilities、Status、READ_ONLY CommandResult与Release ProductCommand
五组payload向量，其中前四组还包含完整frame及CRC。独立Python按`struct`重新解析，Rust按
字段编码，二者不共享序列化实现。

## 7. P2.6C代码冻结边界与下一批次

P2.6C的软件骨架已完成：参数线格式、严格长度/保留位/token/group校验、C镜像、只读错误
响应，以及有界Fake driver的分片、背压、断流与恢复状态机。它故意没有绑定真实配置事务
执行器，也没有声明任何板级transport能力。

后续按以下顺序继续：

1. 批量验证参数codec、Fake driver的64 B分片、背压、断流和恢复；
2. 再分别实现UART/USB流适配及CAN/CAN-FD分片、超时、bus-off和预算；
3. 把真实snapshot provider接入只读状态服务；
4. 最后接CommandService/ConfigService，设备端仍负责权限、租约和安全裁决。
