# A14：不可变 ProductionMotorProfile 与配置 CRC

日期：2026-09-24  
基线：`main` / `41796e52928768c75b5ee9d162ed405bb1d19c0a`，包含 A0～A13 未提交工作区  
目标硬件：NUCLEO-G431RB + X-NUCLEO-IHM16M1 + GBM2804H-100T

## 1. 结论

本轮把 Production 从“没有在线调参，所以永远只能用安全默认值”推进到“能够装入并
验证编译期只读参数档案”，但**没有审批闭环**：

- Diagnostic 与 Production 共用同一个只读档案和同一套校验逻辑；
- 档案带 schema、revision、板卡 ID、电机 ID、运行配置版本、运行配置 CRC、审批位和
  档案自身 CRC；
- Rust 对完整 `foc_runtime_config_t` 做规范化 CRC-32/ISO-HDLC，不读取结构体填充；
- C 应用层校验身份、审批依赖与档案 CRC，并只在全部通过后事务式提交配置；
- 失败时运行配置不变，Rust 控制器不 configure，平台不 bind，`foc_start` 因未绑定而
  拒绝；
- 当前 revision 1 的 `approval_flags=0`，因此 `closed_loop_enable` 必定为 0；Production
  没有 `foc_cfg`，不存在运行现场绕过路径。

这是一道“配置身份与审批门”，不是参数正确性的证明。当前 Rs/Ls/磁链仍来自 MCSDK
Workbench 转写，尚未完成实物辨识。

## 2. 架构落位

```text
Rust 默认完整配置
        │
        ▼
applications/foc_production_profile.c
  - 应用层覆盖值
  - 板卡/电机身份
  - approval 依赖
        │
        ├──→ Rust 规范化 runtime CRC
        ├──→ C 档案 record CRC
        └──→ 全部通过后一次性提交
                     │
                     ▼
              foc_rust_configure()
                     │
                     ▼
              平台 bind（失败则不 bind）
```

职责边界：

- `applications/foc_production_profile.*`：参数来源、身份、审批和提交策略；
- `foc-rt-bridge`：C ABI 布局、运行配置合法性和跨 Host/MCU 一致的 CRC；
- `main.c`：启动顺序、失败关断和只读报告；
- 平台 ISR、`foc-control` 与 `foc-algorithm`：完全不解析档案，不增加实时路径工作。

## 3. revision 1 固定内容

| 字段 | 值 | 含义 |
|---|---:|---|
| magic | `0x46505246` | `FPRF` |
| struct size | 40 B | 10 个连续 `uint32_t` |
| schema / revision | `1 / 1` | 档案格式 / 本参数候选版本 |
| board ID | `0x43116001` | NUCLEO-G431RB + IHM16M1 |
| motor ID | `0x28041007` | GBM2804H-100T，7 极对 |
| runtime config version | 6 | 当前 240 B 配置布局 |
| runtime config CRC | `0x8C7A8DF4` | 完整 60 个 ABI 字的规范化 CRC |
| approval flags | `0x00` | 参数未审批，闭环未审批 |
| record CRC | `0xD9F996C3` | 覆盖前九个档案字 |

审批位必须满足依赖：`CLOSED_LOOP_APPROVED` 只有在 `PARAMETERS_APPROVED` 同时存在时才
合法。当前两个位都为 0。未来不能只手改审批位；任何字段变化都会要求重算 runtime
CRC 和 record CRC，并应关联辨识报告、独立真值和评审记录。

## 4. CRC 语义与边界

算法为 CRC-32/ISO-HDLC：初值与末异或均为 `0xFFFFFFFF`，反射多项式
`0xEDB88320`。运行配置由 60 个连续 `u32/f32` ABI 字组成，编译期断言固定：

- `sizeof(foc_pi_config_t) = 28`；
- `alignof(foc_runtime_config_t) = 4`；
- `sizeof(foc_runtime_config_t) = 240`；
- C 侧继续用 `_Static_assert` 对拍相同尺寸。

每个本机 ABI 字先恢复为逻辑 `u32`，再按小端字节序送入 CRC，因此 x86 Host 与
Cortex-M4F 得到相同结果。Rust 会先执行完整配置合法性检查，NaN、越界、错误版本等
配置不会获得 CRC。

CRC 能发现意外改参、版本漂移和常见位错误，但它不是数字签名，不能防止有意修改源码
后同时重算 CRC。若以后需要防篡改，应再引入签名/安全启动，不能把 CRC 当认证。

## 5. 验证

### S1：Host 与 Rust

`test.ps1` 全部通过：

- `foc-algorithm` 69 项；
- `foc-control` 16 项；
- `foc-rt-bridge` 11 项；
- `foc-sim` 15 项；
- Clippy、CPU-only 和 CORDIC Cortex-M4F 静态库构建；
- C Host 测试验证正确档案、错误 target、非法审批、record/runtime CRC 错误、Rust CRC
  失败以及“失败不修改原配置”。

Rust 测试锁定当前 runtime CRC 为 `0x8C7A8DF4`，并验证单 bit 合法参数变化会改变 CRC，
空指针/NaN 会拒绝且不覆盖输出。

### S2：目标构建

| 指标 | A14 Diagnostic + `s` | A14 Production + `s` |
|---|---:|---:|
| text | 124,984 B | 94,544 B |
| data | 1,764 B | 1,156 B |
| bss | 12,396 B | 7,740 B |
| ROM | 126,748 B（96.70%） | 95,700 B（73.01%） |
| RAM | 14,160 B（43.21%） | 8,896 B（27.15%） |
| 相对 A13 ROM | +1,072 B | +1,080 B |

初版逐字段展开 CRC 曾使 Diagnostic 达到 128,868 B（98.32%）。改成“编译期无填充
断言 + 60 字循环”后，导出的 CRC 函数从 2,242 B 降为 124 B，CRC 值不变，最终回收
2,120 B。Diagnostic 仍只余 4,324 B，后续新增功能必须继续做 Flash 预算。

最终固件 SHA-256：

| 构建档 | BIN | HEX | ELF |
|---|---|---|---|
| Diagnostic | `175CD0D3FCB1F95AB2C7AAC427BC6DBD3C6AFCEE686E3D8A0B23F14FB79A5FC9` | `F6866C57CA4246C24F14EFE69C510E1AA05EC541DD4842BCC797A0F855416A81` | `85425BBD95EC1C0C2F1DCF0ACE33948736D0ED86F01F9EB16D25F8B6498E58A8` |
| Production | `1CB3333A4BA481BF0178443E2D8AC7D95777B65409FF60A3047C35D54E84B743` | `65FDED3F87D9F814CE2967709C210A20BB67EC63F3017ACCCB6D0DAEA2C0E61F` | `B2C54F08A9C852B0FF99B3BC8CDA5C13B81B1C381593EFE449173CBCAF8DB3B2` |

### S4：Diagnostic 启动与只读状态

通过 ST-Link `00400027544B500120343637` 将最终 Diagnostic HEX 下载、读回校验并复位。
没有执行 `foc_start`。串口 `foc_stop`、`foc_cfg show`、`foc_status` 回读：

```text
boot-profile status=0 rev=1
board=0x43116001 motor=0x28041007 approvals=0x00
runtime/record CRC=0x8c7a8df4/0xd9f996c3
closed_loop=0, state=uninitialized, steps/errors/misses=0/0/0
duty=0/0/0, flags=0x0000231f, Vbus≈12.262 V
```

`status=0` 表示“档案结构、身份与 CRC 有效，但未审批”，不表示闭环可用。最终功率级未
armed，输出关闭，电机未运行。

## 6. 下一步

1. 建立参数辨识候选记录格式，保存温度、母线、限流、重复次数、误差和原始数据；
2. 增加离线 profile 生成/复核工具，由工具生成两个 CRC，避免人工抄写；
3. 参数审批与闭环审批分开，闭环审批必须再关联独立转速/角度真值、故障注入和长测；
4. 在上述证据具备前继续保持 revision 1、`approval_flags=0` 和
   `closed_loop_enable=0`。
