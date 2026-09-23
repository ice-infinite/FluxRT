# FluxRT 项目操作记录

## 1. 用途

本文是项目的持续操作账本，用来回答：

- 上一次改了什么、为什么改；
- 哪些文件、配置和参数受到影响；
- 做过哪些检查，证据属于哪一级；
- 当前能回退到哪里；
- 还有哪些风险，下一步应该做什么。

记录与 Git 配合使用，但不能代替 Git。Git 保存可恢复的文件快照，本文保存修改目的、
测试条件、实机条件、结果解释和下一步判断。

所有后续代码、配置、构建、文档、仿真、测试、烧录、实机、Git 和发布操作，在报告
完成前都必须更新本文。记录按时间倒序排列，最新记录放在最前面。

## 2. 当前交接状态

| 项目 | 当前状态 |
|---|---|
| 最近记录 | `LOG-20260923-005` |
| 工程阶段 | 路线阶段 1.1 构建优化矩阵与分档完成；架构工作包 A1 尚未开始，下一步优化共享 Clarke 和 CORDIC 事务 |
| 当前默认安全状态 | `closed_loop_enable=0`，新增补偿/相电压/HFI 等功能均未默认启用 |
| 当前固件基线 | 板上为 Diagnostic + Rust `s`：BIN 122,812 B；开环完整 ISR 8,257/12,500 cycles |
| Git 状态 | `main` 连接 `https://github.com/ice-infinite/FluxRT.git`；本批修改待由 `LOG-20260923-005` 所在提交追踪 |
| 当前可恢复点 | `5e3b85f` A0 基线、GitHub `main`、本条记录所在提交及对应固件 SHA-256 |
| 下一建议动作 | 在 12 kHz 和 Rust `s` 不变时复用 Clarke/中间量并减少 CORDIC 事务；闭环观察器失锁另立整改项 |
| 禁止误读 | 文档中“规划新增”的文件和功能尚未实现 |

## 3. 强制记录规则

### 3.1 哪些操作必须记录

- 新增、删除、移动或修改源码；
- Kconfig、`.config`、CMake、SCons、Cargo、链接脚本和依赖变化；
- 电机、控制器、观察器、保护、时序和硬件参数变化；
- 文档、接口、CSV schema、MATLAB/Simulink 模型变化；
- 格式、静态检查、Host 测试、交叉构建和 size/map 检查；
- 仿真、参数扫描、数据清洗和绘图；
- 烧录、串口操作、示波器测量和电机实机运行；
- Git 初始化、分支、提交、合并、revert、tag 和发布包；
- 失败、回退、临时绕过和没有完成的尝试。

只读查看如果没有形成新判断，可以不单独建记录；如果查看改变了项目结论、参数判断或
下一步方案，也应记录。

### 3.2 证据等级必须分开

| 等级 | 含义 | 不能证明 |
|---|---|---|
| S0 | 设计/源码静态检查 | 能编译或能运行 |
| S1 | 格式、lint、单元测试、Host 测试 | 目标 MCU 可链接或实时性满足 |
| S2 | STM32 交叉构建、size/map | 已成功烧录或硬件正常 |
| S3 | PC/MATLAB/Simulink 仿真 | 实机与模型一致 |
| S4 | 成功烧录、启动和通信 | 功率级或电机已安全运行 |
| S5 | 受限低功率实机验证 | 全工况、长期或量产可用 |
| S6 | 故障注入、长测、多样本和独立真值 | 未覆盖条件下仍一定安全 |

记录中必须写实际达到的等级，不能用 `PASS` 笼统替代。

### 3.3 Git 关联规则

当前项目不在 Git 工作树中，因此现有记录只能写 `NO-GIT`，不能虚构提交号。将项目
纳入 Git 后，每条记录至少包含：

- 开始分支和 `base commit`；
- 关联的实现提交；
- 工作树是否仍有未提交修改；
- 若创建里程碑 tag，记录真实 tag；
- 可执行的回退方式。

推荐两种方式：

1. **同提交记录**：代码、测试和本条日志一起提交。日志中的“结果提交”写
   `本条记录所在提交`，实际哈希用 `git log -1 -- docs/PROJECT_OPERATION_LOG.md` 查询；
2. **实现提交 + 日志提交**：先提交实现，再在日志中写入实现哈希并提交日志。适合实机
   结果需要在实现提交之后补充的情况。

不要为了填写日志而修改已经存在的提交；不要使用破坏性 reset 作为常规恢复方式。
优先使用：

```powershell
git show <commit>
git diff <base>..<result>
git switch -c recovery/<name> <known-good-commit>
git revert <bad-commit>
```

具体操作前仍要检查工作树，保护用户未提交的修改。

### 3.4 实机记录附加字段

涉及功率级或电机时必须额外记录：

- 开发板、驱动板、电机和传感器；
- 电源电压、电流上限、负载和机械安全条件；
- 实际烧录文件的路径、构建时间和 SHA-256；
- 启动命令、目标转速、电流限制、运行时长和停止方式；
- Fault、deadline、采样丢失、温度、声音和波形；
- 结束后是否恢复默认配置、关闭功率级并确认电机停止。

## 4. 新记录模板

复制以下模板并放在“操作记录”标题之后，保持最新记录在最前面：

```markdown
### LOG-YYYYMMDD-NNN｜简短标题

- 时间：YYYY-MM-DD HH:mm +08:00
- 负责人：User / Codex / 其他
- 状态：完成 / 部分完成 / 阻塞 / 已回退
- 类型：代码 / 配置 / 构建 / 文档 / 仿真 / 测试 / 烧录 / 实机 / Git / 发布
- 目标：本次只解决什么问题
- 开始基线：branch + commit；没有 Git 时写 NO-GIT
- 关联提交：commit/tag；没有创建时如实写“未提交”或 NO-GIT

#### 修改内容

| 路径/对象 | 类型 | 修改说明 |
|---|---|---|
| `path` | 新增/修改/删除/参数 | 具体变化和原因 |

#### 关键参数变化

| 参数 | 修改前 | 修改后 | 原因 |
|---|---:|---:|---|
| 无则写“无” | | | |

#### 验证与证据

| 等级 | 命令/场景 | 结果 | 产物/记录 |
|---|---|---|---|
| S0～S6 | 实际执行内容 | PASS/FAIL/未执行及原因 | 路径或摘要 |

#### 实机条件

不涉及写“不涉及”。涉及时填写板卡、电机、电源、限流、负载、固件 SHA-256、命令、
时长、故障和结束安全状态。

#### 风险、回退与遗留

- 风险：
- 回退：
- 未完成：
- 下一步：
```

## 5. 操作记录

### LOG-20260923-005｜阶段 1.1 优化等级矩阵与构建分档

- 时间：2026-09-23 13:42 +08:00
- 负责人：User + Codex
- 状态：完成；构建分档和低功率实机矩阵通过，生产全工况验收未执行
- 类型：代码 / 构建系统 / 测试 / 烧录 / 实机 / 文档
- 目标：实测 Rust `3/s/z` 的 Flash/WCET 取舍，建立不会混用静态库的 Diagnostic/Production 构建档，并回收生产候选固件空间。
- 开始基线：`main`，`5e3b85fd84b2d09f08c896acdd312b3e286afd81`。
- 关联提交：本条记录所在提交。

#### 修改内容

| 路径/对象 | 类型 | 修改说明 |
|---|---|---|
| `rust/Cargo.toml` | 参数 | 默认 release `opt-level` 由 `3` 改为实机选出的 `s`，LTO/单 codegen unit/abort 保持不变 |
| `custom.cmake` | 构建 | 增加 `FLUXRT_BUILD_PROFILE` 和 `FLUXRT_RUST_OPT_LEVEL`；各优化档独立 target 目录并覆盖生成的旧链接路径 |
| `build.ps1` | 构建 | 增加 Diagnostic/Production、`3/s/z` 参数、cache 一致性检查、独立输出目录和进程环境恢复 |
| `applications/main.c` | 构建档 | 显示构建身份；Production 保留 start/stop/status，裁掉在线浮点配置和 trace Shell/输出 |
| `foc/platform/stm32g431/foc_platform_stm32g431.c` | 构建档 | Production 裁掉 trace 缓冲与 ISR 采集，保留控制、保护、关联 WCET 和 deadline |
| `simulation/capture_timing_baseline.py` | 工具 | 增加构建档参数；Production 只允许 trace-off，避免调用已裁剪命令 |
| `.vscode/tasks.json`、`.gitignore` | 工具 | 增加 Production 构建任务并忽略独立构建目录 |
| `docs/BUILD_PROFILES.md`、`docs/performance/2026-09-23-a1-build-profile-matrix.md` | 新增 | 固化使用方法、矩阵、哈希、裁剪边界和证据等级 |
| `README.md`、整改路线 | 文档 | 更新默认优化档、闭环风险、Production 边界和阶段状态 |

#### 关键参数变化

| 参数 | 修改前 | 修改后 | 原因 |
|---|---:|---:|---|
| Rust 默认 `opt-level` | `3` | `s` | Diagnostic 回收 5,080 B，WCET 仅从 8,098 增至 8,257 cycles |
| Diagnostic BIN | 127,892 B（同版 `3`） | 122,812 B（`s`） | 日常调试至少恢复约 8 KiB Flash 余量 |
| Production BIN | 无独立档 | 93,004 B | 裁掉在线浮点调参和 trace，Flash 余量 29.04% |
| Production RAM | 无独立档 | 8,000 B | trace 缓冲和相关状态被裁掉 |
| PWM/控制/安全阈值 | 12 kHz / 12 kHz / 原值 | 未改 | 本批只改构建与诊断能力，不混入控制律调整 |
| 默认闭环 | `0` | `0` | 构建优化不能越过观察器稳定性门 |

#### 验证与证据

| 等级 | 命令/场景 | 结果 | 产物/记录 |
|---|---|---|---|
| S0 | map/nm 检查 | Production 不含 trace、`foc_cfg`、`strtod/strtof`、`vfiprintf`；仍含 start/stop/status、ISR 和平台启停 | 阶段 1.1 报告 |
| S1 | `.\test.ps1` | PASS；Rust 94 项、格式、Clippy、CPU/CORDIC 交叉库、C Host 1/1 和 PMSM 仿真通过 | 控制台记录 |
| S1 | Python 编译、profile 参数拒绝、PowerShell PATH 前后对比 | PASS；Production 非零 trace 频率被拒绝；构建后仍解析到同一个 Python/pyserial | 控制台记录 |
| S2 | Diagnostic `3/s/z`、Production `s` | 全部构建通过；尺寸见阶段 1.1 报告 | 两个 CMake 输出目录 |
| S4 | 四档 SWD 下载、verify、reset | PASS | NUCLEO-G431RB / STM32G43x/G44x |
| S5 | 四档默认开环，582 rpm，各 5 s | PASS；WCET 8,098 / 8,257 / 8,857 / 8,160，全部 0 error、0 miss、0 fault | ignored results + 阶段 1.1 报告 |
| S6 | 示波器、编码器、故障注入、长测、多板多电机 | 未执行 | Production 仍只是候选构建档 |

最终板上 Diagnostic + `s` 固件：

- `fluxrt.bin`：122,812 B，`EF66C0A1FAE31F32C183A3351F6C1DA36526A81D9DF45B0807F44DAD8E4FD9A1`
- `fluxrt.hex`：`CDBFB76B45395BAECE7DF424080181AF753C50191D13D8147C5BC35265AA15E7`
- `rtthread.elf`：`0B9EF74B0BF4EE98FE7848DDA84AEDDBD13E89DAE1B4457419E964A64E640543`

Production + `s` 候选 BIN：93,004 B，
`7FFB36F867E39058F1EDE4EC797DF5513D11A429DCB8E1159FD64CDCE0A0B28E`。

#### 实机条件

- 板卡：NUCLEO-G431RB + X-NUCLEO-IHM16M1；电机：GBM2804H-100T；空载自由旋转。
- 电源：板端约 12.28 V，用户电源上限 2 A，软件 trip 1.15 A。
- 命令：四档均为 582 rpm、默认开环、trace 关闭、5 秒；每组由脚本停机收尾。
- 结束状态：重新烧录 Diagnostic + `s`；回读 `closed_loop=0`、`steps=0`、`errors=0`、duty 0/0/0，trace 已停止。

#### 风险、回退与遗留

- 风险：Production 仍保留 Finsh 基础 Shell，并非最终最小产品镜像；后续正式通信/参数存储接口完成后还可继续裁剪。
- 纠正：首次参数化 `O3` 构建发现生成 CMake 仍链接旧 `build/rust-target`；未将错误标签纳入结果，修复链接目录后重建、重烧并重新实测三档。
- 纠正：同一 PowerShell 中构建脚本曾泄漏 PATH，导致随后 Python 缺少 pyserial；现已恢复进程环境并验证前后解释器一致。
- 回退：`5e3b85f` 是本批前 A0 基线；提交后使用 `git revert <本条记录所在提交>`。
- 未完成：闭环观察器失锁、PA5 示波器对拍、独立真值、故障注入和长测。
- 下一步：保持 Rust `s` 和 12 kHz，开始共享 Clarke/中间量与 CORDIC 事务优化，并重新执行相同 WCET 门。

### LOG-20260923-004｜A0 完整 ISR 关联 WCET 与首轮实机基线

- 时间：2026-09-23 12:59 +08:00
- 负责人：User + Codex
- 状态：部分完成；DWT/size/实机基线完成，PA5 示波器对拍和独立轴端真值待补
- 类型：代码 / 配置 / 构建 / 测试 / 烧录 / 实机 / 文档
- 目标：修正旧 WCET 区间不完整、分段最大值不关联的问题，建立 trace 关闭/10 Hz/50 Hz 的可重复板端基线。
- 开始基线：`main`，`e87079286070155a85ff85540b4d924c91060d1e`。
- 关联提交：本条记录所在提交。

#### 修改内容

| 路径/对象 | 类型 | 修改说明 |
|---|---|---|
| `foc/include/foc_realtime_timing.h`、`foc/runtime/foc_realtime_timing.c` | 新增 | 保存同一控制拍的 total/pre/control/post、trace 状态、独立段峰值及无效样本计数 |
| `foc/platform/stm32g431/foc_platform_stm32g431.c` | 修改 | DWT 区间前移到 ADC IRQ 入口并延长到 ADC flags 清理后；截止判断改用完整区间 |
| `foc/include/foc_platform.h` | 修改 | 增加只读关联时序统计接口 |
| `applications/main.c` | 修改 | `foc_status` 输出可读 WCET 和机器可读 `FTIMING`，明确独立段峰值禁止相加 |
| `board/Kconfig`、`.config`、`rtconfig.h` | 修改 | 增加默认关闭的 `FOC_ISR_TIMING_PROBE`，启用时在 PA5 输出同区间示波器脉冲 |
| `tests/host/*` | 修改 | 加入同拍恒等式、WCET 替换、独立段峰值和无效样本测试 |
| `simulation/capture_timing_baseline.py` | 新增 | 有显式电机运行许可、每组停机和最终恢复的 trace-off/10/50 自动采集脚本 |
| `docs/performance/2026-09-23-a0-correlated-wcet-baseline.md` | 新增 | 固化尺寸、哈希、实机条件、开环/闭环结果和证据边界 |
| `docs/ST_VESC_ENGINEERING_IMPROVEMENT_ROADMAP.md` | 修改 | 更新阶段 0 已完成项，并保留 GPIO/独立真值未完成状态 |

#### 关键参数变化

| 参数 | 修改前 | 修改后 | 原因 |
|---|---:|---:|---|
| ISR `total` 起点 | 电流预处理之后 | `ADC1_2_IRQHandler` 入口 | 覆盖真实中断前段开销 |
| ISR `total` 终点 | ADC flags 清理之前 | ADC1/ADC2 injected flags 清理之后 | 覆盖完整正常 ISR 路径 |
| 分段最大值 | 三个独立最大值，无关联拍 | 同拍 WCET 分段 + 独立段峰值并存 | 禁止错误相加并保留热点定位能力 |
| PA5 测量脉冲 | 无 | 可选，默认 `n` | 后续用示波器校验 DWT；不改变生产默认行为 |
| PWM/控制/失锁阈值 | 12 kHz / 12 kHz / 50 ms | 未改 | 本批只修证据链，不混入控制律或阈值调参 |
| 默认闭环 | `0` | `0` | 闭环重复稳定前不提高权限 |

#### 验证与证据

| 等级 | 命令/场景 | 结果 | 产物/记录 |
|---|---|---|---|
| S0 | 源码和旧日志口径检查 | 旧 total 区间和独立峰值问题已定位并修正 | 本记录与性能报告 |
| S1 | `.\test.ps1` | PASS；Rust 94 项、格式、Clippy、CPU/CORDIC 交叉库和 C Host 1/1 通过 | 控制台记录 |
| S2 | 默认构建及临时 `FOC_ISR_TIMING_PROBE=y` 构建 | PASS；默认 ROM 127,820 B、RAM 11,728 B；探针版 ROM 127,908 B；最终已恢复默认关闭并重建 | `cmake-build/fluxrt.*`、`rtthread.elf` |
| S3 | `test.ps1` 闭环 PMSM 仿真 | PASS；`final_rpm=534.52`，目标 524 rpm | `FOC_SIM_PASS` |
| S4 | STM32CubeProgrammer SWD 下载、verify、reset | PASS；识别 NUCLEO-G431RB / STM32G43x/G44x | 最终固件 SHA-256 如下 |
| S5 | 默认开环，582 rpm，trace off/10/50，各 5 s | PASS；完整 ISR 8,161/10,000/10,030 cycles，均 0 error、0 miss | 性能报告和本机 ignored results |
| S5 | 临时闭环，582 rpm，trace off/10/50，各 5 s | FAIL/PARTIAL；off 和 10 Hz 约 2.3 s 后 `OBSERVER_LOST`，50 Hz 跑满；三组均 0 deadline miss | 性能报告和本机 ignored results |
| S6 | 示波器 DWT 对拍、编码器真值、故障注入和长测 | 未执行 | 不得视为闭环稳定或量产证明 |

最终烧录固件：

- `fluxrt.bin`：127,820 B，`FEE8CFB8E98B0AB382A6FE2E49ECF2C7D398134C9FBE7D48B7DC7720996CBE75`
- `fluxrt.hex`：`AA5EA5A9797BC69E62643556354F89DF2790916DA1F16D604696098BEA6572D7`
- `rtthread.elf`：`2BB7068D2BCD8894602390A131B2F0969CC510095505A0A8601A33E33DBBD32F`

#### 实机条件

- 板卡：NUCLEO-G431RB + X-NUCLEO-IHM16M1；电机：GBM2804H-100T；空载自由旋转。
- 电源：板端回读约 12.26～12.31 V；用户设定最大输出 2 A；软件 trip 1.15 A。
- 命令：`capture_timing_baseline.py`，582 rpm，每种 trace 状态 5 s；闭环只在脚本期间临时打开。
- 保护结果：所有试验均 0 deadline miss；两次观察器失锁均进入故障关断。
- 结束状态：已执行 `foc_stop`、`foc_trace stop`，回读 `closed_loop=0`，功率级关闭。

#### 风险、回退与遗留

- 风险：Flash 已用 97.52%，只余 3,252 B；关联时序相对旧基线增加 768 B ROM、56 B RAM。
- 风险：闭环在相同硬件条件下重复性不足，不能因某个 trace 频率偶然通过就判断其稳定。
- 回退：`e870792` 是本批前基线；提交后可用 `git revert <本条记录所在提交>` 恢复，不使用破坏性 reset。
- 未完成：PA5 示波器与 DWT 对拍、`O3/Os/Oz` 对比、production 配置、独立速度/角度真值、闭环失锁根因。
- 下一步：先做 A0 编译优化矩阵与 Flash 裁剪；观察器问题保持安全阈值不变，另行对比失锁前 BEMF、速度方差和角度误差。

### LOG-20260923-003｜接入 GitHub 并建立首次源码基线

- 时间：2026-09-23 12:22 +08:00
- 负责人：User + Codex
- 状态：完成
- 类型：Git / GitHub / 配置 / 测试
- 目标：在不覆盖 GitHub 初始历史的前提下，将 FluxRT 当前源码、配置、模型和文档建立为可恢复的远端基线。
- 开始基线：远端 `main`，`2cc696d6c36ac4796342a0f877ea9a65f004a1a7`（`Initial commit`）。
- 关联提交：本条记录所在提交；远端 `origin/main`。

#### 修改内容

| 路径/对象 | 类型 | 修改说明 |
|---|---|---|
| `.git/`、`origin`、`main` | 初始化/配置 | 连接 `https://github.com/ice-infinite/FluxRT.git`，以远端初始提交为父提交，不改写已有历史 |
| `.gitignore` | 修改 | 排除 C/SCons/CMake/Cargo 构建物、RT-Thread packages、Simulink 缓存和仿真结果 |
| `.gitattributes` | 新增 | 规范文本换行并明确 MATLAB 模型、固件和图像等二进制类型 |
| `README.md` | 修改 | 用完整项目说明替换远端两行占位说明；远端原说明仍保留在父提交历史中 |
| 当前源码、配置、模型和文档 | 首次纳管 | 建立 FluxRT 的首个完整、可追溯 GitHub 源码基线 |

#### 关键参数变化

| 参数 | 修改前 | 修改后 | 原因 |
|---|---:|---:|---|
| Git 仓库 | 未初始化 | `main` + `origin` | 建立版本恢复和协作入口 |
| 待纳管内容 | 无源码基线 | 131 个新增文件、1 个修改文件，约 1.06 MB | 排除约 900 MB 本机构建/仿真生成物 |
| 控制、观察器和电机参数 | 未改 | 未改 | 本次只建立版本管理基线 |

#### 验证与证据

| 等级 | 命令/场景 | 结果 | 产物/记录 |
|---|---|---|---|
| S0 | `git ls-remote`、浅克隆远端并检查历史 | PASS；远端仅有 `2cc696d...` 和 `README.md`，已作为父提交保留 | `origin/main` |
| S0 | Git 待提交文件、大文件、忽略规则和常见凭据模式检查 | PASS；生成物均被忽略，未发现私钥、PAT、AWS key 或显式密码/token | `.gitignore`、`.gitattributes` |
| S1 | `.\test.ps1` | PASS；Rust 94 个单元测试、格式、Clippy、CPU/CORDIC 交叉库和 C Host 1/1 通过 | 控制台记录 |
| S3 | `test.ps1` 内闭环 PMSM 仿真 | PASS；`final_rpm=534.52`，目标 524 rpm | 控制台 `FOC_SIM_PASS` |
| S2、S4～S6 | 本轮目标构建、烧录和实机 | 未重复；沿用 `LOG-20260923-002` 的同一源码构建证据，本轮未烧录或运行电机 | 不得视为新增实机证明 |

#### 实机条件

不涉及。本次没有烧录、使能功率级或运行电机。

#### 风险、回退与遗留

- 风险：当前 GitHub 仓库是源码基线，不包含 RT-Thread `packages/`、构建缓存、固件和 MATLAB/Simulink 运行结果；复现时必须按 README 获取依赖并重新构建。
- 回退：查看父提交用 `git show 2cc696d...`；恢复当前基线可从 `origin/main` 新建分支，后续错误优先使用 `git revert`。
- 未完成：旧 `STM32G431_FOC` 空目录仍等待 MATLAB 释放句柄；本轮未创建发布 tag。
- 下一步：以当前提交作为整改基线，按架构分配文档开始 A0，并为每个独立改动继续写操作记录和提交。

### LOG-20260923-002｜项目统一更名为 FluxRT

- 时间：2026-09-23 12:09 +08:00
- 负责人：User + Codex
- 状态：部分完成；有效工程已更名并验证，旧空目录等待 MATLAB 释放句柄后清理
- 类型：项目命名 / 目录移动 / 构建配置 / 文档 / 测试
- 目标：将芯片绑定的项目名改为简短、可移植的 `FluxRT`，同时保留 STM32G431 作为当前参考硬件标识。
- 开始基线：`NO-GIT`；原目录 `E:\File\RT-Thread\projects\STM32G431_FOC`。
- 关联提交：`NO-GIT`；本次没有初始化仓库或创建提交。

#### 修改内容

| 路径/对象 | 类型 | 修改说明 |
|---|---|---|
| 项目主目录 | 移动 | 有效工程从 `projects/STM32G431_FOC` 移至 `projects/FluxRT` |
| `README.md`、`AGENTS.md`、主要使用文档 | 修改 | 项目品牌、命令路径和目录示例统一为 FluxRT |
| `Kconfig`、`.config`、`rtconfig.h` | 修改/生成 | 菜单改为 FluxRT，板级选择符改为 `BOARD_FLUXRT_STM32G431_REFERENCE` |
| `rtconfig.py`、`SConstruct`、`build.ps1` | 修改 | SCons 和 CMake 后处理产物改为 `fluxrt.bin/.hex/.map` |
| `.vscode/launch.json`、`.vscode/tasks.json` | 修改 | 调试配置和任务显示名改为 FluxRT |
| `tests/host/CMakeLists.txt` | 修改 | Host 测试工程名改为 `fluxrt_host_tests` |
| `test.ps1` | 修改 | Host CMake 配置增加 `--fresh`，可从目录更名后的旧 cache 自动恢复 |
| `board/board.c`、`board/board.h` | 修改 | 用户可见错误和 include guard 使用 FluxRT；STM32G431 硬件约束保留 |
| `simulation/`、`simulink/`、`rust/README.md` | 修改 | 显示名称和当前绝对路径改为 FluxRT |
| `CMakeLists.txt` | 重新生成 | 清除旧绝对路径，生成 `fluxrt` 后处理产物规则 |

硬件实现文件、`FOC_TARGET_STM32G431`、`foc/platform/stm32g431` 和历史实机记录中的旧
固件文件名刻意保留，因为它们分别表示当前参考硬件和当时实际使用的证据，不属于项目
品牌残留。

#### 关键参数变化

| 参数 | 修改前 | 修改后 | 原因 |
|---|---:|---:|---|
| 项目目录 | `STM32G431_FOC` | `FluxRT` | 使用简短且不绑定 MCU 的项目名 |
| SCons ELF | `stm32g431_foc.elf` | `fluxrt.elf` | 统一项目产物名称 |
| CMake BIN/HEX/MAP | `stm32g431_foc.*` | `fluxrt.*` | 统一项目产物名称 |
| PWM/控制/观察器参数 | 未改 | 未改 | 本次只改项目身份与构建命名 |

#### 验证与证据

| 等级 | 命令/场景 | 结果 | 产物/记录 |
|---|---|---|---|
| S0 | 扫描源码、配置和当前使用文档中的旧项目名/旧绝对路径 | PASS；只保留硬件标识和历史证据 | 本记录 |
| S1 | `.\test.ps1` | PASS；Rust 94 个单元测试、Clippy、CPU/CORDIC 交叉库和 C Host 1/1 通过 | 控制台记录 |
| S2 | `.\build.ps1 -Clean` 后 `.\build.ps1 -Regenerate` | PASS；ROM 127,052/131,072 B，RAM 11,672/32,768 B | `cmake-build/rtthread.elf`、`fluxrt.bin/.hex/.map` |
| S3 | `test.ps1` 内闭环 PMSM 仿真 | PASS；`final_rpm=534.52`，目标 524 rpm | 控制台 `FOC_SIM_PASS` |
| S4～S6 | 烧录、实机和故障注入 | 未执行；项目改名不需要驱动功率级 | 不得视为新固件实机证明 |

新构建 SHA-256：

- `fluxrt.bin`：`0BBC53B6B4AB1B828EF6FE613D17E62EAF9623ABC6D0D82FD6FBF97F0F4ACA45`
- `fluxrt.hex`：`9B04EEE25554D46F0B03CCB4133ECA3794A358D9D2B311BFD6CD9C469F05EE93`
- `rtthread.elf`：`02DB9CCCC0F509F76F34F1BA7B7F5C8F23CF7FAD7300C912D5B557DC0D6209E4`

#### 实机条件

不涉及。本次没有烧录、使能功率级或运行电机。

#### 风险、回退与遗留

- 风险：MATLAB 正在占用原 `simulink` 路径，Windows 暂时保留了只含 0 个文件、8 个空目录的旧目录树。
- 回退：当前无 Git；根目录保留改名前历史 ELF/BIN/HEX/map，新构建位于 `cmake-build`。
- 未完成：MATLAB 释放目录句柄后，仍需删除旧空目录；GitHub 仓库尚未创建或连接。
- 下一步：在 MATLAB 中切换到 `E:\File\RT-Thread\projects\FluxRT\simulink` 或关闭 MATLAB，清理旧空目录；随后初始化 Git 并连接 GitHub `fluxrt`。

### LOG-20260923-001｜建立整改路线、架构分配和持续追溯规则

- 时间：2026-09-23 11:45 +08:00
- 负责人：User + Codex
- 状态：完成
- 类型：文档 / 架构 / 项目治理
- 目标：把 ST/VESC 对比结论转换为可执行整改路线，明确代码落点，并建立每次操作可追溯规则。
- 开始基线：`NO-GIT`；项目目录不在 Git 工作树中。
- 关联提交：`NO-GIT`；本次没有初始化仓库或创建提交。

#### 修改内容

| 路径/对象 | 类型 | 修改说明 |
|---|---|---|
| `docs/ST_VESC_ENGINEERING_IMPROVEMENT_ROADMAP.md` | 新增/修改 | 建立 A0～A9 整改方向、优先级和验收门，并链接架构分配文档 |
| `docs/REMEDIATION_ARCHITECTURE_ALLOCATION.md` | 新增 | 分配 C/Rust/平台/实时层/仿真职责、目录和迁移顺序 |
| `docs/PROJECT_OPERATION_LOG.md` | 新增 | 建立当前交接、记录格式、证据等级、Git 和实机追溯规则 |
| `AGENTS.md` | 新增 | 强制后续实质性操作在完成前更新本记录，并遵守架构和证据边界 |
| `docs/ARCHITECTURE.md` | 修改 | 增加目标整改架构入口 |
| `README.md` | 修改 | 增加整改路线、架构分配和项目操作记录入口 |

#### 关键参数变化

无。本次没有修改 PWM、控制器、观察器、保护、时钟、电机参数或实机默认配置。

#### 验证与证据

| 等级 | 命令/场景 | 结果 | 产物/记录 |
|---|---|---|---|
| S0 | 对照当前 `main.c`、平台 API、C ABI、Rust crate 和 SConscript 分配职责 | PASS | `REMEDIATION_ARCHITECTURE_ALLOCATION.md` |
| S0 | 检查新增及关联文档的本地 Markdown 链接 | PASS，缺失链接 0 个 | README、ARCHITECTURE、整改路线、架构分配和本记录 |
| S1/S2 | Host 测试、目标构建 | 未执行；本次仅修改文档和项目规则 | 不得视为固件回归证明 |
| S3～S6 | 仿真、烧录、实机 | 未执行 | 不涉及 |

#### 实机条件

不涉及。本次没有连接、烧录或启动功率级。

#### 风险、回退与遗留

- 风险：目标目录中的“规划新增”文件尚未建立，当前源码仍是较大的聚合文件。
- 回退：当前无 Git 提交；只能依据现有文件副本手工恢复，因此应尽快决定 Git 仓库归属。
- 未完成：尚未执行整改 A0 基线固化和 A1 结构拆分。
- 下一步：经用户授权后先建立/接入 Git 版本管理，再记录基线 commit；随后执行 A0。
