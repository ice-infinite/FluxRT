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
| 最近记录 | `LOG-20260923-003` |
| 工程阶段 | FluxRT 已接入 GitHub `main`；尚未开始 A0/A1 源码结构整改 |
| 当前默认安全状态 | `closed_loop_enable=0`，新增补偿/相电压/HFI 等功能均未默认启用 |
| 当前固件基线 | `cmake-build/fluxrt.bin` 127,052 B；12 kHz PWM/控制环；闭环历史最大约 10,083/12,500 cycles |
| Git 状态 | `main` 连接 `https://github.com/ice-infinite/FluxRT.git`；本记录随首次源码提交推送 |
| 当前可恢复点 | GitHub `main`、本条记录所在提交、新构建的 `fluxrt.*` 和根目录改名前历史固件 |
| 下一建议动作 | 让 MATLAB 释放旧空目录；以当前 GitHub 基线创建后续整改分支并执行 A0 |
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
