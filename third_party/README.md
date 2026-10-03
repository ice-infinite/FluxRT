# FluxRT 第三方上游区

本目录只承载协议所有者的官方上游依赖、生成产物来源记录和许可证。FluxRT 自己的硬件
port、消息映射和安全接入放在 `protocol/ports/`、`protocol/adapters/` 及目标平台目录，不能
混入上游源码。

## 强制规则

1. 只从协议所有者的官方仓库或官方生成器获取；RT-Thread 包索引中的非官方镜像不能因为
   “能下载”就成为来源。
2. 集成时优先选择最新稳定发布；没有稳定发布的滚动仓库固定到经过评审的完整 commit。
3. 上游源码使用 Git submodule 或等价的独立依赖目录，目录内保持零本地修改。
4. 生成代码必须记录生成器版本、输入 schema/DSDL、命令、commit 和许可证，不手改生成物。
5. 每个上游单独编译为库，只向 FluxRT adapter 暴露最小接口。FOC、MotorService 和
   CommandArbiter 不得直接 include 上游头文件。
6. 没有官方可复用实现时保持 feature 关闭并延期，禁止为了赶进度自写 wire codec、CRC、
   分片重组或协议状态机。

## 更新流程

1. 在干净分支刷新官方仓库的 tag/commit 和安全公告；
2. 审查许可证、API/ABI、生成器、MCU 资源、动态内存和实时性变化；
3. 只更新 submodule/ref 或重新生成 `generated/`，不在上游目录打散修改；
4. 先跑上游自身测试，再跑 FluxRT golden vectors、adapter、fuzz、Host 和目标构建；
5. 比较 ROM/RAM、栈、队列、总线负载和 WCET，保持对应 feature 默认关闭；
6. 更新 `upstreams.catalog.json`、专项报告和 `docs/工程操作日志.md`；
7. 协议兼容或安全门失败时回退依赖引用，不回滚无关 FluxRT 代码。

`upstreams.catalog.json` 是上游发现与候选清单，不代表其中组件已经下载、链接、许可批准或
通过实机验证。真正集成某组件时还要在 catalog 中增加锁定 ref、内容哈希和验收记录。
