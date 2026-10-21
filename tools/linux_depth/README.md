# Linux 控制链时序与定向验证

这里是业务组件的开发工具，不是第二个控制平台。正常产品仍使用根 README 的 CMake/ament/colcon 构建和 bringup；当前阶段、实际验收和限制只在 `docs/00_mvp_evidence_plan.md` 登记。这里不启动常驻监控，不依赖或修改 EdgeRuntime/SelfIPCs，不替代原性能/故障矩阵。

## 运行时改变

- `SocketCanTransport` 继续使用非阻塞 CAN、独立 worker 和 `poll`，增加 `CLOCK_MONOTONIC` 绝对周期 `timerfd`。过期次数在读计数中合并，handler 期间错过的周期跳过，不补发旧周期。
- RX 每轮最多 64 帧、2ms，另外受 TX deadline 和 stop flag 限制；这是 I/O 公平性预算，不是业务时效规则。`receive_budget_yields` 包含 deadline 提前让出，可大于收到的帧数，不代表丢帧。它统计进入预算/deadline分支的次数；同一未读就绪数据可反复触发，不是不同帧或完整poll迭代计数，不能单独据此判定CPU忙循环。
- EAGAIN/EWOULDBLOCK/ENOBUFS 不缓存待重试的运动帧；下一次尝试重新调用 adapter 原来的 mailbox/watchdog 逻辑。CAN write 和 wake 的 EINTR 重试都有上限，退出仍依次请求、唤醒、join、关闭所有 fd。handler 自身必须有限，不承诺硬实时退出。
- Contract、优先级、来源绑定、100ms watchdog、恢复门槛和协议不在这里复制或放宽。fatal CAN 仍按根 README 的人工重建契约处理。
- transport 类和 stats 布局有变化；依赖它的 simulator/消费者必须使用同一源码头文件重编。不能仅靠替换 `.so` 混用旧 ABI。

## 有界应用计时

Gateway 和 SocketCAN adapter 提供只读参数 `runtime_timing_path`。默认空字符串不开线程、不建缓冲、不写文件；开启时必须是绝对且不存在的路径，不能在线改路径或覆盖旧证据。

```bash
ros2 run control_link_gateway control_link_gateway_node --ros-args \
  -r __node:=gateway -r __ns:=/control_link \
  -p runtime_timing_path:=/absolute/new/gateway.csv
```

该片段只说明新增参数；Profile/config_root、同 overlay 和正常生命周期装配仍按根 README 提供，不代替完整启动命令。

`runtime_timing` 是 ROS-free OBJECT 库，私有并入业务库。生产者只 `try_lock` 并复制定长 Event；容量 4096，后台每 20ms 批量写最多 256 条。争用、满、非法身份和关闭后调用分别计数，不等待磁盘，也不悄悄补齐缺事件。owner 必须先停完所有生产者再 drain/join。

CSV 的 `# drain` 只证明写入队列排空阶段；数据 `fsync/close` 后才写伴随状态。只有 `.status.partial` 自身写入、fsync、close 都成功才发布独占 `.status.json`。统计分析同时要求最终状态、无 I/O 错误、queued=written=CSV 行数；非零丢记录意味着仅能报告已观察样本，不能声称完整内部 tick。状态文件不承诺掉电后的目录项持久化；close 返回之后的违规调用不会回写已冻结状态。

### 事件边界

`begin_ns/end_ns` 是共享 Linux 实例的 `CLOCK_MONOTONIC` 纳秒。PID/TID 是实际调用进程/线程；源身份包含 source、publisher GID、原 sequence 和原 SYSTEM/ROS stamp。GID 只作 generation 去歧义，不是认证身份。

| phase | 端点/含义 |
|---|---|
| source_receive / source_result | Gateway callback 进入；后者从相同入口到原 DecisionEngine 验证完成，包含锁等待 |
| source_select / canonical_publish | 决策选择；publish 调用进入/返回；同一 tick frame_id 桥接上游 GID 与 canonical 自己的 publisher GID |
| source_snapshot | 原引擎给出的 command-valid/lease-valid 位与命令年龄，不重新判断时效 |
| mailbox_write | adapter callback 进入到校验/单槽覆盖；result=1 接受，-1 拒绝；value=1 表示覆盖尚未消费的不同身份/模式 |
| mailbox_consume | begin 是 mailbox 写入时刻，end 是本次帧生成；不是 worker 连续执行区间，不能拿整个区间归因某个 TID 的 CPU |
| can_write | 本次真实 write 的进入/返回，result 为字节数或负 errno；身份、控制 counter、frame_id 均在生成时冻结，不在发送后读取新 mailbox |
| can_receive | read 返回后的 MONOTONIC，result=DLC，value=原始 CAN ID；transport 不猜信号偏移 |
| feedback_validate | 同一 receive frame_id 的 codec/counter 结果；接受时 value=解码后的 state counter，拒绝分别保留 codec/counter reason |
| feedback_publish / gateway_feedback | 原 observed_at、状态 counter 和 publisher GID 桥接 adapter 发布与 Gateway callback/验证；发布事件另带最近接受的 receive frame_id |
| resources_closed | result=错过的 TX 周期数，value=RX 让出次数；stop/exit/join/closed 时间按各自实际边界记录 |

CAN 的 4-bit counter **不是业务 sequence**。反馈桥只证明接收/校验/状态回传，不是该业务命令的执行 ACK，更不是物理动作。源→vcan 观察接收使用既有量化目标夹具，整会话唯一 (speed_raw,yaw_raw) 并验证生产 round/scale；不得按最近时间或 counter 回绕配对。

### timer 时钟不能混算

CSV v2 的 `scheduled_ns` 必须和 `schedule_clock` 一起解释：

- `schedule_clock=1`：MONOTONIC，迟到为 `begin_ns - scheduled_ns`（CAN timerfd）。
- `schedule_clock=2`：RCL_STEADY_TIME，迟到为 `schedule_now_ns - scheduled_ns`（Gateway）；value 是入口跨 API 采样 span。

当前锁定 Humble `rcutils_steady_time_now` 实际调用 `CLOCK_MONOTONIC_RAW`，不能从名称 `steady` 推断它与 MONOTONIC 同域。RCL 已跳过的旧周期不能仅从 `get_next_call_time - period` 恢复；该指标是最近当前周期的入口迟到，不冒充所有原始 deadline 的排队分布。跨进程/BPF 关联只使用 MONOTONIC 的 begin/end，SYSTEM stamp 只保留业务身份/时效。订阅回调间隔叫接收间隔，包含 DDS 和 observer 开销。

## 构建和局部检查

```bash
export MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1
cmake -S runtime_timing -B /absolute/new/timing -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build /absolute/new/timing --parallel 1
ctest --test-dir /absolute/new/timing --output-on-failure
cmake -S tools/linux_depth -B /absolute/new/focused -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build /absolute/new/focused --parallel 1
unshare -Urn bash -c 'ip link set lo up; ip link add vcan0 type vcan; ip link set vcan0 up; ctest --test-dir /absolute/new/focused --output-on-failure'
```

需要 Linux CAN/vcan、GTest、Python3/PyYAML。wrapping write 的 ENOBUFS/EINTR 注入仅存在于 focused 测试 ELF，不是生产 feature flag。原 transport 源码可通过 `CONTROL_LINK_TRANSPORT_SOURCE_ROOT` 指定隔离快照，以同一用例作前后对照。

`build_runtime.bash OUT` / `activate.bash OUT` 仅服务本仓库保留的、已核对的 native contract/interfaces 装配，历史目录不存在时必须先明确依赖缺口，不能伪称通用安装器或从 coverage/sanitizer prefix 混装。它窄编 Gateway/adapters 和使用新 transport ABI 的原 simulator，不碰旧安装。`run_scene.py` 有限等待 READY、10s warm-up、固定 20s 窗；每场拥有全部 PID，结束后 wait/flush，再 `analyze.py --freeze` 固定输入。

`summarize.py BASE NEW_OUTPUT` 读取冻结输入，输出阶段统计、资源、未匹配命令和恢复时间线；同一输入重放可逐字节比较。分析必须保留右截止未匹配和 recovery 期间 latest-value 覆盖，不能把它们补成成功或直接判丢命令缺陷。缺少 100 样本不报 p99。`run_scenarios.bash` 为固定批次入口，不是后台监控。

`analyze.py` 的 mailbox/worker 分段以 consume.begin 的内部 mailbox 更新标记为起点，以 consume.end（取值/watchdog/帧生成完成）为终点；包含锁和编码，不叫纯排队等待。必须匹配写回调区间、完整 source/GID/sequence/stamp、该 PID 的 frame_id、冻结 counter 和成功 write；canonical mode 有歧义则不猜。首次已观察 ACTIVE 身份、重复使用 latest-value、HOLD、覆盖标记和未关联记录分列。首次判断含预热历史，分母为 worker ready 落入固定窗的已观察记录，不是源窗 cohort；非零日志丢失不支持完整人口结论。

## BPF C 与 loader

`scheduler.bpf.c` 用实际内核 BTF/CO-RE 读取 task，并使用 raw sched_wakeup/sched_switch/sched_process_exit 与 raw_syscalls sys_enter/sys_exit tracepoint。loader 在挂载前写 TGID、可选 TID 和 leader `/proc/starttime` 区间；线程状态带 task start_boottime，退出删除，TID 复用重置。目标退出/换 PID 后结束，重启需用新 PID/starttime 新开采集；不承诺 exec 同 PID 热替换的连续身份。

thread map 容量 4096，ring buffer 8MiB，统计 reserve/map 失败、cold switch、unmatched syscall、复用和已输出事件。PERCPU 计数原子累加；调度关联仍须在实际内核验收，编译不证明 verifier/attach 或跨 CPU 事件关联通过。`prev_state==0`/preempt 对应可运行切出，其余阻塞状态不自动归因为 mutex、磁盘或 CAN。syscall 保存实际编号，业务序号仅来自同 PID/TID 时间区间相交。

```bash
bash tools/linux_depth/prepare_libbpf.bash /absolute/new/toolchain
bash tools/linux_depth/build_bpf.bash /absolute/new/build-bpf /absolute/new/toolchain/prefix
python3 tools/linux_depth/bpf_preflight.py --loader /absolute/new/build-bpf/trace_loader --output /absolute/new/preflight.json
```

构建脚本复用独立解包的 clang14，但明确要求新的私有 libbpf 1.4.0 prefix，不再使用旧0.5.0。prepare 从官方v1.4.0 tag下载、核对固定archive SHA、串行j1并仅安装到私有prefix；libbpf静态链接，系统工具链不变。BPF obj仍带BTF/CO-RE，不篡改内核BTF。preflight直接执行指定loader的 `--check-btf`，记录其真实libbpf版本、解析结果和ELF SHA，bpftool能解析不再代替loader兼容证明。独立root复审的旧0.5.0加载失败实际是BTF兼容错误，不能继续仅归EPERM；当前固定库解析BTF成功也不证明verifier/attach或采集已通过。

未来由已获授权的宿主管理员在初始 PID namespace、非受限终端使用：

```bash
bash tools/linux_depth/run_privileged_trace.bash /absolute/bpf-build-parent /absolute/new/trace-evidence /absolute/verified/controlled_workload
```

入口自身不 sudo、不索密码、不 mount 或更改 sysctl；要求管理员已准备可读 tracefs、BPF/perf_event 权限及可执行环境。只启动短 controlled_workload 和 8s loader，外层总期限 20s、额外 3s 强制收束，wait/清理本次子进程，无 pin/自动重连。超时/强制终止是 incomplete，不是 graceful/通过。权限失败保留日志，不能靠关闭 suppressions、过滤慢样本或丢失 footer 生成 PASS。

`analyze_bpf.py` 要求完整footer和loader最终I/O/退出状态。accounting_complete只表示计数闭合；空采集的population_complete为false，绝不当smoke成功。受控入口强制 `--require-controlled-workload --target-identity`：核对独立/proc TGID/leader starttime/各TID实例、五个挂点、非零wake/switch/syscall、100轮CPU/sleep/锁/holder应用记录及同TID内核区间交叠，含同序号跨线程holder/lock交叠。当前ABI是x86_64，futex202、clock_nanosleep230；不用于别的架构。缺失/丢弃/身份错误/无交叠均写INCOMPLETE并返回非零，进程存活或exit0不足以通过。

调度等待与syscall区间可重叠，不能相加冒充独占耗时；即使受控夹具通过也不自动判真实业务阻塞根因，更不等于Gateway/CAN业务序号关联或BPF开销已完成。没有特权入口时仅交付有界命令，不修改sudoers/sysctl/挂载，不传播认证信息；旧失败证据保留。

## 边界

所有结果仅 VM/vcan 软件链；CAN write 返回不是物理发送完成、车辆 ACK 或硬实时保证。新批次不能替代旧 50Hz/36.66ms/128 项证据。系统 glibc 的旧退出缺陷和 N2 未确定的中断责任与本工具分开，不宣称已经修复。没有新 CI/CD、MQTT、Bazel、Edge 依赖升级或长期监控。
