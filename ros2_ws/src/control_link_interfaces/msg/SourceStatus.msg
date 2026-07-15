# 供 gateway、diagnostics、测试和 rosbag 共用的稳定结构化拒绝原因。
uint16 REJECT_NONE=0
uint16 REJECT_SOURCE_ID_MISMATCH=1
uint16 REJECT_SOURCE_ENDPOINT_AMBIGUOUS=2
uint16 REJECT_PUBLISHER_GENERATION_UNSTABLE=3
uint16 REJECT_UNKNOWN_MODE=4
uint16 REJECT_NON_FINITE=5
uint16 REJECT_OUT_OF_RANGE=6
uint16 REJECT_ZERO_STAMP=7
uint16 REJECT_FUTURE_STAMP=8
uint16 REJECT_STALE=9
uint16 REJECT_SEQUENCE_NOT_INCREASING=10
uint16 REJECT_CLOCK_INVALID=11
uint16 REJECT_HOLD_NONZERO=12

# 每条消息只描述一个配置来源；consumer 按 source_id 聚合。
# 与本来源状态快照对应的 ROS time 观测时间。
builtin_interfaces/Time observed_at
string source_id
# 字段 priority 与 enabled 均来自已加载的 source policy。
uint8 priority
bool enabled
# 来源 lease 是否有效，与内容合法性和 freshness 分开表达。
bool lease_valid
# 仅当稳定 generation 仍存在 latest-valid snapshot 时为 true。
bool command_valid
# 最近记录的结构化拒绝原因。
uint16 last_reject_reason
# 最近一次已接受命令的 sequence；拒绝样本不得覆盖该值。
uint64 last_source_sequence
# 最近有效命令在 ROS clock domain 内的年龄，单位为 ms。
float64 command_age_ms
# 当前来源累计接受和拒绝的命令数量。
uint64 accepted_count
uint64 rejected_count
