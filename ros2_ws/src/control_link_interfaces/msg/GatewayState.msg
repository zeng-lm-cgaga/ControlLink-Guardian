# 网关数据面状态，与 ROS2 Lifecycle 状态相互独立。
uint8 STANDBY=0
uint8 ACTIVE=1
uint8 DEGRADED=2
uint8 SAFE_STOP=3
uint8 RECOVERING=4
uint8 ERROR=5

# 最近一次状态或来源转换的稳定结构化原因编码。
uint16 REASON_NONE=0
uint16 REASON_FIRST_VALID_COMMAND=1
uint16 REASON_SOURCE_SWITCH=2
uint16 REASON_SOURCE_FALLBACK=3
uint16 REASON_NO_QUALIFIED_SOURCE=4
uint16 REASON_CRITICAL_ENDPOINT_UNHEALTHY=5
uint16 REASON_CRITICAL_QOS_MISMATCH=6
uint16 REASON_VEHICLE_STATE_TIMEOUT=7
uint16 REASON_VEHICLE_SAFE_STOP=8
uint16 REASON_VEHICLE_FAULT=9
uint16 REASON_VEHICLE_STATE_INVALID=10
uint16 REASON_CLOCK_INVALID=11
uint16 REASON_OUTPUT_TICK_OVERRUN=12
uint16 REASON_RECOVERY_COMPLETE=13
uint16 REASON_INTERNAL_INVARIANT=255

# 与本状态快照对应的 ROS time 观测时间。
builtin_interfaces/Time observed_at
# 当前数据面状态，取值使用上述状态常量。
uint8 state
uint16 reason_code
# 当前选中来源及其最近一次已接受的上游 sequence。
string active_source_id
uint64 active_source_sequence
# 当前选中命令在 ROS clock domain 内的年龄，单位为 ms。
float64 active_command_age_ms
# 处于 RECOVERING 状态时的连续有效样本数；其他状态必须为 0。
uint16 recovery_valid_count
# 数据面状态或 active source 发生变化时递增。
uint64 transition_sequence
