# 执行端 adapter 回传给 gateway 的数据面状态。
uint8 STANDBY=0
uint8 RUNNING=1
uint8 SAFE_STOP=2
uint8 FAULT=3

# 供 adapter 与执行平台共用的稳定故障编码。
uint16 FAULT_NONE=0
uint16 FAULT_ROBOT_CONTROLLER_UNAVAILABLE=1
uint16 FAULT_ROBOT_CONTROLLER_STATE_TIMEOUT=2
uint16 FAULT_ROBOT_ODOMETRY_TIMEOUT=3
uint16 FAULT_ROBOT_TF_UNAVAILABLE=4
uint16 FAULT_ROBOT_TF_STALE=5
uint16 FAULT_ROBOT_HARDWARE_INACTIVE=6
uint16 FAULT_ROBOT_HARDWARE_STATE_TIMEOUT=7
uint16 FAULT_ADAPTER_CANONICAL_TIMEOUT=10
uint16 FAULT_ADAPTER_CANONICAL_SOURCE_AMBIGUOUS=11
uint16 FAULT_ADAPTER_CANONICAL_INVALID=12
uint16 FAULT_ADAS_CAN_IO=100
uint16 FAULT_ADAS_CAN_STATE_TIMEOUT=101
# 非零 8-bit demo vehicle raw fault 加到该基值上，映射范围为 257..511。
uint16 FAULT_ADAS_VEHICLE_REPORTED_BASE=256

# 执行端 adapter 组装本状态快照时的 ROS time。
builtin_interfaces/Time observed_at
# 当前 adapter/平台状态，取值使用上述状态常量。
uint8 state
uint16 fault_code
# 执行端 adapter 观测到的平台速度，两项都必须为 finite。
float64 linear_velocity_mps
float64 angular_velocity_radps
# 在 Robot Profile 中固定为 0；在 ADAS Profile 中携带 CAN state counter。
uint8 rolling_counter
