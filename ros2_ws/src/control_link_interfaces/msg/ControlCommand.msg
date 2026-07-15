# 输入边界与 canonical output 边界共用的命令模式。
uint8 MODE_NORMAL=0
uint8 MODE_HOLD=1

# 命令源在统一 ROS clock domain 内生成的时间戳。
builtin_interfaces/Time source_stamp
# 由 Contract 配置的来源标识，例如 nav2、teleop 或 planning。
string source_id
# 在同一 ingress publisher GID generation 内严格递增。
# 固定频率 canonical output 可以重复最近一次已接受的 sequence。
uint64 source_sequence
# 取值必须是 MODE_NORMAL 或 MODE_HOLD。
uint8 mode
# 期望速度必须为 finite 且满足 Contract 边界；HOLD 要求两项均为 0。
float64 linear_velocity_mps
float64 angular_velocity_radps
