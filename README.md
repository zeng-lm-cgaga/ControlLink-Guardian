# ControlLink Guardian

ControlLink Guardian 是面向低速无人车和仓储 AMR 的 C++/ROS2 控制通信网关，位于 Nav2、遥控或规划控制模块与机器人、车辆执行层之间。

它将多个控制来源收敛为唯一的 canonical control output，并集中处理通信契约、DDS QoS、来源绑定、命令合法性、来源仲裁、命令时效、运行状态和诊断信息，避免这些规则分散在上游业务节点和不同执行平台中。

```text
Nav2 / teleop / planning-control
  -> ingress adapters
  -> Contract + QoS boundary
  -> command validation
  -> source arbitration
  -> health state machine
  -> canonical output
  -> robot / vehicle adapters
```

项目采用公共网关核心与平台适配器分离的结构：Robot Profile 对接 Nav2、ros2_control 和 Gazebo；ADAS Profile 对接 SocketCAN 和 vcan 演示协议。两种 Profile 复用同一套控制契约、仲裁和状态治理逻辑。

License: Apache-2.0
