builtin_interfaces/Time stamp

# reference_mode 描述这条 joint-space reference 的语义。
# 在当前 velocity 底层中，MODE_TRAJECTORY/MODE_VELOCITY 都会被解释成 q_des 跟踪。
uint8 MODE_UNSPECIFIED=0
uint8 MODE_TRAJECTORY=1
uint8 MODE_VELOCITY=2
uint8 MODE_EFFORT=3

uint8 reference_mode

# joint_names 明确每个数组元素对应哪个关节，避免 joint_states 或规划结果顺序不一致。
string[] left_joint_names
string[] right_joint_names

# positions 是低层关节速度闭环的主要目标 q_des。
float64[] left_positions
float64[] right_positions

# velocities/efforts 目前作为接口预留，后续可接速度前馈或 effort 控制。
float64[] left_velocities
float64[] right_velocities
float64[] left_efforts
float64[] right_efforts
