builtin_interfaces/Time stamp

# 当前底层控制器类型。
uint8 CONTROLLER_UNSPECIFIED=0
uint8 CONTROLLER_TRAJECTORY=1
uint8 CONTROLLER_VELOCITY=2
uint8 CONTROLLER_EFFORT=3
uint8 controller_type

# 当前激活的参考类型。
uint8 REFERENCE_UNSPECIFIED=0
uint8 REFERENCE_JOINT_SPACE=1
uint8 REFERENCE_TASK_SPACE=2
uint8 active_reference_type

# 低层控制器是否已经收到可用参考并进入闭环运行。
bool has_reference

# 当前目标是否已达到阈值范围。
bool goal_reached

# 对于 joint-space 任务，这里记录最大关节位置误差（绝对值）。
float64 max_joint_position_error

# 对于 task-space 任务，这里记录末端平移误差范数（单位米）。
float64 task_space_position_error_norm

# 对于 task-space 任务，这里记录末端姿态误差范数（单位弧度）。
float64 task_space_orientation_error_norm
