builtin_interfaces/Time stamp

# reference_mode 描述任务空间参考应由底层哪类控制器解释。
# 当前低层已实现 MODE_VELOCITY 的任务空间速度控制链。
uint8 MODE_UNSPECIFIED=0
uint8 MODE_VELOCITY=1
uint8 MODE_EFFORT=2
uint8 MODE_HYBRID_FORCE_POSITION=3

uint8 reference_mode

# use_object_target=true 时，下层或协同层可优先把 object_target_pose 解释成物体中心目标。
bool use_object_target

# enable_force_control=true 时，表示该参考还携带接触力/内力控制语义。
bool enable_force_control
# 当前时刻的任务空间参考点，统一使用位置 + 四元数姿态表示。
# 这样可以与 TaskSpaceTrajectoryPoint 的表示完全一致，避免中途再退回到 RPY。
TaskSpacePose left_target_pose
TaskSpacePose right_target_pose
TaskSpacePose object_target_pose

# 目标内力/夹持力，后续用于接触后的内力闭环。
float64 target_internal_force
