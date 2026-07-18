builtin_interfaces/Time stamp

# command_mode 描述“底层最终采用哪类控制接口”。
# 例如整套系统固定使用速度控制器时，自由空间和约束空间都可以保持 MODE_VELOCITY。
uint8 MODE_UNSPECIFIED=0
uint8 MODE_TRAJECTORY=1
uint8 MODE_VELOCITY=2
uint8 MODE_EFFORT=3
uint8 MODE_HYBRID_FORCE_POSITION=4

# motion_space 描述“当前任务属于哪类运动空间”。
# SPACE_FREE 通常先走 MoveIt 关节轨迹；SPACE_CONSTRAINED 通常进入任务空间/力控制链。
uint8 SPACE_UNSPECIFIED=0
uint8 SPACE_FREE=1
uint8 SPACE_CONSTRAINED=2

uint8 command_mode
uint8 motion_space

# use_object_target=true 时，控制语义从左右末端各自目标扩展为物体/虚拟物体目标。
bool use_object_target

# enable_force_control=true 时，协同层会把该命令解释为带力语义的约束空间任务。
bool enable_force_control

# 左右末端目标位姿，当前约定为 world 下的 [x, y, z, roll, pitch, yaw]。
Pose6D left_target_pose
Pose6D right_target_pose

# 物体/虚拟物体目标位姿，后续双臂搬运时可表示箱体中心或 grasp frame。
Pose6D object_target_pose

# 目标内力/夹持力，当前主要用于接触阶段和后续内力闭环接口。
float64 target_internal_force
