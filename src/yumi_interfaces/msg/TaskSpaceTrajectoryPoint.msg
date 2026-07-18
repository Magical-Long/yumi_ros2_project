# 从轨迹起点累计到当前参考点的时间。
# 协同控制层可以根据这个字段按周期读取对应参考点。
builtin_interfaces/Duration time_from_start

# 是否以物体/虚拟物体为主控制对象。
bool use_object_target

# 是否启用力相关参考。
bool enable_force_control

# 左右末端与物体的任务空间参考位姿。
# 这里使用位置 + 四元数姿态，避免中间再退回到 roll/pitch/yaw 表达。
yumi_interfaces/TaskSpacePose left_target_pose
yumi_interfaces/TaskSpacePose right_target_pose
yumi_interfaces/TaskSpacePose object_target_pose

# 可选的目标内力/夹持力参考。
# 当前轨迹点可以携带这类力语义，但真正的控制律仍应在下层实现。
float64 target_internal_force
