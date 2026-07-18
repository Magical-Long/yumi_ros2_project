builtin_interfaces/Time stamp

# true 表示 cooperative_controller 已经能连接到自由空间规划 action。
bool joint_planning_action_ready

# true 表示 constrained-space 需要的任务空间规划 action 已经可用。
bool task_space_planning_action_ready

# task_manager 只应在该标志为 true 后开始发布任务命令。
bool ready_for_task

# true 表示当前已有规划请求正在等待 planning_interface / MoveIt 返回结果。
bool planning_in_flight

# true 表示已经缓存了可播放的自由空间关节轨迹。
bool has_cached_joint_trajectory

# 简短状态说明，方便 ros2 topic echo 时直接判断卡在哪一步。
string message
