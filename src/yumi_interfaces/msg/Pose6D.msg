# 轻量 6D 位姿表达，主要用于 task_manager 和 planning_interface 之间传目标。
# 位置分量单位为米。
float64 x
float64 y
float64 z

# 姿态分量采用 roll / pitch / yaw，单位为弧度。
# 进入 TaskSpaceReference 前会转换成四元数，避免控制链里直接使用 RPY。
float64 roll
float64 pitch
float64 yaw
