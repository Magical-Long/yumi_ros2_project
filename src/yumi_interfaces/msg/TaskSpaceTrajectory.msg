# 轨迹参考所属参考坐标系，例如 world。
string reference_frame

# 任务空间轨迹采样点序列。
# 每个点都带有 time_from_start，可供协同控制层按周期取点发布。
yumi_interfaces/TaskSpaceTrajectoryPoint[] points
