# 任务空间位置分量，单位为米。
float64 x
float64 y
float64 z

# 任务空间姿态四元数分量。
# 建议始终保持归一化，避免后续插值和姿态误差计算出现数值漂移。
float64 qx
float64 qy
float64 qz
float64 qw
