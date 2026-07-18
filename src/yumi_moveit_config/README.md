# yumi_moveit_config

YuMi 的 MoveIt 配置包。

## 作用

- 提供 SRDF
- 提供规划器配置
- 提供 MoveIt 控制器映射
- 提供 RViz 里的 MoveIt 可视化配置

## 主要目录

- `config/yumi.srdf`
  - MoveIt 规划组、自碰撞矩阵、命名姿态等配置
- `config/ompl_planning.yaml`
  - OMPL 规划参数
- `config/kinematics.yaml`
  - 逆运动学配置
- `config/joint_limits.yaml`
  - 关节速度/加速度限制
- `config/moveit_controllers.yaml`
  - MoveIt 到底层控制器的映射
- `launch/demo.launch.py`
  - MoveIt 演示入口
- `launch/demo_sim.launch.py`
  - 配合仿真的 MoveIt 演示入口
- `launch/move_group_only.launch.py`
  - 仅启动 MoveIt `move_group`

## 使用方式

常见启动方式：

```bash
ros2 launch yumi_moveit_config demo.launch.py
ros2 launch yumi_moveit_config demo_sim.launch.py
```

## 与其他包的关系

- 机器人模型来自 `yumi_description`
- 仿真执行链来自 `yumi_simulation_interfaces`
- 上层规划调用来自 `yumi_planning_interface`

## 当前定位

这个包只负责 MoveIt 配置，不负责：

- 规划接口封装
- 双臂协同控制
- Gazebo 控制器本体
