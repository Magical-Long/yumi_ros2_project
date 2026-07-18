# yumi_low_level_controllers

YuMi 双臂系统的低层控制层骨架包。

## 作用

这一层负责真正把参考量变成机器人可执行的关节命令。

后续应放在这一层的内容包括：

- 关节空间 PD 控制
- 逆动力学补偿
- 速度控制律
- 力矩控制律
- 阻抗控制底层实现

## 当前状态

当前包已经有第一版单节点骨架：

- `yumi_low_level_controller_node`

它目前会：

- 订阅 `joint_space_reference`
- 订阅 `task_space_reference`
- 订阅 `joint_states`
- 在内部按参考模式选择控制分支
- 当前先实现最小 `joint-space velocity tracking`
- 直接向左右臂 velocity controller 发布命令

这样做的目的是先把分层边界固定下来：

- `task_manager` 负责状态机与目标
- `cooperative_controller` 负责协同控制参考生成
- `low_level_controllers` 负责真正控制律

当前推荐的实现方向是：

- 由 `yumi_cooperative_controller` 直接发布 `JointSpaceReference / TaskSpaceReference`
- 由 `yumi_low_level_controllers` 直接订阅这些统一参考
- 在低层节点内部根据 `reference_mode` 选择不同控制算法分支

## 后续建议

后续可在这个包中逐步加入：

- `joint_velocity_tracking_controller`
- `joint_effort_tracking_controller`
- `task_space_impedance_controller`
- `task_space_velocity_controller`

## 当前文件

- `include/yumi_low_level_controllers/yumi_low_level_controller.hpp`
- `src/yumi_low_level_controller.cpp`
- `config/low_level_velocity.yaml`
- `launch/low_level_velocity_bringup.launch.py`

## 启动方式

```bash
ros2 launch yumi_low_level_controllers low_level_velocity_bringup.launch.py
```
