# yumi_planning_interface

YuMi 的上层规划接口包。

## 作用

- 封装 MoveIt C++ API
- 对外提供单臂 / 双臂规划与执行能力
- 对外暴露标准 ROS2 action，便于上层任务调用

## 主要文件

- `src/yumi_planning_interface.cpp`
  - 核心规划逻辑
- `src/yumi_planning_interface_node_main.cpp`
  - 节点入口
- `launch/bringup.launch.py`
  - 规划接口启动入口

## 当前能力

- 单臂位姿规划
- 双臂位姿规划
- 单臂 / 双臂轨迹执行
- 单臂 / 双臂回 `home`
- 随机动作验证

## 对外接口

当前主要通过 `yumi_interfaces` 中的 action 暴露能力：

- `PlanArmPose`
- `PlanBothArmsPoses`
- `ResetToHome`
- `ResetBothArmsToHome`
- `RunRandomMotion`

## 使用方式

```bash
ros2 launch yumi_planning_interface bringup.launch.py
```

## 当前定位

这个包负责：

- 接触前自由空间规划
- 接触后撤离 / 回 home

这个包不负责：

- 接触中的双臂协同控制
- 力反馈
- 导纳控制

这些应由 `yumi_cooperative_controller` 负责。
