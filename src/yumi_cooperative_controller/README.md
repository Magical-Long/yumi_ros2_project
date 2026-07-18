# yumi_cooperative_controller

YuMi 双臂协同控制包。

## 作用

- 承载双臂协同控制逻辑
- 根据上层模式与目标生成统一控制参考
- 和具体底层控制器、仿真 topic、控制律实现解耦
- 为后续接入 MoveIt 规划结果和双臂协作算法预留入口

## 当前实现

当前第一版先实现了最小 reference 生成骨架：

1. 订阅 `cooperative_task_command`
2. 对 `INITIALIZING / HOLDING / FINISHING` 输出标准关节参考
3. 对 `WORKING` 根据模式发布：
   - `JointSpaceReference`
   - `TaskSpaceReference`

当前它**不直接做底层控制输出**，也**不在本层实现 joint-space P/PD 控制律**。

## 主要文件

- `src/yumi_cooperative_controller.cpp`
  - 上层协同控制节点
  - 当前按 `task_state + command_mode` 生成 reference
- `config/cooperative_velocity.yaml`
  - 双臂关节顺序、ready 姿态与参考发布周期
- `launch/cooperative_velocity_bringup.launch.py`
  - 当前协同控制启动入口

## 当前架构

推荐数据流：

```text
MoveIt / 上层任务
    -> yumi_cooperative_controller
    -> JointSpaceReference / TaskSpaceReference
    -> low_level_controller
```

这样做的目的：

- 协同控制层不直接绑定具体控制器 topic
- 初始化、保持、执行任务的语义留在上层状态机
- 底层 velocity / effort / trajectory 控制律被移到低层控制器
- 有利于后续加入物体级协同控制、导纳控制和内力调节
- 有利于后续把 MoveIt 规划结果作为本层输入之一统一组织
- 当前不再单独保留 execution adapter 中间层，统一 reference 直接交给低层控制层消费

## 使用方式

在任务管理层与仿真速度控制器已经启动后：

```bash
ros2 launch yumi_task_manager task_manager_bringup.launch.py
ros2 launch yumi_cooperative_controller cooperative_velocity_bringup.launch.py
```

## 后续扩展方向

- 接入 MoveIt 规划结果，区分规划轨迹参考和在线任务空间参考
- 从标准 ready 关节参考扩展到末端相对约束控制
- 再扩展到物体级协作控制
- 与低层控制层进一步对齐，按 `time_from_start` 周期发布缓存轨迹参考点
