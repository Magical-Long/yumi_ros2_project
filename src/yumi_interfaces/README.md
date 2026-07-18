# yumi_interfaces

YuMi 工程统一使用的 ROS2 接口定义包。

## 作用

- 提供跨包共享的 `msg`
- 提供跨包共享的 `action`
- 保持规划层、控制层和上层任务层的数据格式一致

## 当前接口

### 消息

- `msg/Pose6D.msg`
  - 简化的 6D 位姿表示
- `msg/TaskSpacePose.msg`
  - 任务空间位置 + 四元数姿态表示
- `msg/CooperativeTaskCommand.msg`
  - 任务管理层下发的模式 / 状态 / 目标
- `msg/JointSpaceReference.msg`
  - 协同控制层输出的关节空间参考
- `msg/LowLevelControllerStatus.msg`
  - 低层控制器发布的执行状态与误差反馈
- `msg/TaskSpaceReference.msg`
  - 协同控制层输出的任务空间参考，姿态统一使用四元数
- `msg/TaskSpaceTrajectoryPoint.msg`
  - 带 `time_from_start` 的任务空间轨迹采样点
- `msg/TaskSpaceTrajectory.msg`
  - 整条任务空间轨迹

### Action

- `action/PlanArmPose.action`
- `action/PlanBothArmsPoses.action`
- `action/PlanTaskSpaceTrajectory.action`
- `action/ResetToHome.action`
- `action/ResetBothArmsToHome.action`
- `action/RunRandomMotion.action`

## 设计原则

- 上层尽量不要直接依赖底层控制器 topic 名称
- 跨包共享数据时优先先补接口，再写节点逻辑
- 新增接口后应同步检查依赖包的 `CMakeLists.txt` 和 `package.xml`

## 当前定位

这个包只放接口，不放：

- 控制算法
- MoveIt 逻辑
- Gazebo 启动逻辑

## 构建说明

新增接口后建议先单独编译一次：

```bash
colcon build --symlink-install --packages-select yumi_interfaces
source install/setup.bash
```
