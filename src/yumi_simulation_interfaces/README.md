# yumi_simulation_interfaces

YuMi 的 Gazebo Classic 与 ros2_control 仿真启动包。

## 运行环境

- Ubuntu 20.04 LTS
- ROS 2 Foxy Fitzroy
- Gazebo Classic
- `gazebo_ros2_control`

其他 Ubuntu 或 ROS 2 版本尚未经过验证。

## 功能

- 加载 YuMi 双臂和夹爪仿真模型
- 启动 `robot_state_publisher` 和 Gazebo
- 生成 `gazebo_ros2_control` 所需的运行时 URDF
- 加载 `joint_state_broadcaster`
- 提供关节轨迹与关节速度两套控制器配置
- 可选地将双臂发送到无碰撞 home 姿态

## 启动方式

关节轨迹控制：

```bash
ros2 launch yumi_simulation_interfaces sim_bringup_traj.launch.py
```

关节速度控制：

```bash
ros2 launch yumi_simulation_interfaces sim_bringup_velocity.launch.py
```

常用启动参数：

```bash
ros2 launch yumi_simulation_interfaces sim_bringup_velocity.launch.py \
  gui:=false \
  pause:=false \
  enable_ros2_control:=true
```

默认世界文件为 `world/final.world`。

## URDF 路径说明

`urdf/yumi_gazebo_ros2.urdf` 和 `urdf/yumi_gazebo_ros2_velocity.urdf`
中的 mesh 使用经过当前环境验证的绝对 `file:///` URI。若工作空间位置不同，
请将 `/home/zhangjianglong/yumi_ws` 替换为本机工作空间的绝对路径。

控制器 YAML 的实际安装路径由 Launch 在运行时写入 URDF，不需要手工填写。

## 控制模式

| Launch 文件 | 控制器类型 | 适用场景 |
| --- | --- | --- |
| `sim_bringup_traj.launch.py` | `JointTrajectoryController` | MoveIt 轨迹执行与标准关节轨迹 |
| `sim_bringup_velocity.launch.py` | `JointGroupVelocityController` | 自定义低层速度控制器与协同控制实验 |

该仿真栈仍属于实验版本，控制增益、接触参数和任务空间控制效果需要根据具体实验继续调整。
