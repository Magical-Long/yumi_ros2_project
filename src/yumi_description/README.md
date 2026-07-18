# yumi_description

YuMi 机器人的描述主源包。

## 作用

- 维护机器人的 URDF 结构
- 维护 mesh 几何资源
- 维护基础坐标系命名
- 为上层仿真、MoveIt 和控制包提供统一机器人模型

## 主要目录

- `urdf/yumi.urdf`
  - 机器人主 URDF
- `urdf/yumi_gazebo.urdf.xacro`
  - 面向 Gazebo 封装的 xacro
- `meshes/`
  - 机器人可视化与碰撞网格
- `launch/display.launch.py`
  - 在 RViz 中显示机器人模型

## 依赖关系

这个包应被其他所有层复用：

- `yumi_simulation_interfaces`
- `yumi_moveit_config`
- `yumi_planning_interface`
- `yumi_cooperative_controller`

## 使用方式

编译后可直接启动模型显示：

```bash
ros2 launch yumi_description display.launch.py
```

## 当前定位

这个包只负责“机器人长什么样”，不负责：

- Gazebo 控制器加载
- MoveIt 规划逻辑
- 双臂协作控制

这三类功能应分别放在其他功能包中。
