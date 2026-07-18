# yumi_task_manager

YuMi 双臂系统的任务规划与状态机层骨架包。

## 作用

- 管理控制模式切换
- 管理任务状态机
- 向协同控制层发布模式、状态和目标

## 当前骨架

当前版本先实现最小功能：

- 周期发布 `CooperativeTaskCommand`
- 用参数描述当前：
  - `mode`
  - `state`
  - 左右末端目标
  - 物体目标
  - 目标内力

## 设计定位

这一层只负责“做什么”，不负责“怎么控制”。

它不应直接输出：

- velocity controller topic 命令
- effort controller topic 命令
- 关节控制律

## 主要文件

- `src/yumi_task_manager_node.cpp`
- `config/task_manager_skeleton.yaml`
- `launch/task_manager_bringup.launch.py`

## 使用方式

```bash
ros2 launch yumi_task_manager task_manager_bringup.launch.py
```
