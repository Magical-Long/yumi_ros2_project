# YuMi 双臂协同控制架构设计草案

## 1. 文档目标

本文档用于定义 YuMi 双臂协同控制系统的推荐软件架构，重点解决以下问题：

- 任务规划层如何进行模式切换与状态机管理
- 协同控制层如何与底层控制器解耦
- 不同控制模式下应输出什么类型的参考量
- velocity / effort / trajectory 三类执行方式如何在同一架构下共存

当前目标不是直接实现完整混合力位控制，而是先把**分层边界、输入输出和模式管理结构**定义清楚，为后续逐步实现提供稳定骨架。

---

## 2. 设计原则

### 2.1 任务规划层只管“做什么”

任务规划层负责：

- 模式切换
- 状态机切换
- 任务目标组织
- 上层流程编排

任务规划层不负责：

- 约束方程怎么写
- 物体对象怎么建模
- Jacobian 怎么算
- 关节速度 / 力矩命令怎么生成

一句话说：

**任务规划层负责“当前应该进入什么模式、给什么目标”，不负责“具体怎么控制”。**

### 2.2 协同控制层只管“怎么控制”

协同控制层负责：

- 双臂协同约束
- 任务空间控制
- 物体级控制
- 阻抗 / 导纳 / 混合力位控制
- 把上层目标转换成统一控制参考

协同控制层不应直接耦合：

- 仿真 topic 名称
- 具体底层控制器类型
- Gazebo 启动细节

### 2.3 底层控制层只管“怎么驱动机器人”

底层控制层负责：

- 轨迹跟踪
- 关节速度控制
- 关节力矩控制
- 关节空间 PD
- 逆动力学补偿

一句话说：

**协同控制层决定控制目标，底层控制层决定如何用控制律把目标变成关节命令。**

---

## 3. 推荐分层架构

推荐采用三层半结构：

```text
Task Manager
    -> 模式切换
    -> 状态机切换
    -> 任务目标下发

Cooperative Controller
    -> 双臂协同约束
    -> 任务空间 / 物体空间控制
    -> 统一控制参考生成

Low-level Controller
    -> trajectory / velocity / effort 控制
    -> PD / 逆动力学 / 阻抗等底层控制律
    -> 直接订阅统一 reference
```

---

## 4. 各层职责定义

### 4.1 Task Manager 层

建议后续新增：

- `yumi_task_manager`

其职责为：

- 管理系统主状态机
- 选择当前控制模式
- 组织阶段目标
- 调用 `yumi_planning_interface`
- 启动或切换协同控制流程

推荐由这一层管理的模式包括：

- `TRAJECTORY_MODE`
- `VELOCITY_MODE`
- `EFFORT_MODE`
- `HYBRID_FORCE_POSITION_MODE`

推荐由这一层管理的状态包括：

- `INITIALIZING`
- `HOLDING`
- `WORKING`
- `FINISHING`

其中 `WORKING` 还可以细分为：

- `MOVE_ONLY`
- `GRASP_ONLY`
- `COOPERATIVE_TRANSPORT`
- `HYBRID_FORCE_POSITION_WORK`

这一层的输出建议包括：

- 当前 `mode`
- 当前 `state`
- 末端目标
- 物体目标
- 相对位姿目标
- 目标内力

这一层**不直接输出底层控制器命令**。

### 4.2 Cooperative Controller 层

建议保留：

- `yumi_cooperative_controller`

其职责为：

- 根据 `mode` 和 `state` 选择控制策略
- 根据任务目标组织控制对象
- 实现：
  - 末端任务空间控制
  - 物体级控制
  - 相对位姿约束
  - 阻抗 / 导纳 / 混合力位控制
- 生成统一控制参考

这一层的输入建议包括：

- 当前 `mode`
- 当前 `state`
- 左右臂当前关节状态
- 左右臂当前末端状态
- 物体目标或虚拟物体目标
- 相对位姿目标
- 目标内力
- 力 / 力矩反馈

这一层的输出**根据模式不同而不同**。

### 4.3 Low-level Controller 层

这一层负责真正控制机器人，可包括：

- `joint_trajectory_controller`
- `joint_velocity_controller`
- `joint_effort_controller`
- 或自定义关节空间控制节点

其职责是：

- 直接订阅协同控制层发布的统一参考
- 执行关节空间控制律
- 在节点内部根据 `reference_mode` / 当前控制模式选择算法分支
- 产出最终关节速度 / 力矩 / 轨迹命令

---

## 5. 模式与输出类型建议

这是当前架构里最关键的一条设计规则。

### 5.1 轨迹模式

模式：

- `TRAJECTORY_MODE`

推荐输出：

- **关节轨迹**

原因：

- MoveIt 规划天然输出关节轨迹
- 轨迹模式主要服务于自由空间运动和标准轨迹执行
- 这一模式下直接保留 joint trajectory 语义最自然

因此，协同控制层在轨迹模式下可以输出：

- `JointTrajectoryReference`

### 5.2 速度模式

模式：

- `VELOCITY_MODE`

推荐输出：

- **任务空间轨迹**

例如：

- 左右末端期望位姿 / 速度
- 物体期望位姿 / 速度
- 相对位姿约束目标

原因：

- 速度控制下做末端任务空间控制更自然
- 更方便后续加入物体级控制与相对约束

### 5.3 力矩模式

模式：

- `EFFORT_MODE`

推荐输出：

- **任务空间轨迹**

原因：

- 力矩控制更适合在任务空间做阻抗、逆动力学和力控制
- 协同控制层应更关注控制对象而不是底层关节细节

### 5.4 混合力位模式

模式：

- `HYBRID_FORCE_POSITION_MODE`

推荐输出：

- **任务空间轨迹 + 力参考**

例如：

- 物体位姿轨迹
- 左右末端相对位姿约束
- 目标夹持内力

---

## 6. 推荐的统一参考量定义

建议后续在接口层逐步补齐三类 reference。

### 6.1 JointTrajectoryReference

用于：

- `TRAJECTORY_MODE`

内容建议包括：

- 关节名称
- 时间参数化关节轨迹
- 可选速度 / 加速度

### 6.2 TaskSpaceReference

用于：

- `VELOCITY_MODE`
- `EFFORT_MODE`

内容建议包括：

- 左右末端目标 pose / twist
- 或物体目标 pose / twist
- 或相对位姿目标

### 6.3 ForceReference

用于：

- `HYBRID_FORCE_POSITION_MODE`

内容建议包括：

- 目标夹持力
- 目标内力
- 目标接触法向力

---

## 7. 推荐数据流

推荐完整数据流如下：

```text
Task Manager
    -> mode + state + goal

Low-level Controller
    -> subscribe JointTrajectoryReference / TaskSpaceReference / ForceReference
    -> trajectory / velocity / effort execution

Robot
```

这里最重要的一点是：

- `task_manager` 不直接关心底层话题
- `cooperative_controller` 不直接耦合 Gazebo 控制器
- `low_level_controller` 负责控制律实现
- 当前不再单独保留 execution adapter 中间层

---

## 8. 与 `yumi_planning_interface` 的边界

### 8.1 `yumi_planning_interface` 负责什么

当前规划接口层继续负责：

- 自由空间规划
- 单臂 / 双臂位姿规划
- 标准轨迹执行
- 回 `home`
- 接触前就位
- 接触后撤离

### 8.2 `task_manager` 负责什么

推荐由 `task_manager` 负责：

1. 选择是否调用 `yumi_planning_interface`
2. 决定何时从规划模式切入协同模式
3. 下发：
   - 模式
   - 状态
   - 目标末端位姿
   - 目标物体轨迹
   - 目标内力

### 8.3 `yumi_cooperative_controller` 负责什么

`yumi_cooperative_controller` 负责：

- 在接触阶段解释这些目标
- 生成合适的协同控制参考

一句话说：

- `planning_interface` 负责“接触前后怎么运动”
- `task_manager` 负责“当前进入什么模式、给什么目标”
- `cooperative_controller` 负责“在该模式下怎么协同控制”

---

## 9. 初始化与保持状态机建议

无论后续是 velocity、effort 还是 hybrid mode，都建议保留最小初始化流程：

1. `INITIALIZING`
2. `HOLDING`
3. `WORKING`

### 9.1 INITIALIZING

目标：

- 把机器人带到安全标准起始姿态

该姿态可以是：

- `home`
- `dual_arm_ready`
- `pregrasp_box`

### 9.2 HOLDING

目标：

- 稳定保持当前安全起始姿态
- 等待切换到真正工作模式

### 9.3 WORKING

目标：

- 进入具体实验模式

例如：

- 单纯移动
- 单纯夹取
- 双臂协作搬运
- 混合力位控制

这三阶段应由 `task_manager` 管理，而不是由底层控制器零散拼接。

---

## 10. 当前阶段推荐落地顺序

建议按下面顺序推进：

### 第一步：先把 `task_manager` 概念定清楚

哪怕短期先不完全实现，也要先明确：

- 它只负责状态机切换
- 它只负责发送模式和目标
- 它不直接做控制算法

### 第二步：协同控制层保留最小状态机

当前 `yumi_cooperative_controller` 里可以先保留：

- `INITIALIZING`
- `HOLDING`

作为最小可运行版本。

### 第三步：明确模式下的输出类型

先把规则固定：

- 轨迹模式输出 joint trajectory
- 其他高级控制模式输出 task-space reference

### 第四步：让低层控制层直接消费统一 reference

即：

- `cooperative_controller` 直接发布统一 reference
- `low_level_controller` 直接订阅 reference 并按模式选择算法

### 第五步：逐步加入真正的工作模式

优先顺序建议为：

1. `MOVE_ONLY`
2. `GRASP_ONLY`
3. `COOPERATIVE_TRANSPORT`
4. `HYBRID_FORCE_POSITION_WORK`

---

## 11. 当前阶段推荐结论

对当前 YuMi 工程，推荐结论是：

1. `task_manager` 应定义为状态机切换与模式 / 目标下发层
2. `cooperative_controller` 应根据模式选择不同输出
3. 轨迹模式输出 joint trajectory
4. 其他高级控制模式输出 task-space reference
5. 底层控制层直接订阅统一 reference，并在节点内部按模式选择算法
6. 底层控制层负责根据不同执行方式实现 joint-space 控制律

一句话总结：

**推荐架构不是“上层直接发关节命令”，而是“任务规划层发模式和目标，协同控制层生成统一参考，底层控制层直接消费这些参考并完成真正控制”。**
