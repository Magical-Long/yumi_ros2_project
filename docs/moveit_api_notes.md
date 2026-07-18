# MoveIt API 常用函数速查

## 1. 文档目的

本文档用于记录当前 YuMi ROS2 工程中最常用的 MoveIt API，重点说明：

- 函数作用
- 典型输入
- 典型输出
- 在当前工程中的使用场景
- 使用时容易混淆的点

本文档面向当前工程中的：

- `yumi_moveit_config`
- `yumi_planning_interface`

后续如果新增 `yumi_dual_arm_coordinator` 或任务层，也应继续复用本文档中的术语与理解方式。

---

## 2. 基本理解

在当前工程中，MoveIt 相关调用大致分为三层：

1. 机器人模型层  
   由以下参数提供：
   - `robot_description`
   - `robot_description_semantic`
   - `robot_description_kinematics`
   - `robot_description_planning`

2. 规划接口层  
   主要通过：
   - `moveit::planning_interface::MoveGroupInterface`
   来发起规划与执行请求

3. 执行接口层  
   当前主要有两种：
   - `move_group->execute(plan)`
   - 直接向 `FollowJointTrajectory` action server 发送轨迹

在当前 YuMi 工程中，需要特别注意：

- `MoveGroupInterface` 是 MoveIt 的高层调用接口
- `move_group` 是 MoveIt 运行时内核
- `joint_trajectory_controller` 是底层控制器

这三者不是同一个东西。

另外，当前 `yumi_planning_interface` 已经对外暴露了 4 个 ROS2 action：

- `plan_arm_pose`
- `plan_both_arms_poses`
- `reset_to_home`
- `run_random_motion`

它们的定位是：

- 内部仍然复用已有 C++ 规划/执行函数
- 外部通过标准 action 调用
- 适合后续 `yumi_dual_arm_coordinator` 或任务层直接复用

---

## 3. 常用 API 速查

### 3.1 `MoveGroupInterface` 构造函数

典型写法：

```cpp
auto move_group =
  std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "arm_l");
```

作用：

- 创建某个规划组对应的 MoveIt 高层接口对象

输入：

- 当前 ROS2 节点
- 规划组名称，例如：
  - `arm_l`
  - `arm_r`
  - `both_arms`

输出：

- 一个可用于后续规划、执行、状态读取的 `MoveGroupInterface`

注意：

- 规划组名称必须已经在 SRDF 中定义
- 当前节点必须已经加载：
  - `robot_description`
  - `robot_description_semantic`

---

### 3.2 `setPlanningTime(double seconds)`

典型写法：

```cpp
move_group->setPlanningTime(5.0);
```

作用：

- 设置规划器最长允许求解的时间

输入：

- 一个以秒为单位的浮点数

输出：

- 无显式返回值

理解：

- 这不是轨迹总时长
- 而是“规划器最多花多久去找解”

---

### 3.3 `setMaxVelocityScalingFactor(double factor)`

典型写法：

```cpp
move_group->setMaxVelocityScalingFactor(0.2);
```

作用：

- 按比例缩小本次运动规划的速度上限

输入：

- `0.0 ~ 1.0` 之间的比例值

输出：

- 无显式返回值

理解：

- 不修改机器人模型的固有速度上限
- 只影响当前规划结果的时间参数化

---

### 3.4 `setMaxAccelerationScalingFactor(double factor)`

典型写法：

```cpp
move_group->setMaxAccelerationScalingFactor(0.2);
```

作用：

- 按比例缩小本次运动规划的加速度上限

输入：

- `0.0 ~ 1.0` 之间的比例值

输出：

- 无显式返回值

---

### 3.5 `startStateMonitor(double wait_time)`

典型写法：

```cpp
move_group->startStateMonitor(5.0);
```

作用：

- 显式启动 MoveIt 当前状态监视器

输入：

- 等待监视器建立和接收状态的时间

输出：

- 无显式返回值

在当前工程中的意义：

- MoveIt 不是直接从 Gazebo 读取关节角
- 它依赖：
  - `/joint_states`
  - `CurrentStateMonitor`
  - 内部机器人状态缓存

如果不显式启动状态监视器，后续：

- `getCurrentState()`
- `setStartStateToCurrentState()`

就可能卡在等待状态这一步。

---

### 3.6 `getCurrentState(double wait_seconds)`

典型写法：

```cpp
auto current_state = move_group->getCurrentState(5.0);
```

作用：

- 获取当前机器人状态

输入：

- 最长等待时间

输出：

- `moveit::core::RobotStatePtr`

失败时：

- 返回空指针

理解：

- 读取的是 MoveIt 内部缓存的当前状态
- 不是直接从 Gazebo 或 controller 读一份新数据

在当前工程中的典型用途：

- 随机生成合法末端位姿前，先拿当前状态做采样基准

---

### 3.7 `setStartStateToCurrentState()`

典型写法：

```cpp
move_group->setStartStateToCurrentState();
```

作用：

- 将本次规划的起点设置为机器人当前真实状态

输入：

- 无

输出：

- 无显式返回值

理解：

- 告诉 MoveIt：“这次轨迹从现在这个姿态开始”
- 避免沿用上一次规划残留的旧起点

在当前工程中的意义：

- 每次规划前都建议调用
- 尤其是仿真环境里机器人已经动过之后

---

### 3.8 `setPoseReferenceFrame(const std::string & frame)`

典型写法：

```cpp
move_group->setPoseReferenceFrame("world");
```

作用：

- 指定后续位姿目标属于哪个参考坐标系

输入：

- 坐标系名称，例如：
  - `world`
  - `yumi_body`

输出：

- 无显式返回值

理解：

- 同样的 `x / y / z / roll / pitch / yaw`
- 在不同 frame 下意义完全不同

当前工程约定：

- 简化接口默认使用：
  - `world`

---

### 3.9 `setPoseTarget(const geometry_msgs::msg::Pose &, const std::string & end_effector_link)`

典型写法：

```cpp
move_group->setPoseTarget(target_pose.pose, end_effector_link);
```

作用：

- 设置某个末端 link 的位姿目标

输入：

- 目标位姿
- 末端 link 名称

输出：

- 无显式返回值

理解：

- 这是“根据末端位姿规划”的核心输入

当前工程中典型场景：

- `planToPose(...)`

---

### 3.10 `clearPoseTargets()`

典型写法：

```cpp
move_group->clearPoseTargets();
```

作用：

- 清空之前设置过的位姿目标

输入：

- 无

输出：

- 无显式返回值

理解：

- 避免下一次规划不小心继续沿用旧目标

---

### 3.11 `setNamedTarget(const std::string & name)`

典型写法：

```cpp
move_group->setNamedTarget("home_l");
```

作用：

- 将规划目标设置为 SRDF 中定义好的命名关节姿态

输入：

- 命名姿态名称，例如：
  - `home_l`
  - `home_r`
  - `home`

输出：

- `bool`

成功时：

- 返回 `true`

失败时：

- 返回 `false`

理解：

- 目标不是末端位姿
- 而是一组已经命名好的关节角

当前工程中典型场景：

- `resetArmToHome(...)`
- `resetBothArmsToHome()`

---

### 3.12 `plan(MoveGroupInterface::Plan & plan)`

典型写法：

```cpp
moveit::planning_interface::MoveGroupInterface::Plan local_plan;
const auto success = static_cast<bool>(move_group->plan(local_plan));
```

作用：

- 调用 MoveIt 规划器，生成一条轨迹

输入：

- 一个 `Plan` 对象引用，用于接收规划结果

输出：

- 一个表示是否成功的结果，可转换为 `bool`

`Plan` 内最关键的内容：

- `trajectory_`

而 `trajectory_` 中最关键的是：

- `joint_trajectory`

又包含：

- `joint_names`
- `points`
- 每个点的：
  - `positions`
  - `velocities`
  - `accelerations`
  - `time_from_start`

重要理解：

- `Plan` 本身主要保存的是**相对时间轨迹**
- 它通常不包含“绝对从哪一时刻开始执行”

---

### 3.13 `execute(const MoveGroupInterface::Plan & plan)`

典型写法：

```cpp
move_group->execute(plan);
```

作用：

- 执行已经规划好的轨迹

输入：

- 一条 `Plan`

输出：

- 一个结果码，可转为 `bool`

理解：

- 这是 MoveIt 封装好的高层执行路径
- 最终仍然会把轨迹交给底层控制器

当前工程中：

- 单臂执行仍然可直接用它
- 双臂近同步执行则已改为手动走 `FollowJointTrajectory` action

---

### 3.14 `asyncExecute(const MoveGroupInterface::Plan & plan)`

典型写法：

```cpp
move_group->asyncExecute(plan);
```

作用：

- 异步执行轨迹，不阻塞当前线程

输入：

- 一条 `Plan`

输出：

- 一个结果码

理解：

- 命令很快返回
- 不代表轨迹已经执行完毕

当前工程中：

- 这条接口不再作为双臂同步执行主方案
- 因为我们需要自己更细致地控制左右臂轨迹下发过程

---

## 4. 当前工程中与执行相关的关键数据结构

### 4.1 `moveit::planning_interface::MoveGroupInterface::Plan`

作用：

- 保存一次完整的 MoveIt 规划结果

当前工程里的用法：

- 单臂规划结果保存
- 双臂左右臂各自规划结果保存

重要理解：

- 它是“轨迹计划”
- 不是单个时刻关节角
- 也不是绝对时间调度命令

---

### 4.2 `control_msgs::action::FollowJointTrajectory::Goal`

作用：

- 发送给底层 `joint_trajectory_controller` 的执行目标

当前工程里的用法：

- 左臂发给：
  - `/left_arm_controller/follow_joint_trajectory`
- 右臂发给：
  - `/right_arm_controller/follow_joint_trajectory`

当前工程经验：

- 直接复用 MoveIt 规划出的 `joint_trajectory`
- 不额外手动改绝对起始时间戳
- 可获得更稳定的近同步双臂执行效果

---

## 5. 当前工程已经验证的执行策略

对于 YuMi 当前 ROS2 工程，双臂执行层已验证可行的策略是：

1. 左右臂分别规划
2. 从左右臂各自的 `Plan` 中取出 `joint_trajectory`
3. 分别构造左右臂 `FollowJointTrajectory` goal
4. 分别发送给左右臂控制器
5. 等待左右臂各自动作完成

这应当理解为：

- 双单臂规划
- 双控制器近同步执行

而不是严格的联合双臂轨迹优化。

---

## 6. 当前最常见的误区

### 6.1 误以为 `move_group` 启动了，API 节点就自动有模型参数

错误理解：

- 只要 `move_group` 起了，别的节点就自动能构造 `MoveGroupInterface`

正确理解：

- API 节点自己也必须加载：
  - `robot_description`
  - `robot_description_semantic`
  - 等参数

### 6.2 误以为 `/joint_states` 有数据，`getCurrentState()` 就一定能立即成功

错误理解：

- 只要能 `echo /joint_states`，MoveIt 就肯定能拿到当前状态

正确理解：

- 还需要：
  - 状态监视器已启动
  - 回调线程可正常执行
  - 状态缓存已经建立

### 6.3 误以为双臂同步执行一定要手动改绝对时间戳

当前工程实测结论：

- 手动改绝对 `header.stamp` 容易导致控制器拒绝轨迹
- 当前更稳定的做法是不改绝对时间戳，直接分别发送轨迹

---

## 7. 推荐用法总结

### 7.1 单臂位姿规划

推荐顺序：

1. `setStartStateToCurrentState()`
2. `setPoseReferenceFrame(...)`
3. `setPoseTarget(...)`
4. `plan(...)`
5. `clearPoseTargets()`

### 7.2 单臂回 home

推荐顺序：

1. `setStartStateToCurrentState()`
2. `setNamedTarget("home_l" / "home_r")`
3. `plan(...)`
4. `execute(...)`

### 7.3 双臂回 home

推荐策略：

1. 左右臂分别 `setStartStateToCurrentState()`
2. 左右臂分别 `setNamedTarget(...)`
3. 左右臂分别 `plan(...)`
4. 分别向两个控制器发送轨迹

---

## 8. 一句话总结

在当前 YuMi ROS2 工程中，可以把 MoveIt API 理解成：

- `MoveGroupInterface` 负责告诉 MoveIt “从哪开始、到哪去、怎么规划”
- `Plan` 负责保存 MoveIt 生成的相对时间轨迹
- 底层控制器负责真正执行这条轨迹

对于当前工程，单臂执行可以继续使用 `execute(plan)`，而双臂执行的稳定主线已经收敛为：

- 分别规划
- 分别发送轨迹
- 近同步执行
