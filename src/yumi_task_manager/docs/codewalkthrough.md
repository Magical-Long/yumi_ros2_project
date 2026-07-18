# yumi_task_manager Code Walkthrough

## 核心职责

`yumi_task_manager` 是当前控制链路的任务状态机。它不直接控制 Gazebo 控制器，也不直接调用 MoveIt，而是周期发布 `/cooperative_task_command`，让 `yumi_cooperative_controller` 负责规划或生成 reference。

整体链路可以简化为：

```text
task_manager
  -> /cooperative_task_command
  -> cooperative_controller
  -> /joint_space_reference 或 /task_space_reference
  -> low_level_controller
  -> velocity controller commands
```

## 启动握手机制

启动握手解决的是“什么时候可以开始发任务”的问题。

`cooperative_controller` 会周期发布 `/cooperative_controller_status`，其中 `ready_for_task` 表示协作控制层是否已经准备好接任务。`task_manager` 订阅这个话题，并在 `handleCooperativeControllerStatus()` 中缓存最新状态：

```text
has_cooperative_status_ = true
latest_cooperative_status_ = 当前 cooperative 状态
```

`publishTaskCommand()` 每次准备发布任务前都会检查：

```text
如果 wait_for_cooperative_ready=true
并且还没收到 cooperative 状态
或者 cooperative ready_for_task=false
则本周期不发布任务命令
```

流程图：

```text
cooperative_controller
        |
        | /cooperative_controller_status
        | ready_for_task=true/false
        v
task_manager
        |
        | ready_for_task=true 后
        v
发布 /cooperative_task_command
```

一句话：握手机制防止 `planning_interface`、MoveIt action server 或 cooperative action client 还没准备好时，task manager 过早发出第一条任务。

## 序列任务加载

如果 YAML 中 `enable_sequence_execution=true`，节点启动时会调用 `loadSequenceIfEnabled()` 读取序列任务。

YAML 中的位姿采用扁平数组：

```yaml
sequence_left_target_poses: [
  x1, y1, z1, r1, p1, yaw1,
  x2, y2, z2, r2, p2, yaw2
]
```

代码中 `poseSequenceFromFlatVector()` 每 6 个数字解析成一个 `Pose6D`：

```text
[x, y, z, roll, pitch, yaw] -> Pose6D
```

之后 `loadSequenceIfEnabled()` 会把同一 index 的参数组合成一个 `TaskStep`：

```text
TaskStep i =
  motion_space[i]
  command_mode[i]
  use_object_target[i]
  enable_force_control[i]
  left_target_pose[i]
  right_target_pose[i]
  object_target_pose[i]
```

当前执行到哪一步由 `current_sequence_index_` 表示。

## 周期发布当前 Step

`publishTaskCommand()` 是 task manager 的主定时器回调。它每个周期做三件事：

```text
1. 检查 cooperative 是否 ready
2. 调用 tryAdvanceSequenceAfterHoldTime() 尝试推进序列
3. 发布当前 current_sequence_index_ 对应的 TaskStep
```

如果当前是序列模式，它会从 `task_sequence_` 中取当前 step：

```text
step = task_sequence_[current_sequence_index_]
```

然后填入 `CooperativeTaskCommand` 并发布到 `/cooperative_task_command`。

## 到位反馈机制

`low_level_controller` 在目标到位后，会向 `/low_level_controller_status` 发布一次：

```text
goal_reached=true
```

`task_manager` 在 `handleLowLevelControllerStatus()` 中接收这个反馈。

如果当前启用了序列任务，并且收到 `goal_reached=true`，task manager 会做：

```text
sequence_goal_reached_pending_ = true
tryAdvanceSequenceAfterHoldTime()
```

这里的含义是：当前 step 的到位事件已经来了，但是否马上切下一步，还要看最小保持时间是否满足。

## Step 切换机制

`tryAdvanceSequenceAfterHoldTime()` 负责真正切换 step。

切换条件有两个：

```text
1. sequence_goal_reached_pending_ == true
2. 当前 step 已经执行超过 sequence_min_step_hold_sec
```

满足后执行：

```text
sequence_goal_reached_pending_ = false
current_sequence_index_++
current_sequence_step_start_time_ = now()
```

流程图：

```text
发布 step i
    |
    v
等待 lowlevel goal_reached
    |
    v
sequence_goal_reached_pending_ = true
    |
    v
检查 hold time 是否足够
    |
    +-- 不足：继续等待，pending 保持 true
    |
    +-- 足够：current_sequence_index_++，进入 step i+1
```

## 两个状态标志

`sequence_goal_reached_pending_`

```text
表示当前 step 的到位事件已经收到，但 task manager 还没完成切换处理。
```

它是核心变量。因为 lowlevel 的 `goal_reached=true` 是单次事件，如果到位时 hold time 还没满足，就必须先把这个事件挂起来，等时间满足后再切换。

`sequence_step_goal_consumed_`

```text
表示当前 step 的到位事件是否已经被消费过。
```

它更像防御性保险，主要用于避免同一个完成反馈被重复处理。当前低层已经恢复成“到位后只发一次 status”，所以状态机主逻辑主要依赖 `sequence_goal_reached_pending_`。

## 一句话总结

```text
cooperative status 决定 task_manager 能不能开始发任务；
lowlevel status 决定当前 step 有没有完成；
current_sequence_index_ 决定 task_manager 当前发布 YAML 序列里的哪一组任务。
```
