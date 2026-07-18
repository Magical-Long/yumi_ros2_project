1.yumi_interface里的msg数据结构可能存在冗余，会占用过大的存储空间，后续功能完善好后可以选择进行删减等相关处理
2.感觉planning中的配置参数有点乱，后期可以简化一下
3.感觉好像底层控制器接收上层话题数据时要注意线程安全问题？例如上层发送数据，下层开始计算控制量的时候这时上层又接收到数据是否会发生线程竞争的问题？
4.目前好像任务空间的轨迹没什么问题，但是好像底层执行层无法跟踪任务空间的轨迹，可能是雅可比矩阵那里出了问题，暂且先不管，自由空间先按关节轨迹来
5.目前自由空间的逆解的最终关节角选取还是有点问题，但不是大问题，就是需要优化一下，可以从双臂约束的角度出发，我觉得可以从对称角度出发，也可以从某些关节的权重变大的角度出发。
接入了镜像但是感觉还是有点问题
6.底层控制器每次发送command都要执行发送状态的函数（虽然要进行判断再决定是否发送数据给话题）
7.底层控制器一段轨迹只发一次信息给上游，为了防止上游漏收，最好让到位状态持续发布，或者加 reference_id 做可靠确认。
8.似乎任务管理器填充数据是全部填充并没有进行判断以及分别处理的效果
9.最终效果的误差似乎很大，需要看一下是怎么回事
10.启动文件里的临时 ready joint reference 会绕过 task_manager/cooperative_controller 直接发给 lowlevel。如果 task_manager 启动过早，lowlevel 对 ready 目标的 goal_reached 反馈可能被 task_manager 误认为 sequence step 0 已完成，导致 current_sequence_index_ 提前切换。当前默认延时下 task_manager 晚于 ready publisher，一般不会触发；后续更稳妥的方案是给任务命令和 lowlevel status 增加 command_id/reference_id，让 task_manager 只消费当前任务对应的完成反馈。
11.协作控制层里ready关节角什么的可以删除了，因为目前ready角度的达到是通过启动文件里直接发送关节数据来实现的，并不通过协作控制层了
12.wrench/force 目前属于预留功能，并没有看效果
13.use_joint_trajectory_debug_mode_ 可以删
这个变量之前是为了调试任务空间轨迹问题：当时任务空间跟踪效果不好，所以临时加了一个开关，让 constrained-space 也走 joint trajectory，方便对比“问题在 planning 还是 lowlevel task-space 控制”。但事实是问题应该是在lowlevel task-space
14.协作控制层的handleTaskCommand() 需要重构，特别是判断分支，过于混乱，应该统一一下
15.协作控制层关节轨迹规划时似乎并没有用到那个就近取moveit规划好的轨迹点的index
