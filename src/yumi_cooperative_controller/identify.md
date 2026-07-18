1.void YumiCooperativeControllerNode::runControlLoop()函数中
  // 初始化、保持、结束阶段统一输出标准关节参考。
  // 这些阶段的语义由 task_manager 定义，下层只需要跟踪 reference。
  if (latest_task_command_.task_state ==
      yumi_interfaces::msg::CooperativeTaskCommand::STATE_INITIALIZING ||
    latest_task_command_.task_state ==
      yumi_interfaces::msg::CooperativeTaskCommand::STATE_HOLDING ||
    latest_task_command_.task_state ==
      yumi_interfaces::msg::CooperativeTaskCommand::STATE_FINISHING)
  {
    joint_reference_publisher_->publish(buildJointSpaceReference());
    return;
  }  这行代码目前的功能是不管是什么状态都进行发送home下的关节空间的信息    
