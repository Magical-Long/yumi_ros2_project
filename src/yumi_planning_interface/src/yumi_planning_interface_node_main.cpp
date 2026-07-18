#include <memory>

#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "yumi_planning_interface/yumi_planning_interface.hpp"

int main(int argc, char **argv)
{
  // 初始化 ROS2 运行时，这是后续创建 node、executor 和 MoveIt 接口的前提。
  rclcpp::init(argc, argv);

  // 创建上层规划接口节点。
  auto node = std::make_shared<yumi_planning_interface::YumiPlanningInterfaceNode>();

  // 将 MoveIt 相关对象的构造放到独立初始化函数里，
  // 避免在构造函数阶段过早使用 shared_from_this()。
  if (!node->initialize())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize yumi_planning_interface node.");
    rclcpp::shutdown();
    return 1;
  }

  // MoveIt 在内部需要持续接收 joint_states、scene 和 action 相关回调。
  // 如果仍然使用单线程 spin，那么定时器回调里一旦阻塞等待当前状态，
  // 接收 joint_states 的回调就没有机会执行，容易表现成“始终在监听 joint_states”。
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  // 正常退出前关闭 ROS2 上下文，释放中间件资源。
  rclcpp::shutdown();
  return 0;
}
