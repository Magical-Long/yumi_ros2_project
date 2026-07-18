#include "yumi_cooperative_controller/yumi_cooperative_controller.hpp"

#include <memory>

int main(int argc, char ** argv)
{
  // 初始化 ROS2 运行时，为后续节点通信和参数系统做准备。
  rclcpp::init(argc, argv);
  // 创建协同控制节点实例，内部会完成订阅、发布和定时器构造。
  auto node = std::make_shared<yumi_cooperative_controller::YumiCooperativeControllerNode>();
  // 需要等节点实例真正托管到 shared_ptr 后，再初始化 action client。
  node->initialize();
  // 进入事件循环，让协同控制节点持续处理消息和控制回调。
  rclcpp::spin(node);
  // 正常退出前关闭 ROS2 上下文，释放中间件资源。
  rclcpp::shutdown();
  return 0;
}
