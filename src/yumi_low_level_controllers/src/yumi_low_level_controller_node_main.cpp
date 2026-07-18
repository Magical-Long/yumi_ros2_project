#include "yumi_low_level_controllers/yumi_low_level_controller.hpp"

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<yumi_low_level_controllers::YumiLowLevelControllerNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
