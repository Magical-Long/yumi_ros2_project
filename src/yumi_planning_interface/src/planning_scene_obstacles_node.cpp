#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace
{

geometry_msgs::msg::Pose poseFromXyzRpy(
    const std::vector<double> &poses,
    const std::size_t offset)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = poses[offset + 0U];
  pose.position.y = poses[offset + 1U];
  pose.position.z = poses[offset + 2U];

  // YAML 中用 roll/pitch/yaw 方便人工填写，
  // MoveIt CollisionObject 内部使用四元数姿态。
  tf2::Quaternion quaternion;
  quaternion.setRPY(poses[offset + 3U], poses[offset + 4U], poses[offset + 5U]);
  quaternion.normalize();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

}  // namespace

class YumiPlanningSceneObstaclesNode : public rclcpp::Node
{
public:
  YumiPlanningSceneObstaclesNode()
  : Node("yumi_planning_scene_obstacles")
  {
    declare_parameter<std::string>("frame_id", "world");
    declare_parameter<double>("apply_period_sec", 1.0);
    declare_parameter<int>("max_apply_attempts", 5);
    declare_parameter<std::vector<std::string>>("obstacle_names", {});
    declare_parameter<std::vector<double>>("box_sizes", {});
    declare_parameter<std::vector<double>>("box_poses", {});

    frame_id_ = get_parameter("frame_id").as_string();
    max_apply_attempts_ = get_parameter("max_apply_attempts").as_int();

    const auto apply_period_sec = get_parameter("apply_period_sec").as_double();
    apply_timer_ = create_wall_timer(
        std::chrono::duration<double>(apply_period_sec),
        std::bind(&YumiPlanningSceneObstaclesNode::applyObstaclesOnce, this));

    RCLCPP_INFO(
        get_logger(),
        "Planning scene obstacle node started. Static boxes will be applied in frame '%s'.",
        frame_id_.c_str());
  }

private:
  std::vector<moveit_msgs::msg::CollisionObject> buildBoxCollisionObjects()
  {
    const auto obstacle_names = get_parameter("obstacle_names").as_string_array();
    const auto box_sizes = get_parameter("box_sizes").as_double_array();
    const auto box_poses = get_parameter("box_poses").as_double_array();

    if (obstacle_names.empty())
    {
      RCLCPP_WARN(get_logger(), "No planning scene obstacles were configured.");
      return {};
    }
    if (box_sizes.size() != obstacle_names.size() * 3U)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Parameter 'box_sizes' must contain exactly 3 values for each obstacle.");
      return {};
    }
    if (box_poses.size() != obstacle_names.size() * 6U)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Parameter 'box_poses' must contain exactly 6 values for each obstacle.");
      return {};
    }

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.reserve(obstacle_names.size());
    for (std::size_t index = 0; index < obstacle_names.size(); ++index)
    {
      // CollisionObject 是 MoveIt PlanningScene 里的障碍物表达。
      // 只要 object 加入 planning scene，后续 move_group->plan() 默认会避开它。
      moveit_msgs::msg::CollisionObject object;
      object.header.frame_id = frame_id_;
      object.id = obstacle_names[index];
      object.operation = moveit_msgs::msg::CollisionObject::ADD;

      shape_msgs::msg::SolidPrimitive box;
      box.type = shape_msgs::msg::SolidPrimitive::BOX;
      box.dimensions.resize(3U);
      box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = box_sizes[index * 3U + 0U];
      box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = box_sizes[index * 3U + 1U];
      box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = box_sizes[index * 3U + 2U];

      object.primitives.push_back(box);
      object.primitive_poses.push_back(poseFromXyzRpy(box_poses, index * 6U));
      collision_objects.push_back(object);
    }

    return collision_objects;
  }

  void applyObstaclesOnce()
  {
    if (max_apply_attempts_ > 0 && apply_attempt_count_ >= max_apply_attempts_)
    {
      apply_timer_->cancel();
      return;
    }

    ++apply_attempt_count_;
    const auto collision_objects = buildBoxCollisionObjects();
    if (collision_objects.empty())
    {
      return;
    }

    // applyCollisionObjects 通过 MoveIt 的 PlanningScene service 写入场景。
    // 与单纯 publish topic 相比，它会等待 move_group 确认应用，启动时更稳。
    const bool applied = planning_scene_interface_.applyCollisionObjects(collision_objects);
    if (!applied)
    {
      RCLCPP_WARN(
          get_logger(),
          "Failed to apply planning scene obstacles on attempt %d / %d.",
          apply_attempt_count_,
          max_apply_attempts_);
      return;
    }

    RCLCPP_INFO(
        get_logger(),
        "Applied %zu box obstacles to MoveIt PlanningScene.",
        collision_objects.size());
    apply_timer_->cancel();
  }

  std::string frame_id_{"world"};
  int max_apply_attempts_{5};
  int apply_attempt_count_{0};
  rclcpp::TimerBase::SharedPtr apply_timer_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<YumiPlanningSceneObstaclesNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
