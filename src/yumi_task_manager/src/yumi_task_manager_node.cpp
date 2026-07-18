#include <string>
#include <limits>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <yumi_interfaces/msg/cooperative_controller_status.hpp>
#include <yumi_interfaces/msg/cooperative_task_command.hpp>
#include <yumi_interfaces/msg/low_level_controller_status.hpp>

namespace
{

  // 把 YAML 中的 [x, y, z, roll, pitch, yaw] 数组转成统一 Pose6D 消息。
  yumi_interfaces::msg::Pose6D poseFromVector(const std::vector<double> &values)
  {
    yumi_interfaces::msg::Pose6D pose;
    // 只有长度足够时才写入，避免配置不完整时越界。
    if (values.size() >= 6)
    {
      pose.x = values[0];
      pose.y = values[1];
      pose.z = values[2];
      pose.roll = values[3];
      pose.pitch = values[4];
      pose.yaw = values[5];
    }
    return pose;
  }

} // namespace

class YumiTaskManagerNode : public rclcpp::Node
{
public:
  YumiTaskManagerNode()
      : Node("yumi_task_manager")
  {
    declare_parameter<bool>("enable_sequence_execution", false);
    declare_parameter<double>("sequence_min_step_hold_sec", 0.0);
    declare_parameter<std::vector<int64_t>>("sequence_motion_spaces", {});
    declare_parameter<std::vector<int64_t>>("sequence_command_modes", {});
    declare_parameter<std::vector<bool>>("sequence_use_object_target", {});
    declare_parameter<std::vector<bool>>("sequence_enable_force_control", {});
    declare_parameter<std::vector<double>>("sequence_target_internal_forces", {});
    declare_parameter<std::vector<double>>("sequence_left_target_poses", {});
    declare_parameter<std::vector<double>>("sequence_right_target_poses", {});
    declare_parameter<std::vector<double>>("sequence_object_target_poses", {});

    // 任务管理层当前只保留两类主状态：
    // - FREE_SPACE：自由空间移动，通常走 MoveIt 关节轨迹
    // - CONSTRAINED_SPACE：进入接触/夹持/协作阶段，通常走任务空间控制
    declare_parameter<int>(
        "motion_space_mode",
        static_cast<int>(yumi_interfaces::msg::CooperativeTaskCommand::SPACE_FREE));
    declare_parameter<int>(
        "command_mode",
        static_cast<int>(yumi_interfaces::msg::CooperativeTaskCommand::MODE_VELOCITY));
    declare_parameter<bool>("use_object_target", false);
    declare_parameter<bool>("enable_force_control", false);
    declare_parameter<double>("target_internal_force", 0.0);
    declare_parameter<double>("publish_period_sec", 0.2);
    declare_parameter<bool>("wait_for_cooperative_ready", true);
    declare_parameter<bool>("enable_feedback_driven_transition", false);
    declare_parameter<std::vector<double>>(
        "free_space_left_target_pose",
        {0.476239, 0.313782, 0.823692, -1.423986, 0.788104, -1.512160});
    declare_parameter<std::vector<double>>(
        "free_space_right_target_pose",
        {0.476226, -0.314137, 0.823431, 1.422371, 0.784801, 1.512639});
    declare_parameter<std::vector<double>>(
        "constrained_left_target_pose",
        {0.4, 0.2, 0.3, 0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>(
        "constrained_right_target_pose",
        {0.4, -0.2, 0.3, 0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>(
        "object_target_pose",
        {0.45, 0.0, 0.25, 0.0, 0.0, 0.0});

    // 对外统一发布 CooperativeTaskCommand，供协同控制层消费。
    command_publisher_ =
        create_publisher<yumi_interfaces::msg::CooperativeTaskCommand>("cooperative_task_command", 10);
    controller_status_subscriber_ =
        create_subscription<yumi_interfaces::msg::LowLevelControllerStatus>(
            "low_level_controller_status", 10,
            std::bind(&YumiTaskManagerNode::handleLowLevelControllerStatus, this, std::placeholders::_1));
    // 这条订阅就是“低层反馈 -> 任务管理层决策”的入口。
    // low_level_controller 负责判断是否到位，task_manager 负责据此推进状态机。
    cooperative_status_subscriber_ =
        create_subscription<yumi_interfaces::msg::CooperativeControllerStatus>(
            "cooperative_controller_status", 10,
            std::bind(&YumiTaskManagerNode::handleCooperativeControllerStatus, this, std::placeholders::_1));
    // 这条订阅是启动握手入口：协同层确认 planning action ready 后，任务层才开始发命令。

    // 用较低频率周期广播当前任务命令，模拟最小状态机输出。
    const auto publish_period_sec = get_parameter("publish_period_sec").as_double();
    publish_timer_ = create_wall_timer(
        std::chrono::duration<double>(publish_period_sec),
        std::bind(&YumiTaskManagerNode::publishTaskCommand, this));

    loadSequenceIfEnabled();
  }

private:
  struct TaskStep
  {
    // 一个 TaskStep 表示“一次双臂联合目标”：
    // - left/right/object pose 是同一步里的同时目标
    // - cooperative_controller 会把这一整步当成一个双臂联合规划/控制请求来处理
    uint8_t motion_space{yumi_interfaces::msg::CooperativeTaskCommand::SPACE_UNSPECIFIED};
    uint8_t command_mode{yumi_interfaces::msg::CooperativeTaskCommand::MODE_UNSPECIFIED};
    bool use_object_target{false};
    bool enable_force_control{false};
    double target_internal_force{0.0};
    yumi_interfaces::msg::Pose6D left_target_pose;
    yumi_interfaces::msg::Pose6D right_target_pose;
    yumi_interfaces::msg::Pose6D object_target_pose;
  };

  std::vector<yumi_interfaces::msg::Pose6D> poseSequenceFromFlatVector(
      const std::vector<double> &flat_values) const
  {
    std::vector<yumi_interfaces::msg::Pose6D> poses;
    if (flat_values.empty())
    {
      return poses;
    }
    if (flat_values.size() % 6 != 0)
    {
      throw std::runtime_error("Sequence pose array size must be a multiple of 6.");
    }
    poses.reserve(flat_values.size() / 6U);
    for (std::size_t index = 0; index < flat_values.size(); index += 6U)
    {
      poses.push_back(poseFromVector(
          {flat_values[index + 0U], flat_values[index + 1U], flat_values[index + 2U],
           flat_values[index + 3U], flat_values[index + 4U], flat_values[index + 5U]}));
    }
    return poses;
  }

  void loadSequenceIfEnabled()
  {
    sequence_enabled_ = get_parameter("enable_sequence_execution").as_bool();
    task_sequence_.clear();
    current_sequence_index_ = 0U;
    sequence_step_goal_consumed_ = false;
    sequence_goal_reached_pending_ = false;
    last_published_sequence_index_ = std::numeric_limits<std::size_t>::max();
    current_sequence_step_start_time_ = now();

    if (!sequence_enabled_)
    {
      return;
    }

    const auto motion_spaces = get_parameter("sequence_motion_spaces").as_integer_array();
    const auto command_modes = get_parameter("sequence_command_modes").as_integer_array();
    const auto use_object_targets = get_parameter("sequence_use_object_target").as_bool_array();
    const auto enable_force_controls =
        get_parameter("sequence_enable_force_control").as_bool_array();
    const auto target_internal_forces =
        get_parameter("sequence_target_internal_forces").as_double_array();
    const auto left_target_poses =
        poseSequenceFromFlatVector(get_parameter("sequence_left_target_poses").as_double_array());
    const auto right_target_poses =
        poseSequenceFromFlatVector(get_parameter("sequence_right_target_poses").as_double_array());
    const auto object_target_poses =
        poseSequenceFromFlatVector(get_parameter("sequence_object_target_poses").as_double_array());

    const std::size_t step_count = motion_spaces.size();
    if (step_count == 0U)
    {
      RCLCPP_WARN(
          get_logger(),
          "enable_sequence_execution=true, but sequence_motion_spaces is empty. Falling back to single command mode.");
      sequence_enabled_ = false;
      return;
    }

    /* 检查数组长度的lamda函数 */
    const auto require_size = [this, step_count](std::size_t actual, const char *name)
    {
      if (actual != step_count)
      {
        throw std::runtime_error(
            std::string("Sequence parameter '") + name + "' must have exactly " +
            std::to_string(step_count) + " entries.");
      }
    };
    /* 检查所有sequence数组长度 */
    require_size(command_modes.size(), "sequence_command_modes");
    require_size(use_object_targets.size(), "sequence_use_object_target");
    require_size(enable_force_controls.size(), "sequence_enable_force_control");
    require_size(target_internal_forces.size(), "sequence_target_internal_forces");
    require_size(left_target_poses.size(), "sequence_left_target_poses");
    require_size(right_target_poses.size(), "sequence_right_target_poses");
    require_size(object_target_poses.size(), "sequence_object_target_poses");

    task_sequence_.reserve(step_count);
    /* 是直接全部填充进去，并没有进行任务不同来进行分别处理的算法 */
    for (std::size_t index = 0; index < step_count; ++index)
    {
      TaskStep step;
      step.motion_space = static_cast<uint8_t>(motion_spaces[index]);
      step.command_mode = static_cast<uint8_t>(command_modes[index]);
      step.use_object_target = use_object_targets[index];
      step.enable_force_control = enable_force_controls[index];
      step.target_internal_force = target_internal_forces[index];
      step.left_target_pose = left_target_poses[index];
      step.right_target_pose = right_target_poses[index];
      step.object_target_pose = object_target_poses[index];
      task_sequence_.push_back(step);
    }
  }

  void publishTaskCommand()
  {
    /* 当使能等待协作就绪且未收到协作状态或协作状态未就绪时就打印warn并直接返回 */
    if (get_parameter("wait_for_cooperative_ready").as_bool() &&
        (!has_cooperative_status_ || !latest_cooperative_status_.ready_for_task))
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Waiting for cooperative_controller ready before publishing task commands. last_status='%s'.",
          has_cooperative_status_ ? latest_cooperative_status_.message.c_str() : "no status received");
      return;
    }

    // goal_reached 是低层边沿式单次反馈。
    // 如果反馈早于 min hold 到来，这里会在后续定时器周期里补做推进，避免漏掉那一次 true。
    tryAdvanceSequenceAfterHoldTime();

    // 每次回调都从参数服务器取值，便于用 ros2 param 在运行期改模式。
    yumi_interfaces::msg::CooperativeTaskCommand command_msg;
    command_msg.stamp = now();

    /* 如果使能发送序列任务且任务列表不是空的 */
    if (sequence_enabled_ && !task_sequence_.empty())
    {
      const auto &step = task_sequence_[current_sequence_index_];
      if (last_published_sequence_index_ != current_sequence_index_)
      {
        RCLCPP_INFO(
            get_logger(),
            "Publishing sequence step %zu / %zu (motion_space=%u, command_mode=%u).",
            current_sequence_index_ + 1U,
            task_sequence_.size(),
            static_cast<unsigned int>(step.motion_space),
            static_cast<unsigned int>(step.command_mode));
        last_published_sequence_index_ = current_sequence_index_;
      }
      command_msg.motion_space = step.motion_space;
      command_msg.command_mode = step.command_mode;
      command_msg.use_object_target = step.use_object_target;
      command_msg.enable_force_control = step.enable_force_control;
      command_msg.target_internal_force = step.target_internal_force;
      command_msg.left_target_pose = step.left_target_pose;
      command_msg.right_target_pose = step.right_target_pose;
      command_msg.object_target_pose = step.object_target_pose;
      command_publisher_->publish(command_msg);
      return;
    }

    /* 其他情况 */
    const auto configured_motion_space =
        static_cast<uint8_t>(get_parameter("motion_space_mode").as_int());
    if (!motion_space_overridden_)
    {
      // 默认情况下，motion_space 仍由参数/YAML 主导。
      // 只有反馈驱动切换生效后，current_motion_space_ 才会临时覆盖这个配置值。
      current_motion_space_ = configured_motion_space;
    }
    command_msg.motion_space = current_motion_space_;
    // command_mode 描述的是底层控制器模式，而不是当前属于自由空间还是约束空间。
    // 因此它应当独立于 motion_space 保持一致，例如整套系统固定使用速度控制器时，
    // 无论 FREE_SPACE 还是 CONSTRAINED_SPACE 都持续发布 MODE_VELOCITY。
    command_msg.command_mode =
        static_cast<uint8_t>(get_parameter("command_mode").as_int());
    command_msg.use_object_target = get_parameter("use_object_target").as_bool();
    command_msg.enable_force_control = get_parameter("enable_force_control").as_bool();
    command_msg.target_internal_force = get_parameter("target_internal_force").as_double();
    if (current_motion_space_ ==
        static_cast<uint8_t>(yumi_interfaces::msg::CooperativeTaskCommand::SPACE_FREE))
    {
      // FREE_SPACE 阶段先发布 ready 对应的双臂末端位姿，让协作层统一通过 MoveIt
      // 求出一条去 ready 的关节轨迹，再交给低层速度控制器跟踪。
      command_msg.left_target_pose =
          poseFromVector(get_parameter("free_space_left_target_pose").as_double_array());
      command_msg.right_target_pose =
          poseFromVector(get_parameter("free_space_right_target_pose").as_double_array());
    }
    else
    {
      // 进入 CONSTRAINED_SPACE 后，再切到真正的任务目标位姿。
      command_msg.left_target_pose =
          poseFromVector(get_parameter("constrained_left_target_pose").as_double_array());
      command_msg.right_target_pose =
          poseFromVector(get_parameter("constrained_right_target_pose").as_double_array());
    }
    command_msg.object_target_pose = poseFromVector(get_parameter("object_target_pose").as_double_array());
    // 统一把本周期任务意图发布出去，让下游节点按各自职责解释。
    command_publisher_->publish(command_msg);
  }

  void handleLowLevelControllerStatus(
      const yumi_interfaces::msg::LowLevelControllerStatus::SharedPtr msg)
  {
    // 先缓存最近一次反馈，后续如果要扩展更复杂的状态机条件，可以直接复用这里的数据。
    latest_controller_status_ = *msg;
    has_controller_status_ = true;

    if (!get_parameter("enable_feedback_driven_transition").as_bool())
    {
      return;
    }
    if (!msg->goal_reached)
    {
      sequence_step_goal_consumed_ = false;
      return;
    }
    /* 如果使能了反馈驱动切换且任务列表非空 */
    if (sequence_enabled_ && !task_sequence_.empty())
    {
      if (sequence_step_goal_consumed_) // 如果同一个sequence step已经消费过了就直接返回
      {
        return;
      }
      // 先锁存“当前 step 已到位”这个事件。
      // 即使此时还没满足最小保持时间，也不能丢掉该事件，因为低层不会持续重复上报 true。
      sequence_goal_reached_pending_ = true;
      tryAdvanceSequenceAfterHoldTime();
      return;
    }

    // 两态状态机：
    // - 当前若处于 FREE_SPACE
    // - 且第一次收到“自由空间目标已到位”反馈
    // 就自动切到 CONSTRAINED_SPACE，开始把任务交给任务空间/协作控制链。
    if (current_motion_space_ ==
            static_cast<uint8_t>(yumi_interfaces::msg::CooperativeTaskCommand::SPACE_FREE) &&
        !free_to_constrained_transition_done_)
    {
      current_motion_space_ =
          static_cast<uint8_t>(yumi_interfaces::msg::CooperativeTaskCommand::SPACE_CONSTRAINED);
      motion_space_overridden_ = true;
      free_to_constrained_transition_done_ = true;
      return;
    }
  }

  void handleCooperativeControllerStatus(
      const yumi_interfaces::msg::CooperativeControllerStatus::SharedPtr msg)
  {
    const bool was_ready = has_cooperative_status_ && latest_cooperative_status_.ready_for_task;
    latest_cooperative_status_ = *msg;
    has_cooperative_status_ = true;
    if (!was_ready && msg->ready_for_task)
    {
      RCLCPP_INFO(get_logger(), "Cooperative controller is ready. Task command publishing is enabled.");
    }
  }

  void tryAdvanceSequenceAfterHoldTime()
  {
    if (!sequence_enabled_ || task_sequence_.empty())
    {
      return;
    }
    if (!sequence_goal_reached_pending_ || sequence_step_goal_consumed_)
    {
      return;
    }
    /* 当sequence_goal_reached_pending_为true且sequence_step_goal_consumed_为false时才会计时 */
    const double min_step_hold_sec = get_parameter("sequence_min_step_hold_sec").as_double();
    const double elapsed_sec = (now() - current_sequence_step_start_time_).seconds();
    if (elapsed_sec < min_step_hold_sec)
    {
      return;
    }
    /*  */
    sequence_step_goal_consumed_ = true;
    /* 表示不再等待 */
    sequence_goal_reached_pending_ = false;
    if (current_sequence_index_ + 1U < task_sequence_.size())
    {
      ++current_sequence_index_;
      // 切到下一步后重新开始计时，并允许下一次 goal_reached 事件推进新 step。
      sequence_step_goal_consumed_ = false;
      current_sequence_step_start_time_ = now();
      last_published_sequence_index_ = std::numeric_limits<std::size_t>::max();
    }
  }

  // 一个 publisher 对外广播任务命令，一个 timer 驱动周期性输出。
  rclcpp::Publisher<yumi_interfaces::msg::CooperativeTaskCommand>::SharedPtr command_publisher_;
  rclcpp::Subscription<yumi_interfaces::msg::LowLevelControllerStatus>::SharedPtr
      controller_status_subscriber_;
  rclcpp::Subscription<yumi_interfaces::msg::CooperativeControllerStatus>::SharedPtr
      cooperative_status_subscriber_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  bool has_controller_status_{false};
  bool has_cooperative_status_{false};
  bool motion_space_overridden_{false};
  bool free_to_constrained_transition_done_{false};
  bool sequence_enabled_{false};
  bool sequence_step_goal_consumed_{false};
  bool sequence_goal_reached_pending_{false};
  std::size_t last_published_sequence_index_{std::numeric_limits<std::size_t>::max()};
  uint8_t current_motion_space_{
      static_cast<uint8_t>(yumi_interfaces::msg::CooperativeTaskCommand::SPACE_FREE)};
  std::size_t current_sequence_index_{0U};
  rclcpp::Time current_sequence_step_start_time_{0, 0, RCL_ROS_TIME};
  std::vector<TaskStep> task_sequence_;
  yumi_interfaces::msg::LowLevelControllerStatus latest_controller_status_;
  yumi_interfaces::msg::CooperativeControllerStatus latest_cooperative_status_;
};

int main(int argc, char **argv)
{
  // 初始化 ROS2，准备启动任务管理节点。
  rclcpp::init(argc, argv);
  // 创建节点实例，节点内部会构造参数、publisher 和 timer。
  auto node = std::make_shared<YumiTaskManagerNode>();
  // 持续运行事件循环，让任务命令按设定周期发布。
  rclcpp::spin(node);
  // 结束前释放 ROS2 上下文资源。
  rclcpp::shutdown();
  return 0;
}
