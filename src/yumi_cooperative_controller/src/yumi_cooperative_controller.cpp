#include "yumi_cooperative_controller/yumi_cooperative_controller.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <stdexcept>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace yumi_cooperative_controller
{
  namespace
  {
    /* Pose6D转taskspace */
    yumi_interfaces::msg::TaskSpacePose taskSpacePoseFromPose6D(
        const yumi_interfaces::msg::Pose6D &pose)
    {
      yumi_interfaces::msg::TaskSpacePose task_pose;
      task_pose.x = pose.x;
      task_pose.y = pose.y;
      task_pose.z = pose.z;

      const double half_roll = 0.5 * pose.roll;
      const double half_pitch = 0.5 * pose.pitch;
      const double half_yaw = 0.5 * pose.yaw;
      const double cr = std::cos(half_roll);
      const double sr = std::sin(half_roll);
      const double cp = std::cos(half_pitch);
      const double sp = std::sin(half_pitch);
      const double cy = std::cos(half_yaw);
      const double sy = std::sin(half_yaw);

      task_pose.qw = cr * cp * cy + sr * sp * sy;
      task_pose.qx = sr * cp * cy - cr * sp * sy;
      task_pose.qy = cr * sp * cy + sr * cp * sy;
      task_pose.qz = cr * cp * sy - sr * sp * cy;
      return task_pose;
    }

    /* taskspace变量转齐次变换矩阵 */
    tf2::Transform transformFromTaskSpacePose(
        const yumi_interfaces::msg::TaskSpacePose &pose)
    {
      tf2::Quaternion rotation(pose.qx, pose.qy, pose.qz, pose.qw);
      rotation.normalize();
      return tf2::Transform(rotation, tf2::Vector3(pose.x, pose.y, pose.z));
    }

    tf2::Transform transformFromPose6D(const yumi_interfaces::msg::Pose6D &pose)
    {
      return transformFromTaskSpacePose(taskSpacePoseFromPose6D(pose));
    }

    yumi_interfaces::msg::TaskSpacePose taskSpacePoseFromTransform(
        const tf2::Transform &transform)
    {
      yumi_interfaces::msg::TaskSpacePose pose;
      const auto origin = transform.getOrigin();
      const auto rotation = transform.getRotation().normalized();
      pose.x = origin.x();
      pose.y = origin.y();
      pose.z = origin.z();
      pose.qx = rotation.x();
      pose.qy = rotation.y();
      pose.qz = rotation.z();
      pose.qw = rotation.w();
      return pose;
    }

    tf2::Transform transformFromStamped(const geometry_msgs::msg::TransformStamped &transform)
    {
      const auto &translation = transform.transform.translation;
      const auto &rotation_msg = transform.transform.rotation;
      tf2::Quaternion rotation(rotation_msg.x, rotation_msg.y, rotation_msg.z, rotation_msg.w);
      rotation.normalize();
      return tf2::Transform(
          rotation,
          tf2::Vector3(translation.x, translation.y, translation.z));
    }

  } // namespace

  YumiCooperativeControllerNode::YumiCooperativeControllerNode()
      : Node("yumi_cooperative_controller")
  {
    // 双臂关节顺序是后续 joint trajectory / joint reference 的统一索引基准。
    declare_parameter<std::vector<std::string>>(
        "left_joint_names",
        {"yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l", "yumi_joint_3_l",
         "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l"});
    declare_parameter<std::vector<std::string>>(
        "right_joint_names",
        {"yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r", "yumi_joint_3_r",
         "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r"});

    // ready/home 是初始化和保持阶段的标准关节参考，由下层决定如何去跟踪它。
    declare_parameter<std::vector<double>>(
        "ready_left_positions",
        {0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>(
        "ready_right_positions",
        {0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    // 协同控制层只做 reference 生成，因此这里只保留发布周期参数。
    declare_parameter<double>("control_period_sec", 0.02);
    // planning_interface action 名称与规划参数默认值。
    declare_parameter<std::string>(
        "plan_both_arms_poses_action_name", "plan_both_arms_poses");
    declare_parameter<std::string>(
        "plan_task_space_trajectory_action_name", "plan_task_space_trajectory");
    declare_parameter<double>("planning_action_server_wait_timeout_sec", 5.0);
    declare_parameter<double>("planning_time_sec", 5.0);
    declare_parameter<int>("planning_num_attempts", 3);
    declare_parameter<double>("planning_velocity_scaling", 0.2);
    declare_parameter<double>("planning_acceleration_scaling", 0.2);
    declare_parameter<double>("planning_sample_period_sec", 0.05);
    declare_parameter<double>("joint_planning_result_timeout_sec", 12.0);
    declare_parameter<double>("object_trajectory_duration_sec", 8.0);
    declare_parameter<std::string>("object_reference_frame", "world");
    declare_parameter<std::string>("left_end_effector_frame", "gripper_l_base");
    declare_parameter<std::string>("right_end_effector_frame", "gripper_r_base");

    left_joint_names_ = get_parameter("left_joint_names").as_string_array();
    right_joint_names_ = get_parameter("right_joint_names").as_string_array();
    ready_left_positions_ = get_parameter("ready_left_positions").as_double_array();
    ready_right_positions_ = get_parameter("ready_right_positions").as_double_array();
    control_period_sec_ = get_parameter("control_period_sec").as_double();
    plan_both_arms_poses_action_name_ =
        get_parameter("plan_both_arms_poses_action_name").as_string();
    plan_task_space_trajectory_action_name_ =
        get_parameter("plan_task_space_trajectory_action_name").as_string();
    planning_action_server_wait_timeout_sec_ =
        get_parameter("planning_action_server_wait_timeout_sec").as_double();
    planning_time_sec_ = get_parameter("planning_time_sec").as_double();
    planning_num_attempts_ = get_parameter("planning_num_attempts").as_int();
    planning_velocity_scaling_ = get_parameter("planning_velocity_scaling").as_double();
    planning_acceleration_scaling_ = get_parameter("planning_acceleration_scaling").as_double();
    planning_sample_period_sec_ = get_parameter("planning_sample_period_sec").as_double();
    joint_planning_result_timeout_sec_ =
        get_parameter("joint_planning_result_timeout_sec").as_double();
    object_trajectory_duration_sec_ =
        get_parameter("object_trajectory_duration_sec").as_double();
    object_reference_frame_ = get_parameter("object_reference_frame").as_string();
    left_end_effector_frame_ = get_parameter("left_end_effector_frame").as_string();
    right_end_effector_frame_ = get_parameter("right_end_effector_frame").as_string();

    if (left_joint_names_.size() != ready_left_positions_.size())
    {
      throw std::runtime_error(
          "Parameter 'ready_left_positions' must match 'left_joint_names' in size.");
    }
    if (right_joint_names_.size() != ready_right_positions_.size())
    {
      throw std::runtime_error(
          "Parameter 'ready_right_positions' must match 'right_joint_names' in size.");
    }

    RCLCPP_INFO(
        get_logger(),
        "yumi_cooperative_controller parameters loaded. Call initialize() to create ROS interfaces.");
  }

  bool YumiCooperativeControllerNode::initialize()
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 订阅上层 task_manager 命令。
    task_command_subscriber_ =
        create_subscription<yumi_interfaces::msg::CooperativeTaskCommand>(
            "cooperative_task_command", 10,
            std::bind(&YumiCooperativeControllerNode::handleTaskCommand, this, std::placeholders::_1));

    // 协同控制层只发布 reference，不直接碰具体控制器 topic。
    joint_reference_publisher_ = create_publisher<yumi_interfaces::msg::JointSpaceReference>(
        "joint_space_reference", 10);
    task_reference_publisher_ = create_publisher<yumi_interfaces::msg::TaskSpaceReference>(
        "task_space_reference", 10);
    controller_status_publisher_ =
        create_publisher<yumi_interfaces::msg::CooperativeControllerStatus>(
            "cooperative_controller_status", 10);

    // 定时器保证 reference 连续输出，便于下层控制器统一跟踪。
    control_timer_ = create_wall_timer(
        std::chrono::duration<double>(control_period_sec_),
        std::bind(&YumiCooperativeControllerNode::runControlLoop, this));
    // 状态定时器是启动握手的关键：没有任务命令时也要持续发布 ready 状态。
    status_timer_ = create_wall_timer(
        std::chrono::duration<double>(0.2),
        std::bind(&YumiCooperativeControllerNode::publishControllerStatus, this));

    // action client 依赖 shared_from_this()，因此放在构造函数外的初始化阶段创建。
    plan_both_arms_poses_client_ =
        rclcpp_action::create_client<PlanBothArmsPoses>(
            shared_from_this(), plan_both_arms_poses_action_name_);
    plan_task_space_trajectory_client_ =
        rclcpp_action::create_client<PlanTaskSpaceTrajectory>(
            shared_from_this(), plan_task_space_trajectory_action_name_);

    if (!plan_both_arms_poses_client_->wait_for_action_server(
            std::chrono::duration<double>(planning_action_server_wait_timeout_sec_)))
    {
      RCLCPP_WARN(
          get_logger(),
          "Joint planning action server '%s' was not available within %.2f seconds.",
          plan_both_arms_poses_action_name_.c_str(),
          planning_action_server_wait_timeout_sec_);
      return false;
    }

    RCLCPP_INFO(
        get_logger(),
        "Connected to joint planning action server '%s'.",
        plan_both_arms_poses_action_name_.c_str());

    if (!plan_task_space_trajectory_client_->wait_for_action_server(
            std::chrono::duration<double>(planning_action_server_wait_timeout_sec_)))
    {
      RCLCPP_WARN(
          get_logger(),
          "Task-space planning action server '%s' was not available within %.2f seconds.",
          plan_task_space_trajectory_action_name_.c_str(),
          planning_action_server_wait_timeout_sec_);
      return false;
    }

    RCLCPP_INFO(
        get_logger(),
        "Connected to task-space planning action server '%s'.",
        plan_task_space_trajectory_action_name_.c_str());

    RCLCPP_INFO(
        get_logger(),
        "yumi_cooperative_controller initialized. It now acts as a pure reference generator.");
    return true;
  }

  void YumiCooperativeControllerNode::handleTaskCommand(
      const yumi_interfaces::msg::CooperativeTaskCommand::SharedPtr msg)
  {
    const bool had_previous_command = has_task_command_;
    const auto previous_command = latest_task_command_;

    // 始终缓存最新任务命令，使 reference 生成始终跟随上层状态机。
    latest_task_command_ = *msg;
    has_task_command_ = true;

    // 自由空间：收到新目标后请求 MoveIt 关节轨迹，结果回来后由 runControlLoop 播放。
    if (isFreeSpaceMode(latest_task_command_) &&
        (!had_previous_command || shouldRequestPlan(previous_command, latest_task_command_)))
    {
      object_space_active_ = false;
      object_grasp_locked_ = false;
      joint_planning_failed_waiting_for_new_command_ = false;
      RCLCPP_INFO(
          get_logger(),
          "Received new free-space command. Request joint trajectory plan: left=(%.3f, %.3f, %.3f), right=(%.3f, %.3f, %.3f).",
          latest_task_command_.left_target_pose.x,
          latest_task_command_.left_target_pose.y,
          latest_task_command_.left_target_pose.z,
          latest_task_command_.right_target_pose.x,
          latest_task_command_.right_target_pose.y,
          latest_task_command_.right_target_pose.z);
      joint_planning_request_pending_ = true;
      requestJointTrajectoryPlan();
    }
    // 物体空间：不再请求 MoveIt，由 runControlLoop 内部根据物体轨迹生成双臂末端参考。
    else if (isObjectSpaceCommand(latest_task_command_) &&
             (!had_previous_command || shouldRequestPlan(previous_command, latest_task_command_)))
    {
      task_space_planning_request_in_flight_ = false;
      has_cached_task_space_trajectory_ = false;
      task_space_trajectory_active_ = false;
      object_space_active_ = true;
      object_grasp_locked_ = false;
      RCLCPP_INFO(
          get_logger(),
          "Received object-space constrained command. Lock grasp relation and move virtual object toward (%.3f, %.3f, %.3f).",
          latest_task_command_.object_target_pose.x,
          latest_task_command_.object_target_pose.y,
          latest_task_command_.object_target_pose.z);
    }
    else if (!isFreeSpaceMode(latest_task_command_) &&
             (!had_previous_command || shouldRequestPlan(previous_command, latest_task_command_)))
    {
      object_space_active_ = false;
      object_grasp_locked_ = false;
      RCLCPP_INFO(
          get_logger(),
          "Received regular constrained command. Request task-space trajectory plan.");
      requestTaskSpaceTrajectoryPlan();
    }
  }

  void YumiCooperativeControllerNode::runControlLoop()
  {
    // 在没有收到 task_manager 命令前，不主动发布任何参考。
    if (!has_task_command_)
    {
      return;
    }

    // 自由空间阶段统一输出关节空间参考：
    // - 先根据 task_manager 给出的 ready/接近等末端位姿请求 MoveIt 关节轨迹
    // - joint trajectory 结果返回后，再按时间采样发布 JointSpaceReference
    // - 规划尚未返回前保持静默，避免直接 fallback 到某组预设关节角
    if (isFreeSpaceMode(latest_task_command_))
    {
      /* 构建一个默认的jointreference */
      auto joint_reference = buildJointSpaceReference();
      /* 如果有缓存好的轨迹就按时间采样发布出去 */
      if (tryBuildJointSpaceReferenceFromCachedTrajectory(&joint_reference))
      {
        joint_reference_publisher_->publish(joint_reference);
        return;
      }
      /* 规划请求已发送 */
      if (joint_planning_request_in_flight_)
      {
        // 如果 action goal 已发送但长期没有 result，说明 planning_interface / MoveIt 侧可能卡住。
        // 清掉 in-flight 并重新置 pending，避免协同层一直静默等待导致机器人不动。
        const auto elapsed = std::chrono::steady_clock::now() -
                             joint_planning_request_wall_start_time_;
        const double elapsed_sec = std::chrono::duration<double>(elapsed).count();
        if (elapsed_sec > joint_planning_result_timeout_sec_)
        {
          if (!joint_planning_cancel_requested_ && active_joint_planning_goal_handle_)
          {
            RCLCPP_WARN(
                get_logger(),
                "Joint planning result timed out after %.2f seconds. Cancel the active PlanBothArmsPoses goal.",
                elapsed_sec);
            plan_both_arms_poses_client_->async_cancel_goal(active_joint_planning_goal_handle_);
            joint_planning_cancel_requested_ = true;
            joint_planning_request_pending_ = false;
          }
          else
          {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Waiting for active joint planning goal cancellation to finish.");
          }
        }
      }
      if (joint_planning_failed_waiting_for_new_command_)
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Free-space planning failed or timed out. Waiting for a new task command instead of retrying the same goal.");
        return;
      }
      if (joint_planning_request_pending_ && !joint_planning_request_in_flight_)
      {
        requestJointTrajectoryPlan();
      }
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Free-space command is active, but no cached joint trajectory is available yet. planning_in_flight=%s, has_cached=%s.",
          joint_planning_request_in_flight_ ? "true" : "false",
          has_cached_joint_trajectories_ ? "true" : "false");
      return;
    }

    // 约束空间当前只保留物体空间控制，由协作层内部生成双臂任务空间参考。
    yumi_interfaces::msg::TaskSpaceReference task_reference;
    if (buildTaskSpaceReference(&task_reference))
    {
      task_reference_publisher_->publish(task_reference);
    }
  }

  bool YumiCooperativeControllerNode::isReadyForTask() const
  {
    if (!plan_both_arms_poses_client_)
    {
      return false;
    }

    return plan_both_arms_poses_client_->action_server_is_ready() &&
           plan_task_space_trajectory_client_ &&
           plan_task_space_trajectory_client_->action_server_is_ready();
  }

  void YumiCooperativeControllerNode::publishControllerStatus()
  {
    yumi_interfaces::msg::CooperativeControllerStatus status;
    status.stamp = now();
    status.joint_planning_action_ready =
        plan_both_arms_poses_client_ &&
        plan_both_arms_poses_client_->action_server_is_ready();
    status.task_space_planning_action_ready =
        plan_task_space_trajectory_client_ &&
        plan_task_space_trajectory_client_->action_server_is_ready();
    status.ready_for_task = isReadyForTask();
    status.planning_in_flight =
        joint_planning_request_in_flight_ || task_space_planning_request_in_flight_;
    status.has_cached_joint_trajectory = has_cached_joint_trajectories_;

    if (!status.joint_planning_action_ready)
    {
      status.message = "waiting for plan_both_arms_poses action server";
    }
    else if (!status.task_space_planning_action_ready)
    {
      status.message = "waiting for plan_task_space_trajectory action server";
    }
    else if (joint_planning_cancel_requested_)
    {
      status.message = "canceling active joint planning goal";
    }
    else if (status.planning_in_flight)
    {
      status.message = "planning request is in flight";
    }
    else if (joint_planning_failed_waiting_for_new_command_)
    {
      status.message = "joint planning timed out; waiting for new task command";
    }
    else
    {
      status.message = "ready for task commands";
    }

    controller_status_publisher_->publish(status);
  }

  bool YumiCooperativeControllerNode::shouldRequestPlan(
      const yumi_interfaces::msg::CooperativeTaskCommand &previous_command,
      const yumi_interfaces::msg::CooperativeTaskCommand &next_command) const
  {
    return previous_command.command_mode != next_command.command_mode ||
           previous_command.motion_space != next_command.motion_space ||
           previous_command.use_object_target != next_command.use_object_target ||
           previous_command.enable_force_control != next_command.enable_force_control ||
           previous_command.target_internal_force != next_command.target_internal_force ||
           previous_command.left_target_pose.x != next_command.left_target_pose.x ||
           previous_command.left_target_pose.y != next_command.left_target_pose.y ||
           previous_command.left_target_pose.z != next_command.left_target_pose.z ||
           previous_command.left_target_pose.roll != next_command.left_target_pose.roll ||
           previous_command.left_target_pose.pitch != next_command.left_target_pose.pitch ||
           previous_command.left_target_pose.yaw != next_command.left_target_pose.yaw ||
           previous_command.right_target_pose.x != next_command.right_target_pose.x ||
           previous_command.right_target_pose.y != next_command.right_target_pose.y ||
           previous_command.right_target_pose.z != next_command.right_target_pose.z ||
           previous_command.right_target_pose.roll != next_command.right_target_pose.roll ||
           previous_command.right_target_pose.pitch != next_command.right_target_pose.pitch ||
           previous_command.right_target_pose.yaw != next_command.right_target_pose.yaw ||
           previous_command.object_target_pose.x != next_command.object_target_pose.x ||
           previous_command.object_target_pose.y != next_command.object_target_pose.y ||
           previous_command.object_target_pose.z != next_command.object_target_pose.z ||
           previous_command.object_target_pose.roll != next_command.object_target_pose.roll ||
           previous_command.object_target_pose.pitch != next_command.object_target_pose.pitch ||
           previous_command.object_target_pose.yaw != next_command.object_target_pose.yaw;
  }

  bool YumiCooperativeControllerNode::isFreeSpaceMode(
      const yumi_interfaces::msg::CooperativeTaskCommand &command) const
  {
    // 自由空间命令：motion_space == SPACE_FREE。
    return command.motion_space ==
           yumi_interfaces::msg::CooperativeTaskCommand::SPACE_FREE;
  }

  bool YumiCooperativeControllerNode::isConstrainedSpaceMode(
      const yumi_interfaces::msg::CooperativeTaskCommand &command) const
  {
    // 普通约束空间命令：SPACE_CONSTRAINED + velocity / effort / hybrid force-position。
    if (command.motion_space !=
        yumi_interfaces::msg::CooperativeTaskCommand::SPACE_CONSTRAINED)
    {
      return false;
    }

    return command.command_mode == yumi_interfaces::msg::CooperativeTaskCommand::MODE_VELOCITY ||
           command.command_mode == yumi_interfaces::msg::CooperativeTaskCommand::MODE_EFFORT ||
           command.command_mode ==
               yumi_interfaces::msg::CooperativeTaskCommand::MODE_HYBRID_FORCE_POSITION;
  }

  bool YumiCooperativeControllerNode::isObjectSpaceCommand(
      const yumi_interfaces::msg::CooperativeTaskCommand &command) const
  {
    // 物体空间命令：约束空间 + 速度模式 + 使用 object_target_pose + 不启用力控。
    return command.motion_space ==
               yumi_interfaces::msg::CooperativeTaskCommand::SPACE_CONSTRAINED &&
           command.command_mode ==
               yumi_interfaces::msg::CooperativeTaskCommand::MODE_VELOCITY &&
           command.use_object_target &&
           !command.enable_force_control;
  }

  YumiCooperativeControllerNode::PlanBothArmsPoses::Goal
  YumiCooperativeControllerNode::buildJointPlanningGoal() const
  {
    PlanBothArmsPoses::Goal goal;
    goal.left_target_pose = latest_task_command_.left_target_pose;
    goal.right_target_pose = latest_task_command_.right_target_pose;
    goal.execute = false;
    return goal;
  }

  YumiCooperativeControllerNode::PlanTaskSpaceTrajectory::Goal
  YumiCooperativeControllerNode::buildTaskSpacePlanningGoal() const
  {
    PlanTaskSpaceTrajectory::Goal goal;
    goal.use_both_arms = true;
    goal.arm = PlanTaskSpaceTrajectory::Goal::ARM_LEFT;
    goal.use_object_target = latest_task_command_.use_object_target;
    goal.enable_force_control = latest_task_command_.enable_force_control;
    goal.left_target_pose = latest_task_command_.left_target_pose;
    goal.right_target_pose = latest_task_command_.right_target_pose;
    goal.object_target_pose = latest_task_command_.object_target_pose;
    goal.target_internal_force = latest_task_command_.target_internal_force;
    goal.reference_frame = "world";
    goal.use_cartesian_path = false;
    goal.planning_time = planning_time_sec_;
    goal.num_planning_attempts = planning_num_attempts_;
    goal.velocity_scaling = planning_velocity_scaling_;
    goal.acceleration_scaling = planning_acceleration_scaling_;
    goal.sample_period = planning_sample_period_sec_;
    goal.cartesian_step = 0.01;
    goal.jump_threshold = 0.0;
    goal.avoid_collisions = true;
    return goal;
  }

  void YumiCooperativeControllerNode::requestJointTrajectoryPlan()
  {
    if (joint_planning_request_in_flight_)
    {
      joint_planning_request_pending_ = true;
      RCLCPP_INFO(
          get_logger(),
          "Joint planning request is already in flight. The latest target will be planned after the current result returns.");
      return;
    }

    if (!plan_both_arms_poses_client_->action_server_is_ready())
    {
      RCLCPP_WARN(
          get_logger(),
          "Joint planning action server '%s' is not ready.",
          plan_both_arms_poses_action_name_.c_str());
      return;
    }

    auto goal = buildJointPlanningGoal();
    typename rclcpp_action::Client<PlanBothArmsPoses>::SendGoalOptions send_goal_options;
    send_goal_options.goal_response_callback =
        std::bind(
            &YumiCooperativeControllerNode::handleJointPlanGoalResponse,
            this,
            std::placeholders::_1);
    send_goal_options.result_callback =
        std::bind(
            &YumiCooperativeControllerNode::handleJointPlanResult,
            this,
            std::placeholders::_1);

    joint_planning_request_in_flight_ = true;
    joint_planning_request_pending_ = false;
    joint_planning_cancel_requested_ = false;
    joint_planning_failed_waiting_for_new_command_ = false;
    active_joint_planning_goal_handle_.reset();
    joint_planning_request_start_time_ = now();
    joint_planning_request_wall_start_time_ = std::chrono::steady_clock::now();
    has_cached_joint_trajectories_ = false;
    joint_trajectory_active_ = false;
    current_left_joint_trajectory_point_index_ = 0U;
    current_right_joint_trajectory_point_index_ = 0U;
    plan_both_arms_poses_client_->async_send_goal(goal, send_goal_options);

    RCLCPP_INFO(
        get_logger(),
        "Sent PlanBothArmsPoses goal to '%s' for free-space joint trajectory planning.",
        plan_both_arms_poses_action_name_.c_str());
  }

  void YumiCooperativeControllerNode::handleJointPlanGoalResponse(
      std::shared_future<GoalHandlePlanBothArmsPoses::SharedPtr> future)
  {
    const auto goal_handle = future.get();
    if (!goal_handle)
    {
      /* 如果goal被拒绝，就取消正在请求规划的标志位 */
      joint_planning_request_in_flight_ = false;
      joint_planning_cancel_requested_ = false;
      joint_planning_failed_waiting_for_new_command_ = true;
      active_joint_planning_goal_handle_.reset();
      RCLCPP_WARN(get_logger(), "Joint planning goal was rejected by the planning interface.");
      return;
    }

    /* 保存此次规划的被接受的goal的handle */
    active_joint_planning_goal_handle_ = goal_handle;
    RCLCPP_INFO(get_logger(), "Joint planning goal accepted.");
  }

  void YumiCooperativeControllerNode::handleJointPlanResult(
      const GoalHandlePlanBothArmsPoses::WrappedResult &result)
  {
    /* 得到result就取消正在请求规划的标志位 */
    joint_planning_request_in_flight_ = false;
    joint_planning_cancel_requested_ = false;
    active_joint_planning_goal_handle_.reset();

    if (result.code == rclcpp_action::ResultCode::CANCELED)
    {
      joint_planning_failed_waiting_for_new_command_ = true;
      RCLCPP_WARN(
          get_logger(),
          "Joint planning goal was canceled after timeout. Waiting for a new task command.");
      return;
    }

    if (joint_planning_failed_waiting_for_new_command_)
    {
      RCLCPP_WARN(
          get_logger(),
          "Ignoring a late joint planning result because this command has already timed out.");
      return;
    }

    if (joint_planning_request_pending_)
    {
      // 当前 result 对应的是上一组目标；收到更新目标后应丢弃旧结果，
      // 立即用 latest_task_command_ 重新请求规划，避免序列最后几组被旧轨迹覆盖。
      joint_planning_request_pending_ = false;
      requestJointTrajectoryPlan();
      return;
    }

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_WARN(
          get_logger(),
          "Joint planning did not succeed. Result code: %d, message: %s",
          static_cast<int>(result.code),
          result.result ? result.result->message.c_str() : "no result message");
      return;
    }

    if (!result.result)
    {
      RCLCPP_WARN(get_logger(), "Joint planning returned a null result pointer.");
      return;
    }

    cached_left_joint_trajectory_ = result.result->left_planned_trajectory;
    cached_right_joint_trajectory_ = result.result->right_planned_trajectory;
    has_cached_joint_trajectories_ =
        !cached_left_joint_trajectory_.points.empty() &&
        !cached_right_joint_trajectory_.points.empty();
    joint_trajectory_active_ = has_cached_joint_trajectories_;
    /* 以当前规划结束的此刻时间为轨迹的起点时刻 */
    joint_trajectory_start_time_ = now();
    current_left_joint_trajectory_point_index_ = 0U;
    current_right_joint_trajectory_point_index_ = 0U;

    if (!joint_trajectory_active_)
    {
      RCLCPP_WARN(
          get_logger(),
          "Joint planning succeeded but returned an empty trajectory. Static fallback joint target will be used.");
      return;
    }

    RCLCPP_INFO(
        get_logger(),
        "Cached free-space joint trajectories. Left points=%zu, right points=%zu.",
        cached_left_joint_trajectory_.points.size(),
        cached_right_joint_trajectory_.points.size());
  }

  void YumiCooperativeControllerNode::requestTaskSpaceTrajectoryPlan()
  {
    if (task_space_planning_request_in_flight_)
    {
      RCLCPP_INFO(
          get_logger(),
          "Task-space planning request is already in flight. Skip sending a new goal.");
      return;
    }

    if (!plan_task_space_trajectory_client_->action_server_is_ready())
    {
      RCLCPP_WARN(
          get_logger(),
          "Task-space planning action server '%s' is not ready.",
          plan_task_space_trajectory_action_name_.c_str());
      return;
    }

    auto goal = buildTaskSpacePlanningGoal();
    typename rclcpp_action::Client<PlanTaskSpaceTrajectory>::SendGoalOptions send_goal_options;
    send_goal_options.goal_response_callback =
        std::bind(
            &YumiCooperativeControllerNode::handleTaskSpacePlanGoalResponse,
            this,
            std::placeholders::_1);
    send_goal_options.result_callback =
        std::bind(
            &YumiCooperativeControllerNode::handleTaskSpacePlanResult,
            this,
            std::placeholders::_1);

    task_space_planning_request_in_flight_ = true;
    has_cached_task_space_trajectory_ = false;
    task_space_trajectory_active_ = false;
    current_task_space_point_index_ = 0U;
    plan_task_space_trajectory_client_->async_send_goal(goal, send_goal_options);

    RCLCPP_INFO(
        get_logger(),
        "Sent PlanTaskSpaceTrajectory goal to '%s'.",
        plan_task_space_trajectory_action_name_.c_str());
  }

  void YumiCooperativeControllerNode::handleTaskSpacePlanGoalResponse(
      std::shared_future<GoalHandlePlanTaskSpaceTrajectory::SharedPtr> future)
  {
    const auto goal_handle = future.get();
    if (!goal_handle)
    {
      task_space_planning_request_in_flight_ = false;
      RCLCPP_WARN(get_logger(), "Task-space planning goal was rejected by the planning interface.");
      return;
    }

    RCLCPP_INFO(get_logger(), "Task-space planning goal accepted.");
  }

  void YumiCooperativeControllerNode::handleTaskSpacePlanResult(
      const GoalHandlePlanTaskSpaceTrajectory::WrappedResult &result)
  {
    task_space_planning_request_in_flight_ = false;

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_WARN(
          get_logger(),
          "Task-space planning did not succeed. Result code: %d, message: %s",
          static_cast<int>(result.code),
          result.result ? result.result->message.c_str() : "no result message");
      return;
    }

    if (!result.result)
    {
      RCLCPP_WARN(get_logger(), "Task-space planning returned a null result pointer.");
      return;
    }

    cached_task_space_trajectory_ = result.result->trajectory;
    has_cached_task_space_trajectory_ = true;
    task_space_trajectory_active_ = !cached_task_space_trajectory_.points.empty();
    task_space_trajectory_start_time_ = now();
    current_task_space_point_index_ = 0U;

    if (!task_space_trajectory_active_)
    {
      RCLCPP_WARN(
          get_logger(),
          "Task-space planning succeeded but returned an empty trajectory. Cooperative controller will stay silent.");
      return;
    }

    RCLCPP_INFO(
        get_logger(),
        "Cached task-space trajectory with %zu points in frame '%s'.",
        cached_task_space_trajectory_.points.size(),
        cached_task_space_trajectory_.reference_frame.c_str());
  }

  yumi_interfaces::msg::JointSpaceReference
  YumiCooperativeControllerNode::buildJointSpaceReference() const
  {
    yumi_interfaces::msg::JointSpaceReference joint_reference;
    joint_reference.stamp = now();
    joint_reference.reference_mode =
        mapJointReferenceMode(latest_task_command_.command_mode);

    // 统一填写左右臂关节名和目标关节位置，便于下层控制器做轨迹跟踪或关节伺服。
    joint_reference.left_joint_names = left_joint_names_;
    joint_reference.right_joint_names = right_joint_names_;
    joint_reference.left_positions = ready_left_positions_;
    joint_reference.right_positions = ready_right_positions_;

    // 当前骨架先不在协同层生成速度/力矩控制量，留给低层控制器实现。
    joint_reference.left_velocities.assign(left_joint_names_.size(), 0.0);
    joint_reference.right_velocities.assign(right_joint_names_.size(), 0.0);
    joint_reference.left_efforts.assign(left_joint_names_.size(), 0.0);
    joint_reference.right_efforts.assign(right_joint_names_.size(), 0.0);

    return joint_reference;
  }

  bool YumiCooperativeControllerNode::tryBuildJointSpaceReferenceFromCachedTrajectory(
      yumi_interfaces::msg::JointSpaceReference *joint_reference)
  {
    if (!joint_reference || !has_cached_joint_trajectories_ ||
        cached_left_joint_trajectory_.points.empty() ||
        cached_right_joint_trajectory_.points.empty())
    {
      return false;
    }

    const double elapsed_sec = (now() - joint_trajectory_start_time_).seconds();

    if (joint_trajectory_active_)
    {
      while (current_left_joint_trajectory_point_index_ + 1U < cached_left_joint_trajectory_.points.size() &&
             durationToSeconds(cached_left_joint_trajectory_.points[current_left_joint_trajectory_point_index_ + 1U].time_from_start) <= elapsed_sec)
      {
        ++current_left_joint_trajectory_point_index_;
      }
      while (current_right_joint_trajectory_point_index_ + 1U < cached_right_joint_trajectory_.points.size() &&
             durationToSeconds(cached_right_joint_trajectory_.points[current_right_joint_trajectory_point_index_ + 1U].time_from_start) <= elapsed_sec)
      {
        ++current_right_joint_trajectory_point_index_;
      }

      const auto left_end_time =
          durationToSeconds(cached_left_joint_trajectory_.points.back().time_from_start);
      const auto right_end_time =
          durationToSeconds(cached_right_joint_trajectory_.points.back().time_from_start);
      if (elapsed_sec >= std::max(left_end_time, right_end_time))
      {
        /* 更新完整的双臂轨迹并置位 */
        current_left_joint_trajectory_point_index_ = cached_left_joint_trajectory_.points.size() - 1U;
        current_right_joint_trajectory_point_index_ = cached_right_joint_trajectory_.points.size() - 1U;
        joint_trajectory_active_ = false;
      }
    }

    joint_reference->stamp = now();
    joint_reference->reference_mode = yumi_interfaces::msg::JointSpaceReference::MODE_VELOCITY;
    joint_reference->left_joint_names = left_joint_names_;
    joint_reference->right_joint_names = right_joint_names_;
    /* 插值构造关节轨迹而不是就近取 */
    joint_reference->left_positions =
        interpolateJointPositions(cached_left_joint_trajectory_, elapsed_sec);
    joint_reference->right_positions =
        interpolateJointPositions(cached_right_joint_trajectory_, elapsed_sec);
    joint_reference->left_velocities.assign(left_joint_names_.size(), 0.0);
    joint_reference->right_velocities.assign(right_joint_names_.size(), 0.0);
    joint_reference->left_efforts.assign(left_joint_names_.size(), 0.0);
    joint_reference->right_efforts.assign(right_joint_names_.size(), 0.0);
    return true;
  }

  bool YumiCooperativeControllerNode::buildTaskSpaceReference(
      yumi_interfaces::msg::TaskSpaceReference *task_reference)
  {
    /* 如果task_reference为空指针就直接返回 */
    if (!task_reference)
    {
      return false;
    }

    if (object_space_active_)
    {
      return buildObjectSpaceReference(task_reference);
    }

    if (tryBuildTaskSpaceReferenceFromCachedTrajectory(task_reference))
    {
      return true;
    }

    // 普通约束空间轨迹规划尚未返回时保持静默，避免提前发送最终目标点。
    return false;
  }

  bool YumiCooperativeControllerNode::tryBuildTaskSpaceReferenceFromCachedTrajectory(
      yumi_interfaces::msg::TaskSpaceReference *task_reference)
  {
    if (!task_reference || !has_cached_task_space_trajectory_ ||
        cached_task_space_trajectory_.points.empty())
    {
      return false;
    }

    const double elapsed_sec = (now() - task_space_trajectory_start_time_).seconds();
    const auto &points = cached_task_space_trajectory_.points;

    if (task_space_trajectory_active_)
    {
      while (current_task_space_point_index_ + 1U < points.size() &&
             durationToSeconds(points[current_task_space_point_index_ + 1U].time_from_start) <= elapsed_sec)
      {
        ++current_task_space_point_index_;
      }

      if (elapsed_sec >= durationToSeconds(points.back().time_from_start))
      {
        current_task_space_point_index_ = points.size() - 1U;
        task_space_trajectory_active_ = false;
      }
    }

    const auto &current_point = points[current_task_space_point_index_];
    task_reference->stamp = now();
    task_reference->reference_mode = mapTaskReferenceMode(latest_task_command_.command_mode);
    task_reference->use_object_target = current_point.use_object_target;
    task_reference->enable_force_control = current_point.enable_force_control;
    task_reference->left_target_pose = current_point.left_target_pose;
    task_reference->right_target_pose = current_point.right_target_pose;
    task_reference->object_target_pose = current_point.object_target_pose;
    task_reference->target_internal_force = current_point.target_internal_force;
    return true;
  }

  bool YumiCooperativeControllerNode::lookupCurrentEndEffectorTransforms(
      tf2::Transform *left_end_effector_transform,
      tf2::Transform *right_end_effector_transform)
  {
    if (!left_end_effector_transform || !right_end_effector_transform)
    {
      return false;
    }

    try
    {
      const auto left_transform = tf_buffer_->lookupTransform(
          object_reference_frame_,
          left_end_effector_frame_,
          tf2::TimePointZero);
      const auto right_transform = tf_buffer_->lookupTransform(
          object_reference_frame_,
          right_end_effector_frame_,
          tf2::TimePointZero);
      *left_end_effector_transform = transformFromStamped(left_transform);
      *right_end_effector_transform = transformFromStamped(right_transform);
      return true;
    }
    catch (const tf2::TransformException &exception)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Cannot lookup current end-effector transforms in '%s': %s. Fall back to command poses.",
          object_reference_frame_.c_str(),
          exception.what());
      *left_end_effector_transform = transformFromPose6D(latest_task_command_.left_target_pose);
      *right_end_effector_transform = transformFromPose6D(latest_task_command_.right_target_pose);
      return true;
    }
  }

  bool YumiCooperativeControllerNode::initializeObjectSpaceTrajectory()
  {
    tf2::Transform left_end_effector_pose;
    tf2::Transform right_end_effector_pose;
    if (!lookupCurrentEndEffectorTransforms(&left_end_effector_pose, &right_end_effector_pose))
    {
      return false;
    }

    const tf2::Vector3 object_start_position =
        (left_end_effector_pose.getOrigin() + right_end_effector_pose.getOrigin()) * 0.5;
    const tf2::Quaternion object_orientation =
        transformFromPose6D(latest_task_command_.object_target_pose).getRotation().normalized();

    // 第一版只规划 object 位置，姿态在整段轨迹中保持初始姿态不变。
    object_trajectory_start_pose_ =
        tf2::Transform(object_orientation, object_start_position);
    object_trajectory_goal_pose_ =
        tf2::Transform(
            object_orientation,
            tf2::Vector3(
                latest_task_command_.object_target_pose.x,
                latest_task_command_.object_target_pose.y,
                latest_task_command_.object_target_pose.z));

    object_to_left_grasp_ =
        object_trajectory_start_pose_.inverse() * left_end_effector_pose;
    object_to_right_grasp_ =
        object_trajectory_start_pose_.inverse() * right_end_effector_pose;
    object_trajectory_start_time_ = now();
    object_grasp_locked_ = true;

    RCLCPP_INFO(
        get_logger(),
        "Locked object grasp relation. object_start=(%.3f, %.3f, %.3f), object_goal=(%.3f, %.3f, %.3f), duration=%.2f s.",
        object_trajectory_start_pose_.getOrigin().x(),
        object_trajectory_start_pose_.getOrigin().y(),
        object_trajectory_start_pose_.getOrigin().z(),
        object_trajectory_goal_pose_.getOrigin().x(),
        object_trajectory_goal_pose_.getOrigin().y(),
        object_trajectory_goal_pose_.getOrigin().z(),
        object_trajectory_duration_sec_);
    return true;
  }

  bool YumiCooperativeControllerNode::buildObjectSpaceReference(
      yumi_interfaces::msg::TaskSpaceReference *task_reference)
  {
    if (!task_reference)
    {
      return false;
    }
    /* 如果齐次变换矩阵未求得且初始化（求齐次变换矩阵）失败就直接返回 */
    if (!object_grasp_locked_ && !initializeObjectSpaceTrajectory())
    {
      return false;
    }

    const double elapsed_sec = (now() - object_trajectory_start_time_).seconds();
    const double alpha =
        std::clamp(elapsed_sec / std::max(object_trajectory_duration_sec_, 1e-6), 0.0, 1.0);
    const double smooth_ratio = 0.5 - 0.5 * std::cos(M_PI * alpha);

    const tf2::Vector3 start_position = object_trajectory_start_pose_.getOrigin();
    const tf2::Vector3 goal_position = object_trajectory_goal_pose_.getOrigin();
    const tf2::Vector3 object_position =
        start_position + (goal_position - start_position) * smooth_ratio;
    const tf2::Transform object_pose(
        object_trajectory_start_pose_.getRotation(),
        object_position);
    const tf2::Transform left_end_effector_pose = object_pose * object_to_left_grasp_;
    const tf2::Transform right_end_effector_pose = object_pose * object_to_right_grasp_;

    task_reference->stamp = now();
    task_reference->reference_mode = yumi_interfaces::msg::TaskSpaceReference::MODE_VELOCITY;
    task_reference->use_object_target = true;
    task_reference->enable_force_control = false;
    task_reference->object_target_pose = taskSpacePoseFromTransform(object_pose);
    task_reference->left_target_pose = taskSpacePoseFromTransform(left_end_effector_pose);
    task_reference->right_target_pose = taskSpacePoseFromTransform(right_end_effector_pose);
    task_reference->target_internal_force = latest_task_command_.target_internal_force;
    return true;
  }

  double YumiCooperativeControllerNode::durationToSeconds(
      const builtin_interfaces::msg::Duration &duration) const
  {
    return static_cast<double>(duration.sec) +
           static_cast<double>(duration.nanosec) * 1e-9;
  }

  std::vector<double> YumiCooperativeControllerNode::interpolateJointPositions(
      const trajectory_msgs::msg::JointTrajectory &trajectory,
      double query_time_sec) const
  {
    if (trajectory.points.empty())
    {
      return {};
    }
    if (trajectory.points.size() == 1U)
    {
      return trajectory.points.front().positions;
    }

    /* 如果过去的时间比第一个轨迹点还早就直接返回第一个轨迹点 */
    if (query_time_sec <= durationToSeconds(trajectory.points.front().time_from_start))
    {
      return trajectory.points.front().positions;
    }
    /* 如果过去的时间比最后一个轨迹点还晚就直接返回最后一个轨迹点 */
    if (query_time_sec >= durationToSeconds(trajectory.points.back().time_from_start))
    {
      return trajectory.points.back().positions;
    }

    for (std::size_t index = 0; index + 1U < trajectory.points.size(); ++index)
    {
      const auto &left_point = trajectory.points[index];
      const auto &right_point = trajectory.points[index + 1U];
      const double left_time_sec = durationToSeconds(left_point.time_from_start);
      const double right_time_sec = durationToSeconds(right_point.time_from_start);
      if (query_time_sec < left_time_sec || query_time_sec > right_time_sec)
      {
        continue;
      }

      const double duration_sec = right_time_sec - left_time_sec;
      if (duration_sec <= 1e-9)
      {
        return left_point.positions;
      }

      const double ratio = (query_time_sec - left_time_sec) / duration_sec;
      std::vector<double> interpolated_positions(left_point.positions.size(), 0.0);
      for (std::size_t joint_index = 0; joint_index < left_point.positions.size() &&
                                        joint_index < right_point.positions.size();
           ++joint_index)
      {
        interpolated_positions[joint_index] =
            left_point.positions[joint_index] +
            ratio * (right_point.positions[joint_index] - left_point.positions[joint_index]);
      }
      return interpolated_positions;
    }

    return trajectory.points.back().positions;
  }

  uint8_t YumiCooperativeControllerNode::mapJointReferenceMode(uint8_t command_mode) const
  {
    // trajectory / velocity / effort 三类关节参考都用同一消息表达，具体解释交给下层。
    if (command_mode == yumi_interfaces::msg::CooperativeTaskCommand::MODE_TRAJECTORY)
    {
      return yumi_interfaces::msg::JointSpaceReference::MODE_TRAJECTORY;
    }
    if (command_mode == yumi_interfaces::msg::CooperativeTaskCommand::MODE_VELOCITY)
    {
      return yumi_interfaces::msg::JointSpaceReference::MODE_VELOCITY;
    }
    if (command_mode == yumi_interfaces::msg::CooperativeTaskCommand::MODE_EFFORT)
    {
      return yumi_interfaces::msg::JointSpaceReference::MODE_EFFORT;
    }

    return yumi_interfaces::msg::JointSpaceReference::MODE_UNSPECIFIED;
  }

  uint8_t YumiCooperativeControllerNode::mapTaskReferenceMode(uint8_t command_mode) const
  {
    // 任务空间参考只对应 velocity / effort / hybrid force-position 三类工作模式。
    if (command_mode == yumi_interfaces::msg::CooperativeTaskCommand::MODE_VELOCITY)
    {
      return yumi_interfaces::msg::TaskSpaceReference::MODE_VELOCITY;
    }
    if (command_mode == yumi_interfaces::msg::CooperativeTaskCommand::MODE_EFFORT)
    {
      return yumi_interfaces::msg::TaskSpaceReference::MODE_EFFORT;
    }
    if (command_mode ==
        yumi_interfaces::msg::CooperativeTaskCommand::MODE_HYBRID_FORCE_POSITION)
    {
      return yumi_interfaces::msg::TaskSpaceReference::MODE_HYBRID_FORCE_POSITION;
    }

    return yumi_interfaces::msg::TaskSpaceReference::MODE_UNSPECIFIED;
  }

} // namespace yumi_cooperative_controller
