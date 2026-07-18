#include "yumi_low_level_controllers/yumi_low_level_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace yumi_low_level_controllers
{

  namespace
  {

    constexpr double kDampedLeastSquaresLambda = 1e-4;
    constexpr double kReferenceComparisonTolerance = 1e-9;

    double smoothingFactor(const double cutoff, const double delta_time_sec)
    {
      if (delta_time_sec <= 1e-9)
      {
        return 1.0;
      }

      const double tau = 1.0 / (2.0 * M_PI * std::max(cutoff, 1e-9));
      return 1.0 / (1.0 + tau / delta_time_sec);
    }

    bool nearlyEqual(const double lhs, const double rhs)
    {
      return std::abs(lhs - rhs) <= kReferenceComparisonTolerance;
    }

    bool sameTaskSpacePose(
        const yumi_interfaces::msg::TaskSpacePose &lhs,
        const yumi_interfaces::msg::TaskSpacePose &rhs)
    {
      return nearlyEqual(lhs.x, rhs.x) &&
             nearlyEqual(lhs.y, rhs.y) &&
             nearlyEqual(lhs.z, rhs.z) &&
             nearlyEqual(lhs.qx, rhs.qx) &&
             nearlyEqual(lhs.qy, rhs.qy) &&
             nearlyEqual(lhs.qz, rhs.qz) &&
             nearlyEqual(lhs.qw, rhs.qw);
    }

    bool sameJointVector(
        const std::vector<double> &lhs,
        const std::vector<double> &rhs)
    {
      if (lhs.size() != rhs.size())
      {
        return false;
      }
      for (std::size_t index = 0; index < lhs.size(); ++index)
      {
        if (!nearlyEqual(lhs[index], rhs[index]))
        {
          return false;
        }
      }
      return true;
    }

    bool sameJointSpaceReference(
        const yumi_interfaces::msg::JointSpaceReference &lhs,
        const yumi_interfaces::msg::JointSpaceReference &rhs)
    {
      return lhs.reference_mode == rhs.reference_mode &&
             sameJointVector(lhs.left_positions, rhs.left_positions) &&
             sameJointVector(lhs.right_positions, rhs.right_positions);
    }

    bool sameTaskSpaceReference(
        const yumi_interfaces::msg::TaskSpaceReference &lhs,
        const yumi_interfaces::msg::TaskSpaceReference &rhs)
    {
      return lhs.reference_mode == rhs.reference_mode &&
             lhs.use_object_target == rhs.use_object_target &&
             lhs.enable_force_control == rhs.enable_force_control &&
             nearlyEqual(lhs.target_internal_force, rhs.target_internal_force) &&
             sameTaskSpacePose(lhs.left_target_pose, rhs.left_target_pose) &&
             sameTaskSpacePose(lhs.right_target_pose, rhs.right_target_pose) &&
             sameTaskSpacePose(lhs.object_target_pose, rhs.object_target_pose);
    }

  } // namespace

  YumiLowLevelControllerNode::YumiLowLevelControllerNode()
      : Node("yumi_low_level_controller")
  {
    std::string default_urdf_path;
    // 优先从 yumi_description 包中给出一个稳定默认 URDF 路径。
    // 这样 launch 端即使不显式传参，低层控制器也能独立加载运动学模型。
    try
    {
      default_urdf_path =
          ament_index_cpp::get_package_share_directory("yumi_description") + "/urdf/yumi.urdf";
    }
    catch (const std::exception &exception)
    {
      RCLCPP_WARN(
          get_logger(),
          "Failed to resolve default YuMi URDF path from yumi_description: %s",
          exception.what());
      default_urdf_path.clear();
    }

    declare_parameter<std::vector<std::string>>(
        "left_joint_names",
        {"yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l", "yumi_joint_3_l",
         "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l"});
    declare_parameter<std::vector<std::string>>(
        "right_joint_names",
        {"yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r", "yumi_joint_3_r",
         "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r"});
    declare_parameter<std::vector<double>>(
        "left_nullspace_posture_target_positions",
        {0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>(
        "right_nullspace_posture_target_positions",
        {0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>(
        "nullspace_symmetry_signs",
        {-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0});

    declare_parameter<double>("control_period_sec", 0.01);
    declare_parameter<std::string>("urdf_path", default_urdf_path);
    declare_parameter<std::string>("left_end_effector_frame", "gripper_l_base");
    declare_parameter<std::string>("right_end_effector_frame", "gripper_r_base");
    declare_parameter<double>("joint_position_gain", 1.0);
    declare_parameter<double>("max_joint_velocity", 0.5);
    declare_parameter<double>("joint_goal_tolerance", 0.02);
    declare_parameter<double>("task_space_position_tolerance", 0.01);
    declare_parameter<double>("task_space_orientation_tolerance", 0.05);
    declare_parameter<double>("goal_reached_reference_stable_time_sec", 0.2);
    declare_parameter<double>("task_space_position_gain", 1.0);
    declare_parameter<double>("task_space_orientation_gain", 1.0);
    declare_parameter<double>("task_space_position_damping_gain", 0.0);
    declare_parameter<double>("task_space_orientation_damping_gain", 0.0);
    declare_parameter<double>("nullspace_posture_gain", 0.0);
    declare_parameter<double>("nullspace_symmetry_gain", 0.0);
    declare_parameter<double>("one_euro_min_cutoff", 1.0);
    declare_parameter<double>("one_euro_beta", 0.0);
    declare_parameter<double>("one_euro_d_cutoff", 1.0);
    declare_parameter<std::string>("controller_type", "velocity");

    left_joint_names_ = get_parameter("left_joint_names").as_string_array();
    right_joint_names_ = get_parameter("right_joint_names").as_string_array();
    left_nullspace_posture_target_positions_ =
        get_parameter("left_nullspace_posture_target_positions").as_double_array();
    right_nullspace_posture_target_positions_ =
        get_parameter("right_nullspace_posture_target_positions").as_double_array();
    nullspace_symmetry_signs_ = get_parameter("nullspace_symmetry_signs").as_double_array();
    control_period_sec_ = get_parameter("control_period_sec").as_double();
    urdf_path_ = get_parameter("urdf_path").as_string();
    left_end_effector_frame_ = get_parameter("left_end_effector_frame").as_string();
    right_end_effector_frame_ = get_parameter("right_end_effector_frame").as_string();
    joint_position_gain_ = get_parameter("joint_position_gain").as_double();
    max_joint_velocity_ = get_parameter("max_joint_velocity").as_double();
    joint_goal_tolerance_ = get_parameter("joint_goal_tolerance").as_double();
    task_space_position_tolerance_ = get_parameter("task_space_position_tolerance").as_double();
    task_space_orientation_tolerance_ = get_parameter("task_space_orientation_tolerance").as_double();
    goal_reached_reference_stable_time_sec_ =
        get_parameter("goal_reached_reference_stable_time_sec").as_double();
    task_space_position_gain_ = get_parameter("task_space_position_gain").as_double();
    task_space_orientation_gain_ = get_parameter("task_space_orientation_gain").as_double();
    task_space_position_damping_gain_ =
        get_parameter("task_space_position_damping_gain").as_double();
    task_space_orientation_damping_gain_ =
        get_parameter("task_space_orientation_damping_gain").as_double();
    nullspace_posture_gain_ = get_parameter("nullspace_posture_gain").as_double();
    nullspace_symmetry_gain_ = get_parameter("nullspace_symmetry_gain").as_double();
    one_euro_min_cutoff_ = get_parameter("one_euro_min_cutoff").as_double();
    one_euro_beta_ = get_parameter("one_euro_beta").as_double();
    one_euro_d_cutoff_ = get_parameter("one_euro_d_cutoff").as_double();
    controller_type_name_ = get_parameter("controller_type").as_string();
    controller_type_ = parseControllerType(controller_type_name_);

    if (left_joint_names_.empty() || right_joint_names_.empty())
    {
      throw std::runtime_error("Low-level controller requires non-empty left/right joint name lists.");
    }
    if (left_joint_names_.size() != left_nullspace_posture_target_positions_.size() ||
        right_joint_names_.size() != right_nullspace_posture_target_positions_.size())
    {
      throw std::runtime_error(
          "Nullspace posture target sizes must match left/right joint name list sizes.");
    }
    if (left_joint_names_.size() != nullspace_symmetry_signs_.size() ||
        right_joint_names_.size() != nullspace_symmetry_signs_.size())
    {
      throw std::runtime_error(
          "Parameter 'nullspace_symmetry_signs' must match left/right joint name list sizes.");
    }
    if (urdf_path_.empty())
    {
      throw std::runtime_error("Parameter 'urdf_path' must not be empty.");
    }
    if (left_end_effector_frame_.empty() || right_end_effector_frame_.empty())
    {
      throw std::runtime_error(
          "Parameters 'left_end_effector_frame' and 'right_end_effector_frame' must not be empty.");
    }

    RCLCPP_INFO(
        get_logger(),
        "yumi_low_level_controller parameters loaded. Call initialize() to create ROS interfaces.");
  }

  bool YumiLowLevelControllerNode::initialize()
  {
    if (!initializePinocchioModel())
    {
      return false;
    }

    joint_space_reference_subscriber_ =
        create_subscription<yumi_interfaces::msg::JointSpaceReference>(
            "joint_space_reference", 10,
            std::bind(&YumiLowLevelControllerNode::handleJointSpaceReference, this, std::placeholders::_1));
    task_space_reference_subscriber_ =
        create_subscription<yumi_interfaces::msg::TaskSpaceReference>(
            "task_space_reference", 10,
            std::bind(&YumiLowLevelControllerNode::handleTaskSpaceReference, this, std::placeholders::_1));
    joint_states_subscriber_ =
        create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 50,
            std::bind(&YumiLowLevelControllerNode::handleJointStates, this, std::placeholders::_1));

    left_velocity_command_publisher_ =
        create_publisher<std_msgs::msg::Float64MultiArray>(
            "/left_arm_velocity_controller/commands", 10);
    right_velocity_command_publisher_ =
        create_publisher<std_msgs::msg::Float64MultiArray>(
            "/right_arm_velocity_controller/commands", 10);
    controller_status_publisher_ =
        create_publisher<yumi_interfaces::msg::LowLevelControllerStatus>(
            "low_level_controller_status", 10);
    // 低层反馈话题用于把“执行是否到位”的摘要结果往上送。
    // 这样 task_manager 不必理解底层控制细节，也能根据反馈做状态切换。
    controller_error_debug_publisher_ =
        create_publisher<std_msgs::msg::Float64MultiArray>(
            "low_level_controller_error_debug", 10);
    // debug 话题每次误差计算都会发，不影响 goal_reached 的单次事件语义。
    // 这样调试时可以持续观察误差，而 task_manager 仍只消费 status 完成事件。

    control_timer_ = create_wall_timer(
        std::chrono::duration<double>(control_period_sec_),
        std::bind(&YumiLowLevelControllerNode::runControlLoop, this));

    RCLCPP_INFO(
        get_logger(),
        "yumi_low_level_controller initialized. It subscribes unified references and publishes velocity commands.");
    return true;
  }

  void YumiLowLevelControllerNode::handleJointSpaceReference(
      const yumi_interfaces::msg::JointSpaceReference::SharedPtr msg)
  {
    // joint-space reference 到来后，低层控制器记住：
    // 1. 已经有上层参考
    // 2. 当前激活参考类型是 joint-space
    const bool reference_changed =
        !has_upper_reference_ ||
        active_reference_type_ != ActiveReferenceType::JOINT_SPACE ||
        !sameJointSpaceReference(latest_joint_space_reference_, *msg);

    latest_joint_space_reference_ = *msg;
    has_upper_reference_ = true;
    active_reference_type_ = ActiveReferenceType::JOINT_SPACE;
    // joint-space 参考会打断 task-space 轨迹跟踪，因此需要清空上一次 task-space
    // 参考速度差分状态，避免下次重新进入 task-space 控制时沿用旧速度。
    has_previous_task_space_reference_ = false;
    latest_left_reference_task_space_velocity_.setZero();
    latest_right_reference_task_space_velocity_.setZero();
    if (reference_changed)
    {
      // 只有参考内容真的变化时，才允许下一次“到位事件”重新上报。
      // 如果上层只是周期重发同一个目标，低层仍然会继续控制，但不会反复报告已完成。
      goal_reached_reported_ = false;
      latest_reference_change_time_ = now();
    }
  }

  void YumiLowLevelControllerNode::handleTaskSpaceReference(
      const yumi_interfaces::msg::TaskSpaceReference::SharedPtr msg)
  {
    // task-space reference 到来后，同样刷新“当前激活参考类型”。
    // 后续控制循环会据此走到 task-space 控制分支。
    const bool reference_changed =
        !has_upper_reference_ ||
        active_reference_type_ != ActiveReferenceType::TASK_SPACE ||
        !sameTaskSpaceReference(latest_task_space_reference_, *msg);

    const auto receive_time = now();
    if (has_previous_task_space_reference_)
    {
      const double delta_time_sec =
          (receive_time - previous_task_space_reference_receive_time_).seconds();
      if (delta_time_sec > 1e-9)
      {
        latest_left_reference_task_space_velocity_ =
            computeReferenceTaskSpaceVelocity(
                previous_task_space_reference_.left_target_pose,
                msg->left_target_pose,
                delta_time_sec);
        latest_right_reference_task_space_velocity_ =
            computeReferenceTaskSpaceVelocity(
                previous_task_space_reference_.right_target_pose,
                msg->right_target_pose,
                delta_time_sec);
        latest_left_reference_task_space_velocity_ =
            filterReferenceTaskSpaceVelocity(
                latest_left_reference_task_space_velocity_,
                receive_time,
                &left_reference_velocity_filter_state_);
        latest_right_reference_task_space_velocity_ =
            filterReferenceTaskSpaceVelocity(
                latest_right_reference_task_space_velocity_,
                receive_time,
                &right_reference_velocity_filter_state_);
      }
      else
      {
        latest_left_reference_task_space_velocity_.setZero();
        latest_right_reference_task_space_velocity_.setZero();
      }
    }
    else
    {
      latest_left_reference_task_space_velocity_.setZero();
      latest_right_reference_task_space_velocity_.setZero();
      left_reference_velocity_filter_state_.initialized = false;
      right_reference_velocity_filter_state_.initialized = false;
    }

    latest_task_space_reference_ = *msg;
    has_upper_reference_ = true;
    active_reference_type_ = ActiveReferenceType::TASK_SPACE;
    previous_task_space_reference_ = *msg;
    previous_task_space_reference_receive_time_ = receive_time;
    has_previous_task_space_reference_ = true;
    if (reference_changed)
    {
      // 同样地，task-space 参考只有在目标真变化时才会解除“已上报”锁存。
      goal_reached_reported_ = false;
      latest_reference_change_time_ = receive_time;
    }
  }

  void YumiLowLevelControllerNode::handleJointStates(
      const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // 这里先按名字缓存最新 joint position，后续控制循环再按统一顺序取出。
    // 这样可以把 joint_states 原始顺序与控制器内部顺序解耦。
    latest_joint_positions_.clear();
    latest_joint_velocities_.clear();
    for (size_t index = 0; index < msg->name.size() && index < msg->position.size(); ++index)
    {
      latest_joint_positions_[msg->name[index]] = msg->position[index];
    }
    for (size_t index = 0; index < msg->name.size() && index < msg->velocity.size(); ++index)
    {
      latest_joint_velocities_[msg->name[index]] = msg->velocity[index];
    }
    has_joint_states_ = true;
  }

  void YumiLowLevelControllerNode::runControlLoop()
  {
    // 没有机器人状态就无法做任何闭环控制。
    if (!has_joint_states_)
    {
      return;
    }
    // 没有上层参考则说明当前还没有可跟踪的目标。
    if (!has_upper_reference_)
    {
      return;
    }

    // 第一层分派：先按“底层控制器类型”选大类。
    // 例如 velocity / effort / trajectory 三类控制链的输出接口完全不同，
    // 因此先在这里分流，再进入更细的 reference 类型分支。
    switch (controller_type_)
    {
    case ControllerType::TRAJECTORY:
      runTrajectoryControl();
      return;
    case ControllerType::VELOCITY:
      runVelocityControl();
      return;
    case ControllerType::EFFORT:
      runEffortControl();
      return;
    case ControllerType::UNSPECIFIED:
    default:
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Controller type is UNSPECIFIED. No low-level command will be sent.");
      return;
    }
  }

  void YumiLowLevelControllerNode::runTrajectoryControl()
  {
    // 轨迹控制大类后续可以接 joint trajectory action 或内部时间参数化跟踪器。
    // 当前先把模式入口立住，方便后续补完整控制链。
    switch (active_reference_type_)
    {
    case ActiveReferenceType::JOINT_SPACE:
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Trajectory controller branch is not implemented yet for joint-space references.");
      return;
    case ActiveReferenceType::TASK_SPACE:
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Trajectory controller branch is not implemented yet for task-space references.");
      return;
    case ActiveReferenceType::UNSPECIFIED:
    default:
      return;
    }
  }

  void YumiLowLevelControllerNode::runVelocityControl()
  {
    // 第二层分派：在 velocity 控制链内部，再看当前激活参考是 joint-space 还是 task-space。
    switch (active_reference_type_)
    {
    case ActiveReferenceType::JOINT_SPACE:
    {
      std::vector<double> left_joint_positions;
      std::vector<double> right_joint_positions;
      if (!tryBuildOrderedJointPositions(&left_joint_positions, &right_joint_positions))
      {
        return;
      }

      if (latest_joint_space_reference_.reference_mode ==
              yumi_interfaces::msg::JointSpaceReference::MODE_TRAJECTORY ||
          latest_joint_space_reference_.reference_mode ==
              yumi_interfaces::msg::JointSpaceReference::MODE_VELOCITY)
      {
        // 当前最先落地的是“关节位置误差 -> 关节速度命令”的最小闭环，
        // 用于先把 reference -> velocity command -> Gazebo 运动这条链打通。
        /* 计算关节速度输出 */
        const auto left_joint_velocities =
            computeJointVelocityCommand(left_joint_positions, latest_joint_space_reference_.left_positions);
        const auto right_joint_velocities =
            computeJointVelocityCommand(right_joint_positions, latest_joint_space_reference_.right_positions);
        double max_joint_position_error = 0.0;
        /* 计算最大关节位置误差 */
        for (size_t index = 0; index < left_joint_positions.size() &&
                               index < latest_joint_space_reference_.left_positions.size();
             ++index)
        {
          max_joint_position_error = std::max(
              max_joint_position_error,
              std::abs(latest_joint_space_reference_.left_positions[index] - left_joint_positions[index]));
        }
        for (size_t index = 0; index < right_joint_positions.size() &&
                               index < latest_joint_space_reference_.right_positions.size();
             ++index)
        {
          max_joint_position_error = std::max(
              max_joint_position_error,
              std::abs(latest_joint_space_reference_.right_positions[index] - right_joint_positions[index]));
        }
        publishVelocityCommands(left_joint_velocities, right_joint_velocities);
        // joint-space 任务下，反馈链主要关心“最大关节位置误差”。
        // 这能直接用于判断初始化回 home 或关节保持任务是否基本完成。
        publishControllerStatus(max_joint_position_error, 0.0, 0.0);
        return;
      }

      if (latest_joint_space_reference_.reference_mode ==
          yumi_interfaces::msg::JointSpaceReference::MODE_EFFORT)
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Received joint-space effort reference while controller_type=velocity. Command is ignored.");
        return;
      }

      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Joint-space reference mode is unsupported by the velocity control branch.");
      return;
    }
    case ActiveReferenceType::TASK_SPACE:
    {
      std::vector<double> left_joint_positions;
      std::vector<double> right_joint_positions;
      std::vector<double> left_joint_state_velocities;
      std::vector<double> right_joint_state_velocities;
      if (latest_task_space_reference_.reference_mode !=
          yumi_interfaces::msg::TaskSpaceReference::MODE_VELOCITY)
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Velocity controller currently only handles task-space references in MODE_VELOCITY.");
        return;
      }

      // 任务空间 velocity 控制的第一块基础设施，是先把当前末端位姿和 Jacobian 算出来。
      // 真正的 task-space PD / impedance 下一步会在这个运动学底座上实现。
      if (!has_pinocchio_model_)
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Pinocchio model is not available, so task-space velocity control cannot run.");
        return;
      }
      if (!tryBuildOrderedJointPositions(&left_joint_positions, &right_joint_positions))
      {
        return;
      }
      if (!tryBuildOrderedJointVelocities(&left_joint_state_velocities, &right_joint_state_velocities))
      {
        return;
      }

      Eigen::VectorXd configuration;
      if (!buildPinocchioConfiguration(&configuration))
      {
        return;
      }

      pinocchio::SE3 left_end_effector_pose;
      pinocchio::SE3 right_end_effector_pose;
      Eigen::MatrixXd left_jacobian;
      Eigen::MatrixXd right_jacobian;
      if (!computeTaskSpaceKinematics(
              configuration,
              &left_end_effector_pose,
              &right_end_effector_pose,
              &left_jacobian,
              &right_jacobian))
      {
        return;
      }

      // 先从整机 Jacobian 中抽出左右臂各自的子 Jacobian。
      // 这样后续既可以求当前末端速度，也可以做单臂阻尼伪逆解算。
      const auto left_arm_jacobian = extractArmJacobian(left_jacobian, left_joint_names_);
      const auto right_arm_jacobian = extractArmJacobian(right_jacobian, right_joint_names_);
      if (left_arm_jacobian.cols() != static_cast<Eigen::Index>(left_joint_names_.size()) ||
          right_arm_jacobian.cols() != static_cast<Eigen::Index>(right_joint_names_.size()))
      {
        return;
      }

      // 当前末端速度通过 J(q) * qdot 计算得到。
      // 这里使用 Gazebo / joint_states 提供的关节速度，而不是对位姿做数值差分。
      const Eigen::Map<const Eigen::VectorXd> left_joint_velocity_vector(
          left_joint_state_velocities.data(),
          static_cast<Eigen::Index>(left_joint_state_velocities.size()));
      const Eigen::Map<const Eigen::VectorXd> right_joint_velocity_vector(
          right_joint_state_velocities.data(),
          static_cast<Eigen::Index>(right_joint_state_velocities.size()));
      const Eigen::Matrix<double, 6, 1> left_current_task_space_velocity =
          left_arm_jacobian * left_joint_velocity_vector;
      const Eigen::Matrix<double, 6, 1> right_current_task_space_velocity =
          right_arm_jacobian * right_joint_velocity_vector;

      // 任务空间速度控制现在使用完整的一阶轨迹跟踪形式：
      // v_cmd = v_ref + Kp * e_x + Kd * (v_ref - v_current)
      // 其中 v_ref 由相邻参考点差分近似得到，v_current 由 J(q)qdot 得到。
      const auto left_task_space_velocity =
          computeTaskSpaceVelocityCommand(
              left_end_effector_pose,
              left_current_task_space_velocity,
              latest_left_reference_task_space_velocity_,
              latest_task_space_reference_.left_target_pose);
      const auto right_task_space_velocity =
          computeTaskSpaceVelocityCommand(
              right_end_effector_pose,
              right_current_task_space_velocity,
              latest_right_reference_task_space_velocity_,
              latest_task_space_reference_.right_target_pose);

      // 根据镜像符号构造对侧关节映射后的 symmetry target。
      // 若希望 q_left[j] ~= sign[j] * q_right[j]，则左臂目标来自 sign * q_right，
      // 右臂目标来自 sign * q_left。该项后续只会投影到各自 Jacobian 零空间中。
      std::vector<double> left_symmetry_target_positions(left_joint_positions.size(), 0.0);
      std::vector<double> right_symmetry_target_positions(right_joint_positions.size(), 0.0);
      for (std::size_t index = 0; index < nullspace_symmetry_signs_.size() &&
                                index < left_joint_positions.size() &&
                                index < right_joint_positions.size();
           ++index)
      {
        left_symmetry_target_positions[index] =
            nullspace_symmetry_signs_[index] * right_joint_positions[index];
        right_symmetry_target_positions[index] =
            nullspace_symmetry_signs_[index] * left_joint_positions[index];
      }

      const auto left_joint_velocities =
          computeArmJointVelocityFromTaskSpaceCommand(
              left_arm_jacobian,
              left_task_space_velocity,
              left_joint_positions,
              left_nullspace_posture_target_positions_,
              left_symmetry_target_positions);
      const auto right_joint_velocities =
          computeArmJointVelocityFromTaskSpaceCommand(
              right_arm_jacobian,
              right_task_space_velocity,
              right_joint_positions,
              right_nullspace_posture_target_positions_,
              right_symmetry_target_positions);

      // cooperative_controller 已经把 object target 展开成左右末端位姿。
      // low-level 这里只跟踪 left/right target；真正的力控语义后续再单独实现。
      if (latest_task_space_reference_.enable_force_control)
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Force-control semantics are not implemented in the velocity branch yet.");
      }

      publishVelocityCommands(left_joint_velocities, right_joint_velocities);
      // task-space 任务下，反馈链改为关注末端误差：
      // - 平移误差范数
      // - 姿态误差范数
      // 这样 task_manager 看到的是“末端是否到位”，而不是一串底层关节误差。
      const auto left_pose_error =
          computeTaskSpacePoseError(
              left_end_effector_pose,
              latest_task_space_reference_.left_target_pose);
      const auto right_pose_error =
          computeTaskSpacePoseError(
              right_end_effector_pose,
              latest_task_space_reference_.right_target_pose);
      const double left_position_error_norm = left_pose_error.head<3>().norm();
      const double right_position_error_norm = right_pose_error.head<3>().norm();
      const double left_orientation_error_norm = left_pose_error.tail<3>().norm();
      const double right_orientation_error_norm = right_pose_error.tail<3>().norm();
      publishControllerStatus(
          0.0,
          std::max(left_position_error_norm, right_position_error_norm),
          std::max(left_orientation_error_norm, right_orientation_error_norm));
      return;
    }
    case ActiveReferenceType::UNSPECIFIED:
    default:
      return;
    }
  }

  void YumiLowLevelControllerNode::runEffortControl()
  {
    // effort 控制链后续会接更强的动力学控制能力。
    switch (active_reference_type_)
    {
    case ActiveReferenceType::JOINT_SPACE:
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Joint-space effort control branch is not implemented yet.");
      return;
    case ActiveReferenceType::TASK_SPACE:
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Task-space effort/impedance/admittance branch is not implemented yet.");
      return;
    case ActiveReferenceType::UNSPECIFIED:
    default:
      return;
    }
  }

  YumiLowLevelControllerNode::ControllerType
  YumiLowLevelControllerNode::parseControllerType(const std::string &controller_type) const
  {
    // 这里把 YAML 中的人类可读字符串，统一解析成内部枚举，便于 runControlLoop 里做 switch-case。
    if (controller_type == "trajectory")
    {
      return ControllerType::TRAJECTORY;
    }
    if (controller_type == "velocity")
    {
      return ControllerType::VELOCITY;
    }
    if (controller_type == "effort")
    {
      return ControllerType::EFFORT;
    }

    RCLCPP_WARN(
        get_logger(),
        "Unknown controller_type '%s'. Falling back to UNSPECIFIED.",
        controller_type.c_str());
    return ControllerType::UNSPECIFIED;
  }

  std::vector<double> YumiLowLevelControllerNode::computeJointVelocityCommand(
      const std::vector<double> &current_joint_positions,
      const std::vector<double> &target_joint_positions) const
  {
    std::vector<double> joint_velocities(current_joint_positions.size(), 0.0);
    if (current_joint_positions.size() != target_joint_positions.size())
    {
      return joint_velocities;
    }

    for (size_t index = 0; index < current_joint_positions.size(); ++index)
    {
      const double position_error = target_joint_positions[index] - current_joint_positions[index];
      const double raw_velocity = joint_position_gain_ * position_error;
      joint_velocities[index] = std::clamp(raw_velocity, -max_joint_velocity_, max_joint_velocity_);
    }
    return joint_velocities;
  }

  pinocchio::SE3 YumiLowLevelControllerNode::buildTargetTaskSpacePose(
      const yumi_interfaces::msg::TaskSpacePose &target_pose) const
  {
    // 参考消息中姿态已经统一成四元数，因此这里可以直接恢复目标旋转矩阵。
    Eigen::Quaterniond target_quaternion(
        target_pose.qw,
        target_pose.qx,
        target_pose.qy,
        target_pose.qz);
    target_quaternion.normalize();

    return pinocchio::SE3(
        target_quaternion.toRotationMatrix(),
        Eigen::Vector3d(target_pose.x, target_pose.y, target_pose.z));
  }

  Eigen::Matrix<double, 6, 1> YumiLowLevelControllerNode::computeTaskSpacePoseError(
      const pinocchio::SE3 &current_pose,
      const yumi_interfaces::msg::TaskSpacePose &target_pose) const
  {
    Eigen::Matrix<double, 6, 1> pose_error;
    pose_error.setZero();

    const auto target_se3 = buildTargetTaskSpacePose(target_pose);

    // 平移误差直接在世界坐标系下相减。
    pose_error.head<3>() = target_se3.translation() - current_pose.translation();

    // 姿态误差继续用最短路四元数差表达，避免 RPY 在 pi 附近跳变。
    Eigen::Quaterniond current_quaternion(current_pose.rotation());
    current_quaternion.normalize();
    Eigen::Quaterniond target_quaternion(target_se3.rotation());
    target_quaternion.normalize();

    if (current_quaternion.dot(target_quaternion) < 0.0)
    {
      target_quaternion.coeffs() *= -1.0;
    }

    const Eigen::Quaterniond orientation_error_quaternion =
        target_quaternion * current_quaternion.conjugate();
    Eigen::AngleAxisd orientation_error_axis_angle(orientation_error_quaternion);
    if (std::abs(orientation_error_axis_angle.angle()) > 1e-9)
    {
      pose_error.tail<3>() =
          orientation_error_axis_angle.axis() * orientation_error_axis_angle.angle();
    }

    return pose_error;
  }

  Eigen::Matrix<double, 6, 1> YumiLowLevelControllerNode::computeReferenceTaskSpaceVelocity(
      const yumi_interfaces::msg::TaskSpacePose &previous_target_pose,
      const yumi_interfaces::msg::TaskSpacePose &current_target_pose,
      double delta_time_sec) const
  {
    Eigen::Matrix<double, 6, 1> reference_task_space_velocity;
    reference_task_space_velocity.setZero();

    if (delta_time_sec <= 1e-9)
    {
      return reference_task_space_velocity;
    }

    const auto previous_target_se3 = buildTargetTaskSpacePose(previous_target_pose);
    const auto current_target_se3 = buildTargetTaskSpacePose(current_target_pose);

    // 线速度直接来自相邻参考点的平移差分。
    reference_task_space_velocity.head<3>() =
        (current_target_se3.translation() - previous_target_se3.translation()) / delta_time_sec;

    // 角速度来自相邻四元数的相对旋转差分。
    Eigen::Quaterniond previous_quaternion(previous_target_se3.rotation());
    previous_quaternion.normalize();
    Eigen::Quaterniond current_quaternion(current_target_se3.rotation());
    current_quaternion.normalize();

    if (previous_quaternion.dot(current_quaternion) < 0.0)
    {
      current_quaternion.coeffs() *= -1.0;
    }

    const Eigen::Quaterniond orientation_delta =
        current_quaternion * previous_quaternion.conjugate();
    Eigen::AngleAxisd orientation_delta_axis_angle(orientation_delta);
    if (std::abs(orientation_delta_axis_angle.angle()) > 1e-9)
    {
      reference_task_space_velocity.tail<3>() =
          orientation_delta_axis_angle.axis() *
          (orientation_delta_axis_angle.angle() / delta_time_sec);
    }

    return reference_task_space_velocity;
  }

  Eigen::Matrix<double, 6, 1> YumiLowLevelControllerNode::filterReferenceTaskSpaceVelocity(
      const Eigen::Matrix<double, 6, 1> &raw_reference_task_space_velocity,
      const rclcpp::Time &sample_time,
      OneEuroFilterState *filter_state)
  {
    if (filter_state == nullptr)
    {
      return raw_reference_task_space_velocity;
    }

    if (!filter_state->initialized)
    {
      // 第一个样本直接通过，避免滤波器初值把起始速度拉偏。
      filter_state->initialized = true;
      filter_state->previous_time = sample_time;
      filter_state->previous_value = raw_reference_task_space_velocity;
      filter_state->previous_derivative.setZero();
      return raw_reference_task_space_velocity;
    }

    const double delta_time_sec = (sample_time - filter_state->previous_time).seconds();
    if (delta_time_sec <= 1e-9)
    {
      return filter_state->previous_value;
    }

    Eigen::Matrix<double, 6, 1> filtered_reference_task_space_velocity;
    for (Eigen::Index index = 0; index < raw_reference_task_space_velocity.size(); ++index)
    {
      // 先对速度导数做低通，避免瞬时差分直接把自适应截止频率推得过高。
      const double raw_derivative =
          (raw_reference_task_space_velocity[index] - filter_state->previous_value[index]) /
          delta_time_sec;
      const double derivative_alpha = smoothingFactor(one_euro_d_cutoff_, delta_time_sec);
      const double filtered_derivative =
          derivative_alpha * raw_derivative +
          (1.0 - derivative_alpha) * filter_state->previous_derivative[index];

      // 一欧元滤波器的核心：速度变化越快，cutoff 越高，滤波就越“放行”快速运动。
      const double cutoff =
          one_euro_min_cutoff_ + one_euro_beta_ * std::abs(filtered_derivative);
      const double value_alpha = smoothingFactor(cutoff, delta_time_sec);
      filtered_reference_task_space_velocity[index] =
          value_alpha * raw_reference_task_space_velocity[index] +
          (1.0 - value_alpha) * filter_state->previous_value[index];
      filter_state->previous_derivative[index] = filtered_derivative;
    }

    filter_state->previous_time = sample_time;
    filter_state->previous_value = filtered_reference_task_space_velocity;
    return filtered_reference_task_space_velocity;
  }

  Eigen::Matrix<double, 6, 1> YumiLowLevelControllerNode::computeTaskSpaceVelocityCommand(
      const pinocchio::SE3 &current_pose,
      const Eigen::Matrix<double, 6, 1> &current_task_space_velocity,
      const Eigen::Matrix<double, 6, 1> &reference_task_space_velocity,
      const yumi_interfaces::msg::TaskSpacePose &target_pose) const
  {
    Eigen::Matrix<double, 6, 1> task_space_velocity;
    const auto pose_error = computeTaskSpacePoseError(current_pose, target_pose);
    const Eigen::Matrix<double, 6, 1> velocity_error =
        reference_task_space_velocity - current_task_space_velocity;

    // 任务空间控制律：
    // - v_ref 负责沿参考轨迹前进
    // - Kp * e_x 负责把末端拉回参考轨迹
    // - Kd * (v_ref - v_current) 负责补偿速度跟踪误差
    task_space_velocity = reference_task_space_velocity;
    task_space_velocity.head<3>() +=
        task_space_position_gain_ * pose_error.head<3>() +
        task_space_position_damping_gain_ * velocity_error.head<3>();
    task_space_velocity.tail<3>() +=
        task_space_orientation_gain_ * pose_error.tail<3>() +
        task_space_orientation_damping_gain_ * velocity_error.tail<3>();
    return task_space_velocity;
  }

  Eigen::MatrixXd YumiLowLevelControllerNode::extractArmJacobian(
      const Eigen::MatrixXd &full_jacobian,
      const std::vector<std::string> &arm_joint_names) const
  {
    Eigen::MatrixXd arm_jacobian(6, static_cast<Eigen::Index>(arm_joint_names.size()));
    arm_jacobian.setZero();

    if (!has_pinocchio_model_ || arm_joint_names.empty())
    {
      return Eigen::MatrixXd();
    }

    for (std::size_t column_index = 0; column_index < arm_joint_names.size(); ++column_index)
    {
      if (!robot_model_.existJointName(arm_joint_names[column_index]))
      {
        return Eigen::MatrixXd();
      }

      const auto joint_id = robot_model_.getJointId(arm_joint_names[column_index]);
      const auto &joint_model = robot_model_.joints[joint_id];
      if (joint_model.nv() != 1)
      {
        return Eigen::MatrixXd();
      }

      arm_jacobian.col(static_cast<Eigen::Index>(column_index)) =
          full_jacobian.col(joint_model.idx_v());
    }

    return arm_jacobian;
  }

  Eigen::VectorXd YumiLowLevelControllerNode::computeNullspacePostureVelocity(
      const Eigen::MatrixXd &arm_jacobian,
      const std::vector<double> &current_joint_positions,
      const std::vector<double> &posture_target_positions,
      const std::vector<double> &symmetry_target_positions) const
  {
    // 零空间项只用于冗余机械臂的“构型整理”。
    // 它不应该改变末端主任务，因此后面必须通过 nullspace projector 投影。
    Eigen::VectorXd nullspace_posture_velocity =
        Eigen::VectorXd::Zero(std::max<Eigen::Index>(arm_jacobian.cols(), 0));

    if (arm_jacobian.cols() == 0 ||
        current_joint_positions.size() != posture_target_positions.size() ||
        current_joint_positions.size() != symmetry_target_positions.size() ||
        current_joint_positions.size() != static_cast<std::size_t>(arm_jacobian.cols()))
    {
      return nullspace_posture_velocity;
    }

    // raw_posture_velocity 是“还没有投影”的构型优化速度。
    // posture 项把单臂拉向预设自然构型，symmetry 项把左右臂拉向镜像构型。
    Eigen::VectorXd raw_posture_velocity(static_cast<Eigen::Index>(current_joint_positions.size()));
    raw_posture_velocity.setZero();
    for (std::size_t index = 0; index < current_joint_positions.size(); ++index)
    {
      const double posture_error = posture_target_positions[index] - current_joint_positions[index];
      const double symmetry_error = symmetry_target_positions[index] - current_joint_positions[index];
      raw_posture_velocity[static_cast<Eigen::Index>(index)] =
          nullspace_posture_gain_ * posture_error +
          nullspace_symmetry_gain_ * symmetry_error;
    }

    // 通过 P = I - J#J 投影到零空间，避免姿态偏置直接干扰末端主任务。
    const Eigen::Matrix<double, 6, 6> regularized_task_matrix =
        arm_jacobian * arm_jacobian.transpose() +
        kDampedLeastSquaresLambda * Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::MatrixXd jacobian_pseudoinverse =
        arm_jacobian.transpose() * regularized_task_matrix.ldlt().solve(
                                       Eigen::Matrix<double, 6, 6>::Identity());
    const Eigen::MatrixXd nullspace_projector =
        Eigen::MatrixXd::Identity(arm_jacobian.cols(), arm_jacobian.cols()) -
        jacobian_pseudoinverse * arm_jacobian;

    nullspace_posture_velocity = nullspace_projector * raw_posture_velocity;
    return nullspace_posture_velocity;
  }

  std::vector<double> YumiLowLevelControllerNode::computeArmJointVelocityFromTaskSpaceCommand(
      const Eigen::MatrixXd &arm_jacobian,
      const Eigen::Matrix<double, 6, 1> &task_space_velocity,
      const std::vector<double> &current_joint_positions,
      const std::vector<double> &posture_target_positions,
      const std::vector<double> &symmetry_target_positions) const
  {
    std::vector<double> joint_velocities(
        static_cast<std::size_t>(std::max<Eigen::Index>(arm_jacobian.cols(), 0)),
        0.0);
    if (arm_jacobian.cols() == 0)
    {
      return joint_velocities;
    }

    // 用阻尼最小二乘伪逆把 6 维任务空间速度映射成关节速度，避免奇异位形数值爆炸。
    const Eigen::Matrix<double, 6, 6> regularized_task_matrix =
        arm_jacobian * arm_jacobian.transpose() +
        kDampedLeastSquaresLambda * Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::VectorXd joint_velocity_vector =
        arm_jacobian.transpose() * regularized_task_matrix.ldlt().solve(task_space_velocity);

    // 主任务解之上叠加一个最小零空间姿态偏置，用来抑制双臂执行时的关节乱绕与交叠。
    joint_velocity_vector += computeNullspacePostureVelocity(
        arm_jacobian,
        current_joint_positions,
        posture_target_positions,
        symmetry_target_positions);

    for (std::size_t index = 0; index < joint_velocities.size(); ++index)
    {
      joint_velocities[index] =
          std::clamp(joint_velocity_vector[static_cast<Eigen::Index>(index)],
                     -max_joint_velocity_,
                     max_joint_velocity_);
    }

    return joint_velocities;
  }

  bool YumiLowLevelControllerNode::initializePinocchioModel()
  {
    try
    {
      // 从 URDF 构建 Pinocchio 模型，后续所有 FK / Jacobian 都基于这份模型。
      // 这里导入的是整机模型；后续通过关节名和 frame id 抽取左右臂相关自由度。
      pinocchio::urdf::buildModel(urdf_path_, robot_model_);
      robot_data_ = std::make_unique<pinocchio::Data>(robot_model_);
    }
    catch (const std::exception &exception)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Failed to build Pinocchio model from URDF '%s': %s",
          urdf_path_.c_str(),
          exception.what());
      return false;
    }

    if (!robot_model_.existFrame(left_end_effector_frame_))
    {
      RCLCPP_ERROR(
          get_logger(),
          "Left end-effector frame '%s' does not exist in the Pinocchio model.",
          left_end_effector_frame_.c_str());
      return false;
    }
    if (!robot_model_.existFrame(right_end_effector_frame_))
    {
      RCLCPP_ERROR(
          get_logger(),
          "Right end-effector frame '%s' does not exist in the Pinocchio model.",
          right_end_effector_frame_.c_str());
      return false;
    }

    left_end_effector_frame_id_ = robot_model_.getFrameId(left_end_effector_frame_);
    right_end_effector_frame_id_ = robot_model_.getFrameId(right_end_effector_frame_);
    has_pinocchio_model_ = true;

    RCLCPP_INFO(
        get_logger(),
        "Loaded Pinocchio model from '%s'. Left frame='%s', right frame='%s'.",
        urdf_path_.c_str(),
        left_end_effector_frame_.c_str(),
        right_end_effector_frame_.c_str());
    return true;
  }

  bool YumiLowLevelControllerNode::buildPinocchioConfiguration(Eigen::VectorXd *configuration)
  {
    if (!configuration || !has_pinocchio_model_)
    {
      return false;
    }

    // 先从 neutral configuration 开始，未由 joint_states 覆盖的关节保持中立值。
    *configuration = pinocchio::neutral(robot_model_);

    // 当前 joint_states 里只关心已知名字的关节。
    // 对 YuMi 这样的固定基 1-DoF 关节，这里可以直接把标量写到对应的 q 槽位。
    for (const auto &joint_state_entry : latest_joint_positions_)
    {
      if (!robot_model_.existJointName(joint_state_entry.first))
      {
        continue;
      }

      const auto joint_id = robot_model_.getJointId(joint_state_entry.first);
      const auto &joint_model = robot_model_.joints[joint_id];
      if (joint_model.nq() != 1)
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Joint '%s' has nq=%d. Current helper only writes scalar joints directly.",
            joint_state_entry.first.c_str(),
            joint_model.nq());
        continue;
      }

      (*configuration)[joint_model.idx_q()] = joint_state_entry.second;
    }

    return true;
  }

  bool YumiLowLevelControllerNode::computeTaskSpaceKinematics(
      const Eigen::VectorXd &configuration,
      pinocchio::SE3 *left_end_effector_pose,
      pinocchio::SE3 *right_end_effector_pose,
      Eigen::MatrixXd *left_jacobian,
      Eigen::MatrixXd *right_jacobian)
  {
    if (!has_pinocchio_model_ || !robot_data_)
    {
      return false;
    }
    if (!left_end_effector_pose || !right_end_effector_pose || !left_jacobian || !right_jacobian)
    {
      return false;
    }
    if (configuration.size() != robot_model_.nq)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Pinocchio configuration size mismatch. Expected nq=%d, got %ld.",
          robot_model_.nq,
          configuration.size());
      return false;
    }

    // FK 先更新所有 joint / frame 的世界位姿。
    pinocchio::forwardKinematics(robot_model_, *robot_data_, configuration);
    pinocchio::updateFramePlacements(robot_model_, *robot_data_);

    // Jacobian 计算依赖 joint Jacobians 先被刷新。
    pinocchio::computeJointJacobians(robot_model_, *robot_data_, configuration);

    *left_end_effector_pose = robot_data_->oMf[left_end_effector_frame_id_];
    *right_end_effector_pose = robot_data_->oMf[right_end_effector_frame_id_];

    left_jacobian->resize(6, robot_model_.nv);
    right_jacobian->resize(6, robot_model_.nv);

    // 这里统一使用 WORLD 参考系，便于后续任务空间速度和误差直接在世界坐标系下表达。
    pinocchio::getFrameJacobian(
        robot_model_,
        *robot_data_,
        left_end_effector_frame_id_,
        pinocchio::ReferenceFrame::WORLD,
        *left_jacobian);
    pinocchio::getFrameJacobian(
        robot_model_,
        *robot_data_,
        right_end_effector_frame_id_,
        pinocchio::ReferenceFrame::WORLD,
        *right_jacobian);

    return true;
  }

  void YumiLowLevelControllerNode::publishVelocityCommands(
      const std::vector<double> &left_joint_velocities,
      const std::vector<double> &right_joint_velocities)
  {
    std_msgs::msg::Float64MultiArray left_command;
    std_msgs::msg::Float64MultiArray right_command;
    left_command.data = left_joint_velocities;
    right_command.data = right_joint_velocities;
    left_velocity_command_publisher_->publish(left_command);
    right_velocity_command_publisher_->publish(right_command);
  }

  void YumiLowLevelControllerNode::publishControllerStatus(
      double max_joint_position_error,
      double task_space_position_error_norm,
      double task_space_orientation_error_norm)
  {
    // 这里把当前执行状态压缩成一个轻量反馈消息，供上层状态机消费：
    // - 当前控制器类型
    // - 当前激活参考类型
    // - 当前是否已经达到目标阈值
    yumi_interfaces::msg::LowLevelControllerStatus status_msg;
    status_msg.stamp = now();
    status_msg.has_reference = has_upper_reference_;

    switch (controller_type_)
    {
    case ControllerType::TRAJECTORY:
      status_msg.controller_type =
          yumi_interfaces::msg::LowLevelControllerStatus::CONTROLLER_TRAJECTORY;
      break;
    case ControllerType::VELOCITY:
      status_msg.controller_type =
          yumi_interfaces::msg::LowLevelControllerStatus::CONTROLLER_VELOCITY;
      break;
    case ControllerType::EFFORT:
      status_msg.controller_type =
          yumi_interfaces::msg::LowLevelControllerStatus::CONTROLLER_EFFORT;
      break;
    case ControllerType::UNSPECIFIED:
    default:
      status_msg.controller_type =
          yumi_interfaces::msg::LowLevelControllerStatus::CONTROLLER_UNSPECIFIED;
      break;
    }

    switch (active_reference_type_)
    {
    case ActiveReferenceType::JOINT_SPACE:
      status_msg.active_reference_type =
          yumi_interfaces::msg::LowLevelControllerStatus::REFERENCE_JOINT_SPACE;
      // 关节空间任务只看最大关节误差是否进入阈值。
      status_msg.goal_reached = max_joint_position_error <= joint_goal_tolerance_;
      break;
    case ActiveReferenceType::TASK_SPACE:
      status_msg.active_reference_type =
          yumi_interfaces::msg::LowLevelControllerStatus::REFERENCE_TASK_SPACE;
      // 任务空间任务要求平移和姿态误差都进入阈值，才算真正“到位”。
      status_msg.goal_reached =
          task_space_position_error_norm <= task_space_position_tolerance_ &&
          task_space_orientation_error_norm <= task_space_orientation_tolerance_;
      break;
    case ActiveReferenceType::UNSPECIFIED:
    default:
      status_msg.active_reference_type =
          yumi_interfaces::msg::LowLevelControllerStatus::REFERENCE_UNSPECIFIED;
      status_msg.goal_reached = false;
      break;
    }

    status_msg.max_joint_position_error = max_joint_position_error;
    status_msg.task_space_position_error_norm = task_space_position_error_norm;
    status_msg.task_space_orientation_error_norm = task_space_orientation_error_norm;
    if (status_msg.goal_reached && goal_reached_reference_stable_time_sec_ > 0.0)
    {
      const double reference_stable_time_sec =
          (now() - latest_reference_change_time_).seconds();
      if (reference_stable_time_sec < goal_reached_reference_stable_time_sec_)
      {
        status_msg.goal_reached = false;
      }
    }

    std_msgs::msg::Float64MultiArray debug_msg;
    // 调试数据保持固定顺序，方便 ros2 topic echo 或脚本记录：
    // [0] 最大关节误差 rad，[1] 末端位置误差 m，[2] 末端姿态误差 rad，[3] 是否到位。
    debug_msg.data = {
        max_joint_position_error,
        task_space_position_error_norm,
        task_space_orientation_error_norm,
        status_msg.goal_reached ? 1.0 : 0.0};
    controller_error_debug_publisher_->publish(debug_msg);

    // 反馈链采用事件触发，而不是每个控制周期持续广播：
    // - 首次进入目标阈值时，向上游发布一次 goal_reached=true
    // - 只要目标再次偏离阈值，就解除锁存，允许下一次到位后重新上报
    if (!status_msg.goal_reached)
    {
      goal_reached_reported_ = false;
      return;
    }
    if (goal_reached_reported_)
    {
      return;
    }

    // task_manager 只需要在“刚刚到位”的那个时刻收到一次反馈即可推进状态。
    controller_status_publisher_->publish(status_msg);
    goal_reached_reported_ = true;
  }

  bool YumiLowLevelControllerNode::tryBuildOrderedJointPositions(
      std::vector<double> *left_joint_positions,
      std::vector<double> *right_joint_positions)
  {
    if (left_joint_positions == nullptr || right_joint_positions == nullptr)
    {
      return false;
    }

    left_joint_positions->clear();
    right_joint_positions->clear();

    // 这里按“控制器内部统一关节顺序”重排 joint_states。
    // 这样后续无论 joint_states 原始顺序如何变化，控制算法拿到的向量顺序都稳定一致。
    for (const auto &joint_name : left_joint_names_)
    {
      const auto iterator = latest_joint_positions_.find(joint_name);
      if (iterator == latest_joint_positions_.end())
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Joint state for '%s' is not available yet.", joint_name.c_str());
        return false;
      }
      left_joint_positions->push_back(iterator->second);
    }

    for (const auto &joint_name : right_joint_names_)
    {
      const auto iterator = latest_joint_positions_.find(joint_name);
      if (iterator == latest_joint_positions_.end())
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Joint state for '%s' is not available yet.", joint_name.c_str());
        return false;
      }
      right_joint_positions->push_back(iterator->second);
    }

    return true;
  }

  bool YumiLowLevelControllerNode::tryBuildOrderedJointVelocities(
      std::vector<double> *left_joint_velocities,
      std::vector<double> *right_joint_velocities)
  {
    if (left_joint_velocities == nullptr || right_joint_velocities == nullptr)
    {
      return false;
    }

    left_joint_velocities->clear();
    right_joint_velocities->clear();

    // 速度同样要按内部统一关节顺序重排，才能和 Jacobian 抽列后的顺序严格对齐。
    for (const auto &joint_name : left_joint_names_)
    {
      const auto iterator = latest_joint_velocities_.find(joint_name);
      if (iterator == latest_joint_velocities_.end())
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Joint velocity for '%s' is not available yet.", joint_name.c_str());
        return false;
      }
      left_joint_velocities->push_back(iterator->second);
    }

    for (const auto &joint_name : right_joint_names_)
    {
      const auto iterator = latest_joint_velocities_.find(joint_name);
      if (iterator == latest_joint_velocities_.end())
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Joint velocity for '%s' is not available yet.", joint_name.c_str());
        return false;
      }
      right_joint_velocities->push_back(iterator->second);
    }

    return true;
  }

} // namespace yumi_low_level_controllers
