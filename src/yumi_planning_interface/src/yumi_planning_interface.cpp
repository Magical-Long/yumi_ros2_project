#include "yumi_planning_interface/yumi_planning_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <functional>
#include <limits>
#include <thread>
#include <utility>

#include <moveit/robot_state/robot_state.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace yumi_planning_interface
{

  YumiPlanningInterfaceNode::YumiPlanningInterfaceNode()
      : Node("yumi_planning_interface")
  {
    // 规划组列表决定当前节点会为哪些机械臂实例化 MoveGroupInterface。
    declare_parameter<std::vector<std::string>>("planning_groups", {"arm_l", "arm_r"});

    // 末端 link 名称必须与 URDF / SRDF 完全一致，否则位姿目标无法正确绑定到链末端。
    declare_parameter<std::string>("arm_l_end_effector_link", "yumi_link_7_l");
    declare_parameter<std::string>("arm_r_end_effector_link", "yumi_link_7_r");

    // 随机位姿采样时允许的最大尝试次数，用于平衡搜索成功率和等待时间。
    declare_parameter<int>("random_pose_attempts", 20);

    // 可选演示开关。
    // 当前默认关闭，避免节点一启动就自动驱动仿真。
    declare_parameter<bool>("run_demo_on_startup", true);
    declare_parameter<bool>("execute_demo_plan_on_startup", true);
    declare_parameter<bool>("reset_to_home_after_demo", false);
    declare_parameter<std::string>("demo_mode", "single_arm");
    declare_parameter<std::string>("demo_planning_group", "arm_l");
    declare_parameter<bool>("require_trajectory_action_servers", false);
    declare_parameter<double>("trajectory_action_server_wait_timeout", 10.0);
    declare_parameter<double>("trajectory_execution_result_timeout", 30.0);

    // 简化位姿接口的默认参考坐标系。
    // 当用户只给出 6 个 double 时，将默认按这个 frame 解释目标位姿。
    declare_parameter<std::string>("default_pose_reference_frame", "world");

    // 任务空间轨迹规划的默认参数。
    // action goal 未显式给出时，将回退到这些默认值。
    declare_parameter<double>("default_task_space_planning_time", 5.0);
    declare_parameter<int>("default_task_space_num_planning_attempts", 3);
    declare_parameter<double>("default_task_space_velocity_scaling", 0.2);
    declare_parameter<double>("default_task_space_acceleration_scaling", 0.2);
    declare_parameter<double>("default_task_space_sample_period", 0.02);
    declare_parameter<double>("default_task_space_duration_scale", 1.0);
    declare_parameter<double>("default_task_space_cartesian_step", 0.01);
    declare_parameter<double>("default_task_space_jump_threshold", 0.0);
    declare_parameter<bool>("default_task_space_avoid_collisions", true);
    declare_parameter<int>("preferred_ik_seed_count", 0);
    declare_parameter<std::vector<double>>("left_preferred_ik_seeds", {});
    declare_parameter<std::vector<double>>("right_preferred_ik_seeds", {});
    declare_parameter<double>("preferred_ik_timeout_sec", 0.05);
    declare_parameter<std::vector<double>>(
        "ik_posture_joint_weights",
        {1.0, 1.0, 2.5, 1.8, 2.5, 1.2, 1.2});
    declare_parameter<double>("ik_current_state_weight", 0.1);
    declare_parameter<std::vector<double>>(
        "dual_arm_symmetry_signs",
        {-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0});
    declare_parameter<double>("dual_arm_symmetry_weight", 1.0);

    RCLCPP_INFO(
        get_logger(),
        "yumi_planning_interface node started. Ready for MoveIt API integration.");
  }

  bool YumiPlanningInterfaceNode::initialize()
  {
    // 读取规划组参数，后续为每个规划组分别创建 MoveGroupInterface。
    const auto planning_groups = get_parameter("planning_groups").as_string_array();
    RCLCPP_INFO(
        get_logger(),
        "Starting MoveIt interface initialization. %zu planning groups will be configured.",
        planning_groups.size());

    // 记录每个规划组对应的末端 link，后续生成位姿目标和执行规划都依赖这个映射。
    end_effector_links_["arm_l"] = get_parameter("arm_l_end_effector_link").as_string();
    end_effector_links_["arm_r"] = get_parameter("arm_r_end_effector_link").as_string();
    require_trajectory_action_servers_ =
        get_parameter("require_trajectory_action_servers").as_bool();

    if (require_trajectory_action_servers_)
    {
      // 为左右臂控制器分别建立轨迹 action client。
      // 只有在“规划+执行模式”下，才强制依赖这些 FollowJointTrajectory server。
      trajectory_action_clients_["arm_l"] =
          rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
              shared_from_this(),
              "/left_arm_controller/follow_joint_trajectory");
      trajectory_action_clients_["arm_r"] =
          rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
              shared_from_this(),
              "/right_arm_controller/follow_joint_trajectory");

      const auto action_server_wait_timeout =
          get_parameter("trajectory_action_server_wait_timeout").as_double();
      for (const auto &entry : trajectory_action_clients_)
      {
        RCLCPP_INFO(
            get_logger(),
            "Waiting for trajectory action server of planning group '%s'.",
            entry.first.c_str());
        if (!entry.second->wait_for_action_server(std::chrono::duration<double>(action_server_wait_timeout)))
        {
          RCLCPP_ERROR(
              get_logger(),
              "Trajectory action server for planning group '%s' was not available within %.2f seconds.",
              entry.first.c_str(),
              action_server_wait_timeout);
          return false;
        }
      }
    }
    else
    {
      RCLCPP_INFO(
          get_logger(),
          "Pure planning mode is enabled. FollowJointTrajectory action servers are not required.");
    }

    for (const auto &planning_group : planning_groups)
    {
      // MoveGroupInterface 是对 MoveIt 规划与执行接口的高层封装，
      // 这里按规划组分别实例化，便于后续单臂或双臂扩展。
      auto move_group =
          std::make_shared<moveit::planning_interface::MoveGroupInterface>(
              shared_from_this(), planning_group);

      // 设置保守的规划时间与速度缩放，先优先保证可用性与可观察性。
      move_group->setPlanningTime(5.0);
      move_group->setMaxVelocityScalingFactor(0.2);
      move_group->setMaxAccelerationScalingFactor(0.2);

      // 显式启动当前状态监视器，并给它一点时间等待 joint_states。
      // 这样可以避免首次调用 getCurrentState() 时，状态监视器还没完全建立好。
      RCLCPP_INFO(
          get_logger(),
          "Starting the current state monitor for planning group '%s'.",
          planning_group.c_str());
      move_group->startStateMonitor(5.0);

      move_groups_[planning_group] = move_group;

      RCLCPP_INFO(
          get_logger(),
          "Initialized MoveGroupInterface for group '%s' with end effector '%s'.",
          planning_group.c_str(),
          getEndEffectorLink(planning_group).c_str());
    }

    // 如有需要，提供一次性的启动演示入口，用于快速验证 API 组合是否工作。
    if (get_parameter("run_demo_on_startup").as_bool())
    {
      RCLCPP_INFO(
          get_logger(),
          "Startup demo is enabled. A one-shot planning demo will run after the timer fires.");
      auto self = std::static_pointer_cast<YumiPlanningInterfaceNode>(shared_from_this());
      demo_timer_ = create_wall_timer(
          std::chrono::seconds(2),
          [self]()
          {
            // 定时器回调只负责触发后台线程，避免长时间的规划/执行逻辑阻塞默认回调组。
            std::thread(
                [self]()
                {
                  self->runDemoOnce();
                })
                .detach();
          });
    }
    else
    {
      RCLCPP_INFO(
          get_logger(),
          "Startup demo is disabled. The node will wait for external planning requests.");
    }

    // 初始化完成后立刻创建 action server，
    // 这样外部协调层可以通过标准 ROS2 action 来调用当前规划接口层。
    plan_arm_pose_action_server_ =
        rclcpp_action::create_server<PlanArmPose>(
            shared_from_this(),
            "plan_arm_pose",
            std::bind(&YumiPlanningInterfaceNode::handlePlanArmPoseGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&YumiPlanningInterfaceNode::handlePlanArmPoseCancel, this, std::placeholders::_1),
            std::bind(&YumiPlanningInterfaceNode::handlePlanArmPoseAccepted, this, std::placeholders::_1));

    plan_both_arms_poses_action_server_ =
        rclcpp_action::create_server<PlanBothArmsPoses>(
            shared_from_this(),
            "plan_both_arms_poses",
            std::bind(&YumiPlanningInterfaceNode::handlePlanBothArmsPosesGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&YumiPlanningInterfaceNode::handlePlanBothArmsPosesCancel, this, std::placeholders::_1),
            std::bind(&YumiPlanningInterfaceNode::handlePlanBothArmsPosesAccepted, this, std::placeholders::_1));

    plan_task_space_trajectory_action_server_ =
        rclcpp_action::create_server<PlanTaskSpaceTrajectory>(
            shared_from_this(),
            "plan_task_space_trajectory",
            std::bind(&YumiPlanningInterfaceNode::handlePlanTaskSpaceTrajectoryGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&YumiPlanningInterfaceNode::handlePlanTaskSpaceTrajectoryCancel, this, std::placeholders::_1),
            std::bind(&YumiPlanningInterfaceNode::handlePlanTaskSpaceTrajectoryAccepted, this, std::placeholders::_1));

    reset_to_home_action_server_ =
        rclcpp_action::create_server<ResetToHome>(
            shared_from_this(),
            "reset_to_home",
            std::bind(&YumiPlanningInterfaceNode::handleResetToHomeGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&YumiPlanningInterfaceNode::handleResetToHomeCancel, this, std::placeholders::_1),
            std::bind(&YumiPlanningInterfaceNode::handleResetToHomeAccepted, this, std::placeholders::_1));

    reset_both_arms_to_home_action_server_ =
        rclcpp_action::create_server<ResetBothArmsToHome>(
            shared_from_this(),
            "reset_both_arms_to_home",
            std::bind(&YumiPlanningInterfaceNode::handleResetBothArmsToHomeGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&YumiPlanningInterfaceNode::handleResetBothArmsToHomeCancel, this, std::placeholders::_1),
            std::bind(&YumiPlanningInterfaceNode::handleResetBothArmsToHomeAccepted, this, std::placeholders::_1));

    run_random_motion_action_server_ =
        rclcpp_action::create_server<RunRandomMotion>(
            shared_from_this(),
            "run_random_motion",
            std::bind(&YumiPlanningInterfaceNode::handleRunRandomMotionGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&YumiPlanningInterfaceNode::handleRunRandomMotionCancel, this, std::placeholders::_1),
            std::bind(&YumiPlanningInterfaceNode::handleRunRandomMotionAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "Action servers are ready: plan_arm_pose, plan_both_arms_poses, plan_task_space_trajectory, reset_to_home, reset_both_arms_to_home, run_random_motion.");

    return true;
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
  YumiPlanningInterfaceNode::getMoveGroup(const std::string &planning_group)
  {
    auto it = move_groups_.find(planning_group);
    if (it == move_groups_.end())
    {
      RCLCPP_ERROR(
          get_logger(),
          "Planning group '%s' was not initialized in yumi_planning_interface.",
          planning_group.c_str());
      return nullptr;
    }

    return it->second;
  }

  std::string YumiPlanningInterfaceNode::getHomeTargetName(const std::string &planning_group) const
  {
    // 规划接口层不重复保存一套 home 关节角，而是复用 SRDF 中已经定义好的命名姿态。
    if (planning_group == "arm_l")
    {
      return "home_l";
    }

    if (planning_group == "arm_r")
    {
      return "home_r";
    }

    return "";
  }

  std::string YumiPlanningInterfaceNode::getPlanningGroupFromArmSelector(const uint8_t arm_selector) const
  {
    // action 层用更轻量的左右臂枚举来表示手臂选择，
    // 这里再把它统一映射回内部已经沿用的 arm_l / arm_r 规划组命名。
    if (arm_selector == PlanArmPose::Goal::ARM_LEFT)
    {
      return "arm_l";
    }

    if (arm_selector == PlanArmPose::Goal::ARM_RIGHT)
    {
      return "arm_r";
    }

    return "";
  }

  geometry_msgs::msg::PoseStamped YumiPlanningInterfaceNode::convertPose6DToPoseStamped(
      const yumi_interfaces::msg::Pose6D &pose6d,
      const std::string &reference_frame) const
  {
    geometry_msgs::msg::PoseStamped pose_stamped;

    // 记录位姿所属参考坐标系，
    // 这一步决定了 x/y/z/roll/pitch/yaw 是相对于哪个 frame 定义的。
    pose_stamped.header.frame_id = reference_frame;

    // 记录当前时间戳，便于调试时追踪请求生成时刻。
    pose_stamped.header.stamp = now();

    // 平移分量直接从自定义 6D 位姿拷贝到标准几何消息。
    pose_stamped.pose.position.x = pose6d.x;
    pose_stamped.pose.position.y = pose6d.y;
    pose_stamped.pose.position.z = pose6d.z;

    // 将 roll / pitch / yaw 转换成四元数，
    // 这是 MoveIt / TF 内部统一使用的姿态表达方式。
    tf2::Quaternion quaternion;
    quaternion.setRPY(pose6d.roll, pose6d.pitch, pose6d.yaw);
    quaternion.normalize();

    pose_stamped.pose.orientation.x = quaternion.x();
    pose_stamped.pose.orientation.y = quaternion.y();
    pose_stamped.pose.orientation.z = quaternion.z();
    pose_stamped.pose.orientation.w = quaternion.w();

    return pose_stamped;
  }

  std::string YumiPlanningInterfaceNode::getEndEffectorLink(const std::string &planning_group) const
  {
    auto it = end_effector_links_.find(planning_group);
    if (it == end_effector_links_.end())
    {
      return "";
    }

    return it->second;
  }

  std::optional<geometry_msgs::msg::PoseStamped> YumiPlanningInterfaceNode::generateRandomValidPose(
      const std::string &planning_group,
      int max_attempts)
  {
    RCLCPP_INFO(
        get_logger(),
        "Generating a random valid end-effector pose for planning group '%s' with at most %d attempts.",
        planning_group.c_str(),
        max_attempts);

    auto move_group = getMoveGroup(planning_group);
    if (!move_group)
    {
      return std::nullopt;
    }

    const auto end_effector_link = getEndEffectorLink(planning_group);
    if (end_effector_link.empty())
    {
      RCLCPP_ERROR(
          get_logger(),
          "End effector link for planning group '%s' is empty.",
          planning_group.c_str());
      return std::nullopt;
    }

    // 从当前状态出发复制一个 RobotState，
    // 后续在同一规划组上做随机关节采样，并用 FK 换算成末端位姿。
    RCLCPP_INFO(
        get_logger(),
        "Requesting the current robot state for planning group '%s'.",
        planning_group.c_str());
    auto current_state = move_group->getCurrentState(5.0);
    if (!current_state)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Failed to get current robot state for planning group '%s'.",
          planning_group.c_str());
      return std::nullopt;
    }
    RCLCPP_INFO(
        get_logger(),
        "Current robot state was received successfully for planning group '%s'.",
        planning_group.c_str());

    const auto robot_model = current_state->getRobotModel();
    const auto *joint_model_group = robot_model->getJointModelGroup(planning_group);
    if (!joint_model_group)
    {
      RCLCPP_ERROR(
          get_logger(),
          "JointModelGroup '%s' was not found in the robot model.",
          planning_group.c_str());
      return std::nullopt;
    }

    moveit::core::RobotState random_state(*current_state);
    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
      RCLCPP_INFO(
          get_logger(),
          "Sampling random candidate pose %d / %d for planning group '%s'.",
          attempt + 1,
          max_attempts,
          planning_group.c_str());

      // 在规划组内部随机采样关节位置，先得到一个运动学上可表达的候选姿态。
      random_state.setToRandomPositions(joint_model_group);
      random_state.update();

      geometry_msgs::msg::PoseStamped candidate_pose;

      // 规划参考系应与 MoveIt 当前 planning frame 一致，
      // 这样后续把该位姿重新喂给 MoveGroupInterface 时不会出现坐标系歧义。
      candidate_pose.header.frame_id = move_group->getPlanningFrame();
      candidate_pose.header.stamp = now();

      // 利用 FK 把随机采样到的关节构型转换成末端位姿。
      // 这里得到的是 world / planning frame 下的目标 pose。
      candidate_pose.pose = tf2::toMsg(random_state.getGlobalLinkTransform(end_effector_link));

      moveit::planning_interface::MoveGroupInterface::Plan plan;

      // 用真实规划过程反向验证该位姿是否“好用”，
      // 只有当前状态可达且规划成功的位姿，才作为“合法随机末端位姿”返回。
      RCLCPP_INFO(
          get_logger(),
          "Validating candidate pose %d / %d for planning group '%s' by invoking the planner.",
          attempt + 1,
          max_attempts,
          planning_group.c_str());
      if (planToPose(planning_group, candidate_pose, &plan))
      {
        RCLCPP_INFO(
            get_logger(),
            "Generated a valid random pose for group '%s' on attempt %d.",
            planning_group.c_str(),
            attempt + 1);
        return candidate_pose;
      }

      RCLCPP_WARN(
          get_logger(),
          "Candidate pose %d / %d was rejected for planning group '%s'.",
          attempt + 1,
          max_attempts,
          planning_group.c_str());
    }

    RCLCPP_WARN(
        get_logger(),
        "Failed to generate a valid random pose for planning group '%s' within %d attempts.",
        planning_group.c_str(),
        max_attempts);
    return std::nullopt;
  }

  bool YumiPlanningInterfaceNode::findPostureBiasedIkTarget(
      const std::string &planning_group,
      const geometry_msgs::msg::PoseStamped &target_pose,
      const std::string &end_effector_link,
      const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &move_group,
      std::vector<double> *joint_target)
  {
    double unused_score = 0.0;
    return findPostureBiasedIkTarget(
        planning_group,
        target_pose,
        end_effector_link,
        move_group,
        {},
        joint_target,
        &unused_score);
  }

  bool YumiPlanningInterfaceNode::findPostureBiasedIkTarget(
      const std::string &planning_group,
      const geometry_msgs::msg::PoseStamped &target_pose,
      const std::string &end_effector_link,
      const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &move_group,
      const std::vector<std::vector<double>> &extra_seed_candidates,
      std::vector<double> *joint_target,
      double *selected_score)
  {
    return findPostureBiasedIkTarget(
        planning_group,
        target_pose,
        end_effector_link,
        move_group,
        extra_seed_candidates,
        {},
        0.0,
        joint_target,
        selected_score);
  }

  bool YumiPlanningInterfaceNode::findPostureBiasedIkTarget(
      const std::string &planning_group,
      const geometry_msgs::msg::PoseStamped &target_pose,
      const std::string &end_effector_link,
      const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &move_group,
      const std::vector<std::vector<double>> &extra_seed_candidates,
      const std::vector<double> &mirrored_other_arm_joint_target,
      const double symmetry_weight,
      std::vector<double> *joint_target,
      double *selected_score)
  {
    // 该函数把“末端 pose 目标”先转成“构型更自然的 q_goal”。
    // MoveIt 仍然负责从当前状态到 q_goal 的路径规划；这里主要解决 IK 分支选择问题。
    if (!move_group || joint_target == nullptr)
    {
      return false;
    }

    auto current_state = move_group->getCurrentState();
    if (!current_state)
    {
      return false;
    }

    // JointModelGroup 是 MoveIt 对单臂规划组的关节变量集合。
    // 后面所有 seed、IK 结果、joint target 都必须落在这个 group 的变量顺序里。
    const auto *joint_model_group = current_state->getJointModelGroup(planning_group);
    if (joint_model_group == nullptr)
    {
      return false;
    }

    // 当前只为左右臂单独配置 preferred seed。
    // 其他 planning group 仍然走原始 pose target 规划路径。
    const bool is_left_arm = planning_group == "arm_l";
    const bool is_right_arm = planning_group == "arm_r";
    if (!is_left_arm && !is_right_arm)
    {
      return false;
    }

    // YAML 里的 seed 是按我们统一习惯的 1273456 顺序写的，
    // 每 7 个 double 表示一组“比较自然”的关节构型。
    const auto raw_seed_values = get_parameter(
        is_left_arm ? "left_preferred_ik_seeds" : "right_preferred_ik_seeds").as_double_array();
    const auto configured_seed_count = get_parameter("preferred_ik_seed_count").as_int();
    if (configured_seed_count <= 0 || raw_seed_values.size() < 7U)
    {
      return false;
    }

    const std::size_t seed_count =
        std::min<std::size_t>(static_cast<std::size_t>(configured_seed_count),
                              raw_seed_values.size() / 7U);
    if (seed_count == 0U)
    {
      return false;
    }

    // 这里显式写出 YAML 中的关节名顺序。
    // MoveIt group 内部顺序可能不同，所以不能直接把 YAML 数组原样塞给 setJointGroupPositions。
    const std::vector<std::string> configured_joint_order = is_left_arm ?
        std::vector<std::string>{
            "yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l", "yumi_joint_3_l",
            "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l"} :
        std::vector<std::string>{
            "yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r", "yumi_joint_3_r",
            "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r"};
    const auto &moveit_joint_order = joint_model_group->getVariableNames();
    const auto configured_joint_weights =
        get_parameter("ik_posture_joint_weights").as_double_array();

    // 把一组 YAML seed 重排到 MoveIt group 的变量顺序。
    // 这样用户仍然可以按 1273456 写配置，代码负责转换成规划器真正需要的顺序。
    const auto map_seed_to_moveit_order =
        [&](std::size_t seed_index, std::vector<double> *mapped_seed) -> bool {
          if (mapped_seed == nullptr)
          {
            return false;
          }
          mapped_seed->assign(moveit_joint_order.size(), 0.0);
          for (std::size_t moveit_index = 0; moveit_index < moveit_joint_order.size(); ++moveit_index)
          {
            const auto match = std::find(
                configured_joint_order.begin(),
                configured_joint_order.end(),
                moveit_joint_order[moveit_index]);
            if (match == configured_joint_order.end())
            {
              return false;
            }
            const auto configured_index =
                static_cast<std::size_t>(std::distance(configured_joint_order.begin(), match));
            (*mapped_seed)[moveit_index] = raw_seed_values[seed_index * 7U + configured_index];
          }
          return true;
        };

    const auto map_configured_vector_to_moveit_order =
        [&](const std::vector<double> &configured_values, std::vector<double> *mapped_values) -> bool {
          if (mapped_values == nullptr || configured_values.size() < configured_joint_order.size())
          {
            return false;
          }
          mapped_values->assign(moveit_joint_order.size(), 1.0);
          for (std::size_t moveit_index = 0; moveit_index < moveit_joint_order.size(); ++moveit_index)
          {
            const auto match = std::find(
                configured_joint_order.begin(),
                configured_joint_order.end(),
                moveit_joint_order[moveit_index]);
            if (match == configured_joint_order.end())
            {
              return false;
            }
            const auto configured_index =
                static_cast<std::size_t>(std::distance(configured_joint_order.begin(), match));
            (*mapped_values)[moveit_index] = configured_values[configured_index];
          }
          return true;
        };

    // mapped_seeds 是“IK 初始值集合”，不是最终目标集合。
    // setFromIK 会从这些初始值出发寻找同一个末端 pose 对应的不同 q_goal 分支。
    std::vector<std::vector<double>> mapped_seeds;
    mapped_seeds.reserve(seed_count);
    for (std::size_t seed_index = 0; seed_index < seed_count; ++seed_index)
    {
      std::vector<double> mapped_seed;
      if (map_seed_to_moveit_order(seed_index, &mapped_seed))
      {
        mapped_seeds.push_back(mapped_seed);
      }
    }
    for (const auto &extra_seed : extra_seed_candidates)
    {
      if (extra_seed.size() == moveit_joint_order.size())
      {
        // extra seed 例如“另一只手镜像过来的构型”。
        // 它只负责增加 IK 搜索入口，不直接决定最终选择。
        mapped_seeds.push_back(extra_seed);
      }
    }
    if (mapped_seeds.empty())
    {
      return false;
    }

    std::vector<double> current_joint_positions;
    current_state->copyJointGroupPositions(joint_model_group, current_joint_positions);
    // 当前真实关节状态也作为 IK seed 参与求解。
    // 这样 IK 不只从人为 preferred posture 出发，也会尝试找到一组延续当前构型的解。
    mapped_seeds.insert(mapped_seeds.begin(), current_joint_positions);
    // preferred_target_seed 是“姿态偏好参考”，不是 IK 初值里的唯一正确答案。
    // 当前策略默认把 YAML 最后一组 seed 当作最希望靠近的自然构型。
    std::vector<double> preferred_target_seed;
    if (!map_seed_to_moveit_order(seed_count - 1U, &preferred_target_seed))
    {
      return false;
    }
    std::vector<double> joint_weights;
    if (!map_configured_vector_to_moveit_order(configured_joint_weights, &joint_weights))
    {
      joint_weights.assign(moveit_joint_order.size(), 1.0);
    }
    const auto ik_timeout_sec = get_parameter("preferred_ik_timeout_sec").as_double();
    const auto current_state_weight = get_parameter("ik_current_state_weight").as_double();

    // 评分统一在 MoveIt group 关节顺序下进行。
    // lhs/rhs 都是同一只手的 q 向量，weights 用来强调肘部等更重要的关节。
    const auto squared_distance =
        [](const std::vector<double> &lhs,
           const std::vector<double> &rhs,
           const std::vector<double> &weights) {
          double distance = 0.0;
          for (std::size_t index = 0; index < lhs.size() && index < rhs.size(); ++index)
          {
            const double error = lhs[index] - rhs[index];
            const double weight = index < weights.size() ? weights[index] : 1.0;
            distance += weight * error * error;
          }
          return distance;
        };

    double best_score = std::numeric_limits<double>::infinity();
    std::vector<double> best_candidate;

    for (const auto &seed : mapped_seeds)
    {
      // 每个 seed 都作为 IK 初始构型。
      // 不同初值可能收敛到不同 IK 分支，例如肘部朝向或手腕翻转不同。
      moveit::core::RobotState candidate_state(*current_state);
      candidate_state.setJointGroupPositions(joint_model_group, seed);
      candidate_state.update();

      // setFromIK 把“末端 pose 目标”转换成一组候选 q_goal。
      // 这里仍然使用同一个 end_effector_link，保证 IK 解对应当前规划的实际末端。
      if (!candidate_state.setFromIK(
              joint_model_group,
              target_pose.pose,
              end_effector_link,
              ik_timeout_sec))
      {
        continue;
      }

      std::vector<double> candidate;
      candidate_state.copyJointGroupPositions(joint_model_group, candidate);
      // candidate 是 setFromIK 求出来的候选 q_goal。
      // 后续所有 score 都只是在候选 q_goal 之间做排序，不会改变末端 pose 约束。
      double score =
          squared_distance(candidate, preferred_target_seed, joint_weights) +
          current_state_weight * squared_distance(candidate, current_joint_positions, joint_weights);

      // 镜像项只在双臂规划入口传入参考构型时启用。
      // mirrored_other_arm_joint_target =
      //   “另一只手选出的 q_goal” 经过 dual_arm_symmetry_signs 映射后的当前手 q 参考。
      // 候选解越接近这个参考，左右臂关节构型越接近镜像对称。
      if (symmetry_weight > 0.0 && mirrored_other_arm_joint_target.size() == candidate.size())
      {
        score += symmetry_weight *
            squared_distance(candidate, mirrored_other_arm_joint_target, joint_weights);
      }
      if (score < best_score)
      {
        best_score = score;
        best_candidate = candidate;
      }
    }

    // best_candidate 是所有可行 IK 解中综合代价最低的一组 q_goal。
    // 如果没有任何 seed 求出 IK，外层会退回原始 pose target 规划或返回失败。

    if (best_candidate.empty())
    {
      return false;
    }

    *joint_target = best_candidate;
    if (selected_score != nullptr)
    {
      *selected_score = best_score;
    }
    RCLCPP_INFO(
        get_logger(),
        "Selected posture-biased IK target for group '%s' using %zu preferred seeds.",
        planning_group.c_str(),
        mapped_seeds.size());
    return true;
  }

  bool YumiPlanningInterfaceNode::mirrorJointTargetToOtherArm(
      const std::string &source_group,
      const std::string &target_group,
      const std::vector<double> &source_joint_target,
      std::vector<double> *mirrored_seed)
  {
    if (mirrored_seed == nullptr)
    {
      return false;
    }

    const bool source_is_left = source_group == "arm_l";
    const bool source_is_right = source_group == "arm_r";
    const bool target_is_left = target_group == "arm_l";
    const bool target_is_right = target_group == "arm_r";
    if ((!source_is_left && !source_is_right) || (!target_is_left && !target_is_right))
    {
      return false;
    }

    auto source_move_group = getMoveGroup(source_group);
    auto target_move_group = getMoveGroup(target_group);
    if (!source_move_group || !target_move_group)
    {
      return false;
    }

    auto source_state = source_move_group->getCurrentState();
    auto target_state = target_move_group->getCurrentState();
    if (!source_state || !target_state)
    {
      return false;
    }

    const auto *source_joint_model_group = source_state->getJointModelGroup(source_group);
    const auto *target_joint_model_group = target_state->getJointModelGroup(target_group);
    if (source_joint_model_group == nullptr || target_joint_model_group == nullptr)
    {
      return false;
    }

    const auto source_configured_order = source_is_left ?
        std::vector<std::string>{
            "yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l", "yumi_joint_3_l",
            "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l"} :
        std::vector<std::string>{
            "yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r", "yumi_joint_3_r",
            "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r"};
    const auto target_configured_order = target_is_left ?
        std::vector<std::string>{
            "yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l", "yumi_joint_3_l",
            "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l"} :
        std::vector<std::string>{
            "yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r", "yumi_joint_3_r",
            "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r"};
    const auto &source_moveit_order = source_joint_model_group->getVariableNames();
    const auto &target_moveit_order = target_joint_model_group->getVariableNames();
    const auto symmetry_signs = get_parameter("dual_arm_symmetry_signs").as_double_array();
    if (symmetry_signs.size() < source_configured_order.size())
    {
      return false;
    }

    std::vector<double> source_configured_values(source_configured_order.size(), 0.0);
    for (std::size_t source_index = 0; source_index < source_moveit_order.size() &&
                                      source_index < source_joint_target.size();
         ++source_index)
    {
      const auto match = std::find(
          source_configured_order.begin(),
          source_configured_order.end(),
          source_moveit_order[source_index]);
      if (match == source_configured_order.end())
      {
        return false;
      }
      const auto configured_index =
          static_cast<std::size_t>(std::distance(source_configured_order.begin(), match));
      source_configured_values[configured_index] = source_joint_target[source_index];
    }

    std::vector<double> mirrored_configured_values(target_configured_order.size(), 0.0);
    for (std::size_t index = 0; index < mirrored_configured_values.size(); ++index)
    {
      mirrored_configured_values[index] = symmetry_signs[index] * source_configured_values[index];
    }

    mirrored_seed->assign(target_moveit_order.size(), 0.0);
    for (std::size_t target_index = 0; target_index < target_moveit_order.size(); ++target_index)
    {
      const auto match = std::find(
          target_configured_order.begin(),
          target_configured_order.end(),
          target_moveit_order[target_index]);
      if (match == target_configured_order.end())
      {
        return false;
      }
      const auto configured_index =
          static_cast<std::size_t>(std::distance(target_configured_order.begin(), match));
      (*mirrored_seed)[target_index] = mirrored_configured_values[configured_index];
    }
    return true;
  }

  bool YumiPlanningInterfaceNode::planToPose(
      const std::string &planning_group,
      const geometry_msgs::msg::PoseStamped &target_pose,
      moveit::planning_interface::MoveGroupInterface::Plan *plan_result)
  {
    const std::lock_guard<std::recursive_mutex> planning_lock(planning_mutex_);

    RCLCPP_INFO(
        get_logger(),
        "Planning to a pose target for group '%s' in frame '%s'.",
        planning_group.c_str(),
        target_pose.header.frame_id.c_str());

    auto move_group = getMoveGroup(planning_group);
    if (!move_group)
    {
      return false;
    }

    const auto end_effector_link = getEndEffectorLink(planning_group);
    if (end_effector_link.empty())
    {
      RCLCPP_ERROR(
          get_logger(),
          "End effector link for planning group '%s' is empty.",
          planning_group.c_str());
      return false;
    }

    // 每次规划前都以当前机器人状态作为起点，
    // 这样 API 层行为与 RViz 中的“从当前状态开始规划”保持一致。
    RCLCPP_INFO(
        get_logger(),
        "Using the current robot state as the planning start state for group '%s'.",
        planning_group.c_str());
    move_group->setStartStateToCurrentState();

    // 显式设置参考系，避免目标 pose 的 frame 与 MoveGroup 默认 frame 不一致。
    move_group->setPoseReferenceFrame(target_pose.header.frame_id);

    std::vector<double> posture_biased_joint_target;
    const bool has_posture_biased_target =
        findPostureBiasedIkTarget(
            planning_group,
            target_pose,
            end_effector_link,
            move_group,
            &posture_biased_joint_target);
    if (has_posture_biased_target)
    {
      // 末端 pose 仍然是外部输入，但先由 preferred IK seed 选出更自然的 q_goal，
      // 再让 MoveIt 在关节空间规划到这组明确的目标构型。
      move_group->setJointValueTarget(posture_biased_joint_target);
    }
    else
    {
      // 如果带偏好 IK 没找到可用解，保留原来的 pose target 流程作为回退路径。
      move_group->setPoseTarget(target_pose.pose, end_effector_link);
    }

    moveit::planning_interface::MoveGroupInterface::Plan local_plan;
    RCLCPP_INFO(
        get_logger(),
        "Calling MoveIt planner for group '%s'.",
        planning_group.c_str());
    const auto success = static_cast<bool>(move_group->plan(local_plan));

    // 规划结束后清掉 pose target，避免下一次调用继续沿用旧目标。
    move_group->clearPoseTargets();

    if (!success)
    {
      RCLCPP_WARN(
          get_logger(),
          "Planning to pose failed for planning group '%s'.",
          planning_group.c_str());
      return false;
    }

    if (plan_result)
    {
      *plan_result = local_plan;
    }

    RCLCPP_INFO(
        get_logger(),
        "Planning to pose succeeded for planning group '%s'.",
        planning_group.c_str());
    return true;
  }

  bool YumiPlanningInterfaceNode::planToPose(
      const std::string &planning_group,
      const yumi_interfaces::msg::Pose6D &target_pose,
      moveit::planning_interface::MoveGroupInterface::Plan *plan_result)
  {
    // 读取默认参考坐标系，让最简接口只需要传入 6 个 double。
    const auto reference_frame = get_parameter("default_pose_reference_frame").as_string();
    return planToPose(planning_group, target_pose, reference_frame, plan_result);
  }

  bool YumiPlanningInterfaceNode::planToPose(
      const std::string &planning_group,
      const yumi_interfaces::msg::Pose6D &target_pose,
      const std::string &reference_frame,
      moveit::planning_interface::MoveGroupInterface::Plan *plan_result)
  {
    // 所有简化接口最终统一转换成 PoseStamped，
    // 这样真正的规划逻辑只有一份实现，便于维护与调试。
    const auto target_pose_stamped = convertPose6DToPoseStamped(target_pose, reference_frame);
    return planToPose(planning_group, target_pose_stamped, plan_result);
  }

  bool YumiPlanningInterfaceNode::executePlan(
      const std::string &planning_group,
      const moveit::planning_interface::MoveGroupInterface::Plan &plan)
  {
    if (!require_trajectory_action_servers_)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Trajectory execution for planning group '%s' was requested, but pure planning mode is active.",
          planning_group.c_str());
      return false;
    }

    auto move_group = getMoveGroup(planning_group);
    if (!move_group)
    {
      return false;
    }

    // 执行阶段只消费已经生成好的轨迹计划，
    // 这样规划与执行可以分别调试，也方便后续插入审核或二次处理逻辑。
    RCLCPP_INFO(
        get_logger(),
        "Executing the prepared trajectory plan for planning group '%s'.",
        planning_group.c_str());
    const auto success = static_cast<bool>(move_group->execute(plan));
    if (!success)
    {
      RCLCPP_WARN(
          get_logger(),
          "Trajectory execution failed for planning group '%s'.",
          planning_group.c_str());
      return false;
    }

    RCLCPP_INFO(
        get_logger(),
        "Trajectory execution succeeded for planning group '%s'.",
        planning_group.c_str());
    return true;
  }

  control_msgs::action::FollowJointTrajectory::Goal
  YumiPlanningInterfaceNode::buildFollowJointTrajectoryGoal(
      const moveit::planning_interface::MoveGroupInterface::Plan &plan) const
  {
    control_msgs::action::FollowJointTrajectory::Goal goal;

    // 这里直接复用 MoveIt 规划出的 JointTrajectory。
    // 当前不再人为改写绝对起始时间戳，保持控制器按默认语义立即接收执行。
    goal.trajectory = plan.trajectory_.joint_trajectory;
    return goal;
  }

  bool YumiPlanningInterfaceNode::sendTrajectoryGoal(
      const std::string &planning_group,
      const control_msgs::action::FollowJointTrajectory::Goal &goal,
      rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr *goal_handle)
  {
    if (!require_trajectory_action_servers_)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Trajectory execution for planning group '%s' was requested, but pure planning mode is active.",
          planning_group.c_str());
      return false;
    }

    auto client_it = trajectory_action_clients_.find(planning_group);
    if (client_it == trajectory_action_clients_.end())
    {
      RCLCPP_ERROR(
          get_logger(),
          "No trajectory action client was configured for planning group '%s'.",
          planning_group.c_str());
      return false;
    }

    // action goal 先被对应控制器接受，后面再等待执行结果。
    RCLCPP_INFO(
        get_logger(),
        "Sending trajectory goal to planning group '%s'.",
        planning_group.c_str());
    auto future_goal_handle = client_it->second->async_send_goal(goal);
    if (future_goal_handle.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Timed out while waiting for the controller of planning group '%s' to accept the goal.",
          planning_group.c_str());
      return false;
    }

    auto accepted_goal_handle = future_goal_handle.get();
    if (!accepted_goal_handle)
    {
      RCLCPP_ERROR(
          get_logger(),
          "The controller of planning group '%s' rejected the trajectory goal.",
          planning_group.c_str());
      return false;
    }

    if (goal_handle)
    {
      *goal_handle = accepted_goal_handle;
    }

    return true;
  }

  bool YumiPlanningInterfaceNode::waitForTrajectoryResult(
      const std::string &planning_group,
      const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr &goal_handle)
  {
    if (!require_trajectory_action_servers_)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Trajectory execution result was requested for planning group '%s', but pure planning mode is active.",
          planning_group.c_str());
      return false;
    }

    auto client_it = trajectory_action_clients_.find(planning_group);
    if (client_it == trajectory_action_clients_.end())
    {
      RCLCPP_ERROR(
          get_logger(),
          "No trajectory action client was configured for planning group '%s'.",
          planning_group.c_str());
      return false;
    }

    auto result_future = client_it->second->async_get_result(goal_handle);
    const auto result_timeout = get_parameter("trajectory_execution_result_timeout").as_double();
    if (result_future.wait_for(std::chrono::duration<double>(result_timeout)) != std::future_status::ready)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Timed out while waiting for the execution result of planning group '%s'.",
          planning_group.c_str());
      return false;
    }

    const auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_WARN(
          get_logger(),
          "Trajectory execution finished with a non-success result for planning group '%s'.",
          planning_group.c_str());
      return false;
    }

    return true;
  }

  bool YumiPlanningInterfaceNode::planBothArmsToPoses(
      const geometry_msgs::msg::PoseStamped &left_target_pose,
      const geometry_msgs::msg::PoseStamped &right_target_pose,
      moveit::planning_interface::MoveGroupInterface::Plan *left_plan_result,
      moveit::planning_interface::MoveGroupInterface::Plan *right_plan_result)
  {
    // 双臂接口当前是“统一接收双臂目标，内部左右臂分别规划”。
    // 这样 action 层对外保持双臂语义，同时保留后续替换为真正联合规划的空间。
    const std::lock_guard<std::recursive_mutex> planning_lock(planning_mutex_);

    moveit::planning_interface::MoveGroupInterface::Plan left_plan;
    moveit::planning_interface::MoveGroupInterface::Plan right_plan;

    // 双臂接口当前采用“左右臂分别规划、整体统一判定”的保守策略。
    // 这样先把 API 结构立住，后续如果要做真正耦合规划，再替换内部实现即可。
    RCLCPP_INFO(
        get_logger(),
        "Planning both arms together. Left arm target frame: '%s', right arm target frame: '%s'.",
        left_target_pose.header.frame_id.c_str(),
        right_target_pose.header.frame_id.c_str());

    auto left_move_group = getMoveGroup("arm_l");
    auto right_move_group = getMoveGroup("arm_r");
    const auto left_end_effector_link = getEndEffectorLink("arm_l");
    const auto right_end_effector_link = getEndEffectorLink("arm_r");
    std::vector<double> left_joint_target;
    std::vector<double> right_joint_target;
    double left_score = std::numeric_limits<double>::infinity();
    double right_score = std::numeric_limits<double>::infinity();
    RCLCPP_INFO(get_logger(), "Finding posture-biased IK target for left arm.");
    const bool left_has_biased_ik =
        findPostureBiasedIkTarget(
            "arm_l",
            left_target_pose,
            left_end_effector_link,
            left_move_group,
            {},
            &left_joint_target,
            &left_score);
    RCLCPP_INFO(
        get_logger(),
        "Left posture-biased IK %s.",
        left_has_biased_ik ? "succeeded" : "failed");
    RCLCPP_INFO(get_logger(), "Finding posture-biased IK target for right arm.");
    const bool right_has_biased_ik =
        findPostureBiasedIkTarget(
            "arm_r",
            right_target_pose,
            right_end_effector_link,
            right_move_group,
            {},
            &right_joint_target,
            &right_score);
    RCLCPP_INFO(
        get_logger(),
        "Right posture-biased IK %s.",
        right_has_biased_ik ? "succeeded" : "failed");

    if (left_has_biased_ik && right_has_biased_ik)
    {
      // 第一轮先让左右臂各自独立求一组 posture-biased IK。
      // 这两组解不互相约束，只代表“单臂视角下比较自然”的 q_goal。
      const auto left_independent_joint_target = left_joint_target;
      const auto right_independent_joint_target = right_joint_target;
      const double symmetry_weight = get_parameter("dual_arm_symmetry_weight").as_double();

      // 把右臂独立 q_goal 镜像成“如果左臂要和右臂对称，它大概应该靠近的 q”。
      // 这个 mirrored_right_joint_target_for_left 会同时作为：
      // 1. extra IK seed：帮助左臂从镜像附近搜索 IK 分支；
      // 2. symmetry score reference：惩罚左臂候选解偏离镜像构型。
      std::vector<double> mirrored_right_joint_target_for_left;
      if (mirrorJointTargetToOtherArm(
              "arm_r", "arm_l", right_independent_joint_target, &mirrored_right_joint_target_for_left))
      {
        std::vector<double> left_from_right;
        double left_from_right_score = std::numeric_limits<double>::infinity();
        if (findPostureBiasedIkTarget(
                "arm_l",
                left_target_pose,
                left_end_effector_link,
                left_move_group,
                {mirrored_right_joint_target_for_left},
                mirrored_right_joint_target_for_left,
                symmetry_weight,
                &left_from_right,
                &left_from_right_score))
        {
          left_joint_target = left_from_right;
          left_score = left_from_right_score;
          RCLCPP_INFO(
              get_logger(),
              "Left arm selected symmetry-aware IK candidate. score=%.6f",
              left_score);
        }
      }

      // 右臂采用完全对称的处理：
      // 先把左臂独立 q_goal 镜像到右臂，再用它引导右臂 IK 搜索和评分。
      std::vector<double> mirrored_left_joint_target_for_right;
      if (mirrorJointTargetToOtherArm(
              "arm_l", "arm_r", left_independent_joint_target, &mirrored_left_joint_target_for_right))
      {
        std::vector<double> right_from_left;
        double right_from_left_score = std::numeric_limits<double>::infinity();
        if (findPostureBiasedIkTarget(
                "arm_r",
                right_target_pose,
                right_end_effector_link,
                right_move_group,
                {mirrored_left_joint_target_for_right},
                mirrored_left_joint_target_for_right,
                symmetry_weight,
                &right_from_left,
                &right_from_left_score))
        {
          right_joint_target = right_from_left;
          right_score = right_from_left_score;
          RCLCPP_INFO(
              get_logger(),
              "Right arm selected symmetry-aware IK candidate. score=%.6f",
              right_score);
        }
      }
    }

    const auto plan_to_joint_target =
        [this](
            const std::string &planning_group,
            const geometry_msgs::msg::PoseStamped &target_pose,
            const std::vector<double> &joint_target,
            moveit::planning_interface::MoveGroupInterface::Plan *plan) -> bool {
          auto move_group = getMoveGroup(planning_group);
          if (!move_group || plan == nullptr)
          {
            return false;
          }
          // 每次规划前都以当前 RobotState 为起点。
          // 这避免 MoveIt 沿用旧 start state，尤其适合连续序列任务。
          move_group->setStartStateToCurrentState();
          // 先使用 posture-biased IK 得到 q_goal，再交给 MoveIt 做关节空间路径搜索。
          // 这比裸 pose target 更容易控制肘部/腕部构型。
          move_group->setPoseReferenceFrame(target_pose.header.frame_id);
          move_group->setJointValueTarget(joint_target);
          RCLCPP_INFO(
              get_logger(),
              "Calling MoveIt planner for group '%s' with posture-biased joint target.",
              planning_group.c_str());
          const auto plan_start_time = now();
          const bool success = static_cast<bool>(move_group->plan(*plan));
          RCLCPP_INFO(
              get_logger(),
              "MoveIt planner for group '%s' returned %s in %.3f seconds.",
              planning_group.c_str(),
              success ? "success" : "failure",
              (now() - plan_start_time).seconds());
          return success;
        };

    const bool left_plan_success = left_has_biased_ik ?
        plan_to_joint_target("arm_l", left_target_pose, left_joint_target, &left_plan) :
        planToPose("arm_l", left_target_pose, &left_plan);
    if (!left_plan_success)
    {
      RCLCPP_WARN(
          get_logger(),
          "Dual-arm planning failed because the left arm plan was not generated successfully.");
      return false;
    }

    const bool right_plan_success = right_has_biased_ik ?
        plan_to_joint_target("arm_r", right_target_pose, right_joint_target, &right_plan) :
        planToPose("arm_r", right_target_pose, &right_plan);
    if (!right_plan_success)
    {
      RCLCPP_WARN(
          get_logger(),
          "Dual-arm planning failed because the right arm plan was not generated successfully.");
      return false;
    }

    if (left_plan_result)
    {
      *left_plan_result = left_plan;
    }

    if (right_plan_result)
    {
      *right_plan_result = right_plan;
    }

    RCLCPP_INFO(
        get_logger(),
        "Dual-arm planning succeeded for both left and right arms.");
    return true;
  }

  bool YumiPlanningInterfaceNode::planBothArmsToPoses(
      const yumi_interfaces::msg::Pose6D &left_target_pose,
      const yumi_interfaces::msg::Pose6D &right_target_pose,
      moveit::planning_interface::MoveGroupInterface::Plan *left_plan_result,
      moveit::planning_interface::MoveGroupInterface::Plan *right_plan_result)
  {
    const auto reference_frame = get_parameter("default_pose_reference_frame").as_string();
    return planBothArmsToPoses(
        left_target_pose,
        right_target_pose,
        reference_frame,
        left_plan_result,
        right_plan_result);
  }

  bool YumiPlanningInterfaceNode::planBothArmsToPoses(
      const yumi_interfaces::msg::Pose6D &left_target_pose,
      const yumi_interfaces::msg::Pose6D &right_target_pose,
      const std::string &reference_frame,
      moveit::planning_interface::MoveGroupInterface::Plan *left_plan_result,
      moveit::planning_interface::MoveGroupInterface::Plan *right_plan_result)
  {
    const auto left_target_pose_stamped = convertPose6DToPoseStamped(left_target_pose, reference_frame);
    const auto right_target_pose_stamped = convertPose6DToPoseStamped(right_target_pose, reference_frame);
    return planBothArmsToPoses(
        left_target_pose_stamped,
        right_target_pose_stamped,
        left_plan_result,
        right_plan_result);
  }

  bool YumiPlanningInterfaceNode::executeBothArmPlans(
      const moveit::planning_interface::MoveGroupInterface::Plan &left_plan,
      const moveit::planning_interface::MoveGroupInterface::Plan &right_plan)
  {
    // 双臂执行阶段直接复用 MoveIt 规划出的两条轨迹，
    // 再分别发送给左右臂控制器，以获得稳定的近同步启动效果。
    const auto left_goal = buildFollowJointTrajectoryGoal(left_plan);
    const auto right_goal = buildFollowJointTrajectoryGoal(right_plan);

    rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr left_goal_handle;
    rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr right_goal_handle;

    RCLCPP_INFO(
        get_logger(),
        "Sending left and right arm trajectories for near-synchronous execution.");
    if (!sendTrajectoryGoal("arm_l", left_goal, &left_goal_handle))
    {
      return false;
    }

    if (!sendTrajectoryGoal("arm_r", right_goal, &right_goal_handle))
    {
      return false;
    }

    // 左右臂 goal 都被接受后，再分别等待执行完成。
    // 这样可以在保持近同步启动的同时，拿到两侧控制器的最终执行结果。
    if (!waitForTrajectoryResult("arm_l", left_goal_handle))
    {
      return false;
    }

    if (!waitForTrajectoryResult("arm_r", right_goal_handle))
    {
      return false;
    }

    RCLCPP_INFO(
        get_logger(),
        "Dual-arm trajectories completed successfully.");
    return true;
  }

  bool YumiPlanningInterfaceNode::planAndExecuteToPose(
      const std::string &planning_group,
      const geometry_msgs::msg::PoseStamped &target_pose)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // 组合接口先完成规划，只有规划成功后才会进入执行阶段。
    if (!planToPose(planning_group, target_pose, &plan))
    {
      return false;
    }

    return executePlan(planning_group, plan);
  }

  bool YumiPlanningInterfaceNode::planAndExecuteToPose(
      const std::string &planning_group,
      const yumi_interfaces::msg::Pose6D &target_pose)
  {
    const auto reference_frame = get_parameter("default_pose_reference_frame").as_string();
    return planAndExecuteToPose(planning_group, target_pose, reference_frame);
  }

  bool YumiPlanningInterfaceNode::planAndExecuteToPose(
      const std::string &planning_group,
      const yumi_interfaces::msg::Pose6D &target_pose,
      const std::string &reference_frame)
  {
    const auto target_pose_stamped = convertPose6DToPoseStamped(target_pose, reference_frame);
    return planAndExecuteToPose(planning_group, target_pose_stamped);
  }

  bool YumiPlanningInterfaceNode::resetArmToHome(const std::string &planning_group)
  {
    auto move_group = getMoveGroup(planning_group);
    if (!move_group)
    {
      return false;
    }

    const auto home_target_name = getHomeTargetName(planning_group);
    if (home_target_name.empty())
    {
      RCLCPP_ERROR(
          get_logger(),
          "No home target is defined for planning group '%s'.",
          planning_group.c_str());
      return false;
    }

    // 单臂复位阶段直接复用 MoveIt 的命名姿态能力，
    // 这样 home 姿态的唯一主源仍然保持在 SRDF，而不是分散到多个 C++ 文件里。
    RCLCPP_INFO(
        get_logger(),
        "Resetting planning group '%s' to named target '%s'.",
        planning_group.c_str(),
        home_target_name.c_str());
    // 告诉 MoveIt：这次规划的起点应该使用机器人“此刻的真实状态”，
    // 而不是沿用上一次规划缓存下来的旧起点。
    move_group->setStartStateToCurrentState();
    // 告诉 MoveIt：这次规划的目标不是一个末端位姿，而是 SRDF 中已经命名好的关节构型。
    // 对 arm_l / arm_r 来说，这里分别会切到 home_l / home_r 这组关节角。
    if (!move_group->setNamedTarget(home_target_name))
    {
      RCLCPP_ERROR(
          get_logger(),
          "Failed to set named target '%s' for planning group '%s'.",
          home_target_name.c_str(),
          planning_group.c_str());
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto planning_success = static_cast<bool>(move_group->plan(plan));
    if (!planning_success)
    {
      RCLCPP_WARN(
          get_logger(),
          "Planning home reset trajectory failed for planning group '%s'.",
          planning_group.c_str());
      return false;
    }

    move_group->clearPoseTargets();
    return executePlan(planning_group, plan);
  }

  bool YumiPlanningInterfaceNode::planAndExecuteBothArmsToPoses(
      const geometry_msgs::msg::PoseStamped &left_target_pose,
      const geometry_msgs::msg::PoseStamped &right_target_pose)
  {
    moveit::planning_interface::MoveGroupInterface::Plan left_plan;
    moveit::planning_interface::MoveGroupInterface::Plan right_plan;

    // 组合接口先确保两侧轨迹都已经规划出来，再统一进入执行阶段。
    if (!planBothArmsToPoses(left_target_pose, right_target_pose, &left_plan, &right_plan))
    {
      return false;
    }

    return executeBothArmPlans(left_plan, right_plan);
  }

  bool YumiPlanningInterfaceNode::planAndExecuteBothArmsToPoses(
      const yumi_interfaces::msg::Pose6D &left_target_pose,
      const yumi_interfaces::msg::Pose6D &right_target_pose)
  {
    const auto reference_frame = get_parameter("default_pose_reference_frame").as_string();
    return planAndExecuteBothArmsToPoses(left_target_pose, right_target_pose, reference_frame);
  }

  bool YumiPlanningInterfaceNode::planAndExecuteBothArmsToPoses(
      const yumi_interfaces::msg::Pose6D &left_target_pose,
      const yumi_interfaces::msg::Pose6D &right_target_pose,
      const std::string &reference_frame)
  {
    const auto left_target_pose_stamped = convertPose6DToPoseStamped(left_target_pose, reference_frame);
    const auto right_target_pose_stamped = convertPose6DToPoseStamped(right_target_pose, reference_frame);
    return planAndExecuteBothArmsToPoses(left_target_pose_stamped, right_target_pose_stamped);
  }

  bool YumiPlanningInterfaceNode::resetBothArmsToHome()
  {
    auto left_move_group = getMoveGroup("arm_l");
    auto right_move_group = getMoveGroup("arm_r");
    if (!left_move_group || !right_move_group)
    {
      return false;
    }

    const auto left_home_target_name = getHomeTargetName("arm_l");
    const auto right_home_target_name = getHomeTargetName("arm_r");
    if (left_home_target_name.empty() || right_home_target_name.empty())
    {
      RCLCPP_ERROR(
          get_logger(),
          "Dual-arm home reset failed because one of the home target names is missing.");
      return false;
    }

    // 双臂复位也保持“左右臂分别规划”的当前架构语义，
    // 这样可以继续复用已经验证通过的双臂近同步执行链。
    RCLCPP_INFO(
        get_logger(),
        "Resetting both arms to named targets '%s' and '%s'.",
        left_home_target_name.c_str(),
        right_home_target_name.c_str());
    // 先把左右臂两条规划链的起始状态都对齐到当前真实 joint_states，
    // 避免双臂复位时仍然从旧缓存状态出发。
    left_move_group->setStartStateToCurrentState();
    right_move_group->setStartStateToCurrentState();

    // 将左臂的规划目标设置为 SRDF 里定义好的命名姿态 home_l。
    if (!left_move_group->setNamedTarget(left_home_target_name))
    {
      RCLCPP_ERROR(
          get_logger(),
          "Failed to set left arm home target '%s'.",
          left_home_target_name.c_str());
      return false;
    }

    // 将右臂的规划目标设置为 SRDF 里定义好的命名姿态 home_r。
    if (!right_move_group->setNamedTarget(right_home_target_name))
    {
      RCLCPP_ERROR(
          get_logger(),
          "Failed to set right arm home target '%s'.",
          right_home_target_name.c_str());
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan left_plan;
    moveit::planning_interface::MoveGroupInterface::Plan right_plan;
    const auto left_planning_success = static_cast<bool>(left_move_group->plan(left_plan));
    const auto right_planning_success = static_cast<bool>(right_move_group->plan(right_plan));

    if (!left_planning_success || !right_planning_success)
    {
      RCLCPP_WARN(
          get_logger(),
          "Dual-arm home reset planning failed. Left success: %s, right success: %s.",
          left_planning_success ? "true" : "false",
          right_planning_success ? "true" : "false");
      return false;
    }

    left_move_group->clearPoseTargets();
    right_move_group->clearPoseTargets();
    return executeBothArmPlans(left_plan, right_plan);
  }

  bool YumiPlanningInterfaceNode::planTaskSpaceTrajectory(
      const PlanTaskSpaceTrajectory::Goal & goal,
      yumi_interfaces::msg::TaskSpaceTrajectory * trajectory_result)
  {
    const std::lock_guard<std::recursive_mutex> planning_lock(planning_mutex_);

    // 这条接口的目标不是直接把 MoveIt 原始 joint trajectory 暴露给上层，
    // 而是把它统一转换成“带时间语义的任务空间轨迹”，方便协同控制层和下层控制器使用。
    if (trajectory_result == nullptr)
    {
      RCLCPP_ERROR(get_logger(), "planTaskSpaceTrajectory received a null trajectory_result pointer.");
      return false;
    }

    const auto reference_frame =
        goal.reference_frame.empty() ? get_parameter("default_pose_reference_frame").as_string() : goal.reference_frame;
    const auto planning_time =
        goal.planning_time > 0.0 ? goal.planning_time : get_parameter("default_task_space_planning_time").as_double();
    const auto num_planning_attempts =
        goal.num_planning_attempts > 0 ? goal.num_planning_attempts : get_parameter("default_task_space_num_planning_attempts").as_int();
    const auto velocity_scaling =
        goal.velocity_scaling > 0.0 ? goal.velocity_scaling : get_parameter("default_task_space_velocity_scaling").as_double();
    const auto acceleration_scaling =
        goal.acceleration_scaling > 0.0 ? goal.acceleration_scaling : get_parameter("default_task_space_acceleration_scaling").as_double();
    const auto sample_period =
        goal.sample_period > 0.0 ? goal.sample_period : get_parameter("default_task_space_sample_period").as_double();
    const auto configured_duration_scale = get_parameter("default_task_space_duration_scale").as_double();
    const auto duration_scale = configured_duration_scale > 0.0 ? configured_duration_scale : 1.0;

    // 当前接口先把笛卡尔路径开关暴露出来，便于后续扩展。
    // 第一版仍优先把“任务空间目标规划 -> 任务空间轨迹输出”主链走通。
    if (goal.use_cartesian_path)
    {
      RCLCPP_WARN(
          get_logger(),
          "use_cartesian_path=true is not implemented yet in planTaskSpaceTrajectory. Falling back to standard pose planning.");
    }

    trajectory_result->reference_frame = reference_frame;
    trajectory_result->points.clear();

    // 对 MoveGroupInterface 的规划参数统一在这里下发，
    // 这样单臂/双臂两条路径都使用同一套参数语义。
    auto configure_move_group =
        [&](const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & move_group)
        {
          move_group->setPlanningTime(planning_time);
          move_group->setNumPlanningAttempts(num_planning_attempts);
          move_group->setMaxVelocityScalingFactor(velocity_scaling);
          move_group->setMaxAccelerationScalingFactor(acceleration_scaling);
        };

    auto duration_to_seconds =
        [](const builtin_interfaces::msg::Duration & duration_msg)
        {
          // MoveIt 轨迹点内部使用 sec + nanosec，
          // 这里统一转成 double 秒，方便后续重采样和插值。
          return static_cast<double>(duration_msg.sec) +
                 static_cast<double>(duration_msg.nanosec) * 1e-9;
        };

    auto seconds_to_duration =
        [](double seconds)
        {
          builtin_interfaces::msg::Duration duration_msg;
          if (seconds < 0.0)
          {
            seconds = 0.0;
          }

          duration_msg.sec = static_cast<int32_t>(std::floor(seconds));
          duration_msg.nanosec = static_cast<uint32_t>((seconds - static_cast<double>(duration_msg.sec)) * 1e9);
          return duration_msg;
        };

    auto transform_to_task_space_pose =
        [&](const Eigen::Isometry3d & transform)
        {
          yumi_interfaces::msg::TaskSpacePose pose;
          pose.x = transform.translation().x();
          pose.y = transform.translation().y();
          pose.z = transform.translation().z();

          // 任务空间轨迹直接输出四元数姿态，避免中间再退回到 RPY 表达。
          // 这样后续做姿态误差计算、阻抗控制和 slerp 插值都更自然。
          const Eigen::Quaterniond quaternion(transform.rotation());
          pose.qx = quaternion.x();
          pose.qy = quaternion.y();
          pose.qz = quaternion.z();
          pose.qw = quaternion.w();
          return pose;
        };

    auto pose6d_to_task_space_pose =
        [&](const yumi_interfaces::msg::Pose6D & pose6d)
        {
          // 上层 goal 仍然使用 Pose6D 输入；
          // 这里统一通过 PoseStamped 转成四元数表示，避免手工重复做 RPY -> quaternion。
          const auto pose_stamped = convertPose6DToPoseStamped(pose6d, reference_frame);
          yumi_interfaces::msg::TaskSpacePose pose;
          pose.x = pose_stamped.pose.position.x;
          pose.y = pose_stamped.pose.position.y;
          pose.z = pose_stamped.pose.position.z;
          pose.qx = pose_stamped.pose.orientation.x;
          pose.qy = pose_stamped.pose.orientation.y;
          pose.qz = pose_stamped.pose.orientation.z;
          pose.qw = pose_stamped.pose.orientation.w;
          return pose;
        };

    auto interpolate_positions =
        [&](const trajectory_msgs::msg::JointTrajectory & trajectory, double query_time_sec)
        {
          std::vector<double> interpolated_positions;
          if (trajectory.points.empty())
          {
            return interpolated_positions;
          }

          if (query_time_sec <= duration_to_seconds(trajectory.points.front().time_from_start))
          {
            return trajectory.points.front().positions;
          }

          if (query_time_sec >= duration_to_seconds(trajectory.points.back().time_from_start))
          {
            return trajectory.points.back().positions;
          }

          for (size_t point_index = 1; point_index < trajectory.points.size(); ++point_index)
          {
            const auto & previous_point = trajectory.points[point_index - 1];
            const auto & next_point = trajectory.points[point_index];
            const double previous_time = duration_to_seconds(previous_point.time_from_start);
            const double next_time = duration_to_seconds(next_point.time_from_start);

            if (query_time_sec > next_time)
            {
              continue;
            }

            // 这里采用关节空间线性插值：
            // 先在 MoveIt 规划结果的 joint trajectory 上做时间采样，
            // 再对采样后的关节位形做 FK，得到任务空间轨迹点。
            const double segment_duration = next_time - previous_time;
            const double alpha = segment_duration <= 1e-9 ? 0.0 : (query_time_sec - previous_time) / segment_duration;
            interpolated_positions.resize(previous_point.positions.size(), 0.0);
            for (size_t joint_index = 0; joint_index < previous_point.positions.size(); ++joint_index)
            {
              interpolated_positions[joint_index] =
                  previous_point.positions[joint_index] +
                  alpha * (next_point.positions[joint_index] - previous_point.positions[joint_index]);
            }
            return interpolated_positions;
          }

          return trajectory.points.back().positions;
        };

    auto build_point =
        [&](double sampled_time_sec)
        {
          yumi_interfaces::msg::TaskSpaceTrajectoryPoint point;
          // 输出轨迹点的时间语义可以相对 MoveIt 原始时长做统一缩放，
          // 这样上层在不改变空间路径的前提下，可以整体放慢或加快任务节奏。
          point.time_from_start = seconds_to_duration(sampled_time_sec * duration_scale);
          point.use_object_target = goal.use_object_target;
          point.enable_force_control = goal.enable_force_control;
          point.target_internal_force = goal.target_internal_force;
          point.object_target_pose = pose6d_to_task_space_pose(goal.object_target_pose);
          return point;
        };

    if (!goal.use_both_arms)
    {
      // 单臂模式下，只为 arm_l 或 arm_r 生成一条任务空间轨迹。
      const auto planning_group = getPlanningGroupFromArmSelector(goal.arm);
      if (planning_group.empty())
      {
        RCLCPP_ERROR(get_logger(), "Task-space trajectory goal specified an invalid single-arm selector.");
        return false;
      }

      auto move_group = getMoveGroup(planning_group);
      if (!move_group)
      {
        return false;
      }
      configure_move_group(move_group);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto & target_pose =
          planning_group == "arm_l" ? goal.left_target_pose : goal.right_target_pose;
      if (!planToPose(planning_group, target_pose, reference_frame, &plan))
      {
        RCLCPP_WARN(get_logger(), "Failed to plan single-arm task-space trajectory for group '%s'.", planning_group.c_str());
        return false;
      }

      if (plan.trajectory_.joint_trajectory.points.empty())
      {
        RCLCPP_ERROR(get_logger(), "Single-arm MoveIt plan returned an empty joint trajectory.");
        return false;
      }

      const auto current_state = move_group->getCurrentState(5.0);
      if (!current_state)
      {
        RCLCPP_ERROR(get_logger(), "Failed to get current robot state for task-space trajectory sampling.");
        return false;
      }

      moveit::core::RobotState sampled_state(*current_state);
      const auto total_duration_sec = duration_to_seconds(plan.trajectory_.joint_trajectory.points.back().time_from_start);

      // 对整条 joint trajectory 按 sample_period 均匀采样。
      // 每个采样时刻先得到关节位形，再用 FK 计算末端任务空间点。
      for (double sampled_time_sec = 0.0; sampled_time_sec <= total_duration_sec + 1e-9; sampled_time_sec += sample_period)
      {
        auto point = build_point(sampled_time_sec);
        const auto interpolated_positions =
            interpolate_positions(plan.trajectory_.joint_trajectory, sampled_time_sec);
        sampled_state.setJointGroupPositions(
            planning_group,
            interpolated_positions);
        sampled_state.update();

        const auto end_effector_pose =
            transform_to_task_space_pose(sampled_state.getGlobalLinkTransform(getEndEffectorLink(planning_group)));
        if (planning_group == "arm_l")
        {
          point.left_target_pose = end_effector_pose;
        }
        else
        {
          point.right_target_pose = end_effector_pose;
        }

        trajectory_result->points.push_back(point);
      }

      return !trajectory_result->points.empty();
    }

    // 双臂模式下，先分别规划左右臂，再用统一的采样时轴重建双臂任务空间轨迹。
    auto left_move_group = getMoveGroup("arm_l");
    auto right_move_group = getMoveGroup("arm_r");
    if (!left_move_group || !right_move_group)
    {
      return false;
    }
    configure_move_group(left_move_group);
    configure_move_group(right_move_group);

    moveit::planning_interface::MoveGroupInterface::Plan left_plan;
    moveit::planning_interface::MoveGroupInterface::Plan right_plan;
    if (!planBothArmsToPoses(
            goal.left_target_pose,
            goal.right_target_pose,
            reference_frame,
            &left_plan,
            &right_plan))
    {
      RCLCPP_WARN(get_logger(), "Failed to plan dual-arm task-space trajectory.");
      return false;
    }

    if (left_plan.trajectory_.joint_trajectory.points.empty() ||
        right_plan.trajectory_.joint_trajectory.points.empty())
    {
      RCLCPP_ERROR(get_logger(), "Dual-arm MoveIt plan returned an empty joint trajectory.");
      return false;
    }

    const auto current_state = left_move_group->getCurrentState(5.0);
    if (!current_state)
    {
      RCLCPP_ERROR(get_logger(), "Failed to get current robot state for dual-arm task-space trajectory sampling.");
      return false;
    }

    moveit::core::RobotState sampled_state(*current_state);
    const double total_duration_sec = std::max(
        duration_to_seconds(left_plan.trajectory_.joint_trajectory.points.back().time_from_start),
        duration_to_seconds(right_plan.trajectory_.joint_trajectory.points.back().time_from_start));

    // 双臂采样时以较长的那条轨迹作为总时长，保证两条轨迹都能被完整覆盖。
    for (double sampled_time_sec = 0.0; sampled_time_sec <= total_duration_sec + 1e-9; sampled_time_sec += sample_period)
    {
      auto point = build_point(sampled_time_sec);
      const auto left_interpolated_positions =
          interpolate_positions(left_plan.trajectory_.joint_trajectory, sampled_time_sec);
      const auto right_interpolated_positions =
          interpolate_positions(right_plan.trajectory_.joint_trajectory, sampled_time_sec);

      sampled_state.setJointGroupPositions("arm_l", left_interpolated_positions);
      sampled_state.setJointGroupPositions("arm_r", right_interpolated_positions);
      sampled_state.update();

      point.left_target_pose =
          transform_to_task_space_pose(sampled_state.getGlobalLinkTransform(getEndEffectorLink("arm_l")));
      point.right_target_pose =
          transform_to_task_space_pose(sampled_state.getGlobalLinkTransform(getEndEffectorLink("arm_r")));

      if (goal.use_object_target)
      {
        // 当前第一版先用左右末端位置中点作为物体/虚拟物体的位置近似。
        // 物体姿态仍沿用上层给定的目标四元数语义，后续可以再替换成更严格的构造。
        point.object_target_pose.x = 0.5 * (point.left_target_pose.x + point.right_target_pose.x);
        point.object_target_pose.y = 0.5 * (point.left_target_pose.y + point.right_target_pose.y);
        point.object_target_pose.z = 0.5 * (point.left_target_pose.z + point.right_target_pose.z);
        const auto object_target_pose = pose6d_to_task_space_pose(goal.object_target_pose);
        point.object_target_pose.qx = object_target_pose.qx;
        point.object_target_pose.qy = object_target_pose.qy;
        point.object_target_pose.qz = object_target_pose.qz;
        point.object_target_pose.qw = object_target_pose.qw;
      }

      trajectory_result->points.push_back(point);
    }

    return !trajectory_result->points.empty();
  }

  void YumiPlanningInterfaceNode::runDemoOnce()
  {
    // 取消 timer，确保演示逻辑只执行一次，避免节点循环下发规划请求。
    demo_timer_.reset();

    const auto demo_mode = get_parameter("demo_mode").as_string();
    const auto planning_group = get_parameter("demo_planning_group").as_string();
    const auto max_attempts = get_parameter("random_pose_attempts").as_int();
    const auto execute_demo_plan = get_parameter("execute_demo_plan_on_startup").as_bool();
    const auto reset_to_home_after_demo = get_parameter("reset_to_home_after_demo").as_bool();

    if (demo_mode == "dual_arm")
    {
      RCLCPP_INFO(
          get_logger(),
          "Starting one-shot dual-arm demo with at most %d random pose attempts per arm. Execute after planning: %s. Reset to home after demo: %s.",
          max_attempts,
          execute_demo_plan ? "true" : "false",
          reset_to_home_after_demo ? "true" : "false");

      // 双臂演示阶段先分别为左右臂随机生成一个可规划的目标位姿，
      // 然后再把这两个目标一起交给双臂接口做统一规划/执行验证。
      auto left_random_pose = generateRandomValidPose("arm_l", max_attempts);
      if (!left_random_pose)
      {
        RCLCPP_WARN(
            get_logger(),
            "Dual-arm demo failed because no valid random pose was generated for the left arm.");
        return;
      }

      auto right_random_pose = generateRandomValidPose("arm_r", max_attempts);
      if (!right_random_pose)
      {
        RCLCPP_WARN(
            get_logger(),
            "Dual-arm demo failed because no valid random pose was generated for the right arm.");
        return;
      }

      if (execute_demo_plan)
      {
        RCLCPP_INFO(
            get_logger(),
            "Dual-arm demo will now plan and execute both random target poses together.");
        if (!planAndExecuteBothArmsToPoses(*left_random_pose, *right_random_pose))
        {
          RCLCPP_WARN(
              get_logger(),
              "Dual-arm demo failed during the combined plan-and-execute stage.");
          return;
        }
      }
      else
      {
        moveit::planning_interface::MoveGroupInterface::Plan left_plan;
        moveit::planning_interface::MoveGroupInterface::Plan right_plan;

        RCLCPP_INFO(
            get_logger(),
            "Dual-arm demo will now plan both random target poses without executing them.");
        if (!planBothArmsToPoses(
                *left_random_pose,
                *right_random_pose,
                &left_plan,
                &right_plan))
        {
          RCLCPP_WARN(
              get_logger(),
              "Dual-arm demo failed during the combined planning stage.");
          return;
        }
      }

      RCLCPP_INFO(
          get_logger(),
          "Dual-arm demo completed successfully.");

      if (reset_to_home_after_demo)
      {
        RCLCPP_INFO(
            get_logger(),
            "Dual-arm demo finished. Resetting both arms to home as requested.");
        if (!resetBothArmsToHome())
        {
          RCLCPP_WARN(
              get_logger(),
              "Dual-arm demo succeeded, but resetting both arms to home failed.");
        }
      }
      return;
    }

    RCLCPP_INFO(
        get_logger(),
        "Starting one-shot single-arm demo for group '%s' with at most %d random pose attempts. Execute after planning: %s. Reset to home after demo: %s.",
        planning_group.c_str(),
        max_attempts,
        execute_demo_plan ? "true" : "false",
        reset_to_home_after_demo ? "true" : "false");

    auto random_pose = generateRandomValidPose(planning_group, max_attempts);
    if (!random_pose)
    {
      RCLCPP_WARN(
          get_logger(),
          "Demo failed because no valid random pose was generated for '%s'.",
          planning_group.c_str());
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    RCLCPP_INFO(
        get_logger(),
        "A valid random pose was generated for group '%s'. Planning to that pose now.",
        planning_group.c_str());
    if (!planToPose(planning_group, *random_pose, &plan))
    {
      RCLCPP_WARN(
          get_logger(),
          "Demo failed because planning to the generated pose did not succeed for '%s'.",
          planning_group.c_str());
      return;
    }

    if (execute_demo_plan)
    {
      RCLCPP_INFO(
          get_logger(),
          "The startup demo will now execute the generated plan for group '%s'.",
          planning_group.c_str());
      if (!executePlan(planning_group, plan))
      {
        RCLCPP_WARN(
            get_logger(),
            "Demo failed because execution did not succeed for '%s'.",
            planning_group.c_str());
        return;
      }
    }

    RCLCPP_INFO(
        get_logger(),
        "Demo completed successfully for '%s'.",
        planning_group.c_str());

    if (reset_to_home_after_demo)
    {
      RCLCPP_INFO(
          get_logger(),
          "Single-arm demo finished. Resetting planning group '%s' to home as requested.",
          planning_group.c_str());
      if (!resetArmToHome(planning_group))
      {
        RCLCPP_WARN(
            get_logger(),
            "Single-arm demo succeeded, but resetting planning group '%s' to home failed.",
            planning_group.c_str());
      }
    }
  }

  rclcpp_action::GoalResponse YumiPlanningInterfaceNode::handlePlanArmPoseGoal(
      const rclcpp_action::GoalUUID & /*uuid*/,
      std::shared_ptr<const PlanArmPose::Goal> goal)
  {
    // 单臂 action 只接受左右臂两种有效枚举，
    // 这样可以在请求入口尽早挡住明显非法的调用。
    if (!goal || getPlanningGroupFromArmSelector(goal->arm).empty())
    {
      RCLCPP_WARN(
          get_logger(),
          "Rejected plan_arm_pose goal because arm selector is invalid.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse YumiPlanningInterfaceNode::handlePlanArmPoseCancel(
      const std::shared_ptr<GoalHandlePlanArmPose> /*goal_handle*/)
  {
    // 当前 action 的执行逻辑主要依赖已有同步规划/执行函数，
    // 因此先接受取消请求，并在执行线程中尽早结束后续步骤。
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void YumiPlanningInterfaceNode::handlePlanArmPoseAccepted(
      const std::shared_ptr<GoalHandlePlanArmPose> goal_handle)
  {
    // action accepted 回调必须尽快返回，
    // 真正耗时的 MoveIt 规划/执行逻辑放到后台线程中完成。
    std::thread(
        [this, goal_handle]()
        {
          executePlanArmPose(goal_handle);
        })
        .detach();
  }

  void YumiPlanningInterfaceNode::executePlanArmPose(
      const std::shared_ptr<GoalHandlePlanArmPose> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<PlanArmPose::Result>();
    auto feedback = std::make_shared<PlanArmPose::Feedback>();
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // 单臂 action 不再从外部接收 frame，
    // 简化接口层后统一按节点参数中的默认世界坐标系解释 Pose6D。
    const auto planning_group = getPlanningGroupFromArmSelector(goal->arm);

    // 先给调用方一个明确的阶段反馈，表示已经进入规划接口层。
    feedback->stage = "planning";
    goal_handle->publish_feedback(feedback);

    bool success = false;
    if (goal->execute)
    {
      // 规划+执行模式下仍然先保留规划结果，便于上层调试关节轨迹本身。
      success = planToPose(planning_group, goal->target_pose, &plan);
      if (success)
      {
        success = executePlan(planning_group, plan);
      }
    }
    else
    {
      // 仅规划模式适合上层先验证目标是否可达，再决定是否立即执行。
      success = planToPose(planning_group, goal->target_pose, &plan);
    }

    if (goal_handle->is_canceling())
    {
      result->success = false;
      result->message = "plan_arm_pose was canceled before completion";
      goal_handle->canceled(result);
      return;
    }

    result->success = success;
    result->message = success ? "plan_arm_pose completed successfully"
                              : "plan_arm_pose failed during planning or execution";
    if (success)
    {
      result->planned_trajectory = plan.trajectory_.joint_trajectory;
    }
    if (success)
    {
      feedback->stage = goal->execute ? "executed" : "planned";
      goal_handle->publish_feedback(feedback);
      goal_handle->succeed(result);
      return;
    }

    goal_handle->abort(result);
  }

  rclcpp_action::GoalResponse YumiPlanningInterfaceNode::handlePlanBothArmsPosesGoal(
      const rclcpp_action::GoalUUID & /*uuid*/,
      std::shared_ptr<const PlanBothArmsPoses::Goal> /*goal*/)
  {
    if (plan_both_arms_poses_active_.exchange(true))
    {
      RCLCPP_WARN(
          get_logger(),
          "Reject PlanBothArmsPoses goal because a previous dual-arm planning request is still active.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // 双臂接口不需要额外的组名校验，
    // 因为内部固定使用 arm_l 与 arm_r 两条规划链。
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse YumiPlanningInterfaceNode::handlePlanBothArmsPosesCancel(
      const std::shared_ptr<GoalHandlePlanBothArmsPoses> /*goal_handle*/)
  {
    RCLCPP_WARN(get_logger(), "Received cancel request for active PlanBothArmsPoses goal.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void YumiPlanningInterfaceNode::handlePlanBothArmsPosesAccepted(
      const std::shared_ptr<GoalHandlePlanBothArmsPoses> goal_handle)
  {
    std::thread(
        [this, goal_handle]()
        {
          executePlanBothArmsPoses(goal_handle);
        })
        .detach();
  }

  void YumiPlanningInterfaceNode::executePlanBothArmsPoses(
      const std::shared_ptr<GoalHandlePlanBothArmsPoses> goal_handle)
  {
    // ActiveFlagReset 用 RAII 保证 action 执行线程无论成功、失败还是取消，
    // 都会释放 plan_both_arms_poses_active_，避免后续 goal 被永久拒绝。
    struct ActiveFlagReset
    {
      std::atomic_bool * flag{nullptr};
      ~ActiveFlagReset()
      {
        if (flag != nullptr)
        {
          flag->store(false);
        }
      }
    } active_flag_reset{&plan_both_arms_poses_active_};

    const auto action_start_time = now();
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<PlanBothArmsPoses::Result>();
    auto feedback = std::make_shared<PlanBothArmsPoses::Feedback>();
    moveit::planning_interface::MoveGroupInterface::Plan left_plan;
    moveit::planning_interface::MoveGroupInterface::Plan right_plan;

    feedback->stage = "planning_both_arms";
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(
        get_logger(),
        "PlanBothArmsPoses action started. left=(%.3f, %.3f, %.3f), right=(%.3f, %.3f, %.3f), execute=%s.",
        goal->left_target_pose.x,
        goal->left_target_pose.y,
        goal->left_target_pose.z,
        goal->right_target_pose.x,
        goal->right_target_pose.y,
        goal->right_target_pose.z,
        goal->execute ? "true" : "false");

    if (goal_handle->is_canceling())
    {
      result->success = false;
      result->message = "plan_both_arms_poses was canceled before planning started";
      RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
      goal_handle->canceled(result);
      return;
    }

    bool success = false;
    if (goal->execute)
    {
      // 双臂执行模式下也先保留规划结果，便于上层直接复用或比对关节轨迹。
      success = planBothArmsToPoses(
          goal->left_target_pose,
          goal->right_target_pose,
          &left_plan,
          &right_plan);
      if (success)
      {
        success = executeBothArmPlans(left_plan, right_plan);
      }
    }
    else
    {
      success = planBothArmsToPoses(
          goal->left_target_pose,
          goal->right_target_pose,
          &left_plan,
          &right_plan);
    }

    if (goal_handle->is_canceling())
    {
      result->success = false;
      result->message = "plan_both_arms_poses was canceled before completion";
      goal_handle->canceled(result);
      return;
    }

    result->success = success;
    result->message = success ? "plan_both_arms_poses completed successfully"
                              : "plan_both_arms_poses failed during planning or execution";
    if (success)
    {
      result->left_planned_trajectory = left_plan.trajectory_.joint_trajectory;
      result->right_planned_trajectory = right_plan.trajectory_.joint_trajectory;
    }
    if (success)
    {
      feedback->stage = goal->execute ? "executed_both_arms" : "planned_both_arms";
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(
          get_logger(),
          "PlanBothArmsPoses action succeeded in %.3f seconds.",
          (now() - action_start_time).seconds());
      goal_handle->succeed(result);
      return;
    }

    RCLCPP_WARN(
        get_logger(),
        "PlanBothArmsPoses action failed in %.3f seconds.",
        (now() - action_start_time).seconds());
    goal_handle->abort(result);
  }

  rclcpp_action::GoalResponse YumiPlanningInterfaceNode::handlePlanTaskSpaceTrajectoryGoal(
      const rclcpp_action::GoalUUID & /*uuid*/,
      std::shared_ptr<const PlanTaskSpaceTrajectory::Goal> goal)
  {
    if (!goal)
    {
      RCLCPP_WARN(get_logger(), "Received null goal for plan_task_space_trajectory.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->sample_period <= 0.0)
    {
      RCLCPP_WARN(
          get_logger(),
          "Rejected task-space trajectory goal because sample_period must be positive.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse YumiPlanningInterfaceNode::handlePlanTaskSpaceTrajectoryCancel(
      const std::shared_ptr<GoalHandlePlanTaskSpaceTrajectory> /*goal_handle*/)
  {
    RCLCPP_INFO(get_logger(), "Received cancel request for plan_task_space_trajectory.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void YumiPlanningInterfaceNode::handlePlanTaskSpaceTrajectoryAccepted(
      const std::shared_ptr<GoalHandlePlanTaskSpaceTrajectory> goal_handle)
  {
    std::thread(
        [this, goal_handle]()
        {
          executePlanTaskSpaceTrajectory(goal_handle);
        })
        .detach();
  }

  void YumiPlanningInterfaceNode::executePlanTaskSpaceTrajectory(
      const std::shared_ptr<GoalHandlePlanTaskSpaceTrajectory> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<PlanTaskSpaceTrajectory::Result>();
    auto feedback = std::make_shared<PlanTaskSpaceTrajectory::Feedback>();

    if (goal_handle->is_canceling())
    {
      result->success = false;
      result->message = "Task-space trajectory planning was canceled before execution.";
      goal_handle->canceled(result);
      return;
    }

    feedback->stage = "planning_task_space_trajectory";
    goal_handle->publish_feedback(feedback);

    yumi_interfaces::msg::TaskSpaceTrajectory trajectory;
    if (!planTaskSpaceTrajectory(*goal, &trajectory))
    {
      result->success = false;
      result->message = "Task-space trajectory planning is not implemented yet.";
      result->trajectory = trajectory;
      goal_handle->abort(result);
      return;
    }

    if (goal_handle->is_canceling())
    {
      result->success = false;
      result->message = "Task-space trajectory planning was canceled.";
      result->trajectory = trajectory;
      goal_handle->canceled(result);
      return;
    }

    feedback->stage = "finished";
    goal_handle->publish_feedback(feedback);
    result->success = true;
    result->message = "Task-space trajectory planned successfully.";
    result->trajectory = trajectory;
    goal_handle->succeed(result);
  }

  rclcpp_action::GoalResponse YumiPlanningInterfaceNode::handleResetToHomeGoal(
      const rclcpp_action::GoalUUID & /*uuid*/,
      std::shared_ptr<const ResetToHome::Goal> goal)
  {
    // 单臂 home 复位接口只接受左右臂有效枚举。
    if (!goal || getPlanningGroupFromArmSelector(goal->arm).empty())
    {
      RCLCPP_WARN(
          get_logger(),
          "Rejected reset_to_home goal because arm selector is invalid.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse YumiPlanningInterfaceNode::handleResetToHomeCancel(
      const std::shared_ptr<GoalHandleResetToHome> /*goal_handle*/)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void YumiPlanningInterfaceNode::handleResetToHomeAccepted(
      const std::shared_ptr<GoalHandleResetToHome> goal_handle)
  {
    std::thread(
        [this, goal_handle]()
        {
          executeResetToHome(goal_handle);
        })
        .detach();
  }

  void YumiPlanningInterfaceNode::executeResetToHome(
      const std::shared_ptr<GoalHandleResetToHome> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ResetToHome::Result>();
    auto feedback = std::make_shared<ResetToHome::Feedback>();

    const auto planning_group = getPlanningGroupFromArmSelector(goal->arm);

    feedback->stage = "planning_home";
    goal_handle->publish_feedback(feedback);

    // 单臂 home 复位直接复用已有 resetArmToHome 逻辑。
    const bool success = resetArmToHome(planning_group);

    if (goal_handle->is_canceling())
    {
      result->success = false;
      result->message = "reset_to_home was canceled before completion";
      goal_handle->canceled(result);
      return;
    }

    result->success = success;
    result->message = success ? "reset_to_home completed successfully"
                              : "reset_to_home failed during planning or execution";
    if (success)
    {
      feedback->stage = "home_reached";
      goal_handle->publish_feedback(feedback);
      goal_handle->succeed(result);
      return;
    }

    goal_handle->abort(result);
  }

  rclcpp_action::GoalResponse YumiPlanningInterfaceNode::handleResetBothArmsToHomeGoal(
      const rclcpp_action::GoalUUID & /*uuid*/,
      std::shared_ptr<const ResetBothArmsToHome::Goal> /*goal*/)
  {
    // 双臂同时回 home 不需要任何输入参数，
    // 调用即表示“左右臂一起回到 home 命名姿态”。
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse YumiPlanningInterfaceNode::handleResetBothArmsToHomeCancel(
      const std::shared_ptr<GoalHandleResetBothArmsToHome> /*goal_handle*/)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void YumiPlanningInterfaceNode::handleResetBothArmsToHomeAccepted(
      const std::shared_ptr<GoalHandleResetBothArmsToHome> goal_handle)
  {
    std::thread(
        [this, goal_handle]()
        {
          executeResetBothArmsToHome(goal_handle);
        })
        .detach();
  }

  void YumiPlanningInterfaceNode::executeResetBothArmsToHome(
      const std::shared_ptr<GoalHandleResetBothArmsToHome> goal_handle)
  {
    auto result = std::make_shared<ResetBothArmsToHome::Result>();
    auto feedback = std::make_shared<ResetBothArmsToHome::Feedback>();

    feedback->stage = "planning_home";
    goal_handle->publish_feedback(feedback);

    const bool success = resetBothArmsToHome();

    if (goal_handle->is_canceling())
    {
      result->success = false;
      result->message = "reset_both_arms_to_home was canceled before completion";
      goal_handle->canceled(result);
      return;
    }

    result->success = success;
    result->message = success ? "reset_both_arms_to_home completed successfully"
                              : "reset_both_arms_to_home failed during planning or execution";
    if (success)
    {
      feedback->stage = "home_reached";
      goal_handle->publish_feedback(feedback);
      goal_handle->succeed(result);
      return;
    }

    goal_handle->abort(result);
  }

  rclcpp_action::GoalResponse YumiPlanningInterfaceNode::handleRunRandomMotionGoal(
      const rclcpp_action::GoalUUID & /*uuid*/,
      std::shared_ptr<const RunRandomMotion::Goal> goal)
  {
    // 随机演示接口允许 single_arm / dual_arm 两种模式，
    // 其中 single_arm 还要求调用者额外指明具体规划组。
    if (!goal || (goal->mode != RunRandomMotion::Goal::MODE_SINGLE_ARM &&
                  goal->mode != RunRandomMotion::Goal::MODE_DUAL_ARM))
    {
      RCLCPP_WARN(
          get_logger(),
          "Rejected run_random_motion goal because mode selector is invalid.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->mode == RunRandomMotion::Goal::MODE_SINGLE_ARM &&
        getPlanningGroupFromArmSelector(goal->arm).empty())
    {
      RCLCPP_WARN(
          get_logger(),
          "Rejected run_random_motion goal because single-arm mode requires a valid arm selector.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse YumiPlanningInterfaceNode::handleRunRandomMotionCancel(
      const std::shared_ptr<GoalHandleRunRandomMotion> /*goal_handle*/)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void YumiPlanningInterfaceNode::handleRunRandomMotionAccepted(
      const std::shared_ptr<GoalHandleRunRandomMotion> goal_handle)
  {
    std::thread(
        [this, goal_handle]()
        {
          executeRunRandomMotion(goal_handle);
        })
        .detach();
  }

  void YumiPlanningInterfaceNode::executeRunRandomMotion(
      const std::shared_ptr<GoalHandleRunRandomMotion> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<RunRandomMotion::Result>();
    auto feedback = std::make_shared<RunRandomMotion::Feedback>();

    const auto max_attempts = goal->max_attempts > 0 ? goal->max_attempts : 20;
    const auto planning_group = getPlanningGroupFromArmSelector(goal->arm);
    feedback->stage = "sampling_random_targets";
    goal_handle->publish_feedback(feedback);

    bool success = false;
    if (goal->mode == RunRandomMotion::Goal::MODE_DUAL_ARM)
    {
      // 双臂随机接口会分别为左右臂采样目标，
      // 然后复用已有双臂规划/执行能力做验证。
      auto left_random_pose = generateRandomValidPose("arm_l", max_attempts);
      auto right_random_pose = generateRandomValidPose("arm_r", max_attempts);
      if (left_random_pose && right_random_pose)
      {
        feedback->stage = goal->execute ? "planning_and_executing_both_arms" : "planning_both_arms";
        goal_handle->publish_feedback(feedback);
        success = goal->execute ? planAndExecuteBothArmsToPoses(*left_random_pose, *right_random_pose)
                                : planBothArmsToPoses(*left_random_pose, *right_random_pose, nullptr, nullptr);

        if (success && goal->reset_to_home_after_execution)
        {
          feedback->stage = "resetting_both_arms_home";
          goal_handle->publish_feedback(feedback);
          success = resetBothArmsToHome();
        }
      }
    }
    else
    {
      auto random_pose = generateRandomValidPose(planning_group, max_attempts);
      if (random_pose)
      {
        feedback->stage = goal->execute ? "planning_and_executing_single_arm" : "planning_single_arm";
        goal_handle->publish_feedback(feedback);
        success = goal->execute ? planAndExecuteToPose(planning_group, *random_pose)
                                : planToPose(planning_group, *random_pose, nullptr);

        if (success && goal->reset_to_home_after_execution)
        {
          feedback->stage = "resetting_single_arm_home";
          goal_handle->publish_feedback(feedback);
          success = resetArmToHome(planning_group);
        }
      }
    }

    if (goal_handle->is_canceling())
    {
      result->success = false;
      result->message = "run_random_motion was canceled before completion";
      goal_handle->canceled(result);
      return;
    }

    result->success = success;
    result->message = success ? "run_random_motion completed successfully"
                              : "run_random_motion failed during sampling, planning, or execution";
    if (success)
    {
      feedback->stage = "completed";
      goal_handle->publish_feedback(feedback);
      goal_handle->succeed(result);
      return;
    }

    goal_handle->abort(result);
  }

} // namespace yumi_planning_interface
