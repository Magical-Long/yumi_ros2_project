#ifndef YUMI_PLANNING_INTERFACE__YUMI_PLANNING_INTERFACE_HPP_
#define YUMI_PLANNING_INTERFACE__YUMI_PLANNING_INTERFACE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yumi_interfaces/action/plan_arm_pose.hpp>
#include <yumi_interfaces/action/plan_both_arms_poses.hpp>
#include <yumi_interfaces/action/plan_task_space_trajectory.hpp>
#include <yumi_interfaces/action/reset_to_home.hpp>
#include <yumi_interfaces/action/reset_both_arms_to_home.hpp>
#include <yumi_interfaces/action/run_random_motion.hpp>
#include <yumi_interfaces/msg/pose6_d.hpp>
#include <yumi_interfaces/msg/task_space_trajectory.hpp>

namespace yumi_planning_interface
{

class YumiPlanningInterfaceNode : public rclcpp::Node
{
public:
  using PlanArmPose = yumi_interfaces::action::PlanArmPose;
  using PlanBothArmsPoses = yumi_interfaces::action::PlanBothArmsPoses;
  using PlanTaskSpaceTrajectory = yumi_interfaces::action::PlanTaskSpaceTrajectory;
  using ResetToHome = yumi_interfaces::action::ResetToHome;
  using ResetBothArmsToHome = yumi_interfaces::action::ResetBothArmsToHome;
  using RunRandomMotion = yumi_interfaces::action::RunRandomMotion;

  using GoalHandlePlanArmPose = rclcpp_action::ServerGoalHandle<PlanArmPose>;
  using GoalHandlePlanBothArmsPoses = rclcpp_action::ServerGoalHandle<PlanBothArmsPoses>;
  using GoalHandlePlanTaskSpaceTrajectory = rclcpp_action::ServerGoalHandle<PlanTaskSpaceTrajectory>;
  using GoalHandleResetToHome = rclcpp_action::ServerGoalHandle<ResetToHome>;
  using GoalHandleResetBothArmsToHome = rclcpp_action::ServerGoalHandle<ResetBothArmsToHome>;
  using GoalHandleRunRandomMotion = rclcpp_action::ServerGoalHandle<RunRandomMotion>;

  // 构造函数只负责节点基础初始化与参数声明。
  YumiPlanningInterfaceNode();

  // 初始化 MoveIt 相关对象。
  // 将该步骤单独抽离，避免在构造函数阶段过早使用 shared_from_this()。
  bool initialize();

  // 为指定规划组随机生成一个经过规划验证的合法末端位姿。
  // 返回空值表示在最大尝试次数内没有找到可用目标。
  std::optional<geometry_msgs::msg::PoseStamped> generateRandomValidPose(
    const std::string & planning_group,
    int max_attempts = 20);

  // 核心规划接口：
  // 输入完整 PoseStamped，并按给定参考系与末端 link 进行规划。
  bool planToPose(
    const std::string & planning_group,
    const geometry_msgs::msg::PoseStamped & target_pose,
    moveit::planning_interface::MoveGroupInterface::Plan * plan_result = nullptr);

  // 面向上层调用者的简化接口：
  // 用户只提供 Pose6D，内部自动使用默认参考坐标系。
  bool planToPose(
    const std::string & planning_group,
    const yumi_interfaces::msg::Pose6D & target_pose,
    moveit::planning_interface::MoveGroupInterface::Plan * plan_result = nullptr);

  // 带显式参考系的 Pose6D 版本：
  // 适合用户仍然希望只给 6 个 double，但想明确指定位姿所属 frame。
  bool planToPose(
    const std::string & planning_group,
    const yumi_interfaces::msg::Pose6D & target_pose,
    const std::string & reference_frame,
    moveit::planning_interface::MoveGroupInterface::Plan * plan_result = nullptr);

  // 执行已经生成好的轨迹计划。
  // 该接口只负责执行，不再重复修改目标位姿或重新规划。
  bool executePlan(
    const std::string & planning_group,
    const moveit::planning_interface::MoveGroupInterface::Plan & plan);

  // 组合接口：先规划，再在规划成功后立刻执行。
  // 该接口适合上层想直接得到“末端位姿到动作执行”一站式行为时使用。
  bool planAndExecuteToPose(
    const std::string & planning_group,
    const geometry_msgs::msg::PoseStamped & target_pose);

  // 面向简化 6D 位姿输入的规划+执行接口。
  bool planAndExecuteToPose(
    const std::string & planning_group,
    const yumi_interfaces::msg::Pose6D & target_pose);

  // 带显式参考系的 6D 位姿规划+执行接口。
  bool planAndExecuteToPose(
    const std::string & planning_group,
    const yumi_interfaces::msg::Pose6D & target_pose,
    const std::string & reference_frame);

  // 回到单臂 home 命名姿态。
  // 该接口会根据规划组自动选择对应的 home_l / home_r 命名目标。
  bool resetArmToHome(
    const std::string & planning_group);

  // 双臂联合规划接口：
  // 左右臂分别接收一个完整 PoseStamped 目标，只有两侧都规划成功才返回成功。
  // 注意：当前内部仍是“左右臂各自规划 + 统一返回”，不是 MoveIt 里的全身耦合规划。
  bool planBothArmsToPoses(
    const geometry_msgs::msg::PoseStamped & left_target_pose,
    const geometry_msgs::msg::PoseStamped & right_target_pose,
    moveit::planning_interface::MoveGroupInterface::Plan * left_plan_result = nullptr,
    moveit::planning_interface::MoveGroupInterface::Plan * right_plan_result = nullptr);

  // 双臂 6D 简化接口：
  // 用户只提供左右手的 6D 位姿，内部统一按默认参考坐标系转换后规划。
  bool planBothArmsToPoses(
    const yumi_interfaces::msg::Pose6D & left_target_pose,
    const yumi_interfaces::msg::Pose6D & right_target_pose,
    moveit::planning_interface::MoveGroupInterface::Plan * left_plan_result = nullptr,
    moveit::planning_interface::MoveGroupInterface::Plan * right_plan_result = nullptr);

  // 双臂带显式参考系的 6D 规划接口。
  bool planBothArmsToPoses(
    const yumi_interfaces::msg::Pose6D & left_target_pose,
    const yumi_interfaces::msg::Pose6D & right_target_pose,
    const std::string & reference_frame,
    moveit::planning_interface::MoveGroupInterface::Plan * left_plan_result = nullptr,
    moveit::planning_interface::MoveGroupInterface::Plan * right_plan_result = nullptr);

  // 双臂近同步执行接口：
  // 假设左右臂的轨迹已分别规划好，然后分别发送给两个控制器执行。
  bool executeBothArmPlans(
    const moveit::planning_interface::MoveGroupInterface::Plan & left_plan,
    const moveit::planning_interface::MoveGroupInterface::Plan & right_plan);

  // 双臂“先规划、后执行”的组合接口。
  bool planAndExecuteBothArmsToPoses(
    const geometry_msgs::msg::PoseStamped & left_target_pose,
    const geometry_msgs::msg::PoseStamped & right_target_pose);

  // 双臂 6D 简化版组合接口。
  bool planAndExecuteBothArmsToPoses(
    const yumi_interfaces::msg::Pose6D & left_target_pose,
    const yumi_interfaces::msg::Pose6D & right_target_pose);

  // 双臂带显式参考系的 6D 组合接口。
  bool planAndExecuteBothArmsToPoses(
    const yumi_interfaces::msg::Pose6D & left_target_pose,
    const yumi_interfaces::msg::Pose6D & right_target_pose,
    const std::string & reference_frame);

  // 任务空间轨迹规划接口：
  // 输入双臂/物体的任务空间目标与规划参数，输出带时间语义的任务空间轨迹。
  // 当前实现思路是：
  // 1. 先调用 MoveIt 得到 joint trajectory
  // 2. 按采样周期对 joint trajectory 做时间重采样
  // 3. 对每个采样点做 FK
  // 4. 输出位置 + 四元数姿态的任务空间轨迹
  bool planTaskSpaceTrajectory(
    const PlanTaskSpaceTrajectory::Goal & goal,
    yumi_interfaces::msg::TaskSpaceTrajectory * trajectory_result);

  // 双臂同时回到 home 命名姿态。
  // 当前实现会分别规划左右臂回 home 的轨迹，再复用双臂近同步执行链。
  bool resetBothArmsToHome();

  private:
    // 获取指定规划组对应的 MoveGroupInterface。
    // 若规划组尚未初始化，则返回空指针并打印错误日志。
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> getMoveGroup(
      const std::string & planning_group);

    // 查询规划组对应的 home 命名姿态名称。
    // 该映射依赖 yumi_moveit_config 中 SRDF 已定义的 home_l / home_r。
    std::string getHomeTargetName(const std::string & planning_group) const;

    // 将轻量左右臂选择枚举转换成内部统一使用的规划组名称。
    // 这样 action 层可以避免直接暴露 "arm_l" / "arm_r" 这样的字符串。
    std::string getPlanningGroupFromArmSelector(uint8_t arm_selector) const;

    // 查询规划组对应的末端 link 名称。
    std::string getEndEffectorLink(const std::string & planning_group) const;

    // 将自定义 Pose6D 转换成 MoveIt 需要的 PoseStamped。
    // 所有简化接口最终都会先经过这一步，再汇总到核心规划函数。
    geometry_msgs::msg::PoseStamped convertPose6DToPoseStamped(
      const yumi_interfaces::msg::Pose6D & pose6d,
      const std::string & reference_frame) const;

    // 对 pose target 先做带构型偏好的 IK 选择。
    // 外部接口仍然只给末端 pose；这里内部先挑选更自然的 q_goal，
    // 再交给 MoveIt 做关节空间规划，避免裸 pose target 随机选到别扭构型。
    bool findPostureBiasedIkTarget(
      const std::string & planning_group,
      const geometry_msgs::msg::PoseStamped & target_pose,
      const std::string & end_effector_link,
      const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & move_group,
      std::vector<double> * joint_target);
    bool findPostureBiasedIkTarget(
      const std::string & planning_group,
      const geometry_msgs::msg::PoseStamped & target_pose,
      const std::string & end_effector_link,
      const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & move_group,
      const std::vector<std::vector<double>> & extra_seed_candidates,
      std::vector<double> * joint_target,
      double * selected_score);
    // 带镜像评分项的 IK 选择版本。
    // extra_seed_candidates:
    // - 只作为 IK 初始值，帮助 setFromIK 搜索不同分支；
    // - 它本身不代表最终一定要选中的关节目标。
    // mirrored_other_arm_joint_target:
    // - 是另一只手的候选 q_goal 经过镜像符号映射后的当前手参考构型；
    // - 只进入 score，用于惩罚“不够镜像对称”的候选 IK 解。
    bool findPostureBiasedIkTarget(
      const std::string & planning_group,
      const geometry_msgs::msg::PoseStamped & target_pose,
      const std::string & end_effector_link,
      const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & move_group,
      const std::vector<std::vector<double>> & extra_seed_candidates,
      const std::vector<double> & mirrored_other_arm_joint_target,
      double symmetry_weight,
      std::vector<double> * joint_target,
      double * selected_score);
    // 根据配置的镜像符号映射，把一只手的 q_goal 转换成另一只手的对称构型参考。
    // 它不是最终轨迹，只是 IK seed / symmetry score 的“偏好参考”。
    bool mirrorJointTargetToOtherArm(
      const std::string & source_group,
      const std::string & target_group,
      const std::vector<double> & source_joint_target,
      std::vector<double> * mirrored_seed);

    // 将 MoveIt 规划结果转换成控制器可接收的 action goal。
    // 当前保持 MoveIt 生成的相对时间轨迹，不再额外写绝对起始时间戳。
    control_msgs::action::FollowJointTrajectory::Goal buildFollowJointTrajectoryGoal(
      const moveit::planning_interface::MoveGroupInterface::Plan & plan) const;

    // 通过 action client 把轨迹发送给对应控制器，并等待 goal 被接受。
    bool sendTrajectoryGoal(
      const std::string & planning_group,
      const control_msgs::action::FollowJointTrajectory::Goal & goal,
      rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr * goal_handle);

    // 等待控制器执行完成，并根据 action result 判断成功或失败。
    bool waitForTrajectoryResult(
      const std::string & planning_group,
      const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr & goal_handle);

    // 启动阶段的一次性演示逻辑，用于快速验证接口组合是否正常。
    void runDemoOnce();

    // 下面四组回调用于把现有函数能力封装成 ROS2 action。
    // 这样上层节点不需要直接链接 C++ 类成员函数，只需按标准 action 调用即可。
    rclcpp_action::GoalResponse handlePlanArmPoseGoal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const PlanArmPose::Goal> goal);
    rclcpp_action::CancelResponse handlePlanArmPoseCancel(
      const std::shared_ptr<GoalHandlePlanArmPose> goal_handle);
    void handlePlanArmPoseAccepted(
      const std::shared_ptr<GoalHandlePlanArmPose> goal_handle);
    void executePlanArmPose(
      const std::shared_ptr<GoalHandlePlanArmPose> goal_handle);

    rclcpp_action::GoalResponse handlePlanBothArmsPosesGoal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const PlanBothArmsPoses::Goal> goal);
    rclcpp_action::CancelResponse handlePlanBothArmsPosesCancel(
      const std::shared_ptr<GoalHandlePlanBothArmsPoses> goal_handle);
    void handlePlanBothArmsPosesAccepted(
      const std::shared_ptr<GoalHandlePlanBothArmsPoses> goal_handle);
    void executePlanBothArmsPoses(
      const std::shared_ptr<GoalHandlePlanBothArmsPoses> goal_handle);

    // 任务空间轨迹 action：
    // 对外返回整条带时间语义的任务空间轨迹，供协同控制层缓存并周期发布。
    rclcpp_action::GoalResponse handlePlanTaskSpaceTrajectoryGoal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const PlanTaskSpaceTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handlePlanTaskSpaceTrajectoryCancel(
      const std::shared_ptr<GoalHandlePlanTaskSpaceTrajectory> goal_handle);
    void handlePlanTaskSpaceTrajectoryAccepted(
      const std::shared_ptr<GoalHandlePlanTaskSpaceTrajectory> goal_handle);
    void executePlanTaskSpaceTrajectory(
      const std::shared_ptr<GoalHandlePlanTaskSpaceTrajectory> goal_handle);

    rclcpp_action::GoalResponse handleResetToHomeGoal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const ResetToHome::Goal> goal);
    rclcpp_action::CancelResponse handleResetToHomeCancel(
      const std::shared_ptr<GoalHandleResetToHome> goal_handle);
    void handleResetToHomeAccepted(
      const std::shared_ptr<GoalHandleResetToHome> goal_handle);
    void executeResetToHome(
      const std::shared_ptr<GoalHandleResetToHome> goal_handle);

    rclcpp_action::GoalResponse handleResetBothArmsToHomeGoal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const ResetBothArmsToHome::Goal> goal);
    rclcpp_action::CancelResponse handleResetBothArmsToHomeCancel(
      const std::shared_ptr<GoalHandleResetBothArmsToHome> goal_handle);
    void handleResetBothArmsToHomeAccepted(
      const std::shared_ptr<GoalHandleResetBothArmsToHome> goal_handle);
    void executeResetBothArmsToHome(
      const std::shared_ptr<GoalHandleResetBothArmsToHome> goal_handle);

    rclcpp_action::GoalResponse handleRunRandomMotionGoal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const RunRandomMotion::Goal> goal);
    rclcpp_action::CancelResponse handleRunRandomMotionCancel(
      const std::shared_ptr<GoalHandleRunRandomMotion> goal_handle);
    void handleRunRandomMotionAccepted(
      const std::shared_ptr<GoalHandleRunRandomMotion> goal_handle);
    void executeRunRandomMotion(
      const std::shared_ptr<GoalHandleRunRandomMotion> goal_handle);

    // 每个规划组对应一个 MoveGroupInterface，
    // 便于后续分别支持左臂、右臂以及更复杂的扩展。
    std::unordered_map<std::string, std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>
      move_groups_;

    // 记录规划组与末端 link 的映射关系，
    // 避免在每次规划时重复查找配置。
    std::unordered_map<std::string, std::string> end_effector_links_;

    // 左右臂控制器各自对应一个 FollowJointTrajectory action client。
    // 双臂执行时，会直接向这两个客户端分别发送轨迹目标。
    std::unordered_map<std::string, rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr>
      trajectory_action_clients_;

    // 一次性演示用的 timer 句柄。
    rclcpp::TimerBase::SharedPtr demo_timer_;

    // 对外 action 接口：
    // 单臂位姿规划/执行、双臂位姿规划/执行、回 home、随机规划演示都统一从这里暴露。
    rclcpp_action::Server<PlanArmPose>::SharedPtr plan_arm_pose_action_server_;
    rclcpp_action::Server<PlanBothArmsPoses>::SharedPtr plan_both_arms_poses_action_server_;
    rclcpp_action::Server<PlanTaskSpaceTrajectory>::SharedPtr plan_task_space_trajectory_action_server_;
    rclcpp_action::Server<ResetToHome>::SharedPtr reset_to_home_action_server_;
    rclcpp_action::Server<ResetBothArmsToHome>::SharedPtr reset_both_arms_to_home_action_server_;
    rclcpp_action::Server<RunRandomMotion>::SharedPtr run_random_motion_action_server_;

    // MoveGroupInterface 及其内部状态监视器不是为多条规划请求并发共享而设计的。
    // 这里用递归互斥锁把所有规划入口串行化，既避免不同 action 并发访问，
    // 也允许高层规划函数在内部继续调用 planToPose 这类下层接口而不死锁。
    std::recursive_mutex planning_mutex_;
    // PlanBothArmsPoses 只允许一个 active goal。
    // cooperative_controller 超时取消 goal 后，这个标志会随 action 执行线程退出而释放。
    std::atomic_bool plan_both_arms_poses_active_{false};

    // 是否要求 trajectory action server 必须存在。
    // false 时，当前节点工作在“纯规划模式”：
    // - PlanTaskSpaceTrajectory 这类纯规划 action 仍可正常工作
    // - 但真正的轨迹执行接口会被显式禁用
    bool require_trajectory_action_servers_{true};
};

}  // namespace yumi_planning_interface

#endif  // YUMI_PLANNING_INTERFACE__YUMI_PLANNING_INTERFACE_HPP_
