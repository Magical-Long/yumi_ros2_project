#ifndef YUMI_COOPERATIVE_CONTROLLER__YUMI_COOPERATIVE_CONTROLLER_HPP_
#define YUMI_COOPERATIVE_CONTROLLER__YUMI_COOPERATIVE_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <yumi_interfaces/action/plan_both_arms_poses.hpp>
#include <yumi_interfaces/action/plan_task_space_trajectory.hpp>
#include <yumi_interfaces/msg/cooperative_controller_status.hpp>
#include <yumi_interfaces/msg/cooperative_task_command.hpp>
#include <yumi_interfaces/msg/joint_space_reference.hpp>
#include <yumi_interfaces/msg/task_space_reference.hpp>
#include <yumi_interfaces/msg/task_space_trajectory.hpp>

namespace yumi_cooperative_controller
{

  // 协同控制节点负责承接 task_manager 的模式/目标，并输出统一控制参考。
  class YumiCooperativeControllerNode : public rclcpp::Node
  {
  public:
    YumiCooperativeControllerNode();
    bool initialize();

  private:
    using PlanBothArmsPoses = yumi_interfaces::action::PlanBothArmsPoses;
    using GoalHandlePlanBothArmsPoses =
        rclcpp_action::ClientGoalHandle<PlanBothArmsPoses>;
    using PlanTaskSpaceTrajectory = yumi_interfaces::action::PlanTaskSpaceTrajectory;
    using GoalHandlePlanTaskSpaceTrajectory =
        rclcpp_action::ClientGoalHandle<PlanTaskSpaceTrajectory>;

    // 读取上层任务命令，决定当前目标、自由空间/约束空间模式以及工作模式。
    void handleTaskCommand(const yumi_interfaces::msg::CooperativeTaskCommand::SharedPtr msg);
    // 周期性参考生成主循环，根据自由空间/约束空间模式输出 joint/task space reference。
    void runControlLoop();
    // 为自由空间模式构造关节空间参考。
    yumi_interfaces::msg::JointSpaceReference buildJointSpaceReference() const;
    // 为约束空间模式构造当前时刻的任务空间参考。
    // 如果规划结果尚未返回，则返回 false 并保持静默，不向下层提前发送最终目标点。
    bool buildTaskSpaceReference(
        yumi_interfaces::msg::TaskSpaceReference *task_reference);
    bool tryBuildTaskSpaceReferenceFromCachedTrajectory(
        yumi_interfaces::msg::TaskSpaceReference *task_reference);
    bool tryBuildJointSpaceReferenceFromCachedTrajectory(
        yumi_interfaces::msg::JointSpaceReference *joint_reference);
    // 把 ROS Duration 转成秒，便于和本地 elapsed time 对齐比较。
    double durationToSeconds(const builtin_interfaces::msg::Duration &duration) const;
    std::vector<double> interpolateJointPositions(
        const trajectory_msgs::msg::JointTrajectory &trajectory,
        double query_time_sec) const;
    // 把上层 command mode 映射到关节空间参考类型。
    uint8_t mapJointReferenceMode(uint8_t command_mode) const;
    // 把上层 command mode 映射到任务空间参考类型。
    uint8_t mapTaskReferenceMode(uint8_t command_mode) const;
    // 判断当前任务命令是否需要请求新的规划结果。
    bool shouldRequestPlan(
        const yumi_interfaces::msg::CooperativeTaskCommand &previous_command,
        const yumi_interfaces::msg::CooperativeTaskCommand &next_command) const;
    bool isFreeSpaceMode(const yumi_interfaces::msg::CooperativeTaskCommand &command) const;
    bool isConstrainedSpaceMode(const yumi_interfaces::msg::CooperativeTaskCommand &command) const;
    bool isObjectSpaceCommand(const yumi_interfaces::msg::CooperativeTaskCommand &command) const;
    // 根据当前 task command 组装 planning_interface 的 action goal。
    PlanBothArmsPoses::Goal buildJointPlanningGoal() const;
    PlanTaskSpaceTrajectory::Goal buildTaskSpacePlanningGoal() const;
    void requestJointTrajectoryPlan();
    void requestTaskSpaceTrajectoryPlan();
    // action 回调：自由空间关节规划 goal 是否被 planning_interface 接受。
    // 只有 goal 被接受后，active_joint_planning_goal_handle_ 才能用于后续超时取消。
    void handleJointPlanGoalResponse(
        std::shared_future<GoalHandlePlanBothArmsPoses::SharedPtr> future);
    void handleJointPlanResult(const GoalHandlePlanBothArmsPoses::WrappedResult &result);
    void handleTaskSpacePlanGoalResponse(
        std::shared_future<GoalHandlePlanTaskSpaceTrajectory::SharedPtr> future);
    void handleTaskSpacePlanResult(const GoalHandlePlanTaskSpaceTrajectory::WrappedResult &result);
    // 周期发布握手状态，让 task_manager 知道 cooperative 是否真正 ready。
    void publishControllerStatus();
    bool isReadyForTask() const;
    bool buildObjectSpaceReference(
        yumi_interfaces::msg::TaskSpaceReference *task_reference);
    /* 锁定左右臂相对于object_reference_frame_的位姿 */
    bool initializeObjectSpaceTrajectory();
    bool lookupCurrentEndEffectorTransforms(
        tf2::Transform *left_end_effector_transform,
        tf2::Transform *right_end_effector_transform);
    // 任务命令输入和控制参考输出的 ROS2 通信接口。
    rclcpp::Subscription<yumi_interfaces::msg::CooperativeTaskCommand>::SharedPtr task_command_subscriber_;
    rclcpp::Publisher<yumi_interfaces::msg::JointSpaceReference>::SharedPtr joint_reference_publisher_;
    rclcpp::Publisher<yumi_interfaces::msg::TaskSpaceReference>::SharedPtr task_reference_publisher_;
    rclcpp::Publisher<yumi_interfaces::msg::CooperativeControllerStatus>::SharedPtr
        controller_status_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp_action::Client<PlanBothArmsPoses>::SharedPtr plan_both_arms_poses_client_;
    rclcpp_action::Client<PlanTaskSpaceTrajectory>::SharedPtr plan_task_space_trajectory_client_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 这些向量定义了双臂的统一关节顺序和标准 ready/home 关节目标。
    std::vector<std::string> left_joint_names_;
    std::vector<std::string> right_joint_names_;
    std::vector<double> ready_left_positions_;
    std::vector<double> ready_right_positions_;

    // 协同控制层的参考发布周期。
    double control_period_sec_{0.02};
    double planning_action_server_wait_timeout_sec_{5.0};
    double planning_time_sec_{5.0};
    double planning_velocity_scaling_{0.2};
    double planning_acceleration_scaling_{0.2};
    double planning_sample_period_sec_{0.05};
    double joint_planning_result_timeout_sec_{12.0};
    double object_trajectory_duration_sec_{8.0};
    std::string object_reference_frame_{"world"};
    std::string left_end_effector_frame_{"gripper_l_base"};
    std::string right_end_effector_frame_{"gripper_r_base"};
    int planning_num_attempts_{3};
    std::string plan_both_arms_poses_action_name_{"plan_both_arms_poses"};
    std::string plan_task_space_trajectory_action_name_{"plan_task_space_trajectory"};

    // 当前只缓存最近一次任务命令，便于按固定周期稳定输出 reference。
    bool has_task_command_{false};
    yumi_interfaces::msg::CooperativeTaskCommand latest_task_command_;
    // 自由空间规划请求状态：
    // - in_flight 表示已发 action goal 且正在等待结果
    // - pending 表示已有更新目标，需要当前 result 返回后重新规划
    // - cancel_requested 表示超时后已真实发送 cancel goal 请求
    bool joint_planning_request_in_flight_{false};
    bool joint_planning_request_pending_{false};
    bool joint_planning_cancel_requested_{false};
    // 某个目标规划失败/超时后，不再无限重试同一个命令，等待 task_manager 发新命令。
    bool joint_planning_failed_waiting_for_new_command_{false};
    GoalHandlePlanBothArmsPoses::SharedPtr active_joint_planning_goal_handle_;
    rclcpp::Time joint_planning_request_start_time_{0, 0, RCL_ROS_TIME};
    std::chrono::steady_clock::time_point joint_planning_request_wall_start_time_;
    // planning_interface 返回的左右臂 joint trajectory 缓存在这里，
    // runControlLoop 按 time_from_start 播放成 JointSpaceReference。
    bool has_cached_joint_trajectories_{false};
    bool joint_trajectory_active_{false};
    rclcpp::Time joint_trajectory_start_time_{0, 0, RCL_ROS_TIME};
    bool task_space_planning_request_in_flight_{false};
    bool has_cached_task_space_trajectory_{false};
    bool task_space_trajectory_active_{false};
    bool object_space_active_{false};
    bool object_grasp_locked_{false};
    rclcpp::Time task_space_trajectory_start_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time object_trajectory_start_time_{0, 0, RCL_ROS_TIME};
    tf2::Transform object_trajectory_start_pose_;
    tf2::Transform object_trajectory_goal_pose_;
    tf2::Transform object_to_left_grasp_;
    tf2::Transform object_to_right_grasp_;
    std::size_t current_left_joint_trajectory_point_index_{0U};
    std::size_t current_right_joint_trajectory_point_index_{0U};
    std::size_t current_task_space_point_index_{0U};
    trajectory_msgs::msg::JointTrajectory cached_left_joint_trajectory_;
    trajectory_msgs::msg::JointTrajectory cached_right_joint_trajectory_;
    yumi_interfaces::msg::TaskSpaceTrajectory cached_task_space_trajectory_;
  };

} // namespace yumi_cooperative_controller

#endif // YUMI_COOPERATIVE_CONTROLLER__YUMI_COOPERATIVE_CONTROLLER_HPP_
