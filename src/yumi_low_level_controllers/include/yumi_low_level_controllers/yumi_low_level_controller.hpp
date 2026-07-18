#ifndef YUMI_LOW_LEVEL_CONTROLLERS__YUMI_LOW_LEVEL_CONTROLLER_HPP_
#define YUMI_LOW_LEVEL_CONTROLLERS__YUMI_LOW_LEVEL_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <yumi_interfaces/msg/joint_space_reference.hpp>
#include <yumi_interfaces/msg/low_level_controller_status.hpp>
#include <yumi_interfaces/msg/task_space_reference.hpp>

namespace yumi_low_level_controllers
{

// 低层控制层统一节点：
// 1. 订阅上层 joint/task-space reference
// 2. 订阅 joint_states 形成闭环
// 3. 在内部根据 reference_mode 选择控制算法分支
// 4. 输出左右臂底层命令
class YumiLowLevelControllerNode : public rclcpp::Node
{
public:
  YumiLowLevelControllerNode();
  bool initialize();

private:
  // 底层控制器类型描述的是“当前节点最终准备驱动哪类执行链”。
  // 它与上层 reference 类型不是一回事：
  // - controller_type_ 决定输出 velocity / effort / trajectory 哪一类命令
  // - active_reference_type_ 决定当前收到的是 joint-space 还是 task-space 参考
  enum class ControllerType
  {
    UNSPECIFIED = 0,
    TRAJECTORY,
    VELOCITY,
    EFFORT
  };

  enum class ActiveReferenceType
  {
    UNSPECIFIED = 0,
    JOINT_SPACE,
    TASK_SPACE
  };

  // 一欧元滤波器状态：
  // - previous_value_ 保存上一时刻滤波后的速度
  // - previous_derivative_ 保存上一时刻滤波后的速度导数
  // 这里按 6 维任务空间速度向量整体维护，内部逐分量滤波。
  struct OneEuroFilterState
  {
    bool initialized{false};
    rclcpp::Time previous_time{0, 0, RCL_ROS_TIME};
    Eigen::Matrix<double, 6, 1> previous_value{Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::Matrix<double, 6, 1> previous_derivative{Eigen::Matrix<double, 6, 1>::Zero()};
  };

  // 统一参考输入回调。
  void handleJointSpaceReference(
    const yumi_interfaces::msg::JointSpaceReference::SharedPtr msg);
  void handleTaskSpaceReference(
    const yumi_interfaces::msg::TaskSpaceReference::SharedPtr msg);
  // 关节状态输入回调，用于后续做闭环跟踪。
  void handleJointStates(const sensor_msgs::msg::JointState::SharedPtr msg);
  // 低层控制循环，根据当前参考模式选择不同算法分支。
  void runControlLoop();
  // 根据控制器类型分发到不同控制大类。
  void runTrajectoryControl();
  void runVelocityControl();
  void runEffortControl();
  // 根据字符串参数解析当前底层控制器类型。
  ControllerType parseControllerType(const std::string & controller_type) const;

  // 当前第一版先实现 joint-space velocity tracking。
  std::vector<double> computeJointVelocityCommand(
    const std::vector<double> & current_joint_positions,
    const std::vector<double> & target_joint_positions) const;
  // 初始化 Pinocchio 模型，并解析左右末端 frame id。
  bool initializePinocchioModel();
  // 把 joint_states 中的关节值按名字写入 Pinocchio 配置向量 q。
  bool buildPinocchioConfiguration(Eigen::VectorXd * configuration);
  // 用当前 q 计算双臂末端位姿与 Jacobian，作为后续任务空间控制的底座。
  bool computeTaskSpaceKinematics(
    const Eigen::VectorXd & configuration,
    pinocchio::SE3 * left_end_effector_pose,
    pinocchio::SE3 * right_end_effector_pose,
    Eigen::MatrixXd * left_jacobian,
    Eigen::MatrixXd * right_jacobian);
  // 从统一任务空间参考中恢复目标位姿。
  pinocchio::SE3 buildTargetTaskSpacePose(
    const yumi_interfaces::msg::TaskSpacePose & target_pose) const;
  // 计算当前末端位姿相对目标位姿的 6 维误差。
  Eigen::Matrix<double, 6, 1> computeTaskSpacePoseError(
    const pinocchio::SE3 & current_pose,
    const yumi_interfaces::msg::TaskSpacePose & target_pose) const;
  // 用相邻参考点差分近似参考末端速度，为轨迹跟踪提供一阶前馈项。
  Eigen::Matrix<double, 6, 1> computeReferenceTaskSpaceVelocity(
    const yumi_interfaces::msg::TaskSpacePose & previous_target_pose,
    const yumi_interfaces::msg::TaskSpacePose & current_target_pose,
    double delta_time_sec) const;
  // 对参考任务空间速度做一欧元滤波，抑制相邻轨迹点差分带来的高频抖动。
  Eigen::Matrix<double, 6, 1> filterReferenceTaskSpaceVelocity(
    const Eigen::Matrix<double, 6, 1> & raw_reference_task_space_velocity,
    const rclcpp::Time & sample_time,
    OneEuroFilterState * filter_state);
  // 任务空间速度控制律：
  // v_cmd = v_ref + Kp * (x_ref - x) + Kd * (v_ref - v_current)
  Eigen::Matrix<double, 6, 1> computeTaskSpaceVelocityCommand(
    const pinocchio::SE3 & current_pose,
    const Eigen::Matrix<double, 6, 1> & current_task_space_velocity,
    const Eigen::Matrix<double, 6, 1> & reference_task_space_velocity,
    const yumi_interfaces::msg::TaskSpacePose & target_pose) const;
  // 从整机 Jacobian 中抽出一只机械臂对应的列，形成单臂 6xn Jacobian。
  Eigen::MatrixXd extractArmJacobian(
    const Eigen::MatrixXd & full_jacobian,
    const std::vector<std::string> & arm_joint_names) const;
  // 在 Jacobian 零空间里构造构型优化速度。
  // 当前包含 posture 项和左右臂镜像 symmetry 项。
  Eigen::VectorXd computeNullspacePostureVelocity(
    const Eigen::MatrixXd & arm_jacobian,
    const std::vector<double> & current_joint_positions,
    const std::vector<double> & posture_target_positions,
    const std::vector<double> & symmetry_target_positions) const;
  // 从单臂 Jacobian 与任务空间速度解出关节速度，并叠加零空间构型优化项。
  std::vector<double> computeArmJointVelocityFromTaskSpaceCommand(
    const Eigen::MatrixXd & arm_jacobian,
    const Eigen::Matrix<double, 6, 1> & task_space_velocity,
    const std::vector<double> & current_joint_positions,
    const std::vector<double> & posture_target_positions,
    const std::vector<double> & symmetry_target_positions) const;
  // 根据当前控制分支发布执行状态，供 task_manager 做后续状态机决策。
  void publishControllerStatus(
    double max_joint_position_error,
    double task_space_position_error_norm,
    double task_space_orientation_error_norm);
  // 发布左右臂速度命令。
  void publishVelocityCommands(
    const std::vector<double> & left_joint_velocities,
    const std::vector<double> & right_joint_velocities);
  // 从 joint_states 中按统一顺序取出左右臂关节位置。
  bool tryBuildOrderedJointPositions(
    std::vector<double> * left_joint_positions,
    std::vector<double> * right_joint_positions);
  // 从 joint_states 中按统一顺序取出左右臂关节速度，用于计算当前末端速度 J(q)qdot。
  bool tryBuildOrderedJointVelocities(
    std::vector<double> * left_joint_velocities,
    std::vector<double> * right_joint_velocities);

  rclcpp::Subscription<yumi_interfaces::msg::JointSpaceReference>::SharedPtr
    joint_space_reference_subscriber_;
  rclcpp::Subscription<yumi_interfaces::msg::TaskSpaceReference>::SharedPtr
    task_space_reference_subscriber_;
  // joint_states 是所有闭环控制的状态来源：
  // joint-space 分支用 position 算 q_des-q；task-space 分支同时用 velocity 算 J(q)qdot。
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    left_velocity_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    right_velocity_command_publisher_;
  rclcpp::Publisher<yumi_interfaces::msg::LowLevelControllerStatus>::SharedPtr
    controller_status_publisher_;
  // 调试误差连续发布，不参与状态机切换。
  // data = [joint_error, task_position_error, task_orientation_error, goal_reached_flag]
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    controller_error_debug_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // 统一关节顺序，必须和仿真 velocity controller 配置保持一致。
  std::vector<std::string> left_joint_names_;
  std::vector<std::string> right_joint_names_;
  // 任务空间控制的零空间姿态目标。
  // 它不直接改变末端轨迹，只在 Jacobian 零空间里轻微拉回更自然的关节构型。
  std::vector<double> left_nullspace_posture_target_positions_;
  std::vector<double> right_nullspace_posture_target_positions_;
  // 左右臂镜像符号映射，顺序与 joint_names 保持一致。
  // sign=-1 表示该关节在镜像构型中方向相反。
  std::vector<double> nullspace_symmetry_signs_;

  // 当前先保留最基础的 joint-space velocity 跟踪参数。
  double control_period_sec_{0.01};
  double joint_position_gain_{1.0};
  double max_joint_velocity_{0.5};
  double task_space_position_gain_{1.0};
  double task_space_orientation_gain_{1.0};
  double task_space_position_damping_gain_{0.0};
  double task_space_orientation_damping_gain_{0.0};
  double nullspace_posture_gain_{0.0};
  double nullspace_symmetry_gain_{0.0};
  // 一欧元滤波器参数，用于平滑由相邻 TaskSpaceReference 差分得到的 v_ref。
  double one_euro_min_cutoff_{1.0};
  double one_euro_beta_{0.0};
  double one_euro_d_cutoff_{1.0};
  // 到位阈值用于生成低频语义反馈，而不是参与底层速度控制律。
  double joint_goal_tolerance_{0.02};
  double task_space_position_tolerance_{0.01};
  double task_space_orientation_tolerance_{0.05};
  double goal_reached_reference_stable_time_sec_{0.2};
  std::string controller_type_name_{"velocity"};
  std::string urdf_path_;
  std::string left_end_effector_frame_;
  std::string right_end_effector_frame_;

  // 控制器类型和当前激活参考类型共同决定 runControlLoop 的算法分支。
  // 例如：
  // - VELOCITY + JOINT_SPACE -> joint-space velocity tracking
  // - VELOCITY + TASK_SPACE  -> task-space velocity control
  // - EFFORT   + TASK_SPACE  -> impedance / admittance / inverse dynamics
  ControllerType controller_type_{ControllerType::UNSPECIFIED};
  ActiveReferenceType active_reference_type_{ActiveReferenceType::UNSPECIFIED};

  // 参考缓存与 robot state 缓存。
  bool has_joint_states_{false};
  // 统一表示“至少收到过一次来自上层的有效参考”。
  // 具体是哪种参考，则由 active_reference_type_ 决定。
  bool has_upper_reference_{false};
  // 反馈链采用“到位后单次上报”的边沿触发语义：
  // - false -> true 时发布一次 goal_reached
  // - 目标未到位或目标更新后，重新允许下一次上报
  bool goal_reached_reported_{false};
  yumi_interfaces::msg::JointSpaceReference latest_joint_space_reference_;
  yumi_interfaces::msg::TaskSpaceReference latest_task_space_reference_;
  std::unordered_map<std::string, double> latest_joint_positions_;
  std::unordered_map<std::string, double> latest_joint_velocities_;
  bool has_previous_task_space_reference_{false};
  rclcpp::Time latest_reference_change_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_task_space_reference_receive_time_{0, 0, RCL_ROS_TIME};
  yumi_interfaces::msg::TaskSpaceReference previous_task_space_reference_;
  Eigen::Matrix<double, 6, 1> latest_left_reference_task_space_velocity_{
    Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> latest_right_reference_task_space_velocity_{
    Eigen::Matrix<double, 6, 1>::Zero()};
  OneEuroFilterState left_reference_velocity_filter_state_;
  OneEuroFilterState right_reference_velocity_filter_state_;

  // Pinocchio 运动学后端：
  // - robot_model_ / robot_data_ 提供 FK / Jacobian 计算能力
  // - 左右末端 frame id 用于快速索引目标 frame
  bool has_pinocchio_model_{false};
  pinocchio::Model robot_model_;
  std::unique_ptr<pinocchio::Data> robot_data_;
  pinocchio::FrameIndex left_end_effector_frame_id_{0};
  pinocchio::FrameIndex right_end_effector_frame_id_{0};
};

}  // namespace yumi_low_level_controllers

#endif  // YUMI_LOW_LEVEL_CONTROLLERS__YUMI_LOW_LEVEL_CONTROLLER_HPP_
