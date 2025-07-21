#ifndef DRPOWER_HARDWARE_INTERFACE__DRPOWER_HARDWARE_INTERFACE_HPP_
#define DRPOWER_HARDWARE_INTERFACE__DRPOWER_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "drpower_hardware_interface/can_driver.hpp"
#include "drpower_hardware_interface/motor_state.hpp"

namespace drpower_hardware_interface
{

/**
 * @brief 电机控制配置结构
 */
struct MotorControlConfig
{
  // 角度控制配置
  struct PositionControl {
    int mode = 1;                      // 0=轨迹跟踪, 1=梯形轨迹, 2=前馈控制
    double filter_bandwidth = 50.0;   // mode=0时的滤波带宽 (<300)
    double acceleration = 10.0;        // mode=1时的加速度 (r/min/s)
    double feedforward_velocity = 0.0; // mode=2时的前馈速度 (r/min)
    double feedforward_torque = 0.0;   // mode=2时的前馈力矩 (Nm)
    double max_velocity = 60.0;        // 最大速度限制 (r/min)
  } position_control;
  
  // 速度控制配置
  struct VelocityControl {
    int mode = 1;                      // 0=直接控制, 其他=匀加速控制
    double feedforward_torque = 0.0;   // mode=0时的前馈力矩 (Nm)
    double acceleration = 5.0;         // mode≠0时的角加速度 (r/min/s)
  } velocity_control;
  
  // 力矩控制配置
  struct EffortControl {
    int mode = 1;                      // 控制模式
    double ramp_rate = 1.0;           // 力矩变化率
  } effort_control;
};

/**
 * @brief 电机组配置结构
 */
struct MotorGroup
{
  std::string name;                     // 组名 "arm", "gimbal", "gripper"
  std::vector<uint8_t> motor_ids;       // 电机ID列表
  std::vector<std::string> joint_names; // 关节名称列表
  MotorControlConfig config;            // 控制配置
  bool synchronized = true;             // 是否需要同步运动
  double position_tolerance = 0.01;     // 位置容差 (弧度)
  double velocity_tolerance = 0.1;      // 速度容差 (弧度/秒)
};

/**
 * @brief DrPower电机硬件接口类
 * 
 * 实现ros2_control的SystemInterface接口，为DrPower智能一体化关节
 * 提供标准化的硬件抽象层。支持位置、速度、力矩控制，以及多电机
 * 协调运动和灵活的电机组合配置。
 */
class DrPowerHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DrPowerHardwareInterface)

  /**
   * @brief 初始化硬件接口
   * @param info 硬件信息，包含关节配置和参数
   * @return 初始化结果
   */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief 配置硬件接口
   * @param previous_state 上一个状态
   * @return 配置结果
   */
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief 激活硬件接口
   * @param previous_state 上一个状态  
   * @return 激活结果
   */
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief 停用硬件接口
   * @param previous_state 上一个状态
   * @return 停用结果
   */
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief 导出状态接口
   * @return 状态接口列表
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief 导出命令接口
   * @return 命令接口列表
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief 读取硬件状态
   * @param time 当前时间
   * @param period 周期
   * @return 读取结果
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief 写入硬件命令
   * @param time 当前时间
   * @param period 周期
   * @return 写入结果
   */
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // CAN驱动
  std::unique_ptr<CanDriver> can_driver_;
  
  // 电机状态
  std::vector<MotorState> motor_states_;
  std::map<std::string, size_t> joint_name_to_index_;
  
  // 命令和状态变量
  std::vector<double> hw_position_commands_;
  std::vector<double> hw_velocity_commands_;
  std::vector<double> hw_effort_commands_;
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> hw_effort_states_;
  
  // 控制标志
  std::vector<bool> position_commands_updated_;
  std::vector<bool> velocity_commands_updated_;
  std::vector<bool> effort_commands_updated_;
  
  // 电机组配置
  std::vector<MotorGroup> motor_groups_;
  std::map<std::string, size_t> group_name_to_index_;
  
  // 配置参数
  std::string device_path_;
  int baudrate_;
  double control_frequency_;
  bool enable_state_feedback_;
  int state_feedback_rate_ms_;
  
  /**
   * @brief 加载关节配置
   * @param info 硬件信息
   * @return 加载成功与否
   */
  bool load_joint_configs(const hardware_interface::HardwareInfo & info);
  
  /**
   * @brief 加载电机组配置
   * @param info 硬件信息
   * @return 加载成功与否
   */
  bool load_motor_group_configs(const hardware_interface::HardwareInfo & info);
  
  /**
   * @brief 设置电机组位置
   * @param group_index 组索引
   * @return 设置成功与否
   */
  bool set_group_positions(size_t group_index);
  
  /**
   * @brief 设置单个电机位置
   * @param joint_index 关节索引
   * @return 设置成功与否
   */
  bool set_individual_position(size_t joint_index);
  
  /**
   * @brief 设置电机组速度
   * @param group_index 组索引  
   * @return 设置成功与否
   */
  bool set_group_velocities(size_t group_index);
  
  /**
   * @brief 设置单个电机速度
   * @param joint_index 关节索引
   * @return 设置成功与否
   */
  bool set_individual_velocity(size_t joint_index);
  
  /**
   * @brief 设置电机组力矩
   * @param group_index 组索引
   * @return 设置成功与否
   */
  bool set_group_efforts(size_t group_index);
  
  /**
   * @brief 设置单个电机力矩
   * @param joint_index 关节索引
   * @return 设置成功与否
   */
  bool set_individual_effort(size_t joint_index);
  
  /**
   * @brief 读取所有电机状态
   * @return 读取成功与否
   */
  bool read_all_motor_states();
  
  /**
   * @brief 初始化电机状态反馈
   * @return 初始化成功与否
   */
  bool initialize_state_feedback();
  
  /**
   * @brief 急停所有电机
   * @return 急停成功与否
   */
  bool emergency_stop();
  
  /**
   * @brief 角度单位转换：度 -> 弧度
   * @param degrees 角度值（度）
   * @return 角度值（弧度）
   */
  double degrees_to_radians(double degrees) const;
  
  /**
   * @brief 角度单位转换：弧度 -> 度
   * @param radians 角度值（弧度）
   * @return 角度值（度）
   */
  double radians_to_degrees(double radians) const;
  
  /**
   * @brief 速度单位转换：r/min -> rad/s
   * @param rpm 转速（r/min）
   * @return 角速度（rad/s）
   */
  double rpm_to_rad_per_sec(double rpm) const;
  
  /**
   * @brief 速度单位转换：rad/s -> r/min
   * @param rad_per_sec 角速度（rad/s）
   * @return 转速（r/min）
   */
  double rad_per_sec_to_rpm(double rad_per_sec) const;
};

}  // namespace drpower_hardware_interface

#endif  // DRPOWER_HARDWARE_INTERFACE__DRPOWER_HARDWARE_INTERFACE_HPP_
