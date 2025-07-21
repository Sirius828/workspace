#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/visibility_control.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <termios.h>
#include <string>
#include <vector>

namespace ti_diffbot_hardware
{

class TiDiffBotHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TiDiffBotHardware)

  /**
   * @brief 初始化硬件接口
   * @param info 硬件信息，包含关节配置和参数
   * @return 初始化结果
   */
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo & info) override;

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
   * @brief 激活硬件接口
   * @param previous_state 上一个状态
   * @return 激活结果
   */
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief 停用硬件接口
   * @param previous_state 上一个状态
   * @return 停用结果
   */
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief 读取硬件状态
   * @param time 当前时间
   * @param period 时间间隔
   * @return 读取结果
   */
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief 写入命令到硬件
   * @param time 当前时间
   * @param period 时间间隔
   * @return 写入结果
   */
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 串口参数
  int fd_{-1};
  std::string device_path_;
  int baud_rate_{115200};

  // 模拟模式
  bool simulation_mode_{false};

  // 编码器 & 机械参数
  double ticks_per_rev_{8192.0};
  double gear_ratio_{36.0};
  double wheel_radius_{0.035};
  double wheel_separation_{0.18};

  // 关节状态/指令
  double pos_[2]{0.0, 0.0};   // rad
  double vel_[2]{0.0, 0.0};   // rad/s
  double cmd_[2]{0.0, 0.0};   // rad/s

  // 辅助变量
  double rad_per_tick_{0.0};

  // 模拟变量
  rclcpp::Time last_time_;
  bool first_read_{true};
  
  // 硬件模式状态跟踪
  int hardware_read_count_{0};
  
  // 编码器积分变量
  long last_left_tick_{0};
  long last_right_tick_{0};
  bool encoder_initialized_{false};
  
  // 时间戳同步变量
  rclcpp::Time last_encoder_time_;
  rclcpp::Time last_command_time_;
  bool command_pending_{false};
  double pending_cmd_left_{0.0};
  double pending_cmd_right_{0.0};

  // 串口读写函数
  std::string read_line();
  void write_line(const std::string & str);
};

}  // namespace ti_diffbot_hardware
