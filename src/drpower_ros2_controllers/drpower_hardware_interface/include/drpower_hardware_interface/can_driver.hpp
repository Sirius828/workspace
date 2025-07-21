#ifndef DRPOWER_HARDWARE_INTERFACE__CAN_DRIVER_HPP_
#define DRPOWER_HARDWARE_INTERFACE__CAN_DRIVER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <mutex>

#include "drpower_hardware_interface/motor_state.hpp"

namespace drpower_hardware_interface
{

/**
 * @brief CAN通信驱动类
 * 
 * 实现与DrPower智能一体化关节的CAN通信协议。
 * 基于原始Python实现的通信格式，提供C++接口。
 */
class CanDriver
{
public:
  /**
   * @brief 构造函数
   */
  CanDriver();
  
  /**
   * @brief 析构函数
   */
  ~CanDriver();

  /**
   * @brief 初始化CAN驱动
   * @param device_path 设备路径，如 "/dev/ttyUSB0"
   * @param baudrate 波特率，默认115200
   * @return 初始化成功与否
   */
  bool initialize(const std::string& device_path, int baudrate = 115200);

  /**
   * @brief 关闭CAN驱动
   */
  void close();

  /**
   * @brief 发送位置控制命令（直接模式）
   * @param motor_id 电机ID
   * @param angle 目标角度（度）
   * @param speed 速度参数（含义由mode决定）
   * @param param 运动参数（含义由mode决定）
   * @param mode 控制模式 0=轨迹跟踪, 1=梯形轨迹, 2=前馈控制
   * @return 发送成功与否
   */
  bool send_position_command(uint8_t motor_id, double angle, double speed = 0, 
                           double param = 0, uint32_t mode = 0);

  /**
   * @brief 预设位置命令（用于多电机协调）
   * @param motor_id 电机ID
   * @param angle 目标角度（度）
   * @param time_param 时间参数
   * @param param 运动参数
   * @param mode 控制模式
   * @return 预设成功与否
   */
  bool preset_position_command(uint8_t motor_id, double angle, double time_param = 0, 
                             double param = 0, uint32_t mode = 0);

  /**
   * @brief 发送速度控制命令
   * @param motor_id 电机ID
   * @param speed 目标速度（r/min）
   * @param param 运动参数
   * @param mode 控制模式
   * @return 发送成功与否
   */
  bool send_velocity_command(uint8_t motor_id, double speed, double param = 0, uint32_t mode = 1);

  /**
   * @brief 预设速度命令
   * @param motor_id 电机ID
   * @param speed 目标速度（r/min）
   * @param param 运动参数
   * @param mode 控制模式
   * @return 预设成功与否
   */
  bool preset_velocity_command(uint8_t motor_id, double speed, double param = 0, uint32_t mode = 1);

  /**
   * @brief 发送力矩控制命令
   * @param motor_id 电机ID
   * @param torque 目标力矩（Nm）
   * @param param 运动参数
   * @param mode 控制模式
   * @return 发送成功与否
   */
  bool send_effort_command(uint8_t motor_id, double torque, double param = 0, uint32_t mode = 1);

  /**
   * @brief 预设力矩命令
   * @param motor_id 电机ID
   * @param torque 目标力矩（Nm）
   * @param param 运动参数
   * @param mode 控制模式
   * @return 预设成功与否
   */
  bool preset_effort_command(uint8_t motor_id, double torque, double param = 0, uint32_t mode = 1);

  /**
   * @brief 发送同步启动命令
   * @param start_command 启动命令
   * - 0x10: 轨迹跟踪模式启动
   * - 0x11: 梯形轨迹模式启动  
   * - 0x12: 前馈控制模式启动
   * - 0x13: 速度控制模式启动
   * @return 发送成功与否
   */
  bool send_start_command(uint32_t start_command);

  /**
   * @brief 读取电机状态
   * @param motor_id 电机ID
   * @param state 返回的状态数据
   * @return 读取成功与否
   */
  bool read_motor_state(uint8_t motor_id, MotorState& state);

  /**
   * @brief 获取电机当前角度
   * @param motor_id 电机ID
   * @param angle 返回的角度值（度）
   * @return 读取成功与否
   */
  bool get_motor_angle(uint8_t motor_id, double& angle);

  /**
   * @brief 获取电机当前速度
   * @param motor_id 电机ID
   * @param speed 返回的速度值（r/min）
   * @return 读取成功与否
   */
  bool get_motor_speed(uint8_t motor_id, double& speed);

  /**
   * @brief 获取电机当前力矩
   * @param motor_id 电机ID
   * @param torque 返回的力矩值（Nm）
   * @return 读取成功与否
   */
  bool get_motor_torque(uint8_t motor_id, double& torque);

  /**
   * @brief 开启电机状态实时反馈
   * @param motor_id 电机ID
   * @param rate_ms 反馈时间间隔（毫秒）
   * @return 开启成功与否
   */
  bool enable_state_feedback(uint8_t motor_id, uint16_t rate_ms = 10);

  /**
   * @brief 关闭电机状态实时反馈
   * @param motor_id 电机ID
   * @return 关闭成功与否
   */
  bool disable_state_feedback(uint8_t motor_id);

  /**
   * @brief 设置电机零点
   * @param motor_id 电机ID
   * @return 设置成功与否
   */
  bool set_zero_position(uint8_t motor_id);

  /**
   * @brief 急停电机
   * @param motor_id 电机ID，0表示急停所有电机
   * @return 急停成功与否
   */
  bool emergency_stop(uint8_t motor_id = 0);

  /**
   * @brief 设置电机ID
   * @param current_id 当前ID
   * @param new_id 新ID
   * @return 设置成功与否
   */
  bool set_motor_id(uint8_t current_id, uint8_t new_id);

  /**
   * @brief 检查连接状态
   * @return 连接是否正常
   */
  bool is_connected() const;

private:
  // 串口文件描述符
  int serial_fd_;
  
  // 设备路径和波特率
  std::string device_path_;
  int baudrate_;
  
  // 连接状态
  bool connected_;
  
  // 数据缓冲区
  std::vector<uint8_t> read_buffer_;
  std::vector<uint8_t> write_buffer_;
  
  // 读取标志（对应Python中的READ_FLAG）
  int read_flag_;
  
  // 线程安全
  mutable std::mutex communication_mutex_;

  /**
   * @brief 发送CAN命令的底层函数
   * @param motor_id 电机ID
   * @param cmd 命令字
   * @param data 数据载荷
   * @param rtr 远程帧标志
   * @return 发送成功与否
   */
  bool send_can_command(uint8_t motor_id, uint8_t cmd, const std::vector<uint8_t>& data, uint8_t rtr = 0);

  /**
   * @brief CAN报文转串行帧（对应Python中的can_to_uart）
   * @param can_data CAN数据
   * @param rtr 远程帧标志
   * @return 串行帧数据
   */
  std::vector<uint8_t> can_to_uart(const std::vector<uint8_t>& can_data, uint8_t rtr = 0);

  /**
   * @brief 串行帧转CAN报文（对应Python中的uart_to_can）
   * @param uart_data 串行帧数据
   * @return CAN数据
   */
  std::vector<uint8_t> uart_to_can(const std::vector<uint8_t>& uart_data);

  /**
   * @brief 数据格式转换（对应Python中的format_data）
   * @param data 源数据
   * @param format 格式字符串，如 "f s16 s16"
   * @param encode true=编码, false=解码
   * @return 转换后的数据
   */
  std::vector<uint8_t> format_data(const std::vector<double>& data, const std::string& format, bool encode = true);

  /**
   * @brief 解析数据格式
   * @param data 原始数据
   * @param format 格式字符串
   * @return 解析后的数据
   */
  std::vector<double> parse_data(const std::vector<uint8_t>& data, const std::string& format);

  /**
   * @brief 写入串口数据
   * @param data 数据
   * @return 写入成功与否
   */
  bool write_serial_data(const std::vector<uint8_t>& data);

  /**
   * @brief 读取串口数据
   * @param expected_length 期望的数据长度
   * @return 读取的数据
   */
  std::vector<uint8_t> read_serial_data(size_t expected_length);

  /**
   * @brief 等待数据接收
   * @param timeout_ms 超时时间（毫秒）
   * @return 是否有数据可读
   */
  bool wait_for_data(int timeout_ms = 100);

  /**
   * @brief 计算CAN ID
   * @param motor_id 电机ID
   * @param cmd 命令字
   * @return CAN ID
   */
  uint16_t calculate_can_id(uint8_t motor_id, uint8_t cmd) const;

  /**
   * @brief 获取启动命令字
   * @param mode 控制模式
   * @return 启动命令
   */
  uint32_t get_start_command_for_mode(uint32_t mode) const;
};

}  // namespace drpower_hardware_interface

#endif  // DRPOWER_HARDWARE_INTERFACE__CAN_DRIVER_HPP_
