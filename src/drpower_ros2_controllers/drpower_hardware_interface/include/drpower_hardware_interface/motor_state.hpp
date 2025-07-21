#ifndef DRPOWER_HARDWARE_INTERFACE__MOTOR_STATE_HPP_
#define DRPOWER_HARDWARE_INTERFACE__MOTOR_STATE_HPP_

#include <cstdint>
#include <chrono>

namespace drpower_hardware_interface
{

/**
 * @brief 电机状态结构
 * 
 * 保存单个DrPower电机的实时状态信息，包括位置、速度、
 * 力矩、电压、电流等物理量以及错误状态。
 */
struct MotorState
{
  // 电机基本信息
  uint8_t id;                     ///< 电机ID (1-64)
  
  // 运动状态
  double position;                ///< 当前角度（弧度）
  double velocity;                ///< 当前角速度（弧度/秒）
  double effort;                  ///< 当前力矩（Nm）
  
  // 电气状态  
  double voltage;                 ///< 总线电压（V）
  double current;                 ///< FOC q轴电流（A）
  double temperature;             ///< 电机温度（°C）
  
  // 控制状态
  double position_command;        ///< 位置命令（弧度）
  double velocity_command;        ///< 速度命令（弧度/秒）
  double effort_command;          ///< 力矩命令（Nm）
  
  // 状态标志
  bool online;                    ///< 电机在线状态
  bool enabled;                   ///< 电机使能状态
  bool moving;                    ///< 电机运动状态
  bool position_reached;          ///< 位置到达标志
  
  // 错误状态
  uint32_t error_code;            ///< 错误代码
  bool has_error;                 ///< 是否有错误
  bool emergency_stopped;         ///< 急停状态
  
  // PID参数（可读取）
  double position_kp;             ///< 位置环比例增益
  double position_ki;             ///< 位置环积分增益  
  double velocity_kd;             ///< 速度环微分增益
  
  // 限位信息
  double position_limit_min;      ///< 最小位置限制（弧度）
  double position_limit_max;      ///< 最大位置限制（弧度）
  double velocity_limit;          ///< 速度限制（弧度/秒）
  double effort_limit;            ///< 力矩限制（Nm）
  
  // 时间戳
  std::chrono::steady_clock::time_point last_update_time;  ///< 最后更新时间
  double update_frequency;        ///< 更新频率（Hz）
  
  /**
   * @brief 默认构造函数
   */
  MotorState() 
    : id(0)
    , position(0.0)
    , velocity(0.0)
    , effort(0.0)
    , voltage(0.0)
    , current(0.0)
    , temperature(0.0)
    , position_command(0.0)
    , velocity_command(0.0)
    , effort_command(0.0)
    , online(false)
    , enabled(false)
    , moving(false)
    , position_reached(false)
    , error_code(0)
    , has_error(false)
    , emergency_stopped(false)
    , position_kp(0.0)
    , position_ki(0.0)
    , velocity_kd(0.0)
    , position_limit_min(-6.28)  // -2π
    , position_limit_max(6.28)   // +2π
    , velocity_limit(10.0)       // 10 rad/s
    , effort_limit(5.0)          // 5 Nm
    , last_update_time(std::chrono::steady_clock::now())
    , update_frequency(0.0)
  {
  }

  /**
   * @brief 带ID的构造函数
   * @param motor_id 电机ID
   */
  explicit MotorState(uint8_t motor_id)
    : MotorState()
  {
    id = motor_id;
  }

  /**
   * @brief 重置状态
   */
  void reset()
  {
    position = 0.0;
    velocity = 0.0;
    effort = 0.0;
    voltage = 0.0;
    current = 0.0;
    temperature = 0.0;
    position_command = 0.0;
    velocity_command = 0.0;
    effort_command = 0.0;
    online = false;
    enabled = false;
    moving = false;
    position_reached = false;
    error_code = 0;
    has_error = false;
    emergency_stopped = false;
    last_update_time = std::chrono::steady_clock::now();
    update_frequency = 0.0;
  }

  /**
   * @brief 更新时间戳和频率
   */
  void update_timestamp()
  {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      now - last_update_time).count();
    
    if (duration > 0) {
      update_frequency = 1000000.0 / duration;  // Hz
    }
    
    last_update_time = now;
  }

  /**
   * @brief 检查位置是否到达目标
   * @param tolerance 容差（弧度）
   * @return 是否到达
   */
  bool is_position_reached(double tolerance = 0.01) const
  {
    return std::abs(position - position_command) < tolerance;
  }

  /**
   * @brief 检查速度是否为零
   * @param tolerance 容差（弧度/秒）
   * @return 是否静止
   */
  bool is_stationary(double tolerance = 0.1) const
  {
    return std::abs(velocity) < tolerance;
  }

  /**
   * @brief 检查是否在位置限制范围内
   * @return 是否在范围内
   */
  bool is_within_position_limits() const
  {
    return position >= position_limit_min && position <= position_limit_max;
  }

  /**
   * @brief 检查是否在速度限制范围内
   * @return 是否在范围内
   */
  bool is_within_velocity_limits() const
  {
    return std::abs(velocity) <= velocity_limit;
  }

  /**
   * @brief 检查是否在力矩限制范围内
   * @return 是否在范围内
   */
  bool is_within_effort_limits() const
  {
    return std::abs(effort) <= effort_limit;
  }

  /**
   * @brief 获取错误描述
   * @return 错误描述字符串
   */
  std::string get_error_description() const
  {
    if (!has_error) {
      return "No error";
    }
    
    // 根据错误代码返回描述
    switch (error_code) {
      case 0x01: return "Over voltage";
      case 0x02: return "Under voltage";
      case 0x04: return "Over current";
      case 0x08: return "Over temperature";
      case 0x10: return "Position limit exceeded";
      case 0x20: return "Velocity limit exceeded";
      case 0x40: return "Communication timeout";
      case 0x80: return "Emergency stop";
      default: return "Unknown error: 0x" + std::to_string(error_code);
    }
  }

  /**
   * @brief 转换为字符串表示
   * @return 状态字符串
   */
  std::string to_string() const
  {
    std::string result = "MotorState[ID=" + std::to_string(id) + "]:\n";
    result += "  Position: " + std::to_string(position) + " rad\n";
    result += "  Velocity: " + std::to_string(velocity) + " rad/s\n";
    result += "  Effort: " + std::to_string(effort) + " Nm\n";
    result += "  Voltage: " + std::to_string(voltage) + " V\n";
    result += "  Current: " + std::to_string(current) + " A\n";
    result += "  Temperature: " + std::to_string(temperature) + " °C\n";
    result += "  Online: " + std::string(online ? "true" : "false") + "\n";
    result += "  Enabled: " + std::string(enabled ? "true" : "false") + "\n";
    result += "  Moving: " + std::string(moving ? "true" : "false") + "\n";
    result += "  Error: " + get_error_description() + "\n";
    result += "  Update freq: " + std::to_string(update_frequency) + " Hz\n";
    return result;
  }
};

/**
 * @brief 电机错误代码定义
 */
namespace MotorErrorCodes
{
  constexpr uint32_t NO_ERROR = 0x00;
  constexpr uint32_t OVER_VOLTAGE = 0x01;
  constexpr uint32_t UNDER_VOLTAGE = 0x02;
  constexpr uint32_t OVER_CURRENT = 0x04;
  constexpr uint32_t OVER_TEMPERATURE = 0x08;
  constexpr uint32_t POSITION_LIMIT = 0x10;
  constexpr uint32_t VELOCITY_LIMIT = 0x20;
  constexpr uint32_t COMMUNICATION_TIMEOUT = 0x40;
  constexpr uint32_t EMERGENCY_STOP = 0x80;
}

/**
 * @brief 电机控制模式定义
 */
namespace MotorControlModes
{
  // 位置控制模式
  constexpr uint32_t POSITION_TRAJECTORY_TRACKING = 0;  ///< 轨迹跟踪模式
  constexpr uint32_t POSITION_TRAPEZOIDAL = 1;          ///< 梯形轨迹模式
  constexpr uint32_t POSITION_FEEDFORWARD = 2;          ///< 前馈控制模式
  
  // 速度控制模式
  constexpr uint32_t VELOCITY_DIRECT = 0;               ///< 直接控制模式
  constexpr uint32_t VELOCITY_ACCELERATION = 1;         ///< 匀加速控制模式
  
  // 力矩控制模式
  constexpr uint32_t EFFORT_DIRECT = 0;                 ///< 直接控制模式
  constexpr uint32_t EFFORT_RAMP = 1;                   ///< 斜坡控制模式
}

/**
 * @brief 启动命令定义
 */
namespace StartCommands
{
  constexpr uint32_t TRAJECTORY_TRACKING = 0x10;        ///< 轨迹跟踪模式启动
  constexpr uint32_t TRAPEZOIDAL_TRAJECTORY = 0x11;     ///< 梯形轨迹模式启动
  constexpr uint32_t FEEDFORWARD_CONTROL = 0x12;        ///< 前馈控制模式启动
  constexpr uint32_t VELOCITY_CONTROL = 0x13;           ///< 速度控制模式启动
  constexpr uint32_t EFFORT_CONTROL = 0x14;             ///< 力矩控制模式启动（自定义）
}

}  // namespace drpower_hardware_interface

#endif  // DRPOWER_HARDWARE_INTERFACE__MOTOR_STATE_HPP_
