#include "ti_diffbot_hardware/ti_diffbot_hardware.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <termios.h>
#include <errno.h>
#include <string.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ti_diffbot_hardware
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("TiDiffBotHardware");

// ------------ 串口工具函数 ----------------
static int open_serial(const std::string & dev, int baud)
{
  int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);  // 打开串口
  if (fd < 0) { 
    perror("open"); 
    return -1; 
  }

  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    perror("tcgetattr");
    ::close(fd);
    return -1;
  }

  // 清除所有标志
  tty.c_cflag = 0;
  tty.c_iflag = 0;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  
  // 设置基本配置
  tty.c_cflag = CLOCAL | CREAD | CS8;  // 本地连接，允许读取，8位数据
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);  // 无校验，1停止位，无流控
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);  // 禁用软件流控和CR/LF转换
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // 原始模式
  tty.c_oflag &= ~OPOST;  // 禁用输出处理
  
  // 设置非阻塞读取
  tty.c_cc[VTIME] = 0;  // 不等待
  tty.c_cc[VMIN] = 0;   // 立即返回
  
  // 设置波特率
  speed_t speed = B115200;
  switch (baud) {
    case 9600: speed = B9600; break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    case 57600: speed = B57600; break;
    case 115200: speed = B115200; break;
    default: speed = B115200; break;
  }
  
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);
  
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    perror("tcsetattr");
    ::close(fd);
    return -1;
  }
  
  // 清空输入输出缓冲区
  tcflush(fd, TCIOFLUSH);
  
  return fd;
}

std::string TiDiffBotHardware::read_line()
{
  static std::string buffer;  // 静态缓冲区，保存未完成的行
  char c;
  int bytes_read;
  int read_attempts = 0;
  const int max_read_attempts = 100;  // 限制读取尝试次数，防止阻塞
  
  // 继续读取字符直到找到完整的一行，但限制尝试次数
  while ((bytes_read = ::read(fd_, &c, 1)) == 1 && read_attempts < max_read_attempts)
  {
    read_attempts++;
    
    if (c == '\n') {
      // 找到完整的一行
      std::string line = buffer;
      buffer.clear();  // 清空缓冲区
      return line;
    } else if (c != '\r') {
      // 积累字符（忽略回车符）
      buffer += c;
      
      // 防止缓冲区过大
      if (buffer.length() > 512) {
        buffer.clear();
        break;
      }
    }
  }
  
  // 如果达到最大尝试次数，返回空字符串避免长时间阻塞
  if (read_attempts >= max_read_attempts) {
    static int throttle_count = 0;
    if (++throttle_count % 1000 == 1) {  // 限制日志频率
      RCLCPP_DEBUG(rclcpp::get_logger("TiDiffBotHardware"), 
                   "达到最大读取尝试次数，快速返回避免阻塞");
    }
  }
  
  // 没有完整的行，返回空字符串
  return "";
}

void TiDiffBotHardware::write_line(const std::string & str)
{
  ssize_t written = ::write(fd_, str.c_str(), str.size());
  if (written < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      RCLCPP_WARN(LOGGER, "串口繁忙，丢弃一帧命令");
    } else {
      RCLCPP_ERROR(LOGGER, "串口写入失败: %s", strerror(errno));
    }
  }
  // 发送换行符
  written = ::write(fd_, "\r", 1);
  if (written < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      RCLCPP_WARN(LOGGER, "串口繁忙，丢弃换行符");
    } else {
      RCLCPP_ERROR(LOGGER, "串口写入换行符失败: %s", strerror(errno));
    }
  }
}

// ------------ ros2_control 回调函数 ----------------
hardware_interface::CallbackReturn
TiDiffBotHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(LOGGER, "初始化 TiDiffBot 硬件接口...");

  // 读取硬件参数
  try {
    device_path_ = info_.hardware_parameters.at("device_path");
    baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
    ticks_per_rev_ = std::stod(info_.hardware_parameters.at("enc_ticks_per_rev"));
    gear_ratio_ = std::stod(info_.hardware_parameters.at("gear_ratio"));
    wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));
    wheel_separation_ = std::stod(info_.hardware_parameters.at("wheel_separation"));
    
    // 检查是否启用模拟模式
    if (info_.hardware_parameters.find("simulation_mode") != info_.hardware_parameters.end()) {
      simulation_mode_ = (info_.hardware_parameters.at("simulation_mode") == "true");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(LOGGER, "读取硬件参数失败: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 计算每个编码器刻度对应的弧度
  rad_per_tick_ = 2.0 * M_PI / (ticks_per_rev_ * gear_ratio_);

  RCLCPP_INFO(LOGGER, "硬件参数:");
  RCLCPP_INFO(LOGGER, "  设备路径: %s", device_path_.c_str());
  RCLCPP_INFO(LOGGER, "  波特率: %d", baud_rate_);
  RCLCPP_INFO(LOGGER, "  编码器刻度/转: %.0f", ticks_per_rev_);
  RCLCPP_INFO(LOGGER, "  减速比: %.0f", gear_ratio_);
  RCLCPP_INFO(LOGGER, "  轮半径: %.3f m", wheel_radius_);
  RCLCPP_INFO(LOGGER, "  轮间距: %.3f m", wheel_separation_);
  RCLCPP_INFO(LOGGER, "  模拟模式: %s", simulation_mode_ ? "启用" : "禁用");

  // 验证关节数量
  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(LOGGER, "差速轮小车需要恰好2个关节，但找到 %zu 个", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 验证关节名称
  if (info_.joints[0].name != "left_wheel_joint" || info_.joints[1].name != "right_wheel_joint") {
    RCLCPP_ERROR(LOGGER, "关节名称必须是 'left_wheel_joint' 和 'right_wheel_joint'");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(LOGGER, "TiDiffBot 硬件接口初始化成功");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
TiDiffBotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "left_wheel_joint", hardware_interface::HW_IF_POSITION, &pos_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &vel_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "right_wheel_joint", hardware_interface::HW_IF_POSITION, &pos_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &vel_[1]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
TiDiffBotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &cmd_[0]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &cmd_[1]));

  return command_interfaces;
}

hardware_interface::CallbackReturn
TiDiffBotHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "激活 TiDiffBot 硬件接口...");

  if (simulation_mode_) {
    RCLCPP_INFO(LOGGER, "模拟模式已启用，跳过串口连接");
    fd_ = 1;  // 设置为非负值表示"连接"成功
  } else {
    // 打开串口
    fd_ = open_serial(device_path_, baud_rate_);
    if (fd_ < 0) {
      RCLCPP_ERROR(LOGGER, "无法打开串口 %s", device_path_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(LOGGER, "串口 %s 打开成功", device_path_.c_str());
  }

  // 初始化状态
  pos_[0] = 0.0;
  pos_[1] = 0.0;
  vel_[0] = 0.0;
  vel_[1] = 0.0;
  cmd_[0] = 0.0;
  cmd_[1] = 0.0;
  first_read_ = true;
  
  // 重置编码器状态
  encoder_initialized_ = false;
  last_left_tick_ = 0;
  last_right_tick_ = 0;
  hardware_read_count_ = 0;
  
  // 初始化时间戳和命令状态
  last_encoder_time_ = rclcpp::Time(0);
  last_command_time_ = rclcpp::Time(0);
  command_pending_ = false;
  pending_cmd_left_ = 0.0;
  pending_cmd_right_ = 0.0;
  
  // 初始化时间戳
  last_encoder_time_ = rclcpp::Time(0);
  last_command_time_ = rclcpp::Time(0);
  command_pending_ = false;

  RCLCPP_INFO(LOGGER, "TiDiffBot 硬件接口激活成功");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
TiDiffBotHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "停用 TiDiffBot 硬件接口...");

  if (!simulation_mode_) {
    // 停止电机
    cmd_[0] = 0.0;
    cmd_[1] = 0.0;
    if (fd_ >= 0) {
      write_line("0,0");  // 修改：发送整型格式的停止命令
    }

    // 关闭串口
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }
  
  fd_ = -1;
  RCLCPP_INFO(LOGGER, "TiDiffBot 硬件接口停用成功");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
TiDiffBotHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (fd_ < 0) {
    return hardware_interface::return_type::ERROR;
  }

  if (simulation_mode_) {
    // 模拟模式：简单的积分器模拟
    if (first_read_) {
      last_time_ = time;
      last_encoder_time_ = time;
      first_read_ = false;
      return hardware_interface::return_type::OK;
    }
    
    double dt = period.seconds();
    if (dt > 0.0) {
      // 积分速度得到位置
      pos_[0] += vel_[0] * dt;
      pos_[1] += vel_[1] * dt;
      
      // 简单的一阶滤波模拟速度响应
      double alpha = 0.1;  // 滤波系数
      vel_[0] = (1.0 - alpha) * vel_[0] + alpha * cmd_[0];
      vel_[1] = (1.0 - alpha) * vel_[1] + alpha * cmd_[1];
    }
    
    last_encoder_time_ = time;
    return hardware_interface::return_type::OK;
  }

  // 实际硬件模式：读取串口数据
  hardware_read_count_++;
  
  // 在硬件模式启动初期给出提示
  if (hardware_read_count_ == 50) {
    RCLCPP_INFO(LOGGER, "硬件模式运行中，等待下位机数据...");
  } else if (hardware_read_count_ == 500) {
    RCLCPP_WARN(LOGGER, "数据接收可能不稳定，建议检查下位机发送频率");
  } else if (hardware_read_count_ == 1000) {
    RCLCPP_ERROR(LOGGER, "数据接收间歇性中断，请检查：1)下位机发送稳定性 2)数据格式一致性");
  }
  
  // 检查是否有待处理的命令需要立即响应
  if (command_pending_) {
    auto command_age = (time - last_command_time_).seconds();
    if (command_age < 0.1) {  // 命令发出100ms内，提供即时速度估计
      // 基于命令提供即时速度反馈，避免等待硬件数据
      vel_[0] = 0.8 * pending_cmd_left_ + 0.2 * vel_[0];  // 快速响应新命令
      vel_[1] = 0.8 * pending_cmd_right_ + 0.2 * vel_[1];
      
      RCLCPP_DEBUG(LOGGER, "🚀 立即响应命令: 左=%.3f, 右=%.3f rad/s", vel_[0], vel_[1]);
    }
  }
  
  std::string line = read_line();
  
  if (line.empty()) {
    // 没有新数据时的处理
    auto time_since_last_encoder = (time - last_encoder_time_).seconds();
    
    if (time_since_last_encoder > 0.05) {  // 50ms没有新数据
      // 应用速度衰减避免"冻结"效应
      double decay_factor = std::max(0.8, 1.0 - time_since_last_encoder * 2.0);  // 动态衰减
      vel_[0] *= decay_factor;
      vel_[1] *= decay_factor;
      
      // 如果有待处理命令且时间较短，继续积分位置
      if (command_pending_ && (time - last_command_time_).seconds() < 0.2) {
        double dt = period.seconds();
        pos_[0] += vel_[0] * dt;
        pos_[1] += vel_[1] * dt;
      }
    }
    
    return hardware_interface::return_type::OK;
  }

  // 收到新数据，更新时间戳
  last_encoder_time_ = time;
  command_pending_ = false;  // 清除待处理状态

  // 收到数据时记录日志（第一次收到数据时立即显示）
  static bool first_data_received = false;
  static int data_count = 0;
  static auto last_stat_time = rclcpp::Time(0);
  
  if (!first_data_received) {
    RCLCPP_INFO(LOGGER, "✅ 成功收到下位机数据: %s", line.c_str());
    first_data_received = true;
    last_stat_time = time;
  }
  
  data_count++;
  
  // 每5秒报告一次数据接收统计
  if ((time - last_stat_time).seconds() >= 5.0) {
    double rate = data_count / 5.0;
    RCLCPP_INFO(LOGGER, "📊 数据接收统计: %.1f Hz (%d 条/5秒)", rate, data_count);
    data_count = 0;
    last_stat_time = time;
  }
  
  auto clock = rclcpp::Clock();
  RCLCPP_DEBUG_THROTTLE(LOGGER, clock, 5000, "串口数据: %s", line.c_str());

  std::stringstream ss(line);
  std::string token;
  std::vector<std::string> tokens;

  // 分割字符串
  while (std::getline(ss, token, ',')) {
    tokens.push_back(token);
  }

  if (tokens.size() >= 4) {
    try {
      long left_tick = std::stol(tokens[0]);
      long right_tick = std::stol(tokens[1]);
      // 修改：接收整型RPM，除以100得到实际RPM值
      int left_rpm_int = std::stoi(tokens[2]);
      int right_rpm_int = std::stoi(tokens[3]);
      double left_rpm = static_cast<double>(left_rpm_int) / 100.0;
      double right_rpm = static_cast<double>(right_rpm_int) / 100.0;
      
      // 调试：显示整型转换过程
      if (data_count % 200 == 1) {
        RCLCPP_DEBUG(LOGGER, "📥 RPM解析: 接收整型 L=%d→%.2f, R=%d→%.2f", 
                    left_rpm_int, left_rpm, right_rpm_int, right_rpm);
      }

      // 首次读取时，初始化编码器基准值
      if (!encoder_initialized_) {
        last_left_tick_ = left_tick;
        last_right_tick_ = right_tick;
        encoder_initialized_ = true;
        RCLCPP_INFO(LOGGER, "🎯 编码器初始化: 左轮=%ld, 右轮=%ld", left_tick, right_tick);
        return hardware_interface::return_type::OK;
      }

      // 计算编码器增量（处理可能的溢出）
      long left_delta = left_tick - last_left_tick_;
      long right_delta = right_tick - last_right_tick_;
      
      // 处理编码器溢出（假设32位有符号整数）
      const long OVERFLOW_THRESHOLD = 1000000000;  // 10亿，远小于2^31
      if (std::abs(left_delta) > OVERFLOW_THRESHOLD) {
        RCLCPP_WARN(LOGGER, "检测到左编码器可能溢出，忽略此次读取");
        return hardware_interface::return_type::OK;
      }
      if (std::abs(right_delta) > OVERFLOW_THRESHOLD) {
        RCLCPP_WARN(LOGGER, "检测到右编码器可能溢出，忽略此次读取");
        return hardware_interface::return_type::OK;
      }

      // 右轮编码器方向修正：右轮前进时编码器减小，所以需要取反
      right_delta = -right_delta;

      // 更新位置（累积编码器增量）
      pos_[0] += left_delta * rad_per_tick_;   // 左轮：正方向
      pos_[1] += right_delta * rad_per_tick_;  // 右轮：已修正方向

      // 计算基于ROS时间的速度（更准确的实时速度）
      static rclcpp::Time last_encoder_update_time = time;
      static double last_left_vel = 0.0;
      static double last_right_vel = 0.0;
      
      double dt_seconds = (time - last_encoder_update_time).seconds();
      
      if (dt_seconds > 0.001 && encoder_initialized_) {  // 确保时间差合理且已初始化
        // 基于编码器增量和ROS时间差计算速度
        double calc_vel_left = (left_delta * rad_per_tick_) / dt_seconds;
        double calc_vel_right = (right_delta * rad_per_tick_) / dt_seconds;
        
        // 与RPM速度融合（加权平均）
        double rpm_vel_left = left_rpm * 2.0 * M_PI / 60.0;
        double rpm_vel_right = -right_rpm * 2.0 * M_PI / 60.0;  // 右轮取反
        
        // 根据数据质量调整融合权重
        double encoder_weight = 0.7;
        double rpm_weight = 0.3;
        
        // 如果编码器增量过大或过小，降低其权重
        if (std::abs(left_delta) > 1000 || std::abs(right_delta) > 1000) {
          encoder_weight = 0.3;
          rpm_weight = 0.7;
        } else if (std::abs(left_delta) < 2 && std::abs(right_delta) < 2) {
          encoder_weight = 0.5;
          rpm_weight = 0.5;
        }
        
        vel_[0] = encoder_weight * calc_vel_left + rpm_weight * rpm_vel_left;
        vel_[1] = encoder_weight * calc_vel_right + rpm_weight * rpm_vel_right;
        
        // 速度平滑滤波（时间相关的滤波系数）
        double alpha = std::min(0.8, dt_seconds * 5.0);  // 动态滤波系数
        vel_[0] = alpha * vel_[0] + (1.0 - alpha) * last_left_vel;
        vel_[1] = alpha * vel_[1] + (1.0 - alpha) * last_right_vel;
        
        // 调试输出（显示速度计算细节）
        if (data_count % 100 == 1) {
          RCLCPP_DEBUG(LOGGER, "🔄 速度计算: dt=%.3fs, 编码器权重=%.1f, 最终速度: L=%.3f, R=%.3f", 
                      dt_seconds, encoder_weight, vel_[0], vel_[1]);
        }
      } else {
        // 时间差过小或未初始化时，直接使用RPM数据
        vel_[0] = left_rpm * 2.0 * M_PI / 60.0;
        vel_[1] = -right_rpm * 2.0 * M_PI / 60.0;
      }
      
      // 保存状态用于下次计算
      last_left_vel = vel_[0];
      last_right_vel = vel_[1];
      last_encoder_update_time = time;

      // 保存当前编码器值用于下次计算增量
      last_left_tick_ = left_tick;
      last_right_tick_ = right_tick;

      // 调试输出（每100次读取输出一次）
      if (data_count % 100 == 1) {
        RCLCPP_DEBUG(LOGGER, "📊 编码器增量: 左=%ld, 右=%ld(已修正), 位置: 左=%.3f, 右=%.3f", 
                    left_delta, right_delta, pos_[0], pos_[1]);
      }

    } catch (const std::exception & e) {
      auto clock = rclcpp::Clock();
      RCLCPP_WARN_THROTTLE(LOGGER, clock, 1000,
                           "解析串口数据失败: %s", e.what());
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
TiDiffBotHardware::write(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (fd_ < 0) {
    return hardware_interface::return_type::ERROR;
  }

  if (simulation_mode_) {
    // 模拟模式：不需要发送串口数据
    return hardware_interface::return_type::OK;
  }

  // 命令防抖：避免发送相同的命令
  static int last_cmd_left = 99999;  // 初始化为不可能的整型值
  static int last_cmd_right = 99999;
  static rclcpp::Time last_write_time = time;
  
  auto write_time_diff = (time - last_write_time).seconds();
  
  // 将速度命令从 rad/s 转换为 RPM
  double rpm_l = cmd_[0] * 60.0 / (2.0 * M_PI);
  // 右轮控制方向修正：由于右轮编码器反向，控制命令也需要反向
  double rpm_r = cmd_[1] * 60.0 / (2.0 * M_PI);

  // 修改：将RPM转换为整型进行通信（乘以100）
  int rpm_l_int = static_cast<int>(std::round(rpm_l * 100.0));
  int rpm_r_int = static_cast<int>(std::round(rpm_r * 100.0));

  // 检查命令是否有显著变化或时间间隔足够长
  const int cmd_threshold = 10;  // 整型RPM差异阈值（相当于0.1 RPM）
  const double min_interval_s = 0.01;  // 最小发送间隔10ms
  
  bool cmd_changed = (std::abs(rpm_l_int - last_cmd_left) > cmd_threshold) || 
                     (std::abs(rpm_r_int - last_cmd_right) > cmd_threshold);
  bool time_elapsed = write_time_diff >= min_interval_s;
  
  if (cmd_changed || time_elapsed) {
    // 修改：发送整型RPM
    std::ostringstream oss;
    oss << rpm_l_int << "," << rpm_r_int;
    
    write_line(oss.str());
    
    // 更新命令跟踪状态（保存整型值）
    last_cmd_left = rpm_l_int;
    last_cmd_right = rpm_r_int;
    last_write_time = time;
    
    // 设置待处理命令状态（用于立即响应）
    if (cmd_changed) {
      command_pending_ = true;
      last_command_time_ = time;
      pending_cmd_left_ = cmd_[0];   // 保存原始rad/s命令
      pending_cmd_right_ = cmd_[1];
      
      RCLCPP_DEBUG(LOGGER, "🚀 新命令发送: 左=%d (%.2f RPM), 右=%d (%.2f RPM), rad/s=(%.3f, %.3f)", 
                  rpm_l_int, rpm_l, rpm_r_int, rpm_r, cmd_[0], cmd_[1]);
    }
    
    // 调试输出（限制频率）
    static int write_count = 0;
    write_count++;
    if (write_count % 50 == 1) {  // 每50次输出一次
      RCLCPP_DEBUG(LOGGER, "📤 命令统计: 左=%d (%.2f RPM), 右=%d (%.2f RPM)", 
                  rpm_l_int, rpm_l, rpm_r_int, rpm_r);
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace ti_diffbot_hardware

// 导出插件
PLUGINLIB_EXPORT_CLASS(ti_diffbot_hardware::TiDiffBotHardware, hardware_interface::SystemInterface)
