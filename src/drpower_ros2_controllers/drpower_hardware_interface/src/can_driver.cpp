#include "drpower_hardware_interface/can_driver.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace drpower_hardware_interface
{

CanDriver::CanDriver()
  : serial_fd_(-1)
  , baudrate_(115200)
  , connected_(false)
  , read_flag_(0)
{
  read_buffer_.reserve(1024);
  write_buffer_.reserve(1024);
}

CanDriver::~CanDriver()
{
  close();
}

bool CanDriver::initialize(const std::string& device_path, int baudrate)
{
  std::lock_guard<std::mutex> lock(communication_mutex_);
  
  device_path_ = device_path;
  baudrate_ = baudrate;

  // 打开串口
  serial_fd_ = open(device_path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0)
  {
    std::cerr << "Error opening " << device_path << ": " << strerror(errno) << std::endl;
    return false;
  }

  // 配置串口参数
  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0)
  {
    std::cerr << "Error getting terminal attributes: " << strerror(errno) << std::endl;
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // 设置波特率
  speed_t speed;
  switch (baudrate)
  {
    case 9600: speed = B9600; break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    case 57600: speed = B57600; break;
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;
    case 460800: speed = B460800; break;
    default:
      std::cerr << "Unsupported baudrate: " << baudrate << std::endl;
      ::close(serial_fd_);
      serial_fd_ = -1;
      return false;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // 设置串口参数
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  tty.c_iflag &= ~IGNBRK;                         // disable break processing
  tty.c_lflag = 0;                                // no signaling chars, no echo,
  tty.c_oflag = 0;                                // no remapping, no delays
  tty.c_cc[VMIN] = 0;                             // read doesn't block
  tty.c_cc[VTIME] = 5;                            // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xon/xoff ctrl
  tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls,
  tty.c_cflag &= ~(PARENB | PARODD);              // shut off parity
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
  {
    std::cerr << "Error setting terminal attributes: " << strerror(errno) << std::endl;
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  connected_ = true;
  std::cout << "CAN driver initialized successfully on " << device_path << " at " << baudrate << " baud" << std::endl;
  return true;
}

void CanDriver::close()
{
  std::lock_guard<std::mutex> lock(communication_mutex_);
  
  if (serial_fd_ >= 0)
  {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
  connected_ = false;
}

bool CanDriver::send_position_command(uint8_t motor_id, double angle, double speed, double param, uint32_t mode)
{
  const double factor = 0.01;
  std::vector<uint8_t> data;

  if (mode == 0) {
    // 轨迹跟踪模式
    float f_angle = static_cast<float>(angle);
    int16_t s16_speed = static_cast<int16_t>(std::abs(speed) / factor);
    if (param > 300) param = 300;
    int16_t s16_width = static_cast<int16_t>(std::abs(param) / factor);
    data = format_data({f_angle, static_cast<double>(s16_speed), static_cast<double>(s16_width)}, "f s16 s16");
    return send_can_command(motor_id, 0x19, data);
  }
  else if (mode == 1) {
    // 梯形轨迹模式
    if (speed > 0 && param > 0) {
      float f_angle = static_cast<float>(angle);
      int16_t s16_speed = static_cast<int16_t>(std::abs(speed) / factor);
      int16_t s16_accel = static_cast<int16_t>(std::abs(param) / factor);
      data = format_data({f_angle, static_cast<double>(s16_speed), static_cast<double>(s16_accel)}, "f s16 s16");
      return send_can_command(motor_id, 0x1A, data);
    }
  }
  else if (mode == 2) {
    // 前馈控制模式
    float f_angle = static_cast<float>(angle);
    int16_t s16_speed_ff = static_cast<int16_t>(speed / factor);
    int16_t s16_torque_ff = static_cast<int16_t>(param / factor);
    data = format_data({f_angle, static_cast<double>(s16_speed_ff), static_cast<double>(s16_torque_ff)}, "f s16 s16");
    return send_can_command(motor_id, 0x1B, data);
  }

  return false;
}

bool CanDriver::preset_position_command(uint8_t motor_id, double angle, double time_param, double param, uint32_t mode)
{
  const double factor = 0.01;
  std::vector<uint8_t> data;

  if (mode == 0) {
    // 轨迹跟踪模式
    float f_angle = static_cast<float>(angle);
    int16_t s16_time = static_cast<int16_t>(std::abs(time_param) / factor);
    if (param > 300) param = 300;
    int16_t s16_width = static_cast<int16_t>(std::abs(param) / factor);
    data = format_data({f_angle, static_cast<double>(s16_time), static_cast<double>(s16_width)}, "f s16 s16");
  }
  else if (mode == 1) {
    // 梯形轨迹模式
    float f_angle = static_cast<float>(angle);
    int16_t s16_time = static_cast<int16_t>(std::abs(time_param) / factor);
    int16_t s16_accel = static_cast<int16_t>(std::abs(param) / factor);
    data = format_data({f_angle, static_cast<double>(s16_time), static_cast<double>(s16_accel)}, "f s16 s16");
  }
  else if (mode == 2) {
    // 前馈控制模式
    float f_angle = static_cast<float>(angle);
    int16_t s16_speed_ff = static_cast<int16_t>(time_param / factor);
    int16_t s16_torque_ff = static_cast<int16_t>(param / factor);
    data = format_data({f_angle, static_cast<double>(s16_speed_ff), static_cast<double>(s16_torque_ff)}, "f s16 s16");
  }

  return send_can_command(motor_id, 0x0C, data);
}

bool CanDriver::send_velocity_command(uint8_t motor_id, double speed, double param, uint32_t mode)
{
  const double factor = 0.01;
  float f_speed = static_cast<float>(speed);
  std::vector<uint8_t> data;

  if (mode == 0) {
    // 直接控制模式
    int16_t s16_torque = static_cast<int16_t>(param / factor);
    if (f_speed == 0) s16_torque = 0;
    uint16_t u16_input_mode = 1;
    data = format_data({f_speed, static_cast<double>(s16_torque), static_cast<double>(u16_input_mode)}, "f s16 u16");
  }
  else {
    // 匀加速控制模式
    int16_t s16_ramp_rate = static_cast<int16_t>(param / factor);
    uint16_t u16_input_mode = 2;
    data = format_data({f_speed, static_cast<double>(s16_ramp_rate), static_cast<double>(u16_input_mode)}, "f s16 u16");
  }

  return send_can_command(motor_id, 0x1C, data);
}

bool CanDriver::preset_velocity_command(uint8_t motor_id, double speed, double param, uint32_t mode)
{
  // 预设速度命令与直接发送相同，只是不立即执行
  return send_velocity_command(motor_id, speed, param, mode);
}

bool CanDriver::send_effort_command(uint8_t motor_id, double torque, double param, uint32_t mode)
{
  const double factor = 0.01;
  float f_torque = static_cast<float>(torque);
  int16_t s16_ramp_rate = static_cast<int16_t>(param / factor);
  uint16_t u16_input_mode = static_cast<uint16_t>(mode);

  std::vector<uint8_t> data = format_data({f_torque, static_cast<double>(s16_ramp_rate), static_cast<double>(u16_input_mode)}, "f s16 u16");
  return send_can_command(motor_id, 0x1D, data);  // 假设0x1D是力矩控制命令
}

bool CanDriver::preset_effort_command(uint8_t motor_id, double torque, double param, uint32_t mode)
{
  return send_effort_command(motor_id, torque, param, mode);
}

bool CanDriver::send_start_command(uint32_t start_command)
{
  std::vector<uint8_t> data = format_data({static_cast<double>(start_command), 0}, "u32 u16");
  return send_can_command(0, 0x08, data);  // id=0表示广播
}

bool CanDriver::read_motor_state(uint8_t motor_id, MotorState& state)
{
  // 读取角度
  double angle;
  if (get_motor_angle(motor_id, angle)) {
    state.position = angle * M_PI / 180.0;  // 转换为弧度
  }

  // 读取速度
  double speed;
  if (get_motor_speed(motor_id, speed)) {
    state.velocity = speed * 2.0 * M_PI / 60.0;  // r/min转rad/s
  }

  // 读取力矩
  double torque;
  if (get_motor_torque(motor_id, torque)) {
    state.effort = torque;
  }

  return true;
}

bool CanDriver::get_motor_angle(uint8_t motor_id, double& angle)
{
  if (!send_can_command(motor_id, 0x21, std::vector<uint8_t>(8, 0), 1)) {  // rtr=1表示远程帧
    return false;
  }

  auto response = read_serial_data(16);
  if (response.size() == 16) {
    auto can_data = uart_to_can(response);
    if (can_data.size() >= 11) {
      auto parsed = parse_data(std::vector<uint8_t>(can_data.begin() + 3, can_data.begin() + 11), "f");
      if (!parsed.empty()) {
        angle = parsed[0];
        return true;
      }
    }
  }
  return false;
}

bool CanDriver::get_motor_speed(uint8_t motor_id, double& speed)
{
  if (!send_can_command(motor_id, 0x22, std::vector<uint8_t>(8, 0), 1)) {
    return false;
  }

  auto response = read_serial_data(16);
  if (response.size() == 16) {
    auto can_data = uart_to_can(response);
    if (can_data.size() >= 11) {
      auto parsed = parse_data(std::vector<uint8_t>(can_data.begin() + 3, can_data.begin() + 11), "f");
      if (!parsed.empty()) {
        speed = parsed[0];
        return true;
      }
    }
  }
  return false;
}

bool CanDriver::get_motor_torque(uint8_t motor_id, double& torque)
{
  if (!send_can_command(motor_id, 0x23, std::vector<uint8_t>(8, 0), 1)) {
    return false;
  }

  auto response = read_serial_data(16);
  if (response.size() == 16) {
    auto can_data = uart_to_can(response);
    if (can_data.size() >= 11) {
      auto parsed = parse_data(std::vector<uint8_t>(can_data.begin() + 3, can_data.begin() + 11), "f");
      if (!parsed.empty()) {
        torque = parsed[0];
        return true;
      }
    }
  }
  return false;
}

bool CanDriver::enable_state_feedback(uint8_t motor_id, uint16_t rate_ms)
{
  std::vector<uint8_t> data = format_data({static_cast<double>(rate_ms)}, "u16");
  return send_can_command(motor_id, 0x24, data);
}

bool CanDriver::disable_state_feedback(uint8_t motor_id)
{
  std::vector<uint8_t> data(8, 0);
  return send_can_command(motor_id, 0x25, data);
}

bool CanDriver::set_zero_position(uint8_t motor_id)
{
  std::vector<uint8_t> data(8, 0);
  return send_can_command(motor_id, 0x26, data);
}

bool CanDriver::emergency_stop(uint8_t motor_id)
{
  std::vector<uint8_t> data(8, 0);
  return send_can_command(motor_id, 0xFF, data);  // 0xFF作为急停命令
}

bool CanDriver::set_motor_id(uint8_t current_id, uint8_t new_id)
{
  std::vector<uint8_t> data = format_data({static_cast<double>(new_id)}, "u8");
  return send_can_command(current_id, 0x27, data);
}

bool CanDriver::is_connected() const
{
  return connected_ && serial_fd_ >= 0;
}

bool CanDriver::send_can_command(uint8_t motor_id, uint8_t cmd, const std::vector<uint8_t>& data, uint8_t rtr)
{
  std::lock_guard<std::mutex> lock(communication_mutex_);
  
  if (!is_connected()) {
    return false;
  }

  // 构造CAN数据包
  std::vector<uint8_t> can_data(11);
  can_data[0] = 0x08;  // DLC
  
  uint16_t can_id = calculate_can_id(motor_id, cmd);
  can_data[1] = (can_id >> 8) & 0xFF;
  can_data[2] = can_id & 0xFF;
  
  // 复制数据，确保8字节
  for (size_t i = 0; i < 8; ++i) {
    can_data[3 + i] = (i < data.size()) ? data[i] : 0x00;
  }

  // 转换为串行帧
  auto uart_data = can_to_uart(can_data, rtr);
  
  // 发送数据
  return write_serial_data(uart_data);
}

std::vector<uint8_t> CanDriver::can_to_uart(const std::vector<uint8_t>& can_data, uint8_t rtr)
{
  std::vector<uint8_t> uart_data(16);
  uart_data[0] = 0xAA;  // 帧头
  uart_data[1] = 0x00;  // IDE
  uart_data[2] = rtr;   // RTR
  uart_data[3] = 0x08;  // DLC
  uart_data[4] = 0x00;  // Reserved
  uart_data[5] = 0x00;  // Reserved

  if (can_data.size() >= 11) {
    for (size_t i = 0; i < 10; ++i) {
      uart_data[6 + i] = can_data[1 + i];
    }
  }

  return uart_data;
}

std::vector<uint8_t> CanDriver::uart_to_can(const std::vector<uint8_t>& uart_data)
{
  std::vector<uint8_t> can_data(11);
  can_data[0] = 0x08;

  if (uart_data.size() == 16 && uart_data[3] == 0x08) {
    for (size_t i = 0; i < 10; ++i) {
      can_data[1 + i] = uart_data[6 + i];
    }
    read_flag_ = 1;
  } else {
    read_flag_ = -1;
  }

  return can_data;
}

std::vector<uint8_t> CanDriver::format_data(const std::vector<double>& data, const std::string& format, bool /* encode */)
{
  std::vector<uint8_t> result;
  std::istringstream iss(format);
  std::string token;
  size_t data_index = 0;

  while (std::getline(iss, token, ' ') && data_index < data.size()) {
    if (token == "f") {
      // float (4 bytes)
      float value = static_cast<float>(data[data_index]);
      uint8_t* bytes = reinterpret_cast<uint8_t*>(&value);
      for (int i = 0; i < 4; ++i) {
        result.push_back(bytes[i]);
      }
    }
    else if (token == "s16") {
      // signed 16-bit (2 bytes)
      int16_t value = static_cast<int16_t>(data[data_index]);
      result.push_back(value & 0xFF);
      result.push_back((value >> 8) & 0xFF);
    }
    else if (token == "u16") {
      // unsigned 16-bit (2 bytes)
      uint16_t value = static_cast<uint16_t>(data[data_index]);
      result.push_back(value & 0xFF);
      result.push_back((value >> 8) & 0xFF);
    }
    else if (token == "s32") {
      // signed 32-bit (4 bytes)
      int32_t value = static_cast<int32_t>(data[data_index]);
      result.push_back(value & 0xFF);
      result.push_back((value >> 8) & 0xFF);
      result.push_back((value >> 16) & 0xFF);
      result.push_back((value >> 24) & 0xFF);
    }
    else if (token == "u32") {
      // unsigned 32-bit (4 bytes)
      uint32_t value = static_cast<uint32_t>(data[data_index]);
      result.push_back(value & 0xFF);
      result.push_back((value >> 8) & 0xFF);
      result.push_back((value >> 16) & 0xFF);
      result.push_back((value >> 24) & 0xFF);
    }
    else if (token == "u8") {
      // unsigned 8-bit (1 byte)
      uint8_t value = static_cast<uint8_t>(data[data_index]);
      result.push_back(value);
    }
    
    data_index++;
  }

  // 填充到8字节
  while (result.size() < 8) {
    result.push_back(0x00);
  }

  return result;
}

std::vector<double> CanDriver::parse_data(const std::vector<uint8_t>& data, const std::string& format)
{
  std::vector<double> result;
  std::istringstream iss(format);
  std::string token;
  size_t byte_index = 0;

  while (std::getline(iss, token, ' ') && byte_index < data.size()) {
    if (token == "f" && byte_index + 4 <= data.size()) {
      // float (4 bytes)
      float value;
      std::memcpy(&value, &data[byte_index], 4);
      result.push_back(static_cast<double>(value));
      byte_index += 4;
    }
    else if (token == "s16" && byte_index + 2 <= data.size()) {
      // signed 16-bit (2 bytes)
      int16_t value = data[byte_index] | (data[byte_index + 1] << 8);
      result.push_back(static_cast<double>(value));
      byte_index += 2;
    }
    else if (token == "u16" && byte_index + 2 <= data.size()) {
      // unsigned 16-bit (2 bytes)
      uint16_t value = data[byte_index] | (data[byte_index + 1] << 8);
      result.push_back(static_cast<double>(value));
      byte_index += 2;
    }
    // 可以添加更多格式解析
  }

  return result;
}

bool CanDriver::write_serial_data(const std::vector<uint8_t>& data)
{
  if (!is_connected()) {
    return false;
  }

  ssize_t bytes_written = write(serial_fd_, data.data(), data.size());
  return bytes_written == static_cast<ssize_t>(data.size());
}

std::vector<uint8_t> CanDriver::read_serial_data(size_t expected_length)
{
  std::vector<uint8_t> result;
  
  if (!is_connected()) {
    return result;
  }

  // 等待数据
  if (!wait_for_data(100)) {  // 100ms超时
    return result;
  }

  result.resize(expected_length);
  ssize_t bytes_read = read(serial_fd_, result.data(), expected_length);
  
  if (bytes_read > 0) {
    result.resize(bytes_read);
  } else {
    result.clear();
  }

  return result;
}

bool CanDriver::wait_for_data(int timeout_ms)
{
  if (!is_connected()) {
    return false;
  }

  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(serial_fd_, &read_fds);

  struct timeval timeout;
  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = (timeout_ms % 1000) * 1000;

  int result = select(serial_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
  return result > 0 && FD_ISSET(serial_fd_, &read_fds);
}

uint16_t CanDriver::calculate_can_id(uint8_t motor_id, uint8_t cmd) const
{
  return (static_cast<uint16_t>(motor_id) << 5) + cmd;
}

uint32_t CanDriver::get_start_command_for_mode(uint32_t mode) const
{
  switch (mode) {
    case 0: return 0x10;  // 轨迹跟踪
    case 1: return 0x11;  // 梯形轨迹
    case 2: return 0x12;  // 前馈控制
    default: return 0x11;
  }
}

}  // namespace drpower_hardware_interface
