#include "drpower_hardware_interface/drpower_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace drpower_hardware_interface
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("DrPowerHardwareInterface");

hardware_interface::CallbackReturn DrPowerHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(LOGGER, "Initializing DrPower Hardware Interface...");

  // 初始化基本参数
  device_path_ = info_.hardware_parameters.at("device_path");
  baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));
  control_frequency_ = std::stod(info_.hardware_parameters.count("control_frequency") ? 
                                info_.hardware_parameters.at("control_frequency") : "100.0");
  enable_state_feedback_ = info_.hardware_parameters.count("enable_state_feedback") ? 
                          (info_.hardware_parameters.at("enable_state_feedback") == "true") : true;
  state_feedback_rate_ms_ = std::stoi(info_.hardware_parameters.count("state_feedback_rate_ms") ? 
                                     info_.hardware_parameters.at("state_feedback_rate_ms") : "10");

  RCLCPP_INFO(LOGGER, "Device path: %s", device_path_.c_str());
  RCLCPP_INFO(LOGGER, "Baudrate: %d", baudrate_);
  RCLCPP_INFO(LOGGER, "Control frequency: %.1f Hz", control_frequency_);

  // 加载关节配置
  if (!load_joint_configs(info))
  {
    RCLCPP_ERROR(LOGGER, "Failed to load joint configurations");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 加载电机组配置
  if (!load_motor_group_configs(info))
  {
    RCLCPP_ERROR(LOGGER, "Failed to load motor group configurations");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 初始化状态和命令向量
  const size_t num_joints = info_.joints.size();
  hw_position_commands_.resize(num_joints, 0.0);
  hw_velocity_commands_.resize(num_joints, 0.0);
  hw_effort_commands_.resize(num_joints, 0.0);
  hw_position_states_.resize(num_joints, 0.0);
  hw_velocity_states_.resize(num_joints, 0.0);
  hw_effort_states_.resize(num_joints, 0.0);
  
  position_commands_updated_.resize(num_joints, false);
  velocity_commands_updated_.resize(num_joints, false);
  effort_commands_updated_.resize(num_joints, false);

  // 初始化电机状态
  motor_states_.resize(num_joints);
  for (size_t i = 0; i < num_joints; ++i)
  {
    uint8_t motor_id = std::stoi(info_.joints[i].parameters.at("id"));
    motor_states_[i] = MotorState(motor_id);
    joint_name_to_index_[info_.joints[i].name] = i;
  }

  // 创建CAN驱动
  can_driver_ = std::make_unique<CanDriver>();

  RCLCPP_INFO(LOGGER, "DrPower Hardware Interface initialized successfully with %zu joints", num_joints);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DrPowerHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Configuring DrPower Hardware Interface...");

  // 初始化CAN驱动
  if (!can_driver_->initialize(device_path_, baudrate_))
  {
    RCLCPP_ERROR(LOGGER, "Failed to initialize CAN driver");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(LOGGER, "CAN driver initialized successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DrPowerHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Activating DrPower Hardware Interface...");

  // 检查连接状态
  if (!can_driver_->is_connected())
  {
    RCLCPP_ERROR(LOGGER, "CAN driver is not connected");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 初始化状态反馈
  if (enable_state_feedback_)
  {
    if (!initialize_state_feedback())
    {
      RCLCPP_WARN(LOGGER, "Failed to initialize state feedback, continuing without it");
    }
  }

  // 读取初始状态
  if (!read_all_motor_states())
  {
    RCLCPP_WARN(LOGGER, "Failed to read initial motor states");
  }

  // 设置初始命令为当前位置
  for (size_t i = 0; i < motor_states_.size(); ++i)
  {
    hw_position_commands_[i] = hw_position_states_[i];
    hw_velocity_commands_[i] = 0.0;
    hw_effort_commands_[i] = 0.0;
  }

  RCLCPP_INFO(LOGGER, "DrPower Hardware Interface activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DrPowerHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Deactivating DrPower Hardware Interface...");

  // 停止所有电机
  emergency_stop();

  // 关闭状态反馈
  if (enable_state_feedback_)
  {
    for (const auto& state : motor_states_)
    {
      can_driver_->disable_state_feedback(state.id);
    }
  }

  RCLCPP_INFO(LOGGER, "DrPower Hardware Interface deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DrPowerHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DrPowerHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type DrPowerHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 读取所有电机状态
  if (!read_all_motor_states())
  {
    auto clock = rclcpp::Clock();
    RCLCPP_WARN_THROTTLE(LOGGER, clock, 1000, "Failed to read motor states");
    return hardware_interface::return_type::ERROR;
  }

  // 更新硬件接口状态
  for (size_t i = 0; i < motor_states_.size(); ++i)
  {
    hw_position_states_[i] = motor_states_[i].position;
    hw_velocity_states_[i] = motor_states_[i].velocity;
    hw_effort_states_[i] = motor_states_[i].effort;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DrPowerHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 检查命令更新
  for (size_t i = 0; i < hw_position_commands_.size(); ++i)
  {
    if (std::abs(hw_position_commands_[i] - motor_states_[i].position_command) > 1e-6)
    {
      position_commands_updated_[i] = true;
      motor_states_[i].position_command = hw_position_commands_[i];
    }
    
    if (std::abs(hw_velocity_commands_[i] - motor_states_[i].velocity_command) > 1e-6)
    {
      velocity_commands_updated_[i] = true;
      motor_states_[i].velocity_command = hw_velocity_commands_[i];
    }
    
    if (std::abs(hw_effort_commands_[i] - motor_states_[i].effort_command) > 1e-6)
    {
      effort_commands_updated_[i] = true;
      motor_states_[i].effort_command = hw_effort_commands_[i];
    }
  }

  // 处理电机组同步控制
  for (size_t group_idx = 0; group_idx < motor_groups_.size(); ++group_idx)
  {
    const auto& group = motor_groups_[group_idx];
    
    if (group.synchronized)
    {
      // 检查组内是否有更新
      bool group_needs_update = false;
      for (const auto& joint_name : group.joint_names)
      {
        size_t joint_idx = joint_name_to_index_.at(joint_name);
        if (position_commands_updated_[joint_idx] || 
            velocity_commands_updated_[joint_idx] ||
            effort_commands_updated_[joint_idx])
        {
          group_needs_update = true;
          break;
        }
      }

      if (group_needs_update)
      {
        // 根据主要控制接口选择控制方式
        bool has_position_cmd = false, has_velocity_cmd = false, has_effort_cmd = false;
        
        for (const auto& joint_name : group.joint_names)
        {
          size_t joint_idx = joint_name_to_index_.at(joint_name);
          if (position_commands_updated_[joint_idx]) has_position_cmd = true;
          if (velocity_commands_updated_[joint_idx]) has_velocity_cmd = true;
          if (effort_commands_updated_[joint_idx]) has_effort_cmd = true;
        }

        // 优先级：位置 > 速度 > 力矩
        if (has_position_cmd)
        {
          set_group_positions(group_idx);
        }
        else if (has_velocity_cmd)
        {
          set_group_velocities(group_idx);
        }
        else if (has_effort_cmd)
        {
          set_group_efforts(group_idx);
        }
      }
    }
    else
    {
      // 独立控制每个电机
      for (const auto& joint_name : group.joint_names)
      {
        size_t joint_idx = joint_name_to_index_.at(joint_name);
        
        if (position_commands_updated_[joint_idx])
        {
          set_individual_position(joint_idx);
        }
        else if (velocity_commands_updated_[joint_idx])
        {
          set_individual_velocity(joint_idx);
        }
        else if (effort_commands_updated_[joint_idx])
        {
          set_individual_effort(joint_idx);
        }
      }
    }
  }

  // 清除更新标志
  std::fill(position_commands_updated_.begin(), position_commands_updated_.end(), false);
  std::fill(velocity_commands_updated_.begin(), velocity_commands_updated_.end(), false);
  std::fill(effort_commands_updated_.begin(), effort_commands_updated_.end(), false);

  return hardware_interface::return_type::OK;
}

bool DrPowerHardwareInterface::load_joint_configs(const hardware_interface::HardwareInfo & info)
{
  for (size_t i = 0; i < info.joints.size(); ++i)
  {
    const auto& joint = info.joints[i];
    
    // 检查必需参数
    if (joint.parameters.find("id") == joint.parameters.end())
    {
      RCLCPP_ERROR(LOGGER, "Joint %s missing required parameter 'id'", joint.name.c_str());
      return false;
    }

    uint8_t motor_id = std::stoi(joint.parameters.at("id"));
    if (motor_id < 1 || motor_id > 64)
    {
      RCLCPP_ERROR(LOGGER, "Joint %s has invalid motor ID: %d (must be 1-64)", 
                  joint.name.c_str(), motor_id);
      return false;
    }

    RCLCPP_INFO(LOGGER, "Loaded joint %s with motor ID %d", joint.name.c_str(), motor_id);
  }

  return true;
}

bool DrPowerHardwareInterface::load_motor_group_configs(const hardware_interface::HardwareInfo & info)
{
  // 默认创建一个包含所有关节的组
  MotorGroup default_group;
  default_group.name = "default";
  default_group.synchronized = true;
  default_group.config.position_control.mode = 1;  // 梯形轨迹模式

  for (const auto& joint : info.joints)
  {
    uint8_t motor_id = std::stoi(joint.parameters.at("id"));
    default_group.motor_ids.push_back(motor_id);
    default_group.joint_names.push_back(joint.name);
    
    // 加载单独的控制配置
    if (joint.parameters.count("position_mode"))
    {
      default_group.config.position_control.mode = std::stoi(joint.parameters.at("position_mode"));
    }
    if (joint.parameters.count("position_acceleration"))
    {
      default_group.config.position_control.acceleration = std::stod(joint.parameters.at("position_acceleration"));
    }
    if (joint.parameters.count("velocity_mode"))
    {
      default_group.config.velocity_control.mode = std::stoi(joint.parameters.at("velocity_mode"));
    }
  }

  motor_groups_.push_back(default_group);
  group_name_to_index_[default_group.name] = 0;

  RCLCPP_INFO(LOGGER, "Created default motor group with %zu joints", default_group.joint_names.size());
  return true;
}

bool DrPowerHardwareInterface::set_group_positions(size_t group_index)
{
  if (group_index >= motor_groups_.size()) return false;

  const auto& group = motor_groups_[group_index];
  const auto& config = group.config.position_control;

  // 第一阶段：预设所有电机位置
  for (size_t i = 0; i < group.motor_ids.size(); ++i)
  {
    uint8_t motor_id = group.motor_ids[i];
    const std::string& joint_name = group.joint_names[i];
    size_t joint_idx = joint_name_to_index_.at(joint_name);
    
    double position_deg = radians_to_degrees(hw_position_commands_[joint_idx]);
    double velocity = config.feedforward_velocity;
    double param = config.acceleration;
    
    if (!can_driver_->preset_position_command(motor_id, position_deg, velocity, param, config.mode))
    {
      RCLCPP_ERROR(LOGGER, "Failed to preset position for motor %d", motor_id);
      return false;
    }
  }

  // 第二阶段：发送同步启动命令
  uint32_t start_command = (config.mode == 0) ? 0x10 : 
                          (config.mode == 1) ? 0x11 : 0x12;
  
  if (!can_driver_->send_start_command(start_command))
  {
    RCLCPP_ERROR(LOGGER, "Failed to send start command for position control");
    return false;
  }

  return true;
}

bool DrPowerHardwareInterface::set_individual_position(size_t joint_index)
{
  if (joint_index >= motor_states_.size()) return false;

  const auto& motor_state = motor_states_[joint_index];
  double position_deg = radians_to_degrees(hw_position_commands_[joint_index]);
  
  // 使用直接位置控制命令
  return can_driver_->send_position_command(motor_state.id, position_deg, 0, 10.0, 1);
}

bool DrPowerHardwareInterface::set_group_velocities(size_t group_index)
{
  if (group_index >= motor_groups_.size()) return false;

  const auto& group = motor_groups_[group_index];
  const auto& config = group.config.velocity_control;

  // 预设所有电机速度
  for (size_t i = 0; i < group.motor_ids.size(); ++i)
  {
    uint8_t motor_id = group.motor_ids[i];
    const std::string& joint_name = group.joint_names[i];
    size_t joint_idx = joint_name_to_index_.at(joint_name);
    
    double velocity_rpm = rad_per_sec_to_rpm(hw_velocity_commands_[joint_idx]);
    
    if (!can_driver_->preset_velocity_command(motor_id, velocity_rpm, config.acceleration, config.mode))
    {
      RCLCPP_ERROR(LOGGER, "Failed to preset velocity for motor %d", motor_id);
      return false;
    }
  }

  // 发送速度控制启动命令
  return can_driver_->send_start_command(0x13);
}

bool DrPowerHardwareInterface::set_individual_velocity(size_t joint_index)
{
  if (joint_index >= motor_states_.size()) return false;

  const auto& motor_state = motor_states_[joint_index];
  double velocity_rpm = rad_per_sec_to_rpm(hw_velocity_commands_[joint_index]);
  
  return can_driver_->send_velocity_command(motor_state.id, velocity_rpm, 5.0, 1);
}

bool DrPowerHardwareInterface::set_group_efforts(size_t group_index)
{
  if (group_index >= motor_groups_.size()) return false;

  const auto& group = motor_groups_[group_index];

  // 预设所有电机力矩
  for (size_t i = 0; i < group.motor_ids.size(); ++i)
  {
    uint8_t motor_id = group.motor_ids[i];
    const std::string& joint_name = group.joint_names[i];
    size_t joint_idx = joint_name_to_index_.at(joint_name);
    
    if (!can_driver_->preset_effort_command(motor_id, hw_effort_commands_[joint_idx], 1.0, 1))
    {
      RCLCPP_ERROR(LOGGER, "Failed to preset effort for motor %d", motor_id);
      return false;
    }
  }

  // 发送力矩控制启动命令（自定义）
  return can_driver_->send_start_command(0x14);
}

bool DrPowerHardwareInterface::set_individual_effort(size_t joint_index)
{
  if (joint_index >= motor_states_.size()) return false;

  const auto& motor_state = motor_states_[joint_index];
  return can_driver_->send_effort_command(motor_state.id, hw_effort_commands_[joint_index], 1.0, 1);
}

bool DrPowerHardwareInterface::read_all_motor_states()
{
  bool success = true;
  
  for (auto& state : motor_states_)
  {
    if (!can_driver_->read_motor_state(state.id, state))
    {
      success = false;
      state.online = false;
    }
    else
    {
      state.online = true;
      state.update_timestamp();
    }
  }
  
  return success;
}

bool DrPowerHardwareInterface::initialize_state_feedback()
{
  bool success = true;
  
  for (const auto& state : motor_states_)
  {
    if (!can_driver_->enable_state_feedback(state.id, state_feedback_rate_ms_))
    {
      RCLCPP_WARN(LOGGER, "Failed to enable state feedback for motor %d", state.id);
      success = false;
    }
  }
  
  return success;
}

bool DrPowerHardwareInterface::emergency_stop()
{
  RCLCPP_WARN(LOGGER, "Emergency stop triggered!");
  return can_driver_->emergency_stop(0);  // 0 = 急停所有电机
}

double DrPowerHardwareInterface::degrees_to_radians(double degrees) const
{
  return degrees * M_PI / 180.0;
}

double DrPowerHardwareInterface::radians_to_degrees(double radians) const
{
  return radians * 180.0 / M_PI;
}

double DrPowerHardwareInterface::rpm_to_rad_per_sec(double rpm) const
{
  return rpm * 2.0 * M_PI / 60.0;
}

double DrPowerHardwareInterface::rad_per_sec_to_rpm(double rad_per_sec) const
{
  return rad_per_sec * 60.0 / (2.0 * M_PI);
}

}  // namespace drpower_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  drpower_hardware_interface::DrPowerHardwareInterface,
  hardware_interface::SystemInterface
)
