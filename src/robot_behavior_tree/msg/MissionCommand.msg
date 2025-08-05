# MissionCommand.msg - 任务命令消息

# 时间戳
builtin_interfaces/Time stamp

# 命令类型
string command_type                 # "start", "stop", "pause", "resume", "abort"

# 任务参数
string mission_name                 # 任务名称
string behavior_tree_file          # 行为树文件路径（可选）
string target_class                # 目标类别（可选）

# 位置参数
geometry_msgs/Point[] waypoints    # 路径点列表（可选）
geometry_msgs/Point target_position # 目标位置（可选）

# 控制参数
float32 max_speed                  # 最大速度（可选）
float32 timeout                    # 超时时间（可选）
bool enable_safety                 # 是否启用安全模式

# 自定义参数
string[] parameter_keys            # 参数键列表
string[] parameter_values          # 参数值列表
