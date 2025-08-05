# BehaviorTreeStatus.msg - 行为树状态消息

# 时间戳
builtin_interfaces/Time stamp

# 行为树状态
string tree_name                    # 当前执行的行为树名称
string current_node                 # 当前执行的节点名称
string status                       # 状态: "RUNNING", "SUCCESS", "FAILURE", "IDLE"
float32 execution_time             # 执行时间（秒）
int32 tick_count                   # tick计数

# 黑板变量
string[] blackboard_keys           # 黑板键列表
string[] blackboard_values         # 黑板值列表

# 性能信息
float32 tick_frequency             # 实际tick频率
float32 average_tick_duration      # 平均tick持续时间（毫秒）

# 节点统计
int32 total_nodes                  # 总节点数
int32 active_nodes                 # 活跃节点数
string[] failed_nodes              # 失败的节点列表
