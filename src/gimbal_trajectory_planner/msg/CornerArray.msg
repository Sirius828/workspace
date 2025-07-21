# CornerArray.msg
# 角点数组消息，用于批量传输角点数据

std_msgs/Header header
geometry_msgs/Point32[] corners
string trajectory_type  # "open_path", "closed_loop", "point_to_point"
