# DetectionBox.msg
# 检测框消息类型，包含矩形框的四个角点坐标

# 矩形框左上角坐标
float32 x1
float32 y1

# 矩形框右下角坐标  
float32 x2
float32 y2

# 检测置信度
float32 confidence

# 检测类别ID
int32 class_id

# 检测类别名称
string class_name

# 是否检测到目标（如果为false，其他字段无效）
bool detected
