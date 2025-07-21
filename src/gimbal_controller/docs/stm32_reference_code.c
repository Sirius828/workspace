/*
STM32端云台控制参考代码
配合ROS2改进的云台控制节点使用

数据通信格式：
ROS2 -> STM32: "chassis_vx,chassis_vy,chassis_omega,gimbal_yaw_target,gimbal_pitch_target\n"
STM32 -> ROS2: "timestamp,odom_x,odom_y,odom_yaw,odom_vx,odom_vy,odom_omega,gimbal_yaw_current,gimbal_pitch_current\n"
*/

#include "main.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

// 云台控制结构体
typedef struct {
    float target_yaw;      // 目标偏航角(度)
    float target_pitch;    // 目标俯仰角(度)
    float current_yaw;     // 当前偏航角(度)
    float current_pitch;   // 当前俯仰角(度)
    
    // PID控制器状态
    float yaw_error_last;
    float yaw_integral;
    float pitch_error_last;
    float pitch_integral;
    
    // PID参数
    float kp_yaw, ki_yaw, kd_yaw;
    float kp_pitch, ki_pitch, kd_pitch;
    
    uint32_t last_update_time;
} GimbalController_t;

// 底盘控制结构体
typedef struct {
    float vx, vy, omega;
} ChassisCmd_t;

// 里程计数据结构体
typedef struct {
    uint32_t timestamp_ms;
    float position_x, position_y, orientation_yaw;
    float velocity_x, velocity_y, angular_velocity;
} Odometry_t;

// 全局变量
GimbalController_t gimbal = {0};
ChassisCmd_t chassis_cmd = {0};
Odometry_t odom = {0};

// 初始化云台控制器
void Gimbal_Init(void) {
    // 设置PID参数（这些参数需要根据实际云台调整）
    gimbal.kp_yaw = 2.0f;
    gimbal.ki_yaw = 0.1f;
    gimbal.kd_yaw = 0.5f;
    
    gimbal.kp_pitch = 2.0f;
    gimbal.ki_pitch = 0.1f;
    gimbal.kd_pitch = 0.5f;
    
    // 初始化角度
    gimbal.target_yaw = 0.0f;
    gimbal.target_pitch = 0.0f;
    gimbal.current_yaw = 0.0f;    // 从编码器读取
    gimbal.current_pitch = 0.0f;  // 从编码器读取
    
    gimbal.last_update_time = HAL_GetTick();
}

// 解析ROS2发送的控制命令
void Parse_Control_Command(char* cmd_str) {
    // 解析格式: "vx,vy,omega,yaw_target,pitch_target"
    char* token;
    int param_index = 0;
    
    token = strtok(cmd_str, ",");
    while (token != NULL && param_index < 5) {
        switch (param_index) {
            case 0: chassis_cmd.vx = atof(token); break;
            case 1: chassis_cmd.vy = atof(token); break;
            case 2: chassis_cmd.omega = atof(token); break;
            case 3: gimbal.target_yaw = atof(token); break;
            case 4: gimbal.target_pitch = atof(token); break;
        }
        token = strtok(NULL, ",");
        param_index++;
    }
}

// PID控制函数
float PID_Control(float error, float* error_last, float* integral, 
                  float kp, float ki, float kd, float dt) {
    
    // 积分项
    *integral += error * dt;
    
    // 积分限幅，防止积分饱和
    if (*integral > 100.0f) *integral = 100.0f;
    if (*integral < -100.0f) *integral = -100.0f;
    
    // 微分项
    float derivative = (error - *error_last) / dt;
    
    // PID输出
    float output = kp * error + ki * (*integral) + kd * derivative;
    
    // 更新上次误差
    *error_last = error;
    
    return output;
}

// 云台控制主函数
void Gimbal_Control_Update(void) {
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - gimbal.last_update_time) / 1000.0f;
    
    if (dt > 0.001f) {  // 最小控制周期1ms
        
        // 读取当前云台角度（从编码器或其他传感器）
        gimbal.current_yaw = Get_Gimbal_Yaw_Angle();      // 需要实现
        gimbal.current_pitch = Get_Gimbal_Pitch_Angle();  // 需要实现
        
        // 计算误差
        float yaw_error = gimbal.target_yaw - gimbal.current_yaw;
        float pitch_error = gimbal.target_pitch - gimbal.current_pitch;
        
        // 角度误差归一化到[-180, 180]
        while (yaw_error > 180.0f) yaw_error -= 360.0f;
        while (yaw_error < -180.0f) yaw_error += 360.0f;
        
        // PID控制计算
        float yaw_output = PID_Control(yaw_error, &gimbal.yaw_error_last, 
                                       &gimbal.yaw_integral, 
                                       gimbal.kp_yaw, gimbal.ki_yaw, gimbal.kd_yaw, dt);
        
        float pitch_output = PID_Control(pitch_error, &gimbal.pitch_error_last, 
                                         &gimbal.pitch_integral, 
                                         gimbal.kp_pitch, gimbal.ki_pitch, gimbal.kd_pitch, dt);
        
        // 输出限幅
        if (yaw_output > 100.0f) yaw_output = 100.0f;
        if (yaw_output < -100.0f) yaw_output = -100.0f;
        if (pitch_output > 100.0f) pitch_output = 100.0f;
        if (pitch_output < -100.0f) pitch_output = -100.0f;
        
        // 发送到电机控制器
        Set_Gimbal_Yaw_Output(yaw_output);     // 需要实现
        Set_Gimbal_Pitch_Output(pitch_output); // 需要实现
        
        gimbal.last_update_time = current_time;
    }
}

// 发送反馈数据到ROS2
void Send_Feedback_Data(void) {
    // 更新里程计数据（这部分需要根据实际传感器实现）
    Update_Odometry(&odom);  // 需要实现
    
    // 发送格式: "timestamp,x,y,yaw,vx,vy,omega,gimbal_yaw,gimbal_pitch"
    printf("%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f\n",
           odom.timestamp_ms,
           odom.position_x,
           odom.position_y, 
           odom.orientation_yaw,
           odom.velocity_x,
           odom.velocity_y,
           odom.angular_velocity,
           gimbal.current_yaw,
           gimbal.current_pitch);
}

// 主循环中调用的函数
void Control_Loop(void) {
    static uint32_t last_feedback_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 云台控制更新（高频率）
    Gimbal_Control_Update();
    
    // 底盘控制更新
    Update_Chassis_Control(&chassis_cmd);  // 需要实现
    
    // 定期发送反馈数据（例如20Hz）
    if (current_time - last_feedback_time > 50) {
        Send_Feedback_Data();
        last_feedback_time = current_time;
    }
}

/*
需要实现的函数：
1. float Get_Gimbal_Yaw_Angle(void) - 读取当前yaw角度
2. float Get_Gimbal_Pitch_Angle(void) - 读取当前pitch角度  
3. void Set_Gimbal_Yaw_Output(float output) - 设置yaw电机输出
4. void Set_Gimbal_Pitch_Output(float output) - 设置pitch电机输出
5. void Update_Odometry(Odometry_t* odom) - 更新里程计数据
6. void Update_Chassis_Control(ChassisCmd_t* cmd) - 更新底盘控制
*/
