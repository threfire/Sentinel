/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      四轴云台任务与控制对象
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include <stdint.h>
#include <stdbool.h>

#include "robot_param.h"
#include "remote_control.h"
#include "pid.h"
#include "gravity_comp.h"

typedef enum
{
    GIMBAL_MOTOR_RAW = 0,
    GIMBAL_MOTOR_GYRO,
    GIMBAL_MOTOR_ENCODE,
    GIMBAL_MOTOR_MIT_POSITION,       // 电机内部位置控制
} gimbal_motor_mode_e;

/** @brief 目标前馈的来源，分别保存遥控与自瞄的轨迹历史。 */
typedef enum
{
    GIMBAL_CONTROL_SOURCE_RC = 0,
    GIMBAL_CONTROL_SOURCE_AUTO,
} gimbal_control_source_e;

/** @brief 云台轴编号，用于选择反馈来源和控制算法。 */
typedef enum
{
    GIMBAL_AXIS_YAW = 0,
    GIMBAL_AXIS_SMALL_YAW,
    GIMBAL_AXIS_PITCH,
    GIMBAL_AXIS_ROLL,
} gimbal_axis_e;

/** @brief MIT 位置控制的本周期命令。 */
typedef struct
{
    float position_rad;             // 位置目标，rad
    float speed_rad_s;              // 速度目标，rad/s
    float kp;                       // 电机内部位置增益
    float kd;                       // 电机内部速度增益
    float torque_nm;                // 前馈力矩，N*m
} gimbal_position_cmd_t;

typedef pid_type_def gimbal_pid_t;

typedef struct
{
    gimbal_axis_e axis;              // 轴编号
    gimbal_motor_mode_e mode;        // 当前电机控制模式
    gimbal_motor_mode_e last_mode;   // 上一次电机控制模式，用于模式切换过渡
    uint8_t control_allowed;         // 本周期执行许可
    uint8_t feedback_ready;          // 设备反馈有效标志

    float relative_angle;            // 相对角度反馈值
    float relative_angle_unwrapped;  // 大 yaw 连续累计相对角度，单位 rad
    float relative_angle_feedback_last; // 大 yaw 上一帧包角反馈，单位 rad
    uint8_t relative_angle_unwrap_init; // 大 yaw 连续角累计初始化标志
    float relative_angle_set;        // 相对角度目标值
    float relative_angle_last;       // 上一次相对角度反馈值
    float relative_speed;            // 相对角编码器差分速度
    uint8_t relative_speed_update_init; // 相对角速度差分初始化标志

    float absolute_angle;            // 绝对角度反馈值
    float absolute_angle_set;        // 绝对角度目标值

    float raw_absolute_angle;        // 原始电机绝对角度反馈值
    float raw_relative_angle;        // 原始电机相对角度反馈值
    float raw_relative_speed;        // 原始电机相对角速度反馈值

    float angle_offset;              // 上电/初始化时记录的角度零偏
    uint8_t angle_offset_init;       // 角度零偏是否已初始化

    float gyro;                      // 角速度反馈值
    float gyro_set;                  // 角速度目标值
    float gyro_last;                 // 上一次角速度反馈值
    float gyro_accel;                // 角加速度反馈值，由角速度差分得到
    uint8_t gyro_update_init;        // 角速度差分初始化标志

    float ref_vel;                   // 前馈轨迹参考速度
    float ref_vel_last;              // 上一次前馈轨迹参考速度
    float ref_accel;                 // 前馈轨迹参考加速度
    float rc_ref_vel;
    float rc_ref_vel_last;
    float rc_ref_accel;
    float rc_ref_target_last;
    uint8_t rc_ref_target_init;
    float auto_ref_vel;
    float auto_ref_vel_last;
    float auto_ref_accel;
    float auto_ref_target_last;
    uint8_t auto_ref_target_init;
    float pid_torque;                // PID 反馈输出力矩
    float ff_torque;                 // 惯量前馈输出力矩
    float static_friction_comp;      // 静摩擦补偿输出力矩
    float inertia_kgm2;              // 转动惯量参数 J，单位 kg*m^2

    float raw_cmd;                   // RAW 模式下直接输出命令
    float output;                    // 控制器最终输出值
    float current_set;               // 目标电流/目标力矩命令
    float given_current;             // 实际发送给电机的电流/力矩命令

    float max_relative_angle;        // 相对角度上限
    float min_relative_angle;        // 相对角度下限

    float rc_relative_angle_set;
    float rc_absolute_angle_set;
    float auto_relative_angle_set;
    float auto_absolute_angle_set;
    uint8_t rc_control_enable;
    uint8_t auto_control_enable;
    gimbal_pid_t absolute_angle_pid;
    gimbal_pid_t relative_angle_pid;
    gimbal_pid_t speed_pid;          // 小 yaw 速度内环，输出 A
    float auto_target_vel;           // 自瞄目标规划速度，rad/s
    uint16_t last_ecd;               // 小 yaw 上一帧编码器值
    uint32_t move_last_tick;         // roll 位置规划上次时间，ms
    uint8_t move_target_init;        // roll 位置规划初始化标志
    gimbal_position_cmd_t position_cmd; // roll 最终位置命令
} gimbal_motor_t;

/** @brief 小 yaw 轴控制与 CAN 发送观测。 */
typedef struct
{
    gimbal_motor_t motor;              // 与其他轴相同的控制对象
    int16_t current_cmd_raw;           // 小 yaw 电流命令原始值
    uint8_t can_send_status;           // 小 yaw CAN 发送状态
} gimbal_small_yaw_t;

/** @brief 云台目标生成与跨模式过渡状态。 */
typedef struct
{
    float add_yaw;                     // 行为层给出的大 yaw 增量，rad
    float add_pitch;                   // 行为层给出的 pitch 增量，rad
    uint8_t follow_origin_init;        // 跟随模式航向基准已初始化
    uint8_t last_behaviour;            // 上次目标更新时的云台行为
    uint8_t pitch_lift_mid_pending;    // 抬升后等待设置 pitch 中位
    uint8_t scan_active;               // 正弦扫描已初始化
    uint32_t scan_start_tick;          // 扫描起始时间，ms
    float small_yaw_start_rad;         // 扫描切入时小 yaw 角，rad
    float pitch_start_rad;             // 扫描切入时 pitch 角，rad
} gimbal_target_state_t;

typedef struct
{
    float max_yaw;
    float min_yaw;
    float max_pitch;
    float min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;

typedef struct
{
    uint8_t total_mode;
    bool roll_lifted;
    bool roll_target_lifted;
    bool roll_motion_active;
    bool roll_state_initialized;
    bool roll_control_allowed;
    bool roll_safe_yaw_pending;
    bool friction_enabled;
    bool strum_enabled;
} gimbal_behaviour_state_t;

typedef struct gimbal_control_t
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const float *gimbal_INT_angle_point;
    const float *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
    gravity_comp_t gimbal_pitch_gravity_comp;
    gimbal_small_yaw_t small_yaw;
    gimbal_motor_t roll;               // roll 轴反馈、目标与位置命令
    gimbal_target_state_t target;      // 四轴目标生成和切换历史
    gimbal_behaviour_state_t behaviour_state;
} gimbal_control_t;

/** @brief 清除传入轴的全部前馈轨迹历史。 */
void gimbal_feedforward_clear(gimbal_motor_t *motor);

/** @brief 清除传入轴指定来源的前馈轨迹历史。 */
void gimbal_feedforward_clear_source(gimbal_motor_t *motor, gimbal_control_source_e source);

/** @brief 由目标角 rad 更新指定来源的速度和加速度前馈历史。 */
void gimbal_feedforward_track_target(gimbal_motor_t *motor, float target_angle, gimbal_control_source_e source);

/** @brief 读取指定轴的掉线状态与反馈时间戳，返回反馈是否有效。 */
uint8_t gimbal_feedback_ready(gimbal_axis_e axis);

/** @brief 将当前轴反馈写入各来源目标，角度单位 rad。 */
void gimbal_motor_target_reset(gimbal_motor_t *motor);

/** @brief 清除当前轴控制历史并生成零输出命令。 */
void gimbal_motor_zero_output(gimbal_motor_t *motor);

/** @brief 按当前轴的新旧模式初始化控制目标。 */
void gimbal_motor_mode_transit(gimbal_motor_t *motor);

/** @brief 由设备反馈更新当前轴角度 rad、速度 rad/s 与有效状态。 */
void gimbal_motor_feedback_update(gimbal_motor_t *motor, const gimbal_control_t *control);

/** @brief 由当前轴目标和反馈生成 MIT 位置命令。 */
void gimbal_motor_position_control(gimbal_motor_t *motor);

/** @brief 将小 yaw 原始编码值转换为相对前向零点的角度，返回 rad。 */
float gimbal_ecd_angle(uint16_t ecd);

extern gimbal_control_t gimbal_control;
extern float yaw_can_set_current;
extern float pitch_can_set_current;
extern int16_t shoot_can_set_current;

/**
  * @brief          创建云台控制任务
  * @retval         none
  */
void GimbalTask_Init(void);

/**
  * @brief          获取 yaw 电机控制结构体指针
  * @retval         yaw 电机控制结构体只读指针
  */
const gimbal_motor_t *get_yaw_motor_point(void);

/**
  * @brief          获取 pitch 电机控制结构体指针
  * @retval         pitch 电机控制结构体只读指针
  */
const gimbal_motor_t *get_pitch_motor_point(void);

/**
  * @brief          更新绝对角目标并执行限幅
  * @retval         none
  */
void gimbal_absolute_angle_limit(gimbal_motor_t *motor, float add);

/**
  * @brief          执行云台电机绝对角控制
  * @retval         none
  */
void gimbal_motor_absolute_angle_control(gimbal_motor_t *motor);

/**
  * @brief          执行云台电机相对角控制
  * @retval         none
  */
void gimbal_motor_relative_angle_control(gimbal_motor_t *motor);


/**
  * @brief          初始化云台 PID 控制器
  * @retval         none
  */
void gimbal_pid_init(gimbal_pid_t *pid, float kp, float ki, float kd, float max_out, float max_iout);

/**
  * @brief          清空云台 PID 控制器状态
  * @retval         none
  */
void gimbal_pid_clear(gimbal_pid_t *pid);


/**
  * @brief          初始化云台控制结构体
  * @retval         none
  */
void gimbal_init(gimbal_control_t *control);
/**
  * @brief          设置云台控制模式
  * @retval         none
  */
void gimbal_set_mode(gimbal_control_t *control);
/**
  * @brief          更新云台反馈数据
  * @retval         none
  */
void gimbal_feedback_update(gimbal_control_t *control);
/**
  * @brief          初始化云台控制结构体
  * @retval         none
  */
void gimbal_mode_change_control_transit(gimbal_control_t *control);
/**
  * @brief          生成云台目标控制量
  * @retval         none
  */
void gimbal_set_control(gimbal_control_t *control);
/**
  * @brief          设置云台控制模式
  * @retval         none
  */
void gimbal_control_loop(gimbal_control_t *control);
/**
  * @brief          设置云台控制模式
  * @retval         none
  */
void gimbal_send_cmd(gimbal_control_t *control);
#endif
