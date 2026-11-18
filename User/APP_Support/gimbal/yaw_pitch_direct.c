/**
  * @file       yaw_pitch_direct.c
  * @brief      yaw/pitch 直控云台控制实现
  * @note       实现云台反馈更新、目标限幅、前馈补偿、PID 闭环、在线保护和 MIT 力矩发送。
  */
#include "yaw_pitch_direct.h"
#include "gimbal_behaviour.h"
#include "auto_aim.h"
#include "hwt_imu.h"
#include "bsp_fdcan.h"
#include "detect_task.h"
#include "robot_param.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include <math.h>
#include <string.h>

#if (ROBOT_GIMBAL == ROBOT_GIMBAL_YAW_PITCH_DIRECT)

float yaw_can_set_current = 0.0f;
float pitch_can_set_current = 0.0f;
int16_t shoot_can_set_current = 0;

static float yaw_auto_aim_target_vel = 0.0f;
static float pitch_auto_aim_target_vel = 0.0f;
#define small_yaw_angle_pid          (gimbal_control.small_yaw.angle_pid)
#define small_yaw_speed_pid          (gimbal_control.small_yaw.speed_pid)
#define small_yaw_current_cmd_a      (gimbal_control.small_yaw.current_cmd_a)
#define small_yaw_current_cmd_raw    (gimbal_control.small_yaw.current_cmd_raw)
#define small_yaw_control_allowed    (gimbal_control.small_yaw.control_allowed)
#define small_yaw_can_send_status    (gimbal_control.small_yaw.can_send_status)
#define small_yaw_angle_rad          (gimbal_control.small_yaw.angle_rad)
#define small_yaw_target_rad         (gimbal_control.small_yaw.target_rad)
#define small_yaw_speed_feedback_rad_s (gimbal_control.small_yaw.speed_feedback_rad_s)
#define small_yaw_last_ecd           (gimbal_control.small_yaw.last_ecd)
#define small_yaw_feedback_init      (gimbal_control.small_yaw.feedback_init)
#define small_yaw_scan_yaw_direction  (gimbal_control.small_yaw.scan_yaw_direction)
#define small_yaw_scan_pitch_direction (gimbal_control.small_yaw.scan_pitch_direction)
#define scan_pitch_target_rad         (gimbal_control.small_yaw.scan_pitch_target_rad)

static void small_yaw_feedback_update(void);
static float small_yaw_ecd_to_angle_rad(uint16_t ecd);
static void small_yaw_scan_update(gimbal_control_t *control, uint8_t auto_enable, float add_yaw, float add_pitch);
static void gimbal_debug_sine_target_update(gimbal_control_t *control);
static void small_yaw_control_update(void);
static int16_t small_yaw_current_to_cmd(float current_a);

/**
  * @brief          声明 yaw 电机在线检测函数
  * @retval         none
  */
static uint8_t gimbal_yaw_online(void);
/**
  * @brief          声明 pitch MIT 反馈检测函数
  * @retval         none
  */
static uint8_t gimbal_pitch_mit_feedback_ready(void);
/**
  * @brief          声明单轴零输出清理函数
  * @retval         none
  */
static void gimbal_motor_zero_output(gimbal_motor_t *motor);
/**
  * @brief          声明角度归一化函数
  * @retval         归一化后的角度
  */
static float gimbal_wrap_angle(float angle);
/**
  * @brief          更新大 yaw 连续累计相对角度
  * @retval         连续累计相对角度，单位 rad
  */
static float gimbal_yaw_unwrap_update(gimbal_motor_t *motor, float wrapped_angle);
/**
  * @brief          声明自瞄补偿读取函数
  * @retval         自瞄补偿量
  */
static float gimbal_take_auto_aim_bias(gimbal_motor_t *motor);
/**
  * @brief          声明浮点限幅函数
  * @retval         限幅后的值
  */
static float gimbal_clamp(float value, float min_value, float max_value);
/**
  * @brief          声明静摩擦补偿原始量计算函数
  * @retval         静摩擦补偿原始量
  */
static float gimbal_calc_static_friction_comp_raw(const gimbal_motor_t *motor, float angle_error);
/**
  * @brief          声明静摩擦补偿更新函数
  * @retval         静摩擦补偿输出
  */
static float gimbal_update_static_friction_comp(gimbal_motor_t *motor, float angle_error);
/**
  * @brief          声明力矩命令限幅函数
  * @retval         力矩命令
  */
static float gimbal_float_to_torque_cmd(float output);
/**
  * @brief          声明惯量前馈计算函数
  * @retval         前馈力矩
  */
static float gimbal_calc_feedforward(gimbal_motor_t *motor);
/**
  * @brief          声明角度反馈力矩计算函数
  * @retval         反馈力矩
  */
static float gimbal_calc_feedback_torque(gimbal_motor_t *motor, gimbal_pid_t *angle_pid, float angle_get, float angle_set);
/**
  * @brief          声明角速度力矩计算函数
  * @retval         控制力矩
  */
static float gimbal_calc_angle_speed_torque(gimbal_motor_t *motor, gimbal_pid_t *pid,
                                           float angle_error, float speed_get);
/**
  * @brief          声明自瞄目标速度清理函数
  * @retval         none
  */
static void gimbal_auto_aim_clear_target_vel(gimbal_motor_t *motor);

/**
  * @brief          获取 INS 姿态角数组弱接口
  * @retval         姿态角数组指针
  */
__attribute__((weak)) const float *get_INS_angle_point(void)
{
    return 0;
}

/**
  * @brief          获取陀螺仪角速度数组弱接口
  * @retval         角速度数组指针
  */
__attribute__((weak)) const float *get_gyro_data_point(void)
{
    return 0;
}

/**
  * @brief          获取加速度数组弱接口
  * @retval         加速度数组指针
  */
__attribute__((weak)) const float *get_accel_data_point(void)
{
    return 0;
}

/**
  * @brief          对 MIT 力矩命令执行限幅
  * @retval         限幅后的值
  */
static float gimbal_mit_clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

/**
  * @brief          将控制器输出转换为 MIT 力矩命令
  * @retval         none
  */
static float gimbal_output_to_mit_torque(float output)
{
    return gimbal_mit_clamp(output, T_MIN, T_MAX);
}

/**
  * @brief          判断 yaw 电机是否在线
  * @retval         none
  */
static uint8_t gimbal_yaw_online(void)
{
    return (uint8_t)(toe_is_error(YAW_GIMBAL_MOTOR_TOE) == 0U);
}

static uint8_t gimbal_hwt906_yaw_ready(void)
{
    const hwt_imu_info_t *hwt906 = hwt906_get_info();

    return (uint8_t)((hwt906 != 0) && (hwt906->angle.yaw_init_flag != 0u));
}

/**
  * @brief          判断 pitch MIT 反馈是否可用
  * @retval         none
  */
static uint8_t gimbal_pitch_mit_feedback_ready(void)
{
    const MITMeasure_t *measure = &MIT_MOTOR_MEASURE[GIMBAL_PITCH_MIT_INDEX];
    uint32_t now = HAL_GetTick();

    return (uint8_t)((toe_is_error(PITCH_GIMBAL_MOTOR_TOE) == 0U) &&
                     (measure->fdb.last_fdb_time != 0U) &&
                     ((uint32_t)(now - measure->fdb.last_fdb_time) <=
                      (uint32_t)GIMBAL_PITCH_MIT_FDB_TIMEOUT));
}

/**
  * @brief          清除单轴前馈轨迹状态
  * @retval         none
  */
static void gimbal_feedforward_clear(gimbal_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    motor->ref_vel = 0.0f;
    motor->ref_vel_last = 0.0f;
    motor->ref_accel = 0.0f;
    motor->rc_ref_vel = 0.0f;
    motor->rc_ref_vel_last = 0.0f;
    motor->rc_ref_accel = 0.0f;
    motor->rc_ref_target_last = 0.0f;
    motor->rc_ref_target_init = 0u;
    motor->auto_ref_vel = 0.0f;
    motor->auto_ref_vel_last = 0.0f;
    motor->auto_ref_accel = 0.0f;
    motor->auto_ref_target_last = 0.0f;
    motor->auto_ref_target_init = 0u;
    motor->ff_torque = 0.0f;
    gimbal_auto_aim_clear_target_vel(motor);
}

/**
  * @brief          清除单轴控制状态并置零输出
  * @retval         none
  */
static void gimbal_motor_zero_output(gimbal_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    gimbal_feedforward_clear(motor);
    gimbal_pid_clear(&motor->absolute_angle_pid);
    gimbal_pid_clear(&motor->relative_angle_pid);

    motor->mode = GIMBAL_MOTOR_RAW;
    motor->last_mode = GIMBAL_MOTOR_RAW;
    motor->raw_cmd = 0.0f;
    motor->current_set = 0.0f;
    motor->output = 0.0f;
    motor->given_current = 0.0f;
    motor->pid_torque = 0.0f;
    motor->static_friction_comp = 0.0f;
    motor->rc_control_enable = 0u;
    motor->auto_control_enable = 0u;
    motor->absolute_angle_set = motor->absolute_angle;
    motor->relative_angle_set = motor->relative_angle;
    motor->rc_absolute_angle_set = motor->absolute_angle;
    motor->rc_relative_angle_set = motor->relative_angle;
    motor->auto_absolute_angle_set = motor->absolute_angle;
    motor->auto_relative_angle_set = motor->relative_angle;
    motor->gyro_set = motor->gyro;
}

/**
  * @brief          清除指定控制来源的前馈状态
  * @retval         none
  */
static void gimbal_feedforward_clear_source(gimbal_motor_t *motor,
                                            gimbal_control_source_e source)
{
    if (motor == 0)
    {
        return;
    }

    if (source == GIMBAL_CONTROL_SOURCE_AUTO)
    {
        motor->auto_ref_vel = 0.0f;
        motor->auto_ref_vel_last = 0.0f;
        motor->auto_ref_accel = 0.0f;
        motor->auto_ref_target_last = 0.0f;
        motor->auto_ref_target_init = 0u;
        gimbal_auto_aim_clear_target_vel(motor);
        return;
    }

    motor->rc_ref_vel = 0.0f;
    motor->rc_ref_vel_last = 0.0f;
    motor->rc_ref_accel = 0.0f;
    motor->rc_ref_target_last = 0.0f;
    motor->rc_ref_target_init = 0u;
}

/**
  * @brief          根据目标角跟踪更新前馈速度和加速度
  * @retval         none
  */
static void gimbal_feedforward_track_target(gimbal_motor_t *motor,
                                            float target_angle,
                                            gimbal_control_source_e source)
{
    float ref_vel_cmd;
    float ref_accel_cmd;
    float alpha;
    float accel_limit;
    float *ref_vel;
    float *ref_vel_last;
    float *ref_accel;
    float *target_last;
    uint8_t *target_init;
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;

    if (motor == 0 || control_dt <= 0.0f)
    {
        return;
    }

    if (source == GIMBAL_CONTROL_SOURCE_AUTO)
    {
        ref_vel = &motor->auto_ref_vel;
        ref_vel_last = &motor->auto_ref_vel_last;
        ref_accel = &motor->auto_ref_accel;
        target_last = &motor->auto_ref_target_last;
        target_init = &motor->auto_ref_target_init;
    }
    else
    {
        ref_vel = &motor->rc_ref_vel;
        ref_vel_last = &motor->rc_ref_vel_last;
        ref_accel = &motor->rc_ref_accel;
        target_last = &motor->rc_ref_target_last;
        target_init = &motor->rc_ref_target_init;
    }

    if (*target_init == 0u)
    {
        *target_last = target_angle;
        *target_init = 1u;
        *ref_vel_last = 0.0f;
        *ref_vel = 0.0f;
        *ref_accel = 0.0f;
        return;
    }

    alpha = gimbal_mit_clamp(YAW_REF_VEL_FILTER_ALPHA, 0.0f, 1.0f);
    ref_vel_cmd = (target_angle - *target_last) / control_dt;

    *ref_vel_last = *ref_vel;
    *ref_vel += alpha * (ref_vel_cmd - *ref_vel);

    ref_accel_cmd = (*ref_vel - *ref_vel_last) / control_dt;
    accel_limit = YAW_REF_ACCEL_LIMIT;
    if (accel_limit > 0.0f)
    {
        ref_accel_cmd = gimbal_mit_clamp(ref_accel_cmd,
                                         -accel_limit,
                                         accel_limit);
    }

    *ref_accel = ref_accel_cmd;
    *target_last = target_angle;
}

/**
  * @brief          将角度归一化到 [-pi, pi]
  * @retval         none
  */
static float yaw_pitch_direct_wrap_angle(float angle)
{
    while (angle > YAW_PITCH_DIRECT_PI)
    {
        angle -= 2.0f * YAW_PITCH_DIRECT_PI;
    }
    while (angle < -YAW_PITCH_DIRECT_PI)
    {
        angle += 2.0f * YAW_PITCH_DIRECT_PI;
    }
    return angle;
}

static float gimbal_yaw_unwrap_update(gimbal_motor_t *motor, float wrapped_angle)
{
    float delta;

    if (motor == 0)
    {
        return wrapped_angle;
    }

    if (motor->relative_angle_unwrap_init == 0u)
    {
        motor->relative_angle_feedback_last = wrapped_angle;
        motor->relative_angle_unwrapped = wrapped_angle;
        motor->relative_angle_unwrap_init = 1u;
        return motor->relative_angle_unwrapped;
    }

    delta = wrapped_angle - motor->relative_angle_feedback_last;
    if (delta > YAW_PITCH_DIRECT_PI)
    {
        delta -= 2.0f * YAW_PITCH_DIRECT_PI;
    }
    else if (delta < -YAW_PITCH_DIRECT_PI)
    {
        delta += 2.0f * YAW_PITCH_DIRECT_PI;
    }

    motor->relative_angle_unwrapped += delta;
    motor->relative_angle_feedback_last = wrapped_angle;
    return motor->relative_angle_unwrapped;
}

/**
  * @brief          规划自瞄目标角速度受限的目标角
  * @retval         none
  */
static float gimbal_auto_aim_plan_target(float target_set,
                                         float desired_target,
                                         float *target_vel,
                                         float kp,
                                         float max_speed,
                                         float max_accel,
                                         float dt)
{
    float err;
    float vel_cmd;
    float vel_delta;
    float vel_delta_max;
    float step;

    if (target_vel == 0 || dt <= 0.0f)
    {
        return target_set;
    }

    err = desired_target - target_set;
    vel_cmd = gimbal_mit_clamp(kp * err, -max_speed, max_speed);
    vel_delta_max = max_accel * dt;
    vel_delta = gimbal_mit_clamp(vel_cmd - *target_vel,
                                 -vel_delta_max,
                                  vel_delta_max);
    *target_vel += vel_delta;

    step = *target_vel * dt;
    if ((err > 0.0f && step > err) || (err < 0.0f && step < err))
    {
        step = err;
        *target_vel = 0.0f;
    }

    return target_set + step;
}

/**
  * @brief          清除单轴自瞄目标速度状态
  * @retval         none
  */
static void gimbal_auto_aim_clear_target_vel(gimbal_motor_t *motor)
{
    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        yaw_auto_aim_target_vel = 0.0f;
    }
    else if (motor == &gimbal_control.gimbal_pitch_motor)
    {
        pitch_auto_aim_target_vel = 0.0f;
    }
}

/**
  * @brief          更新 yaw 绝对角目标并执行限幅
  * @retval         none
  */
static void gimbal_yaw_absolute_angle_limit(gimbal_control_t *control,
                                            float add,
                                            uint8_t rc_enable,
                                            uint8_t auto_enable)
{
    gimbal_motor_t *yaw_motor;
    float chassis_yaw;
    float rc_relative_angle_set;
    float auto_relative_angle_set;
    float desired_relative_angle;
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;
    if (control == 0)
    {
        return;
    }

    yaw_motor = &control->gimbal_yaw_motor;
    chassis_yaw = yaw_motor->absolute_angle - yaw_motor->relative_angle_unwrapped;
    rc_relative_angle_set =
        yaw_motor->rc_absolute_angle_set -
        chassis_yaw;

    if (rc_enable != 0u)
    {
        rc_relative_angle_set += add;
    }
    else
    {
        rc_relative_angle_set = yaw_motor->relative_angle;
        gimbal_feedforward_clear_source(yaw_motor, GIMBAL_CONTROL_SOURCE_RC);
    }

    yaw_motor->rc_relative_angle_set = rc_relative_angle_set;
    yaw_motor->rc_absolute_angle_set =
        chassis_yaw + rc_relative_angle_set;

    if (auto_enable != 0u)
    {
        desired_relative_angle =
            yaw_motor->relative_angle_unwrapped +
            auto_aim_get_yaw_err_rad();
        auto_relative_angle_set =
            gimbal_auto_aim_plan_target(yaw_motor->auto_relative_angle_set,
                                        desired_relative_angle,
                                        &yaw_auto_aim_target_vel,
                                        GIMBAL_AUTO_AIM_YAW_KP,
                                        GIMBAL_AUTO_AIM_YAW_MAX_SPEED,
                                        GIMBAL_AUTO_AIM_YAW_MAX_ACCEL,
                                        control_dt);
    }
    else
    {
        auto_relative_angle_set = rc_relative_angle_set;
    }

    yaw_motor->auto_relative_angle_set = auto_relative_angle_set;
    yaw_motor->auto_absolute_angle_set =
        chassis_yaw + auto_relative_angle_set;

    yaw_motor->relative_angle_set =
        rc_relative_angle_set +
        ((auto_enable != 0u) ? (auto_relative_angle_set - yaw_motor->relative_angle) : 0.0f);
    yaw_motor->absolute_angle_set =
        chassis_yaw + yaw_motor->relative_angle_set;
}

/**
  * @brief          更新 pitch 相对角目标并执行限幅
  * @retval         none
  */
static void gimbal_pitch_relative_angle_limit(gimbal_control_t *control,
                                              float add,
                                              uint8_t rc_enable,
                                              uint8_t auto_enable)
{
    gimbal_motor_t *pitch_motor;
    float rc_relative_angle_set;
    float desired_relative_angle;
    float auto_relative_angle_set;
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;

    if (control == 0)
    {
        return;
    }

    pitch_motor = &control->gimbal_pitch_motor;

    if (rc_enable != 0u)
    {
        rc_relative_angle_set = pitch_motor->rc_relative_angle_set + add;
    }
    else
    {
        rc_relative_angle_set = pitch_motor->relative_angle;
        gimbal_feedforward_clear_source(pitch_motor, GIMBAL_CONTROL_SOURCE_RC);
    }

    pitch_motor->rc_relative_angle_set =
        gimbal_mit_clamp(rc_relative_angle_set,
                         pitch_motor->min_relative_angle,
                         pitch_motor->max_relative_angle);
    pitch_motor->rc_absolute_angle_set = pitch_motor->rc_relative_angle_set;

    if (auto_enable != 0u)
    {
        desired_relative_angle =
            pitch_motor->relative_angle +
            auto_aim_get_pitch_err_rad();
        desired_relative_angle =
            gimbal_mit_clamp(desired_relative_angle,
                             pitch_motor->min_relative_angle,
                             pitch_motor->max_relative_angle);
        auto_relative_angle_set =
            gimbal_auto_aim_plan_target(pitch_motor->auto_relative_angle_set,
                                        desired_relative_angle,
                                        &pitch_auto_aim_target_vel,
                                        GIMBAL_AUTO_AIM_PITCH_KP,
                                        GIMBAL_AUTO_AIM_PITCH_MAX_SPEED,
                                        GIMBAL_AUTO_AIM_PITCH_MAX_ACCEL,
                                        control_dt);
        pitch_motor->auto_relative_angle_set =
            gimbal_mit_clamp(auto_relative_angle_set,
                             pitch_motor->min_relative_angle,
                             pitch_motor->max_relative_angle);
    }
    else
    {
        pitch_motor->auto_relative_angle_set = pitch_motor->rc_relative_angle_set;
    }

    pitch_motor->auto_absolute_angle_set = pitch_motor->auto_relative_angle_set;
    pitch_motor->relative_angle_set =
        pitch_motor->rc_relative_angle_set +
        ((auto_enable != 0u) ?
             (pitch_motor->auto_relative_angle_set - pitch_motor->relative_angle) :
             0.0f);
    pitch_motor->absolute_angle_set = pitch_motor->relative_angle_set;
}

/**
  * @brief          更新绝对角目标并执行限幅
  * @retval         none
  */
__attribute__((used)) void gimbal_absolute_angle_limit(gimbal_motor_t *motor, float add)
{
    const float bias = gimbal_take_auto_aim_bias(motor);

    if (motor == 0)
    {
        return;
    }

    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        motor->absolute_angle_set += add + bias;
    }
    else
    {
        motor->absolute_angle_set =
            gimbal_clamp(motor->absolute_angle_set + add + bias,
                         motor->min_relative_angle,
                         motor->max_relative_angle);
    }
}

/**
  * @brief          执行云台电机绝对角控制
  * @retval         none
  */
__attribute__((used)) void gimbal_motor_absolute_angle_control(gimbal_motor_t *motor)
{
    float angle_get;
    float angle_set;

    if (motor == 0)
    {
        return;
    }

    motor->ref_vel =
        ((motor->rc_control_enable != 0u) ? motor->rc_ref_vel : 0.0f) +
        ((motor->auto_control_enable != 0u) ? motor->auto_ref_vel : 0.0f);
    motor->ref_accel =
        ((motor->rc_control_enable != 0u) ? motor->rc_ref_accel : 0.0f) +
        ((motor->auto_control_enable != 0u) ? motor->auto_ref_accel : 0.0f);

    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        angle_get = motor->absolute_angle;
        gimbal_calc_angle_speed_torque(motor,
                                       &motor->absolute_angle_pid,
                                       gimbal_wrap_angle(motor->absolute_angle_set - angle_get),
                                       motor->gyro);
    }
    else
    {
        angle_get = gimbal_wrap_angle(motor->absolute_angle);
        angle_set = gimbal_wrap_angle(motor->absolute_angle_set);
        gimbal_calc_feedback_torque(motor, &motor->absolute_angle_pid, angle_get, angle_set);
    }

    motor->current_set = motor->pid_torque + gimbal_calc_feedforward(motor);
    motor->output = motor->current_set;
    motor->given_current = gimbal_float_to_torque_cmd(motor->output);
}

/**
  * @brief          执行云台电机绝对角控制
  * @retval         none
  */
__attribute__((used)) void gimbal_motor_relative_angle_control(gimbal_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    motor->ref_vel =
        ((motor->rc_control_enable != 0u) ? motor->rc_ref_vel : 0.0f) +
        ((motor->auto_control_enable != 0u) ? motor->auto_ref_vel : 0.0f);
    motor->ref_accel =
        ((motor->rc_control_enable != 0u) ? motor->rc_ref_accel : 0.0f) +
        ((motor->auto_control_enable != 0u) ? motor->auto_ref_accel : 0.0f);

    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        gimbal_calc_angle_speed_torque(motor,
                                       &motor->relative_angle_pid,
                                       gimbal_wrap_angle(motor->relative_angle_set - motor->relative_angle),
                                       GIMBAL_YAW_FEEDBACK_SIGN *
                                           MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].fdb.vel);
    }
    else
    {
        gimbal_calc_angle_speed_torque(motor,
                                       &motor->relative_angle_pid,
                                       motor->relative_angle_set - motor->relative_angle,
                                       motor->gyro);
    }

    motor->current_set = motor->pid_torque + gimbal_calc_feedforward(motor);
    motor->output = motor->current_set;
    motor->given_current = gimbal_float_to_torque_cmd(motor->output);
}

/**
  * @brief          执行云台电机 RAW 输出控制
  * @retval         none
  */
__attribute__((used)) void gimbal_motor_raw_angle_control(gimbal_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    motor->current_set = motor->raw_cmd;
    motor->output = motor->raw_cmd;
    motor->pid_torque = 0.0f;
    motor->ff_torque = 0.0f;
    motor->static_friction_comp = 0.0f;
    motor->rc_control_enable = 0u;
    motor->auto_control_enable = 0u;
    motor->given_current = gimbal_float_to_torque_cmd(motor->output);
}

/**
  * @brief          初始化云台 PID 控制器
  * @retval         none
  */
__attribute__((used)) void gimbal_pid_init(gimbal_pid_t *pid, float kp, float ki, float kd, float max_out, float max_iout)
{
    float pid_param[3];

    if (pid == NULL)
    {
        return;
    }

    pid_param[0] = kp;
    pid_param[1] = ki;
    pid_param[2] = kd;
    PID_init(pid, PID_POSITION, pid_param, max_out, max_iout);
}

/**
  * @brief          清空云台 PID 控制器状态
  * @retval         none
  */
__attribute__((used)) void gimbal_pid_clear(gimbal_pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    PID_clear(pid);
}

/**
  * @brief          计算云台 PID 输出
  * @retval         none
  */
__attribute__((used)) float gimbal_pid_calc(gimbal_pid_t *pid, float get, float set, float error_delta)
{
    (void)error_delta;

    if (pid == NULL)
    {
        return 0.0f;
    }

    return PID_Calc(pid, get, set);
}

/**
  * @brief          将角度归一化到 [-pi, pi]
  * @retval         none
  */
static float gimbal_wrap_angle(float angle)
{
    while (angle > GIMBAL_PI)
    {
        angle -= 2.0f * GIMBAL_PI;
    }
    while (angle < -GIMBAL_PI)
    {
        angle += 2.0f * GIMBAL_PI;
    }
    return angle;
}

/**
  * @brief          读取并清除单轴自瞄角度补偿量
  * @retval         none
  */
static float gimbal_take_auto_aim_bias(gimbal_motor_t *motor)
{
    float bias = 0.0f;

    (void)motor;

    return bias;
}

/**
  * @brief          对浮点数执行区间限幅
  * @retval         none
  */
static float gimbal_clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

/**
  * @brief          计算单轴静摩擦补偿原始量
  * @retval         静摩擦补偿原始量
  */
static float gimbal_calc_static_friction_comp_raw(const gimbal_motor_t *motor, float angle_error)
{
    float abs_error;
    float comp_scale;
    float deadband;
    float fullband;

    if (motor == &gimbal_control.gimbal_pitch_motor)
    {
        deadband = GIMBAL_PITCH_STATIC_FRICTION_DEADBAND;
        fullband = GIMBAL_PITCH_STATIC_FRICTION_FULLBAND;
    }
    else
    {
        deadband = GIMBAL_YAW_STATIC_FRICTION_DEADBAND;
        fullband = GIMBAL_YAW_STATIC_FRICTION_FULLBAND;
    }

    abs_error = fabsf(angle_error);
    if (abs_error <= deadband)
    {
        return 0.0f;
    }

    if (fullband <= deadband ||
        abs_error >= fullband)
    {
        comp_scale = 1.0f;
    }
    else
    {
        comp_scale =
            (abs_error - deadband) /
            (fullband - deadband);
    }

    if (angle_error > 0.0f)
    {
        if (motor == &gimbal_control.gimbal_pitch_motor)
        {
            return GIMBAL_PITCH_STATIC_FRICTION_COMP_UP * comp_scale;
        }
        return GIMBAL_YAW_STATIC_FRICTION_COMP * comp_scale;
    }
    if (angle_error < 0.0f)
    {
        if (motor == &gimbal_control.gimbal_pitch_motor)
        {
            return -GIMBAL_PITCH_STATIC_FRICTION_COMP_DOWN * comp_scale;
        }
        return -GIMBAL_YAW_STATIC_FRICTION_COMP * comp_scale;
    }
    return 0.0f;
}

/**
  * @brief          更新单轴静摩擦补偿输出
  * @retval         静摩擦补偿输出
  */
static float gimbal_update_static_friction_comp(gimbal_motor_t *motor, float angle_error)
{
    float alpha;
    float comp_cmd;

    comp_cmd = gimbal_calc_static_friction_comp_raw(motor, angle_error);
    if (motor == 0)
    {
        return comp_cmd;
    }

    if (motor == &gimbal_control.gimbal_pitch_motor)
    {
        alpha = gimbal_clamp(GIMBAL_PITCH_STATIC_FRICTION_FILTER_ALPHA, 0.0f, 1.0f);
    }
    else
    {
        alpha = gimbal_clamp(GIMBAL_YAW_STATIC_FRICTION_FILTER_ALPHA, 0.0f, 1.0f);
    }
    motor->static_friction_comp +=
        alpha * (comp_cmd - motor->static_friction_comp);

    return motor->static_friction_comp;
}

/**
  * @brief          将浮点输出限制为力矩命令范围
  * @retval         力矩命令
  */
static float gimbal_float_to_torque_cmd(float output)
{
    return gimbal_clamp(output, T_MIN, T_MAX);
}

/**
  * @brief          计算单轴惯量前馈力矩
  * @retval         前馈力矩
  */
static float gimbal_calc_feedforward(gimbal_motor_t *motor)
{
    float velocity_torque = 0.0f;

    if (motor == 0)
    {
        return 0.0f;
    }

    if (motor == &gimbal_control.gimbal_pitch_motor)
    {
        velocity_torque = PITCH_VELOCITY_FF_GAIN * motor->ref_vel;
    }

    motor->ff_torque = velocity_torque + motor->inertia_kgm2 * motor->ref_accel;

    return motor->ff_torque;
}

/**
  * @brief          计算单轴角度反馈力矩
  * @retval         反馈力矩
  */
static float gimbal_calc_feedback_torque(gimbal_motor_t *motor, gimbal_pid_t *angle_pid, float angle_get, float angle_set)
{
    float angle_torque;
    float angle_error;

    if (motor == 0 || angle_pid == 0)
    {
        return 0.0f;
    }

    motor->gyro_set = motor->ref_vel;
    angle_error = angle_set - angle_get;
    angle_torque = gimbal_pid_calc(angle_pid, angle_get, angle_set, 0.0f);
    motor->pid_torque =
        gimbal_clamp(angle_torque + gimbal_update_static_friction_comp(motor, angle_error),
                     -angle_pid->max_out,
                     angle_pid->max_out);

    return motor->pid_torque;
}

/**
  * @brief          根据角度误差和角速度误差计算控制力矩
  * @retval         控制力矩
  */
static float gimbal_calc_angle_speed_torque(gimbal_motor_t *motor, gimbal_pid_t *pid,
                                           float angle_error, float speed_get)
{
    float speed_error;
    float output;

    if (motor == 0 || pid == 0)
    {
        return 0.0f;
    }

    motor->gyro_set = motor->ref_vel;
    speed_error = motor->gyro_set - speed_get;

    pid->set = angle_error;
    pid->fdb = 0.0f;
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = angle_error;
    pid->Pout = pid->Kp * angle_error;
    pid->Iout = 0.0f;
    pid->Dout = pid->Kd * speed_error;

    output = pid->Pout + pid->Dout + gimbal_update_static_friction_comp(motor, angle_error);
    output = gimbal_clamp(output, -pid->max_out, pid->max_out);
    pid->out = output;
    motor->pid_torque = output;

    return motor->pid_torque;
}

/**
  * @brief          清空云台两轴全部 PID 状态
  * @retval         none
  */
static void gimbal_total_pid_clear(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    gimbal_pid_clear(&control->gimbal_yaw_motor.absolute_angle_pid);
    gimbal_pid_clear(&control->gimbal_yaw_motor.relative_angle_pid);
    gimbal_pid_clear(&control->gimbal_pitch_motor.absolute_angle_pid);
    gimbal_pid_clear(&control->gimbal_pitch_motor.relative_angle_pid);
}

/**
  * @brief          初始化云台控制结构体
  * @retval         none
  */
__attribute__((used)) void gimbal_init(gimbal_control_t *control)
{
    gravity_comp_param_t gravity_comp_param;

    if (control == 0)
    {
        return;
    }

    memset(control, 0, sizeof(*control));

    control->gimbal_rc_ctrl = get_remote_control_point();
    control->gimbal_INT_angle_point = get_INS_angle_point();
    control->gimbal_INT_gyro_point = get_gyro_data_point();

    control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_RAW;
    control->gimbal_yaw_motor.last_mode = GIMBAL_MOTOR_RAW;
    control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_RAW;
    control->gimbal_pitch_motor.last_mode = GIMBAL_MOTOR_RAW;
    control->small_yaw.can_send_status = 1U;
    small_yaw_scan_yaw_direction = 1U;
    small_yaw_scan_pitch_direction = 1U;

    gimbal_pid_init(&control->gimbal_yaw_motor.absolute_angle_pid,
                    YAW_GYRO_ABSOLUTE_PID_KP,
                    YAW_GYRO_ABSOLUTE_PID_KI,
                    YAW_GYRO_ABSOLUTE_PID_KD,
                    YAW_GYRO_ABSOLUTE_PID_MAX_OUT,
                    YAW_GYRO_ABSOLUTE_PID_MAX_IOUT);
    gimbal_pid_init(&control->gimbal_yaw_motor.relative_angle_pid,
                    YAW_ENCODE_RELATIVE_PID_KP,
                    YAW_ENCODE_RELATIVE_PID_KI,
                    YAW_ENCODE_RELATIVE_PID_KD,
                    YAW_ENCODE_RELATIVE_PID_MAX_OUT,
                    YAW_ENCODE_RELATIVE_PID_MAX_IOUT);

    gimbal_pid_init(&control->gimbal_pitch_motor.relative_angle_pid,
                    PITCH_ENCODE_RELATIVE_PID_KP,
                    PITCH_ENCODE_RELATIVE_PID_KI,
                    PITCH_ENCODE_RELATIVE_PID_KD,
                    PITCH_ENCODE_RELATIVE_PID_MAX_OUT,
                    PITCH_ENCODE_RELATIVE_PID_MAX_IOUT);

    control->gimbal_pitch_motor.max_relative_angle = 0.0f;
    control->gimbal_pitch_motor.min_relative_angle = 0.0f;
    control->gimbal_yaw_motor.inertia_kgm2 = YAW_INERTIA_KGM2;
    control->gimbal_pitch_motor.inertia_kgm2 = PITCH_INERTIA_KGM2;
		
//		Motor_MIT_MODE(&hfdcan1, DM_YAW_CAN_ID);
    gimbal_pid_init(&small_yaw_angle_pid,
                    SMALL_YAW_ANGLE_PID_KP,
                    SMALL_YAW_ANGLE_PID_KI,
                    SMALL_YAW_ANGLE_PID_KD,
                    SMALL_YAW_ANGLE_PID_MAX_OUT,
                    0.0f);
    gimbal_pid_init(&small_yaw_speed_pid,
                    SMALL_YAW_SPEED_PID_KP,
                    SMALL_YAW_SPEED_PID_KI,
                    SMALL_YAW_SPEED_PID_KD,
                    SMALL_YAW_SPEED_PID_MAX_OUT,
                    SMALL_YAW_SPEED_PID_MAX_IOUT);
    Motor_ENABLE(&hfdcan1, DM_BIG_YAW_CAN_ID);
//		Motor_MIT_MODE(&hfdcan2, DM_PIT_CAN_ID);
    Motor_ENABLE(&hfdcan2, DM_PIT_CAN_ID);
    Motor_ENABLE(&hfdcan2, DM_ROLL_CAN_ID);

    gimbal_total_pid_clear(control);
    gimbal_feedback_update(control);

    control->gimbal_yaw_motor.absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
    control->gimbal_yaw_motor.relative_angle_set = control->gimbal_yaw_motor.relative_angle;
    control->gimbal_yaw_motor.rc_absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
    control->gimbal_yaw_motor.rc_relative_angle_set = control->gimbal_yaw_motor.relative_angle;
    control->gimbal_yaw_motor.auto_absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
    control->gimbal_yaw_motor.auto_relative_angle_set = control->gimbal_yaw_motor.relative_angle;
    control->gimbal_yaw_motor.gyro_set = control->gimbal_yaw_motor.gyro;

    control->gimbal_pitch_motor.absolute_angle_set = control->gimbal_pitch_motor.absolute_angle;
    control->gimbal_pitch_motor.relative_angle_set = control->gimbal_pitch_motor.relative_angle;
    control->gimbal_pitch_motor.rc_absolute_angle_set = control->gimbal_pitch_motor.absolute_angle;
    control->gimbal_pitch_motor.rc_relative_angle_set = control->gimbal_pitch_motor.relative_angle;
    control->gimbal_pitch_motor.auto_absolute_angle_set = control->gimbal_pitch_motor.absolute_angle;
    control->gimbal_pitch_motor.auto_relative_angle_set = control->gimbal_pitch_motor.relative_angle;
    control->gimbal_pitch_motor.gyro_set = control->gimbal_pitch_motor.gyro;

    gravity_comp_param.mass_kg = PITCH_GRAVITY_COMP_MASS_KG;
    gravity_comp_param.com_forward_m = PITCH_GRAVITY_COMP_COM_FORWARD_M;
    gravity_comp_param.com_up_m = PITCH_GRAVITY_COMP_COM_UP_M;
    gravity_comp_param.gravity_mps2 = GRAVITY_COMP_DEFAULT_GRAVITY;
    control->gimbal_pitch_gravity_comp.output_scale = OUTPUT_SCALE;
    control->gimbal_pitch_gravity_comp.output_limit = PITCH_GRAVITY_COMP_OUTPUT_LIMIT;
    gravity_comp_init(&control->gimbal_pitch_gravity_comp, &gravity_comp_param);
}

/**
  * @brief          设置云台控制模式
  * @retval         none
  */
__attribute__((used)) void gimbal_set_mode(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    gimbal_behaviour_mode_set(control);
    if (gimbal_pitch_mit_feedback_ready() == 0u)
    {
        gimbal_motor_zero_output(&control->gimbal_pitch_motor);
    }
}

/**
  * @brief          更新云台反馈数据
  * @retval         none
  */
__attribute__((used)) void gimbal_feedback_update(gimbal_control_t *control)
{
    float pitch_motor_pos = 0.0f;
    float yaw_gyro_last = 0.0f;
    float pitch_gyro_last = 0.0f;
    float relative_speed_cmd = 0.0f;
    float relative_speed_alpha = 0.0f;
    float yaw_relative_wrapped = 0.0f;
    uint8_t pitch_feedback_ready = gimbal_pitch_mit_feedback_ready();
    const hwt_imu_info_t *hwt906 = hwt906_get_info();
    const float *imu_gyro = hwt906_get_gimbal_gyro_point();
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;

    if (control == 0)
    {
        return;
    }

    small_yaw_feedback_update();

    if ((control->gimbal_INT_angle_point != 0) &&
        (hwt906 != 0) && (hwt906->angle.yaw_init_flag != 0u))
    {
        control->gimbal_yaw_motor.absolute_angle = hwt906->angle.yaw_total_rad;
        if (pitch_feedback_ready != 0u)
        {
            pitch_motor_pos = MIT_MOTOR_MEASURE[GIMBAL_PITCH_MIT_INDEX].fdb.pos;
        }

        if (gimbal_yaw_online() != 0u &&
            MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].fdb.last_fdb_time != 0U &&
            control->gimbal_yaw_motor.angle_offset_init == 0u)
        {
            control->gimbal_yaw_motor.angle_offset = GIMBAL_ROLL_YAW_SAFE_ANGLE_RAD;

            yaw_relative_wrapped =
                GIMBAL_YAW_FEEDBACK_SIGN * yaw_pitch_direct_wrap_angle(
                    MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].fdb.pos -
                    control->gimbal_yaw_motor.angle_offset);
            control->gimbal_yaw_motor.relative_angle_unwrap_init = 0u;
            control->gimbal_yaw_motor.relative_angle = yaw_relative_wrapped;
            control->gimbal_yaw_motor.relative_angle_unwrapped =
                gimbal_yaw_unwrap_update(&control->gimbal_yaw_motor,
                                         yaw_relative_wrapped);
            control->gimbal_yaw_motor.relative_angle_set = control->gimbal_yaw_motor.relative_angle;
            control->gimbal_yaw_motor.rc_relative_angle_set = control->gimbal_yaw_motor.relative_angle;
            control->gimbal_yaw_motor.auto_relative_angle_set = control->gimbal_yaw_motor.relative_angle;
            control->gimbal_yaw_motor.absolute_angle_set =
                control->gimbal_yaw_motor.absolute_angle;
            control->gimbal_yaw_motor.rc_absolute_angle_set =
                control->gimbal_yaw_motor.absolute_angle;
            control->gimbal_yaw_motor.auto_absolute_angle_set =
                control->gimbal_yaw_motor.absolute_angle;
            control->gimbal_yaw_motor.angle_offset_init = 1u;
        }
        else if (gimbal_yaw_online() != 0u &&
                 control->gimbal_yaw_motor.angle_offset_init != 0u)
        {
            yaw_relative_wrapped =
                GIMBAL_YAW_FEEDBACK_SIGN * yaw_pitch_direct_wrap_angle(
                    MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].fdb.pos -
                    control->gimbal_yaw_motor.angle_offset);
            control->gimbal_yaw_motor.relative_angle = yaw_relative_wrapped;
            control->gimbal_yaw_motor.relative_angle_unwrapped =
                gimbal_yaw_unwrap_update(&control->gimbal_yaw_motor,
                                         yaw_relative_wrapped);
        }

        if (pitch_feedback_ready == 0u)
        {
            control->gimbal_pitch_motor.relative_speed = 0.0f;
            control->gimbal_pitch_motor.relative_speed_update_init = 0u;
        }
        else if (control->gimbal_pitch_motor.angle_offset_init == 0u)
        {
            /* pitch 使用 MIT 绝对位置，angle_offset 仅保留为初始化标志。 */
            control->gimbal_pitch_motor.angle_offset = 0.0f;
            control->gimbal_pitch_motor.absolute_angle = pitch_motor_pos;
            control->gimbal_pitch_motor.relative_angle = pitch_motor_pos;
            control->gimbal_pitch_motor.relative_angle_set = pitch_motor_pos;
            control->gimbal_pitch_motor.rc_relative_angle_set = pitch_motor_pos;
            control->gimbal_pitch_motor.auto_relative_angle_set = pitch_motor_pos;
            control->gimbal_pitch_motor.relative_angle_last = pitch_motor_pos;
            control->gimbal_pitch_motor.relative_speed = 0.0f;
            control->gimbal_pitch_motor.relative_speed_update_init = 1u;
            control->gimbal_pitch_motor.absolute_angle_set =
                control->gimbal_pitch_motor.absolute_angle;
            control->gimbal_pitch_motor.rc_absolute_angle_set =
                control->gimbal_pitch_motor.absolute_angle;
            control->gimbal_pitch_motor.auto_absolute_angle_set =
                control->gimbal_pitch_motor.absolute_angle;
            control->gimbal_pitch_motor.angle_offset_init = 1u;
        }
        else
        {
            /* pitch 反馈和控制目标统一使用 MIT 绝对位置。 */
            control->gimbal_pitch_motor.absolute_angle = pitch_motor_pos;
            control->gimbal_pitch_motor.relative_angle = pitch_motor_pos;

            if (control->gimbal_pitch_motor.relative_speed_update_init == 0u)
            {
                control->gimbal_pitch_motor.relative_angle_last =
                    control->gimbal_pitch_motor.relative_angle;
                control->gimbal_pitch_motor.relative_speed = 0.0f;
                control->gimbal_pitch_motor.relative_speed_update_init = 1u;
            }
            else
            {
                relative_speed_cmd =
                    (control->gimbal_pitch_motor.relative_angle -
                     control->gimbal_pitch_motor.relative_angle_last) / control_dt;
                relative_speed_alpha =
                    gimbal_mit_clamp(PITCH_RELATIVE_SPEED_FILTER_ALPHA, 0.0f, 1.0f);

                control->gimbal_pitch_motor.relative_speed +=
                    relative_speed_alpha *
                    (relative_speed_cmd - control->gimbal_pitch_motor.relative_speed);
                control->gimbal_pitch_motor.relative_angle_last =
                    control->gimbal_pitch_motor.relative_angle;
            }
        }
        if (control->gimbal_pitch_motor.angle_offset_init != 0u)
        {
            /* pitch MIT 绝对位置限幅：1.0 rad 抬头，1.85 rad 低头。 */
            control->gimbal_pitch_motor.min_relative_angle =
                PITCH_HEAD_UP_MOTOR_ANGLE_RAD;
            control->gimbal_pitch_motor.max_relative_angle =
                PITCH_HEAD_DOWN_MOTOR_ANGLE_RAD;
        }
    }

    if (control->gimbal_INT_gyro_point != 0)
    {
        yaw_gyro_last = control->gimbal_yaw_motor.gyro;
        pitch_gyro_last = control->gimbal_pitch_motor.gyro;

        control->gimbal_yaw_motor.gyro =
            control->gimbal_INT_gyro_point[INS_GYRO_Z_ADDRESS_OFFSET];
        control->gimbal_pitch_motor.gyro =
            imu_gyro[HWT_AXIS_PITCH];

        if (control->gimbal_yaw_motor.gyro_update_init == 0u)
        {
            control->gimbal_yaw_motor.gyro_last = control->gimbal_yaw_motor.gyro;
            control->gimbal_yaw_motor.gyro_accel = 0.0f;
            control->gimbal_yaw_motor.gyro_update_init = 1u;
        }
        else
        {
            control->gimbal_yaw_motor.gyro_last = yaw_gyro_last;
            control->gimbal_yaw_motor.gyro_accel =
                (control->gimbal_yaw_motor.gyro - yaw_gyro_last) / control_dt;
        }

        if (control->gimbal_pitch_motor.gyro_update_init == 0u)
        {
            control->gimbal_pitch_motor.gyro_last = control->gimbal_pitch_motor.gyro;
            control->gimbal_pitch_motor.gyro_accel = 0.0f;
            control->gimbal_pitch_motor.gyro_update_init = 1u;
        }
        else
        {
            control->gimbal_pitch_motor.gyro_last = pitch_gyro_last;
            control->gimbal_pitch_motor.gyro_accel =
                (control->gimbal_pitch_motor.gyro - pitch_gyro_last) / control_dt;
        }
    }
}

/**
  * @brief          处理云台模式切换过渡
  * @retval         none
  */
__attribute__((used)) void gimbal_mode_change_control_transit(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    if (control->gimbal_yaw_motor.last_mode != GIMBAL_MOTOR_RAW && control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_RAW)
    {
        control->gimbal_yaw_motor.raw_cmd = control->gimbal_yaw_motor.current_set = control->gimbal_yaw_motor.given_current;
    }
    else if (control->gimbal_yaw_motor.last_mode != GIMBAL_MOTOR_GYRO && control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_GYRO)
    {
        control->gimbal_yaw_motor.absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
        control->gimbal_yaw_motor.rc_absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
        control->gimbal_yaw_motor.auto_absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
        control->gimbal_yaw_motor.gyro_set = control->gimbal_yaw_motor.gyro;
    }
    else if (control->gimbal_yaw_motor.last_mode != GIMBAL_MOTOR_ENCODE && control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        control->gimbal_yaw_motor.relative_angle_set = control->gimbal_yaw_motor.relative_angle;
        control->gimbal_yaw_motor.rc_relative_angle_set = control->gimbal_yaw_motor.relative_angle;
        control->gimbal_yaw_motor.auto_relative_angle_set = control->gimbal_yaw_motor.relative_angle;
        control->gimbal_yaw_motor.gyro_set = control->gimbal_yaw_motor.gyro;
    }
    control->gimbal_yaw_motor.last_mode = control->gimbal_yaw_motor.mode;

    if (gimbal_pitch_mit_feedback_ready() == 0u)
    {
        gimbal_motor_zero_output(&control->gimbal_pitch_motor);
        return;
    }

    if (control->gimbal_pitch_motor.last_mode != GIMBAL_MOTOR_RAW && control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_RAW)
    {
        control->gimbal_pitch_motor.raw_cmd = control->gimbal_pitch_motor.current_set = control->gimbal_pitch_motor.given_current;
    }
    else if (control->gimbal_pitch_motor.last_mode != GIMBAL_MOTOR_ENCODE && control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        control->gimbal_pitch_motor.relative_angle_set = control->gimbal_pitch_motor.relative_angle;
        control->gimbal_pitch_motor.rc_relative_angle_set = control->gimbal_pitch_motor.relative_angle;
        control->gimbal_pitch_motor.auto_relative_angle_set = control->gimbal_pitch_motor.relative_angle;
        control->gimbal_pitch_motor.gyro_set = control->gimbal_pitch_motor.gyro;
    }

    control->gimbal_pitch_motor.last_mode = control->gimbal_pitch_motor.mode;
}

/**
  * @brief          生成云台目标控制量
  * @retval         none
  */
__attribute__((used)) void gimbal_set_control(gimbal_control_t *control)
{
    static float add_yaw = 0.0f;
    static float add_pitch = 0.0f;
    static uint8_t follow_origin_init = 0u;
    uint8_t auto_enable;
    uint8_t yaw_rc_enable;
    uint8_t pitch_rc_enable;
    gimbal_motor_t *yaw_motor;
    gimbal_motor_t *pitch_motor;

    if (control == 0)
    {
        return;
    }

    gimbal_behaviour_control_set(&add_yaw, &add_pitch, control);
    add_yaw *= -GIMBAL_YAW_FEEDBACK_SIGN;
    if (gimbal_roll_motion_is_active() ?
        gimbal_roll_target_is_lifted() : gimbal_roll_is_lifted())
    {
        /* roll 抬起状态和抬起过程中锁定前向零位 ecd=670。 */
        small_yaw_target_rad = 0.0f;
    }
    else
    {
        /* roll 倒下状态和倒下过程中保持小 yaw 目标 ecd=400。 */
        small_yaw_target_rad = small_yaw_ecd_to_angle_rad(SMALL_YAW_LOWER_ECD);
    }
    auto_enable = (uint8_t)((gimbal_behaviour == GIMBAL_AUTO_SENTINEL) &&
                            (auto_aim_is_active() != 0U));
    yaw_rc_enable =
        (uint8_t)((auto_enable == 0u) || (add_yaw != 0.0f));
    pitch_rc_enable =
        (uint8_t)((auto_enable == 0u) || (add_pitch != 0.0f));
    yaw_motor = &control->gimbal_yaw_motor;
    pitch_motor = &control->gimbal_pitch_motor;

    if ((gimbal_behaviour != GIMBAL_RELATIVE_ANGLE) ||
        (gimbal_roll_is_lifted() == 0u))
    {
        follow_origin_init = 0u;
    }
    else if (follow_origin_init == 0u)
    {
        yaw_motor->rc_absolute_angle_set =
            yaw_motor->absolute_angle;
        yaw_motor->auto_absolute_angle_set = yaw_motor->rc_absolute_angle_set;
        follow_origin_init = 1u;
    }

    if (yaw_motor->mode == GIMBAL_MOTOR_RAW)
    {
        auto_aim_reset_delta_accum();
        gimbal_feedforward_clear(yaw_motor);
        yaw_motor->rc_control_enable = 0u;
        yaw_motor->auto_control_enable = 0u;
        yaw_motor->raw_cmd = add_yaw;
    }
    else if (yaw_motor->mode == GIMBAL_MOTOR_GYRO)
    {
        yaw_motor->rc_control_enable = yaw_rc_enable;
        yaw_motor->auto_control_enable = auto_enable;
        gimbal_yaw_absolute_angle_limit(control,
                                        add_yaw,
                                        yaw_rc_enable,
                                        auto_enable);
        if (yaw_rc_enable != 0u)
        {
            gimbal_feedforward_track_target(yaw_motor,
                                            yaw_motor->rc_absolute_angle_set,
                                            GIMBAL_CONTROL_SOURCE_RC);
        }
        if (auto_enable != 0u)
        {
            gimbal_feedforward_track_target(yaw_motor,
                                            yaw_motor->auto_absolute_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }
        else
        {
            gimbal_feedforward_track_target(yaw_motor,
                                            yaw_motor->auto_absolute_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }
    }
    else if (yaw_motor->mode == GIMBAL_MOTOR_ENCODE)
    {
        yaw_motor->rc_control_enable = 0u;
        yaw_motor->auto_control_enable = 0u;
        yaw_motor->rc_relative_angle_set = 0.0f;
        yaw_motor->auto_relative_angle_set = 0.0f;
        yaw_motor->relative_angle_set = 0.0f;
        gimbal_feedforward_clear(yaw_motor);
    }

    if (gimbal_pitch_mit_feedback_ready() == 0u)
    {
        gimbal_motor_zero_output(pitch_motor);
        return;
    }

    if (pitch_motor->mode == GIMBAL_MOTOR_RAW)
    {
        auto_aim_reset_delta_accum();
        gimbal_feedforward_clear(pitch_motor);
        pitch_motor->rc_control_enable = 0u;
        pitch_motor->auto_control_enable = 0u;
        pitch_motor->raw_cmd = add_pitch;
    }
    else if (pitch_motor->mode == GIMBAL_MOTOR_ENCODE)
    {
        if (gimbal_roll_motion_is_active() && !gimbal_roll_target_is_lifted())
        {
            /* roll 倒下期间保持 pitch 低头位置 1.85 rad。 */
            const float down_relative_angle = PITCH_HEAD_DOWN_MOTOR_ANGLE_RAD;
            pitch_motor->rc_relative_angle_set = down_relative_angle;
            pitch_motor->auto_relative_angle_set = down_relative_angle;
        }
        pitch_motor->rc_control_enable = pitch_rc_enable;
        pitch_motor->auto_control_enable = auto_enable;
        gimbal_pitch_relative_angle_limit(control,
                                          add_pitch,
                                          pitch_rc_enable,
                                          auto_enable);
        if (pitch_rc_enable != 0u)
        {
            gimbal_feedforward_track_target(pitch_motor,
                                            pitch_motor->rc_relative_angle_set,
                                            GIMBAL_CONTROL_SOURCE_RC);
        }
        if (auto_enable != 0u)
        {
            gimbal_feedforward_track_target(pitch_motor,
                                            pitch_motor->auto_relative_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }
        else
        {
            gimbal_feedforward_track_target(pitch_motor,
                                            pitch_motor->auto_relative_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }

    }

    if (gimbal_behaviour == GIMBAL_DEBUG)
    {
        gimbal_debug_sine_target_update(control);
    }
    else if (gimbal_behaviour == GIMBAL_AUTO_SENTINEL)
    {
        small_yaw_scan_update(control, auto_enable, add_yaw, add_pitch);
    }
}

/**
  * @brief          生成云台目标控制量
  * @retval         none
  */
__attribute__((used)) void gimbal_control_loop(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    small_yaw_control_allowed = 0U;
    if ((control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_RAW) ||
        (gimbal_hwt906_yaw_ready() == 0u) ||
        (gimbal_yaw_online() == 0u) || toe_is_error(DBUS_TOE))
    {
        /* RAW 无力模式和 yaw 离线保护直接零输出，不进入 yaw 闭环。 */
        gimbal_motor_zero_output(&control->gimbal_yaw_motor);
    }
    else if (control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control->gimbal_yaw_motor);
    }
    else if (control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        gimbal_motor_relative_angle_control(&control->gimbal_yaw_motor);
    }

    if (gimbal_pitch_mit_feedback_ready() == 0u)
    {
        gimbal_motor_zero_output(&control->gimbal_pitch_motor);
    }
    else if (control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_RAW)
    {
        /* RAW 无力模式直接零输出，不进入 pitch 闭环。 */
        gimbal_motor_zero_output(&control->gimbal_pitch_motor);
    }
    else if (control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        gimbal_motor_relative_angle_control(&control->gimbal_pitch_motor);
    }

    small_yaw_control_update();
}

/**
  * @brief          发送云台电机命令
  * @retval         none
  */
__attribute__((used)) void gimbal_send_cmd(gimbal_control_t *control)
{
    static float roll_position_set = 0.0f;
    static uint32_t roll_last_tick = 0U;
    static uint8_t roll_target_init = 0U;
    float yaw_cmd_torque;
    float pitch_cmd_torque;
    uint8_t small_yaw_data[8] = {0};
    int16_t small_yaw_cmd = 0;
    const uint8_t small_yaw_roll_hold = (uint8_t)(
        gimbal_roll_control_is_allowed() &&
        (gimbal_roll_motion_is_active() || !gimbal_roll_is_lifted()));

    if (control == 0)
    {
        return;
    }

    /* Prioritize the small yaw FDCAN2 frame. */
    small_yaw_current_cmd_a = 0.0f;
    small_yaw_current_cmd_raw = 0;
    if ((control->gimbal_yaw_motor.mode != GIMBAL_MOTOR_RAW ||
         small_yaw_roll_hold != 0U) &&
        small_yaw_control_allowed != 0U &&
        !toe_is_error(DBUS_TOE) &&
        DJI_MOTOR_MEASURE[3].last_fdb_time != 0U &&
        (HAL_GetTick() - DJI_MOTOR_MEASURE[3].last_fdb_time) <= 100U)
    {
        small_yaw_current_cmd_a = small_yaw_speed_pid.out;
        small_yaw_cmd = small_yaw_current_to_cmd(small_yaw_current_cmd_a);
        small_yaw_current_cmd_raw = small_yaw_cmd;
    }
    small_yaw_data[0] = (uint8_t)((uint16_t)small_yaw_cmd >> 8);
    small_yaw_data[1] = (uint8_t)small_yaw_cmd;
    small_yaw_can_send_status =
        canx_send_data(&hfdcan2, CAN_SMALL_YAW_CURRENT_CMD_ID, small_yaw_data, 8U);

    if ((control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_RAW) ||
        (gimbal_yaw_online() == 0u) || toe_is_error(DBUS_TOE))
    {
        yaw_can_set_current = 0.0f;
    }
    else
    {
        yaw_can_set_current = control->gimbal_yaw_motor.given_current;
    }

    if ((control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_RAW) ||
        (gimbal_pitch_mit_feedback_ready() == 0u) || toe_is_error(DBUS_TOE))
    {
        pitch_can_set_current = 0.0f;
    }
    else
    {
        pitch_can_set_current = control->gimbal_pitch_motor.given_current;
    }

    yaw_cmd_torque = gimbal_output_to_mit_torque(
        GIMBAL_YAW_FEEDBACK_SIGN * yaw_can_set_current);
    pitch_cmd_torque = gimbal_output_to_mit_torque(pitch_can_set_current);

    CAN_cmd_MIT(&hfdcan1, DM_BIG_YAW_CAN_ID, 0.0f, 0.0f, 0.0f, 0.0f, yaw_cmd_torque);
    CAN_cmd_MIT(&hfdcan2, DM_PIT_CAN_ID, 0.0f, 0.0f, 0.0f, 0.0f, pitch_cmd_torque);
    {
        const MITMeasure_t *roll = &MIT_MOTOR_MEASURE[GIMBAL_ROLL_MIT_INDEX];
        const MITMeasure_t *yaw = &MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX];
        const uint32_t now = HAL_GetTick();
        const uint8_t feedback_ready = (uint8_t)(
            roll->fdb.last_fdb_time != 0U &&
            now - roll->fdb.last_fdb_time <= GIMBAL_ROLL_FEEDBACK_TIMEOUT_MS &&
            yaw->fdb.last_fdb_time != 0U &&
            now - yaw->fdb.last_fdb_time <= GIMBAL_ROLL_FEEDBACK_TIMEOUT_MS &&
            toe_is_error(ROLL_GIMBAL_MOTOR_TOE) == 0U &&
            toe_is_error(YAW_GIMBAL_MOTOR_TOE) == 0U);

        if (gimbal_roll_control_is_allowed() &&
            gimbal_roll_motion_is_active() &&
            feedback_ready != 0U &&
            !toe_is_error(DBUS_TOE))
        {
            const float target = gimbal_roll_target_is_lifted() ?
                                 GIMBAL_ROLL_LIFT_ANGLE_RAD : GIMBAL_ROLL_LOWER_ANGLE_RAD;
            const float max_step = GIMBAL_ROLL_MOVE_SPEED_RAD_S *
                                   (float)(now - roll_last_tick) * 0.001f;
            const float remaining = target - roll_position_set;

            if (roll_target_init == 0U)
            {
                roll_position_set = roll->fdb.pos;
                roll_target_init = 1U;
            }
            else if (fabsf(remaining) <= max_step)
            {
                roll_position_set = target;
            }
            else
            {
                roll_position_set += (remaining > 0.0f) ? max_step : -max_step;
            }
            roll_last_tick = now;

            if (fabsf(target - roll->fdb.pos) <= GIMBAL_ROLL_ARRIVE_ERROR_RAD)
            {
                gimbal_roll_motion_complete();
                roll_target_init = 0U;
                CAN_cmd_MIT(&hfdcan2, DM_ROLL_CAN_ID, target, 0.0f,
                            GIMBAL_ROLL_POSITION_KP, GIMBAL_ROLL_POSITION_KD,
                            gimbal_roll_target_is_lifted() ?
                            GIMBAL_ROLL_LIFT_FEEDFORWARD_TORQUE_NM : 0.0f);
            }
            else
            {
                CAN_cmd_MIT(&hfdcan2, DM_ROLL_CAN_ID, roll_position_set, 0.0f,
                            GIMBAL_ROLL_POSITION_KP, GIMBAL_ROLL_POSITION_KD,
                            gimbal_roll_target_is_lifted() ?
                            GIMBAL_ROLL_LIFT_FEEDFORWARD_TORQUE_NM : 0.0f);
            }
        }
        else
        {
            if (gimbal_roll_motion_is_active())
            {
                gimbal_roll_motion_cancel();
            }
            roll_target_init = 0U;
            roll_last_tick = now;
            if (gimbal_roll_control_is_allowed() &&
                feedback_ready != 0U &&
                !toe_is_error(DBUS_TOE))
            {
                const float hold_target = gimbal_roll_is_lifted() ?
                                          GIMBAL_ROLL_LIFT_ANGLE_RAD :
                                          GIMBAL_ROLL_LOWER_ANGLE_RAD;
                CAN_cmd_MIT(&hfdcan2, DM_ROLL_CAN_ID, hold_target, 0.0f,
                            GIMBAL_ROLL_POSITION_KP, GIMBAL_ROLL_POSITION_KD,
                            gimbal_roll_is_lifted() ?
                            GIMBAL_ROLL_LIFT_FEEDFORWARD_TORQUE_NM : 0.0f);
            }
            else
            {
                CAN_cmd_MIT(&hfdcan2, DM_ROLL_CAN_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            }
        }
    }
}

static void small_yaw_feedback_update(void)
{
    const motor_measure_t *measure = &DJI_MOTOR_MEASURE[3];
    uint16_t ecd = measure->ecd;
    int16_t delta;
    if (measure->last_fdb_time == 0U)
    {
        return;
    }
    if (small_yaw_feedback_init == 0U)
    {
        small_yaw_angle_rad = small_yaw_ecd_to_angle_rad(ecd);
        small_yaw_target_rad = small_yaw_angle_rad;
        small_yaw_last_ecd = ecd;
        small_yaw_feedback_init = 1U;
        return;
    }
    delta = (int16_t)(ecd - small_yaw_last_ecd);
    if (delta > 4096) delta -= 8192;
    if (delta < -4096) delta += 8192;
    small_yaw_angle_rad += (float)delta * (2.0f * YAW_PITCH_DIRECT_PI / 8192.0f);
    small_yaw_last_ecd = ecd;
}

static float small_yaw_ecd_to_angle_rad(uint16_t ecd)
{
    int16_t delta = (int16_t)ecd - (int16_t)SMALL_YAW_FORWARD_ECD;
    if (delta > 4096)
    {
        delta -= 8192;
    }
    if (delta < -4096)
    {
        delta += 8192;
    }
    return (float)delta * (2.0f * YAW_PITCH_DIRECT_PI / 8192.0f);
}

static void gimbal_debug_sine_target_update(gimbal_control_t *control)
{
    const uint32_t phase_ms = HAL_GetTick() % SMALL_YAW_DEBUG_SINE_PERIOD_MS;
    const float sine_value = sinf(2.0f * YAW_PITCH_DIRECT_PI *
                                  (float)phase_ms /
                                  (float)SMALL_YAW_DEBUG_SINE_PERIOD_MS);
    gimbal_motor_t *pitch_motor;

    if (control == 0)
    {
        return;
    }

#if (GIMBAL_DEBUG_SMALL_YAW_SINE_ENABLE != 0U)
    small_yaw_target_rad = gimbal_mit_clamp(
        SMALL_YAW_SCAN_LIMIT_RAD * sine_value,
        -SMALL_YAW_SCAN_LIMIT_RAD,
        SMALL_YAW_SCAN_LIMIT_RAD);
#endif

#if (GIMBAL_DEBUG_PITCH_SINE_ENABLE != 0U)
    pitch_motor = &control->gimbal_pitch_motor;
    {
        const float pitch_center =
            0.5f * (pitch_motor->min_relative_angle + pitch_motor->max_relative_angle);
        float pitch_half_range =
            0.5f * (pitch_motor->max_relative_angle - pitch_motor->min_relative_angle);
        if (pitch_half_range > PITCH_SCAN_LIMIT_RAD)
        {
            pitch_half_range = PITCH_SCAN_LIMIT_RAD;
        }
        const float pitch_target = gimbal_mit_clamp(
            pitch_center + pitch_half_range * sine_value,
            pitch_motor->min_relative_angle,
            pitch_motor->max_relative_angle);

        pitch_motor->rc_relative_angle_set = pitch_target;
        pitch_motor->auto_relative_angle_set = pitch_target;
        pitch_motor->relative_angle_set = pitch_target;
        pitch_motor->absolute_angle_set = pitch_target;
    }
#else
    (void)pitch_motor;
#endif
}

static void small_yaw_scan_update(gimbal_control_t *control, uint8_t auto_enable, float add_yaw, float add_pitch)
{
    const float dt = (float)GIMBAL_CONTROL_TIME * 0.001f;
    if (auto_enable != 0U)
    {
        small_yaw_target_rad = small_yaw_angle_rad + auto_aim_get_yaw_err_rad();
        small_yaw_target_rad = gimbal_mit_clamp(small_yaw_target_rad,
                                                -SMALL_YAW_SCAN_LIMIT_RAD,
                                                SMALL_YAW_SCAN_LIMIT_RAD);
        return;
    }
    if (fabsf(add_yaw) > 0.0001f || fabsf(add_pitch) > 0.0001f)
    {
        return;
    }
    small_yaw_target_rad += (small_yaw_scan_yaw_direction != 0U ? 1.0f : -1.0f) * SMALL_YAW_SCAN_SPEED_RAD_S * dt;
    scan_pitch_target_rad += (small_yaw_scan_pitch_direction != 0U ? 1.0f : -1.0f) * PITCH_SCAN_SPEED_RAD_S * dt;
    if (small_yaw_target_rad >= SMALL_YAW_SCAN_LIMIT_RAD) { small_yaw_target_rad = SMALL_YAW_SCAN_LIMIT_RAD; small_yaw_scan_yaw_direction = 0U; }
    if (small_yaw_target_rad <= -SMALL_YAW_SCAN_LIMIT_RAD) { small_yaw_target_rad = -SMALL_YAW_SCAN_LIMIT_RAD; small_yaw_scan_yaw_direction = 1U; }
    if (scan_pitch_target_rad >= PITCH_SCAN_LIMIT_RAD) { scan_pitch_target_rad = PITCH_SCAN_LIMIT_RAD; small_yaw_scan_pitch_direction = 0U; }
    if (scan_pitch_target_rad <= -PITCH_SCAN_LIMIT_RAD) { scan_pitch_target_rad = -PITCH_SCAN_LIMIT_RAD; small_yaw_scan_pitch_direction = 1U; }
    small_yaw_target_rad = gimbal_mit_clamp(small_yaw_target_rad, -SMALL_YAW_SCAN_LIMIT_RAD, SMALL_YAW_SCAN_LIMIT_RAD);
    control->gimbal_pitch_motor.absolute_angle_set = gimbal_mit_clamp(scan_pitch_target_rad, -PITCH_SCAN_LIMIT_RAD, PITCH_SCAN_LIMIT_RAD);
}

static void small_yaw_control_update(void)
{
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;
    const float speed_feedback_alpha = gimbal_mit_clamp(SMALL_YAW_SPEED_FEEDBACK_ALPHA, 0.0f, 1.0f);
    float angle_error;
    float speed_set;
    float speed_feedback;
    float speed_error;
    float output;
    float integral_candidate;
    uint8_t outward_limit = 0U;

    small_yaw_control_allowed =
        (uint8_t)(gimbal_small_yaw_control_is_allowed() != false);
    if (small_yaw_control_allowed == 0U)
    {
        gimbal_pid_clear(&small_yaw_angle_pid);
        gimbal_pid_clear(&small_yaw_speed_pid);
        small_yaw_speed_feedback_rad_s = 0.0f;
        small_yaw_speed_pid.out = 0.0f;
        return;
    }

    if ((gimbal_behaviour == GIMBAL_DEBUG) &&
        (GIMBAL_DEBUG_SMALL_YAW_SINE_ENABLE != 0U))
    {
        /* 调试目标已在 gimbal_set_control() 中生成。 */
    }
    else if (gimbal_roll_motion_is_active() ?
             gimbal_roll_target_is_lifted() : gimbal_roll_is_lifted())
    {
        small_yaw_target_rad = 0.0f;
    }
    else
    {
        small_yaw_target_rad = small_yaw_ecd_to_angle_rad(SMALL_YAW_LOWER_ECD);
    }

    /* DJI 原始速度反馈与编码器角度增量同向，直接用于阻尼和速度环。 */
    speed_feedback = (float)DJI_MOTOR_MEASURE[3].speed_rpm *
                     (2.0f * YAW_PITCH_DIRECT_PI / 60.0f);
    small_yaw_speed_feedback_rad_s +=
        speed_feedback_alpha * (speed_feedback - small_yaw_speed_feedback_rad_s);

    angle_error = small_yaw_target_rad - small_yaw_angle_rad;
    small_yaw_angle_pid.set = small_yaw_target_rad;
    small_yaw_angle_pid.fdb = small_yaw_angle_rad;
    small_yaw_angle_pid.error[2] = small_yaw_angle_pid.error[1];
    small_yaw_angle_pid.error[1] = small_yaw_angle_pid.error[0];
    small_yaw_angle_pid.error[0] = angle_error;
    small_yaw_angle_pid.Pout = small_yaw_angle_pid.Kp * angle_error;
    small_yaw_angle_pid.Iout = 0.0f;
    small_yaw_angle_pid.Dout = -small_yaw_angle_pid.Kd * small_yaw_speed_feedback_rad_s;
    speed_set = gimbal_mit_clamp(small_yaw_angle_pid.Pout +
                                 small_yaw_angle_pid.Dout,
                                 -small_yaw_angle_pid.max_out,
                                 small_yaw_angle_pid.max_out);
    small_yaw_angle_pid.out = speed_set;

    if ((small_yaw_target_rad >= SMALL_YAW_SCAN_LIMIT_RAD &&
         small_yaw_angle_rad >= SMALL_YAW_SCAN_LIMIT_RAD && speed_set > 0.0f) ||
        (small_yaw_target_rad <= -SMALL_YAW_SCAN_LIMIT_RAD &&
         small_yaw_angle_rad <= -SMALL_YAW_SCAN_LIMIT_RAD && speed_set < 0.0f))
    {
        speed_set = 0.0f;
        outward_limit = 1U;
    }

    if (outward_limit != 0U)
    {
        small_yaw_speed_pid.Iout = 0.0f;
    }

    speed_error = speed_set - small_yaw_speed_feedback_rad_s;
    speed_error = gimbal_mit_clamp(speed_error,
                                   -SMALL_YAW_SPEED_ERROR_LIMIT_RAD_S,
                                   SMALL_YAW_SPEED_ERROR_LIMIT_RAD_S);
    if ((small_yaw_speed_pid.error[0] != 0.0f) &&
        (speed_error * small_yaw_speed_pid.error[0] < 0.0f))
    {
        small_yaw_speed_pid.Iout = 0.0f;
    }
    small_yaw_speed_pid.set = speed_set;
    small_yaw_speed_pid.fdb = small_yaw_speed_feedback_rad_s;
    small_yaw_speed_pid.error[2] = small_yaw_speed_pid.error[1];
    small_yaw_speed_pid.error[1] = small_yaw_speed_pid.error[0];
    small_yaw_speed_pid.error[0] = speed_error;
    small_yaw_speed_pid.Pout = small_yaw_speed_pid.Kp * speed_error;
    integral_candidate = small_yaw_speed_pid.Iout +
                         small_yaw_speed_pid.Ki * speed_error * control_dt;
    integral_candidate = gimbal_mit_clamp(integral_candidate,
                                          -small_yaw_speed_pid.max_iout,
                                          small_yaw_speed_pid.max_iout);
    output = small_yaw_speed_pid.Pout + integral_candidate;
    if ((output < small_yaw_speed_pid.max_out && output > -small_yaw_speed_pid.max_out) ||
        (output >= small_yaw_speed_pid.max_out && speed_error < 0.0f) ||
        (output <= -small_yaw_speed_pid.max_out && speed_error > 0.0f))
    {
        small_yaw_speed_pid.Iout = integral_candidate;
    }
    output = gimbal_mit_clamp(small_yaw_speed_pid.Pout + small_yaw_speed_pid.Iout,
                              -small_yaw_speed_pid.max_out,
                              small_yaw_speed_pid.max_out);
    small_yaw_speed_pid.Dout = 0.0f;
    small_yaw_speed_pid.out = output;
}

static int16_t small_yaw_current_to_cmd(float current_a)
{
    float cmd = current_a * (16384.0f / SMALL_YAW_6020_CURRENT_LIMIT_A);
    if (cmd > 16384.0f) cmd = 16384.0f;
    if (cmd < -16384.0f) cmd = -16384.0f;
    return (int16_t)cmd;
}
#endif
