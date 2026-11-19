/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c
  * @brief      云台任务调度、弱接口与公共轴运算
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "hwt_imu.h"
#include "bsp_fdcan.h"
#include "detect_task.h"
#include "user_lib.h"
#include <math.h>
#include <string.h>
#include "shoot_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

gimbal_control_t gimbal_control;

static osThreadId_t gimbalTaskHandle = NULL;

/**
  * @brief          云台任务入口
  * @retval         none
  */
static void gimbal_task(void *pvParameters);

/**
  * @brief          创建云台控制任务
  * @retval         none
  */
void GimbalTask_Init(void)
{
    static const osThreadAttr_t gimbalTask_attributes = {
        .name = "gimbalTask",
        .stack_size = 1024 * 4,
        .priority = (osPriority_t) osPriorityHigh,
    };
    gimbalTaskHandle = osThreadNew(gimbal_task, NULL, &gimbalTask_attributes);
    (void)gimbalTaskHandle;
}

/**
  * @brief          云台任务入口
  * @retval         none
  */
static void gimbal_task(void *pvParameters)
{
    TickType_t last_wake_time;

    (void)pvParameters;

    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    gimbal_init(&gimbal_control);
    shoot_init();
    last_wake_time = xTaskGetTickCount();

    while (1)
    {
        gimbal_set_mode(&gimbal_control);
        gimbal_mode_change_control_transit(&gimbal_control);
        gimbal_feedback_update(&gimbal_control);
        gimbal_set_control(&gimbal_control);
        gimbal_control_loop(&gimbal_control);
        shoot_control_loop();
        gimbal_send_cmd(&gimbal_control);

        vTaskDelayUntil(&last_wake_time, GIMBAL_CONTROL_TIME);
    }
}

/**
  * @brief          获取 yaw 电机控制结构体指针
  * @retval         yaw 电机控制结构体只读指针
  */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

/**
  * @brief          获取 pitch 电机控制结构体指针
  * @retval         pitch 电机控制结构体只读指针
  */
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

/**
  * @brief          云台初始化弱接口
  * @retval         none
  */
__weak void gimbal_init(gimbal_control_t *control)
{
    (void)control;
}

/**
  * @brief          云台模式设置弱接口
  * @retval         none
  */
__weak void gimbal_set_mode(gimbal_control_t *control)
{
    (void)control;
}

/**
  * @brief          云台反馈更新弱接口
  * @retval         none
  */
__weak void gimbal_feedback_update(gimbal_control_t *control)
{
    (void)control;
}

/**
  * @brief          云台模式切换过渡弱接口
  * @retval         none
  */
__weak void gimbal_mode_change_control_transit(gimbal_control_t *control)
{
    (void)control;
}

/**
  * @brief          云台控制量设置弱接口
  * @retval         none
  */
__weak void gimbal_set_control(gimbal_control_t *control)
{
    (void)control;
}

/**
  * @brief          云台闭环控制弱接口
  * @retval         none
  */
__weak void gimbal_control_loop(gimbal_control_t *control)
{
    (void)control;
}

/**
  * @brief          云台命令发送弱接口
  * @retval         none
  */
__weak void gimbal_send_cmd(gimbal_control_t *control)
{
    (void)control;
}

/* 公共轴运算：反馈组装、切换初始化、前馈补偿与控制命令计算。 */

/**
  * @brief          清除单轴前馈轨迹状态
  * @retval         none
  */
void gimbal_feedforward_clear(gimbal_motor_t *motor)
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
    motor->auto_target_vel = 0.0f;
}

/**
  * @brief          清除指定控制来源的前馈状态
  * @retval         none
  */
void gimbal_feedforward_clear_source(gimbal_motor_t *motor, gimbal_control_source_e source)
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
        motor->auto_target_vel = 0.0f;
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
void gimbal_feedforward_track_target(gimbal_motor_t *motor, float target_angle, gimbal_control_source_e source)
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

    alpha = fp32_constrain(YAW_REF_VEL_FILTER_ALPHA, 0.0f, 1.0f);
    ref_vel_cmd = (target_angle - *target_last) / control_dt;

    *ref_vel_last = *ref_vel;
    *ref_vel += alpha * (ref_vel_cmd - *ref_vel);

    ref_accel_cmd = (*ref_vel - *ref_vel_last) / control_dt;
    accel_limit = YAW_REF_ACCEL_LIMIT;
    if (accel_limit > 0.0f)
    {
        ref_accel_cmd = fp32_constrain(ref_accel_cmd, -accel_limit, accel_limit);
    }

    *ref_accel = ref_accel_cmd;
    *target_last = target_angle;
}

/** @brief 累计编码角跨零增量，生成连续相对角，单位 rad。 */
static float gimbal_motor_unwrap(gimbal_motor_t *motor, float wrapped_angle)
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
    if (delta > GIMBAL_PI)
    {
        delta -= 2.0f * GIMBAL_PI;
    }
    else if (delta < -GIMBAL_PI)
    {
        delta += 2.0f * GIMBAL_PI;
    }

    motor->relative_angle_unwrapped += delta;
    motor->relative_angle_feedback_last = wrapped_angle;
    return motor->relative_angle_unwrapped;
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
  * @brief          计算单轴静摩擦补偿原始量
  * @retval         静摩擦补偿原始量
  */
static float gimbal_calc_static_friction_comp_raw(const gimbal_motor_t *motor, float angle_error)
{
    float abs_error;
    float comp_scale;
    float deadband;
    float fullband;

    if (motor->axis == GIMBAL_AXIS_PITCH)
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

    if (fullband <= deadband || abs_error >= fullband)
    {
        comp_scale = 1.0f;
    }
    else
    {
        comp_scale = (abs_error - deadband) / (fullband - deadband);
    }

    if (angle_error > 0.0f)
    {
        if (motor->axis == GIMBAL_AXIS_PITCH)
        {
            return GIMBAL_PITCH_STATIC_FRICTION_COMP_UP * comp_scale;
        }
        return GIMBAL_YAW_STATIC_FRICTION_COMP * comp_scale;
    }
    if (angle_error < 0.0f)
    {
        if (motor->axis == GIMBAL_AXIS_PITCH)
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

    if (motor->axis == GIMBAL_AXIS_PITCH)
    {
        alpha = fp32_constrain(GIMBAL_PITCH_STATIC_FRICTION_FILTER_ALPHA, 0.0f, 1.0f);
    }
    else
    {
        alpha = fp32_constrain(GIMBAL_YAW_STATIC_FRICTION_FILTER_ALPHA, 0.0f, 1.0f);
    }
    motor->static_friction_comp += alpha * (comp_cmd - motor->static_friction_comp);

    return motor->static_friction_comp;
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

    if (motor->axis == GIMBAL_AXIS_PITCH)
    {
        velocity_torque = PITCH_VELOCITY_FF_GAIN * motor->ref_vel;
    }

    motor->ff_torque = velocity_torque + motor->inertia_kgm2 * motor->ref_accel;

    return motor->ff_torque;
}

/**
  * @brief          根据角度误差和角速度误差计算控制力矩
  * @retval         控制力矩
  */
static float gimbal_calc_angle_speed_torque(gimbal_motor_t *motor, gimbal_pid_t *pid, float angle_error, float speed_get)
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
    output = fp32_constrain(output, -pid->max_out, pid->max_out);
    pid->out = output;
    motor->pid_torque = output;

    return motor->pid_torque;
}

/** @brief 按轴检查设备反馈时间戳与掉线状态。 */
uint8_t gimbal_feedback_ready(gimbal_axis_e axis)
{
    const uint32_t now = HAL_GetTick();
    const MITMeasure_t *measure;
    if (axis == GIMBAL_AXIS_YAW)
    {
        return (uint8_t)!toe_is_error(YAW_GIMBAL_MOTOR_TOE);
    }
    if (axis == GIMBAL_AXIS_SMALL_YAW)
    {
        const uint32_t tick = DJI_MOTOR_MEASURE[GIMBAL_SMALL_YAW_DJI_INDEX].last_fdb_time;
        return (uint8_t)(tick != 0U && now - tick <= 100U);
    }
    measure = &MIT_MOTOR_MEASURE[axis == GIMBAL_AXIS_PITCH ? GIMBAL_PITCH_MIT_INDEX : GIMBAL_ROLL_MIT_INDEX];
    if (axis == GIMBAL_AXIS_PITCH)
    {
        return (uint8_t)(!toe_is_error(PITCH_GIMBAL_MOTOR_TOE) && measure->fdb.last_fdb_time != 0U && now - measure->fdb.last_fdb_time <= GIMBAL_PITCH_MIT_FDB_TIMEOUT);
    }
    return (uint8_t)(!toe_is_error(ROLL_GIMBAL_MOTOR_TOE) && !toe_is_error(YAW_GIMBAL_MOTOR_TOE) && measure->fdb.last_fdb_time != 0U && now - measure->fdb.last_fdb_time <= GIMBAL_ROLL_FEEDBACK_TIMEOUT_MS && MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].fdb.last_fdb_time != 0U && now - MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].fdb.last_fdb_time <= GIMBAL_ROLL_FEEDBACK_TIMEOUT_MS);
}

/** @brief 用当前反馈初始化单轴各来源目标。 */
void gimbal_motor_target_reset(gimbal_motor_t *motor)
{
    if (motor == NULL)
    {
        return;
    }
    motor->absolute_angle_set = motor->absolute_angle;
    motor->rc_absolute_angle_set = motor->absolute_angle;
    motor->auto_absolute_angle_set = motor->absolute_angle;
    motor->relative_angle_set = motor->relative_angle;
    motor->rc_relative_angle_set = motor->relative_angle;
    motor->auto_relative_angle_set = motor->relative_angle;
    motor->gyro_set = motor->gyro;
}

/** @brief 清除单轴控制历史与命令，执行零输出。 */
void gimbal_motor_zero_output(gimbal_motor_t *motor)
{
    if (motor == NULL)
    {
        return;
    }
    gimbal_feedforward_clear(motor);
    gimbal_pid_clear(&motor->absolute_angle_pid);
    gimbal_pid_clear(&motor->relative_angle_pid);
    gimbal_pid_clear(&motor->speed_pid);
    motor->mode = GIMBAL_MOTOR_RAW;
    motor->last_mode = GIMBAL_MOTOR_RAW;
    motor->control_allowed = 0U;
    motor->raw_cmd = 0.0f;
    motor->current_set = 0.0f;
    motor->output = 0.0f;
    motor->given_current = 0.0f;
    motor->pid_torque = 0.0f;
    motor->static_friction_comp = 0.0f;
    motor->rc_control_enable = 0U;
    motor->auto_control_enable = 0U;
    if (motor->axis == GIMBAL_AXIS_SMALL_YAW)
    {
        motor->gyro = 0.0f;
    }
    else if (motor->axis == GIMBAL_AXIS_ROLL)
    {
        if (gimbal_roll_motion_is_active())
        {
            gimbal_roll_motion_cancel();
        }
        motor->move_target_init = 0U;
        motor->move_last_tick = HAL_GetTick();
        memset(&motor->position_cmd, 0, sizeof(motor->position_cmd));
    }
    else
    {
        gimbal_motor_target_reset(motor);
    }
}

/** @brief 按新模式初始化单轴目标，保留原有控制器历史处理顺序。 */
void gimbal_motor_mode_transit(gimbal_motor_t *motor)
{
    if (motor == NULL)
    {
        return;
    }
    if (motor->last_mode != motor->mode)
    {
        if (motor->mode == GIMBAL_MOTOR_RAW)
        {
            motor->raw_cmd = motor->given_current;
            motor->current_set = motor->given_current;
        }
        else if (motor->mode == GIMBAL_MOTOR_GYRO)
        {
            motor->absolute_angle_set = motor->absolute_angle;
            motor->rc_absolute_angle_set = motor->absolute_angle;
            motor->auto_absolute_angle_set = motor->absolute_angle;
            motor->gyro_set = motor->gyro;
        }
        else if (motor->mode == GIMBAL_MOTOR_ENCODE)
        {
            motor->relative_angle_set = motor->relative_angle;
            motor->rc_relative_angle_set = motor->relative_angle;
            motor->auto_relative_angle_set = motor->relative_angle;
            motor->gyro_set = motor->gyro;
        }
    }
    motor->last_mode = motor->mode;
}

/** @brief 将设备原始数据转换为传入轴的控制反馈。 */
void gimbal_motor_feedback_update(gimbal_motor_t *motor, const gimbal_control_t *control)
{
    if (motor == NULL || control == NULL)
    {
        return;
    }
    const hwt_imu_info_t *imu = hwt906_get_info();
    const float *gyro = hwt906_get_gimbal_gyro_point();
    const float dt = (float)GIMBAL_CONTROL_TIME * 0.001f;
    const uint8_t angle_ready = (uint8_t)(control->gimbal_INT_angle_point != NULL && imu != NULL && imu->angle.yaw_init_flag != 0U);
    const MITMeasure_t *measure;
    float speed;
    float last_gyro = motor->gyro;
    motor->feedback_ready = gimbal_feedback_ready(motor->axis);

    if (motor->axis == GIMBAL_AXIS_SMALL_YAW)
    {
        const motor_measure_t *dji = &DJI_MOTOR_MEASURE[GIMBAL_SMALL_YAW_DJI_INDEX];
        if (dji->last_fdb_time != 0U)
        {
            if (motor->angle_offset_init == 0U)
            {
                motor->relative_angle = gimbal_ecd_angle(dji->ecd);
                motor->relative_angle_set = motor->relative_angle;
                motor->angle_offset_init = 1U;
            }
            else
            {
                int16_t delta = (int16_t)(dji->ecd - motor->last_ecd);
                if (delta > 4096) delta -= 8192;
                if (delta < -4096) delta += 8192;
                motor->relative_angle += (float)delta * (2.0f * GIMBAL_PI / 8192.0f);
            }
            motor->last_ecd = dji->ecd;
            motor->relative_speed = (float)dji->speed_rpm * (2.0f * GIMBAL_PI / 60.0f);
        }
        return;
    }

    measure = &MIT_MOTOR_MEASURE[motor->axis == GIMBAL_AXIS_YAW ? GIMBAL_YAW_MIT_INDEX : (motor->axis == GIMBAL_AXIS_PITCH ? GIMBAL_PITCH_MIT_INDEX : GIMBAL_ROLL_MIT_INDEX)];
    if (motor->axis == GIMBAL_AXIS_ROLL)
    {
        motor->relative_angle = measure->fdb.pos;
        motor->relative_speed = measure->fdb.vel;
        return;
    }
    if (angle_ready && motor->axis == GIMBAL_AXIS_YAW)
    {
        motor->absolute_angle = imu->angle.yaw_total_rad;
        if (motor->feedback_ready && (motor->angle_offset_init || measure->fdb.last_fdb_time != 0U))
        {
            motor->angle_offset = GIMBAL_YAW_FOWARD_ANGLE_RAD;
            motor->relative_angle = GIMBAL_YAW_FEEDBACK_SIGN * rad_format(measure->fdb.pos - motor->angle_offset);
            motor->relative_angle_unwrapped = gimbal_motor_unwrap(motor, motor->relative_angle);
            if (!motor->angle_offset_init)
            {
                gimbal_motor_target_reset(motor);
                motor->angle_offset_init = 1U;
            }
        }
        motor->relative_speed = GIMBAL_YAW_FEEDBACK_SIGN * measure->fdb.vel;
    }
    else if (angle_ready && motor->axis == GIMBAL_AXIS_PITCH)
    {
        if (motor->feedback_ready)
        {
            motor->raw_absolute_angle = measure->fdb.pos;
            motor->raw_relative_angle = measure->fdb.pos;
            motor->raw_relative_speed = measure->fdb.vel;
            motor->relative_angle = PITCH_FEEDBACK_SIGN * measure->fdb.pos + PITCH_FEEDBACK_OFFSET_RAD;
            motor->absolute_angle = motor->relative_angle;
            if (!motor->angle_offset_init)
            {
                gimbal_motor_target_reset(motor);
                motor->relative_angle_last = motor->relative_angle;
                motor->relative_speed = 0.0f;
                motor->relative_speed_update_init = 1U;
                motor->angle_offset_init = 1U;
            }
            else if (!motor->relative_speed_update_init)
            {
                motor->relative_angle_last = motor->relative_angle;
                motor->relative_speed = 0.0f;
                motor->relative_speed_update_init = 1U;
            }
            else
            {
                speed = (motor->relative_angle - motor->relative_angle_last) / dt;
                motor->relative_speed += fp32_constrain(PITCH_RELATIVE_SPEED_FILTER_ALPHA, 0.0f, 1.0f) * (speed - motor->relative_speed);
                motor->relative_angle_last = motor->relative_angle;
            }
        }
        else
        {
            motor->relative_speed = 0.0f;
            motor->relative_speed_update_init = 0U;
        }
        if (motor->angle_offset_init)
        {
            motor->min_relative_angle = fminf(PITCH_HEAD_UP_CONTROL_ANGLE_RAD, PITCH_HEAD_DOWN_CONTROL_ANGLE_RAD);
            motor->max_relative_angle = fmaxf(PITCH_HEAD_UP_CONTROL_ANGLE_RAD, PITCH_HEAD_DOWN_CONTROL_ANGLE_RAD);
        }
    }
    if (control->gimbal_INT_gyro_point != NULL)
    {
        motor->gyro = motor->axis == GIMBAL_AXIS_YAW ? control->gimbal_INT_gyro_point[INS_GYRO_Z_ADDRESS_OFFSET] : PITCH_FEEDBACK_SIGN * gyro[HWT_AXIS_PITCH];
        motor->gyro_last = motor->gyro_update_init ? last_gyro : motor->gyro;
        motor->gyro_accel = motor->gyro_update_init ? (motor->gyro - last_gyro) / dt : 0.0f;
        motor->gyro_update_init = 1U;
    }
}

/** @brief 设置航向目标，保留连续角和遥控前馈。 */
__attribute__((used)) void gimbal_absolute_angle_limit(gimbal_motor_t *motor, float add)
{
    float chassis_yaw;
    if (motor == NULL)
    {
        return;
    }
    chassis_yaw = motor->absolute_angle - motor->relative_angle_unwrapped;
    motor->rc_control_enable = 1U;
    motor->auto_control_enable = 0U;
    motor->rc_relative_angle_set = motor->rc_absolute_angle_set - chassis_yaw + add;
    motor->rc_absolute_angle_set = chassis_yaw + motor->rc_relative_angle_set;
    motor->auto_relative_angle_set = motor->rc_relative_angle_set;
    motor->auto_absolute_angle_set = motor->rc_absolute_angle_set;
    motor->relative_angle_set = motor->rc_relative_angle_set;
    motor->absolute_angle_set = chassis_yaw + motor->relative_angle_set;
    gimbal_feedforward_track_target(motor, motor->rc_absolute_angle_set, GIMBAL_CONTROL_SOURCE_RC);
    gimbal_feedforward_clear_source(motor, GIMBAL_CONTROL_SOURCE_AUTO);
}

/** @brief 规划单轴 MIT 位置命令，保留 roll 抬放到位通知。 */
void gimbal_motor_position_control(gimbal_motor_t *motor)
{
    if (motor == NULL)
    {
        return;
    }
    const uint32_t now = HAL_GetTick();
    const float target = motor->relative_angle_set;
    const float max_step = GIMBAL_ROLL_MOVE_SPEED_RAD_S * (float)(now - motor->move_last_tick) * 0.001f;
    const float remaining = target - motor->position_cmd.position_rad;
    if (gimbal_roll_motion_is_active())
    {
        if (!motor->move_target_init)
        {
            motor->position_cmd.position_rad = motor->relative_angle;
            motor->move_target_init = 1U;
        }
        else if (fabsf(remaining) <= max_step)
        {
            motor->position_cmd.position_rad = target;
        }
        else
        {
            motor->position_cmd.position_rad += remaining > 0.0f ? max_step : -max_step;
        }
        if (fabsf(target - motor->relative_angle) <= GIMBAL_ROLL_ARRIVE_ERROR_RAD)
        {
            gimbal_roll_motion_complete();
            motor->move_target_init = 0U;
            motor->position_cmd.position_rad = target;
        }
    }
    else
    {
        motor->move_target_init = 0U;
        motor->position_cmd.position_rad = target;
    }
    motor->move_last_tick = now;
    motor->position_cmd.speed_rad_s = 0.0f;
    motor->position_cmd.kp = GIMBAL_ROLL_POSITION_KP;
    motor->position_cmd.kd = GIMBAL_ROLL_POSITION_KD;
    motor->position_cmd.torque_nm = target == GIMBAL_ROLL_LIFT_ANGLE_RAD ? GIMBAL_ROLL_LIFT_FEEDFORWARD_TORQUE_NM : 0.0f;
    motor->given_current = motor->position_cmd.torque_nm;
}

/** @brief 执行编码角位置环和速度内环，输出电流 A。 */
static void gimbal_motor_current_control(gimbal_motor_t *motor)
{
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;
    const float speed_feedback_alpha = fp32_constrain(SMALL_YAW_SPEED_FEEDBACK_ALPHA, 0.0f, 1.0f);
    float angle_error;
    float speed_set;
    float speed_feedback;
    float speed_error;
    float output;
    float integral_candidate;
    uint8_t outward_limit = 0U;

    /* DJI 原始速度反馈与编码器角度增量同向，直接用于阻尼和速度环。 */
    speed_feedback = motor->relative_speed;
    motor->gyro += speed_feedback_alpha * (speed_feedback - motor->gyro);

    angle_error = motor->relative_angle_set - motor->relative_angle;
    motor->relative_angle_pid.set = motor->relative_angle_set;
    motor->relative_angle_pid.fdb = motor->relative_angle;
    motor->relative_angle_pid.error[2] = motor->relative_angle_pid.error[1];
    motor->relative_angle_pid.error[1] = motor->relative_angle_pid.error[0];
    motor->relative_angle_pid.error[0] = angle_error;
    motor->relative_angle_pid.Pout = motor->relative_angle_pid.Kp * angle_error;
    motor->relative_angle_pid.Iout = 0.0f;
    motor->relative_angle_pid.Dout = -motor->relative_angle_pid.Kd * motor->gyro;
    speed_set = fp32_constrain(motor->relative_angle_pid.Pout + motor->relative_angle_pid.Dout, -motor->relative_angle_pid.max_out, motor->relative_angle_pid.max_out);
    motor->relative_angle_pid.out = speed_set;

    if ((motor->relative_angle_set >= SMALL_YAW_SCAN_LIMIT_RAD && motor->relative_angle >= SMALL_YAW_SCAN_LIMIT_RAD && speed_set > 0.0f) || (motor->relative_angle_set <= -SMALL_YAW_SCAN_LIMIT_RAD && motor->relative_angle <= -SMALL_YAW_SCAN_LIMIT_RAD && speed_set < 0.0f))
    {
        speed_set = 0.0f;
        outward_limit = 1U;
    }

    if (outward_limit != 0U)
    {
        motor->speed_pid.Iout = 0.0f;
    }

    speed_error = speed_set - motor->gyro;
    speed_error = fp32_constrain(speed_error, -SMALL_YAW_SPEED_ERROR_LIMIT_RAD_S, SMALL_YAW_SPEED_ERROR_LIMIT_RAD_S);
    if ((motor->speed_pid.error[0] != 0.0f) && (speed_error * motor->speed_pid.error[0] < 0.0f))
    {
        motor->speed_pid.Iout = 0.0f;
    }
    motor->speed_pid.set = speed_set;
    motor->speed_pid.fdb = motor->gyro;
    motor->speed_pid.error[2] = motor->speed_pid.error[1];
    motor->speed_pid.error[1] = motor->speed_pid.error[0];
    motor->speed_pid.error[0] = speed_error;
    motor->speed_pid.Pout = motor->speed_pid.Kp * speed_error;
    integral_candidate = motor->speed_pid.Iout + motor->speed_pid.Ki * speed_error * control_dt;
    integral_candidate = fp32_constrain(integral_candidate, -motor->speed_pid.max_iout, motor->speed_pid.max_iout);
    output = motor->speed_pid.Pout + integral_candidate;
    if ((output < motor->speed_pid.max_out && output > -motor->speed_pid.max_out) || (output >= motor->speed_pid.max_out && speed_error < 0.0f) || (output <= -motor->speed_pid.max_out && speed_error > 0.0f))
    {
        motor->speed_pid.Iout = integral_candidate;
    }
    output = fp32_constrain(motor->speed_pid.Pout + motor->speed_pid.Iout, -motor->speed_pid.max_out, motor->speed_pid.max_out);
    motor->speed_pid.Dout = 0.0f;
    motor->speed_pid.out = output;
    motor->gyro_set = speed_set;
    motor->current_set = output;
    motor->output = output;
    motor->given_current = output;
}

/**
  * @brief          执行云台电机绝对角控制
  * @retval         none
  */
__attribute__((used)) void gimbal_motor_absolute_angle_control(gimbal_motor_t *motor)
{
    float angle_get;

    if (motor == 0)
    {
        return;
    }

    motor->ref_vel = ((motor->rc_control_enable != 0u) ? motor->rc_ref_vel : 0.0f) + ((motor->auto_control_enable != 0u) ? motor->auto_ref_vel : 0.0f);
    motor->ref_accel = ((motor->rc_control_enable != 0u) ? motor->rc_ref_accel : 0.0f) + ((motor->auto_control_enable != 0u) ? motor->auto_ref_accel : 0.0f);

    angle_get = motor->absolute_angle;
    gimbal_calc_angle_speed_torque(motor, &motor->absolute_angle_pid, rad_format(motor->absolute_angle_set - angle_get), motor->gyro);

    motor->current_set = motor->pid_torque + gimbal_calc_feedforward(motor);
    motor->output = motor->current_set;
    motor->given_current = fp32_constrain(motor->output, T_MIN, T_MAX);
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

    if (motor->axis == GIMBAL_AXIS_SMALL_YAW)
    {
        gimbal_motor_current_control(motor);
        return;
    }

    motor->ref_vel = ((motor->rc_control_enable != 0u) ? motor->rc_ref_vel : 0.0f) + ((motor->auto_control_enable != 0u) ? motor->auto_ref_vel : 0.0f);
    motor->ref_accel = ((motor->rc_control_enable != 0u) ? motor->rc_ref_accel : 0.0f) + ((motor->auto_control_enable != 0u) ? motor->auto_ref_accel : 0.0f);

    if (motor->axis == GIMBAL_AXIS_YAW)
    {
        gimbal_calc_angle_speed_torque(motor, &motor->relative_angle_pid, rad_format(motor->relative_angle_set - motor->relative_angle), motor->relative_speed);
    }
    else
    {
        gimbal_calc_angle_speed_torque(motor, &motor->relative_angle_pid, motor->relative_angle_set - motor->relative_angle, motor->gyro);
    }

    motor->current_set = motor->pid_torque + gimbal_calc_feedforward(motor);
    motor->output = motor->current_set;
    motor->given_current = fp32_constrain(motor->output, T_MIN, T_MAX);
}

/** @brief 将小 yaw 编码器值转换为前向零点附近的弧度角。 */
float gimbal_ecd_angle(uint16_t ecd)
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
    return (float)delta * (2.0f * GIMBAL_PI / 8192.0f);
}
