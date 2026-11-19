/**
  * @file       gimbal_control.c
  * @brief      大 yaw、小 yaw、pitch、roll 四轴云台控制
  * @note       按模式、过渡、反馈、目标、控制和发送阶段组织四轴数据流。
  */
#include "gimbal_control.h"
#include "gimbal_behaviour.h"
#include "hwt_imu.h"
#include "bsp_fdcan.h"
#include "detect_task.h"
#include "robot_param.h"
#include "user_lib.h"
#include <string.h>

#if (ROBOT_GIMBAL == ROBOT_GIMBAL_YAW_PITCH_DIRECT)

float yaw_can_set_current = 0.0f;
float pitch_can_set_current = 0.0f;
int16_t shoot_can_set_current = 0;

/**
  * @brief          初始化云台控制结构体
  * @retval         none
  */
__attribute__((used)) void gimbal_init(gimbal_control_t *control)
{
    gravity_comp_param_t gravity_comp_param;
    gimbal_motor_t *motors[4];
    uint8_t i;

    if (control == 0)
    {
        return;
    }

    memset(control, 0, sizeof(*control));
    motors[0] = &control->gimbal_yaw_motor;
    motors[1] = &control->small_yaw.motor;
    motors[2] = &control->gimbal_pitch_motor;
    motors[3] = &control->roll;
    for (i = 0U; i < 4U; i++)
    {
        motors[i]->axis = (gimbal_axis_e)i;
    }

    control->gimbal_rc_ctrl = get_remote_control_point();
    control->gimbal_INT_angle_point = get_INS_angle_point();
    control->gimbal_INT_gyro_point = get_gyro_data_point();

    control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_RAW;
    control->gimbal_yaw_motor.last_mode = GIMBAL_MOTOR_RAW;
    control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_RAW;
    control->gimbal_pitch_motor.last_mode = GIMBAL_MOTOR_RAW;
    control->small_yaw.can_send_status = 1U;

    gimbal_pid_init(&control->gimbal_yaw_motor.absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT);
    gimbal_pid_init(&control->gimbal_yaw_motor.relative_angle_pid, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT);

    gimbal_pid_init(&control->gimbal_pitch_motor.relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT);

    control->gimbal_pitch_motor.max_relative_angle = 0.0f;
    control->gimbal_pitch_motor.min_relative_angle = 0.0f;
    control->gimbal_yaw_motor.inertia_kgm2 = YAW_INERTIA_KGM2;
    control->gimbal_pitch_motor.inertia_kgm2 = PITCH_INERTIA_KGM2;

    gimbal_pid_init(&control->small_yaw.motor.relative_angle_pid, SMALL_YAW_ANGLE_PID_KP, SMALL_YAW_ANGLE_PID_KI, SMALL_YAW_ANGLE_PID_KD, SMALL_YAW_ANGLE_PID_MAX_OUT, 0.0f);
    gimbal_pid_init(&control->small_yaw.motor.speed_pid, SMALL_YAW_SPEED_PID_KP, SMALL_YAW_SPEED_PID_KI, SMALL_YAW_SPEED_PID_KD, SMALL_YAW_SPEED_PID_MAX_OUT, SMALL_YAW_SPEED_PID_MAX_IOUT);
    Motor_ENABLE(&hfdcan1, DM_BIG_YAW_CAN_ID);
    Motor_ENABLE(&hfdcan2, DM_PIT_CAN_ID);
    Motor_ENABLE(&hfdcan2, DM_ROLL_CAN_ID);

    gimbal_feedback_update(control);

    for (i = 0U; i < 4U; i++)
    {
        gimbal_motor_target_reset(motors[i]);
    }

    gravity_comp_param.mass_kg = PITCH_GRAVITY_COMP_MASS_KG;
    gravity_comp_param.com_forward_m = PITCH_GRAVITY_COMP_COM_FORWARD_M;
    gravity_comp_param.com_up_m = PITCH_GRAVITY_COMP_COM_UP_M;
    gravity_comp_param.gravity_mps2 = GRAVITY_COMP_DEFAULT_GRAVITY;
    control->gimbal_pitch_gravity_comp.output_scale = OUTPUT_SCALE;
    control->gimbal_pitch_gravity_comp.output_limit = PITCH_GRAVITY_COMP_OUTPUT_LIMIT;
    gravity_comp_init(&control->gimbal_pitch_gravity_comp, &gravity_comp_param);
}

/** @brief 设置四轴模式与本周期执行许可。 */
__attribute__((used)) void gimbal_set_mode(gimbal_control_t *control)
{
    const hwt_imu_info_t *imu = hwt906_get_info();
    uint8_t remote_ready;
    uint8_t roll_hold;
    if (control == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(control);
    remote_ready = (uint8_t)!toe_is_error(DBUS_TOE);
    roll_hold = (uint8_t)(gimbal_roll_control_is_allowed() && (gimbal_roll_motion_is_active() || !gimbal_roll_is_lifted()));

    control->gimbal_yaw_motor.control_allowed = (uint8_t)(remote_ready && gimbal_feedback_ready(GIMBAL_AXIS_YAW) && imu != NULL && imu->angle.yaw_init_flag != 0U);

    control->small_yaw.motor.control_allowed = (uint8_t)(remote_ready && gimbal_small_yaw_control_is_allowed() && gimbal_feedback_ready(GIMBAL_AXIS_SMALL_YAW) && ((control->gimbal_yaw_motor.mode != GIMBAL_MOTOR_RAW && control->gimbal_yaw_motor.control_allowed) || roll_hold));
    control->small_yaw.motor.mode = control->small_yaw.motor.control_allowed ? GIMBAL_MOTOR_ENCODE : GIMBAL_MOTOR_RAW;

    control->gimbal_pitch_motor.control_allowed = (uint8_t)(remote_ready && gimbal_feedback_ready(GIMBAL_AXIS_PITCH));
    if (!gimbal_feedback_ready(GIMBAL_AXIS_PITCH))
    {
        gimbal_motor_zero_output(&control->gimbal_pitch_motor);
    }

    control->roll.control_allowed = (uint8_t)(remote_ready && gimbal_roll_control_is_allowed() && gimbal_feedback_ready(GIMBAL_AXIS_ROLL));
    control->roll.mode = control->roll.control_allowed ? GIMBAL_MOTOR_MIT_POSITION : GIMBAL_MOTOR_RAW;
}

/** @brief 依次处理四轴模式切换。 */
__attribute__((used)) void gimbal_mode_change_control_transit(gimbal_control_t *control)
{
    if (control == NULL)
    {
        return;
    }
    gimbal_motor_mode_transit(&control->gimbal_yaw_motor);
    gimbal_motor_mode_transit(&control->small_yaw.motor);
    gimbal_motor_mode_transit(&control->gimbal_pitch_motor);
    gimbal_motor_mode_transit(&control->roll);
}

/** @brief 依次更新四轴反馈，完成零点、方向和单位转换。 */
__attribute__((used)) void gimbal_feedback_update(gimbal_control_t *control)
{
    if (control == NULL)
    {
        return;
    }
    gimbal_motor_feedback_update(&control->gimbal_yaw_motor, control);
    gimbal_motor_feedback_update(&control->small_yaw.motor, control);
    gimbal_motor_feedback_update(&control->gimbal_pitch_motor, control);
    gimbal_motor_feedback_update(&control->roll, control);
}

/** @brief 根据行为输入，按四轴模式设置本周期目标。 */
__attribute__((used)) void gimbal_set_control(gimbal_control_t *control)
{
    gimbal_motor_t *motor;
    float hold_angle;
    if (control == NULL)
    {
        return;
    }
    gimbal_behaviour_control_set(&control->target.add_yaw, &control->target.add_pitch, control);
    control->target.add_yaw *= -GIMBAL_YAW_FEEDBACK_SIGN;
    gimbal_target_transit(control);

    motor = &control->gimbal_yaw_motor;
    if (motor->mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_target(motor, control->target.add_yaw);
    }
    else if (motor->mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_absolute_angle_limit(motor, control->target.add_yaw);
    }
    else if (motor->mode == GIMBAL_MOTOR_ENCODE)
    {
        hold_angle = (gimbal_roll_safe_yaw_request_is_active() || gimbal_roll_motion_is_active() || !gimbal_roll_is_lifted()) ? GIMBAL_YAW_FEEDBACK_SIGN * rad_format( GIMBAL_ROLL_YAW_SAFE_ANGLE_RAD - GIMBAL_YAW_FOWARD_ANGLE_RAD) : 0.0f;
        gimbal_relative_angle_limit(motor, hold_angle);
    }

    motor = &control->small_yaw.motor;
    hold_angle = (!gimbal_roll_motion_is_active() && gimbal_roll_is_lifted()) ? 0.0f : gimbal_ecd_angle(SMALL_YAW_LOWER_ECD);
    if (motor->mode == GIMBAL_MOTOR_RAW)
    {
        motor->relative_angle_set = hold_angle;
        gimbal_motor_raw_target(motor, 0.0f);
    }
    else if (motor->mode == GIMBAL_MOTOR_ENCODE)
    {
        gimbal_relative_angle_limit(motor, hold_angle);
    }

    motor = &control->gimbal_pitch_motor;
    if (!gimbal_feedback_ready(GIMBAL_AXIS_PITCH))
    {
        gimbal_motor_zero_output(motor);
    }
    else if (motor->mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_target(motor, control->target.add_pitch);
    }
    else if (motor->mode == GIMBAL_MOTOR_ENCODE)
    {
        gimbal_relative_angle_limit(motor, control->target.add_pitch);
    }

    motor = &control->roll;
    if (motor->mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_target(motor, 0.0f);
    }
    else if (motor->mode == GIMBAL_MOTOR_MIT_POSITION)
    {
        hold_angle = (gimbal_roll_motion_is_active() ? gimbal_roll_target_is_lifted() : gimbal_roll_is_lifted()) ? GIMBAL_ROLL_LIFT_ANGLE_RAD : GIMBAL_ROLL_LOWER_ANGLE_RAD;
        gimbal_relative_angle_limit(motor, hold_angle);
    }
    gimbal_scan_target_update(control);
}

/** @brief 按四轴模式读取目标与反馈，生成最终控制命令。 */
__attribute__((used)) void gimbal_control_loop(gimbal_control_t *control)
{
    gimbal_motor_t *motor;
    if (control == NULL)
    {
        return;
    }
    motor = &control->gimbal_yaw_motor;
    if (motor->mode == GIMBAL_MOTOR_RAW || !motor->control_allowed)
    {
        gimbal_motor_zero_output(motor);
    }
    else if (motor->mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(motor);
    }
    else if (motor->mode == GIMBAL_MOTOR_ENCODE)
    {
        gimbal_motor_relative_angle_control(motor);
    }

    motor = &control->small_yaw.motor;
    if (motor->mode == GIMBAL_MOTOR_RAW || !motor->control_allowed)
    {
        gimbal_motor_zero_output(motor);
    }
    else if (motor->mode == GIMBAL_MOTOR_ENCODE)
    {
        gimbal_motor_relative_angle_control(motor);
    }

    motor = &control->gimbal_pitch_motor;
    if (motor->mode == GIMBAL_MOTOR_RAW || !motor->control_allowed)
    {
        gimbal_motor_zero_output(motor);
    }
    else if (motor->mode == GIMBAL_MOTOR_ENCODE)
    {
        gimbal_motor_relative_angle_control(motor);
    }

    motor = &control->roll;
    if (motor->mode == GIMBAL_MOTOR_RAW || !motor->control_allowed)
    {
        gimbal_motor_zero_output(motor);
    }
    else if (motor->mode == GIMBAL_MOTOR_MIT_POSITION)
    {
        gimbal_motor_position_control(motor);
    }
    gravity_comp_execute(control);
}

/**
  * @brief          发送云台电机命令
  * @retval         none
  */
__attribute__((used)) void gimbal_send_cmd(gimbal_control_t *control)
{
    const gimbal_position_cmd_t *roll;
    float yaw_cmd_torque;
    float pitch_cmd_torque;
    uint8_t small_yaw_data[8] = {0};
    int16_t small_yaw_cmd = 0;

    if (control == 0)
    {
        return;
    }

    roll = &control->roll.position_cmd;

    /* 保留现有顺序，优先发送 CAN2 小 yaw 电流帧。 */
    control->small_yaw.current_cmd_raw = 0;
    if (control->small_yaw.motor.control_allowed != 0U && !toe_is_error(DBUS_TOE) && DJI_MOTOR_MEASURE[GIMBAL_SMALL_YAW_DJI_INDEX].last_fdb_time != 0U && (HAL_GetTick() - DJI_MOTOR_MEASURE[GIMBAL_SMALL_YAW_DJI_INDEX].last_fdb_time) <= 100U)
    {
        small_yaw_cmd = (int16_t)fp32_constrain(control->small_yaw.motor.given_current * (16384.0f / SMALL_YAW_6020_CURRENT_LIMIT_A), -16384.0f, 16384.0f);
        control->small_yaw.current_cmd_raw = small_yaw_cmd;
    }
    small_yaw_data[0] = (uint8_t)((uint16_t)small_yaw_cmd >> 8);
    small_yaw_data[1] = (uint8_t)small_yaw_cmd;
    control->small_yaw.can_send_status = canx_send_data(&hfdcan2, CAN_SMALL_YAW_CURRENT_CMD_ID, small_yaw_data, 8U);

    if ((control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_RAW) || (gimbal_feedback_ready(GIMBAL_AXIS_YAW) == 0u) || toe_is_error(DBUS_TOE))
    {
        yaw_can_set_current = 0.0f;
    }
    else
    {
        yaw_can_set_current = control->gimbal_yaw_motor.given_current;
    }

    if ((control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_RAW) || (gimbal_feedback_ready(GIMBAL_AXIS_PITCH) == 0u) || toe_is_error(DBUS_TOE))
    {
        pitch_can_set_current = 0.0f;
    }
    else
    {
        pitch_can_set_current = control->gimbal_pitch_motor.given_current;
    }

    yaw_cmd_torque = fp32_constrain(GIMBAL_YAW_FEEDBACK_SIGN * yaw_can_set_current, T_MIN, T_MAX);
    pitch_cmd_torque = fp32_constrain(PITCH_FEEDBACK_SIGN * pitch_can_set_current, T_MIN, T_MAX);

    CAN_cmd_MIT(&hfdcan1, DM_BIG_YAW_CAN_ID, 0.0f, 0.0f, 0.0f, 0.0f, yaw_cmd_torque);
    CAN_cmd_MIT(&hfdcan2, DM_PIT_CAN_ID, 0.0f, 0.0f, 0.0f, 0.0f, pitch_cmd_torque);
    CAN_cmd_MIT(&hfdcan2, DM_ROLL_CAN_ID, roll->position_rad, roll->speed_rad_s, roll->kp, roll->kd, roll->torque_nm);
}
#endif
