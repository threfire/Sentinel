/**
  * @file       Omni_chassis.c
  * @brief      十字全向底盘控制阶段入口
  * @note       仅保留初始化、模式、过渡、反馈、目标、控制和发送七个主体阶段。
  */
#include "bsp_usart.h"
#include "bsp_fdcan.h"
#include "chassis_behaviour.h"
#include "chassis_calculate.h"
#include "chassis_control.h"
#include "chassis_power_control.h"
#include "Omni_chassis.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "hwt_imu.h"
#include "gimbal_behaviour.h"
#include "pid.h"
#include "robot_param.h"
#include "remote_control.h"
#include "user_lib.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#if (ROBOT_CHASSIS == ROBOT_CHASSIS_OMNI)

static const fp32 chassis_yaw_pid_param[3] = {
    CHASSIS_ANGLE_PD_KP,
    0.0f,
    CHASSIS_ANGLE_PD_KD
};
static const fp32 chassis_x_order_filter = CHASSIS_ACCEL_X_NUM;
static const fp32 chassis_y_order_filter = CHASSIS_ACCEL_Y_NUM;
static const fp32 chassis_yaw_rate_filter_tau = CHASSIS_YAW_RATE_FILTER_TAU;
static const fp32 chassis_yaw_set_vel_filter_tau = CHASSIS_YAW_RATE_FILTER_TAU;
static const fp32 chassis_wheel_speed_filter_tau = CHASSIS_WHEEL_SPEED_FILTER_TAU;

/**
  * @brief          更新底盘电机反馈、正解车速和姿态反馈
  * @retval         none
  */
__attribute__((used)) void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	if (chassis_move_update == NULL) return;
	fp32 wheel_speed[CHASSIS_MODULE_NUM] = {0.0f};
	fp32 yaw_relative_angle = 0.0f;
	const hwt_imu_info_t *hwt906 = hwt906_get_info();

	for (uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move_update->chassis_3508[i].speed = chassis_move_update->chassis_3508[i].chassis_motor_measure->speed_rpm / MPS_to_RPM;
		first_order_filter_cali(&chassis_move_update->wheel_speed_filter[i], chassis_move_update->chassis_3508[i].speed);
		chassis_move_update->chassis_3508[i].speed_filtered = chassis_move_update->wheel_speed_filter[i].out;
		chassis_move_update->chassis_3508[i].speed_rad_s = (fp32)chassis_move_update->chassis_3508[i].chassis_motor_measure->speed_rpm * CHASSIS_RPM_TO_RAD_PER_SEC;
		chassis_move_update->chassis_3508[i].given_current_a = (fp32)chassis_move_update->chassis_3508[i].chassis_motor_measure->given_current * CHASSIS_CURRENT_CMD_TO_A;
		wheel_speed[i] = chassis_move_update->chassis_3508[i].speed_filtered;
	}

	chas_for_cal(wheel_speed, &chassis_move_update->vx, &chassis_move_update->vy, &chassis_move_update->wz);

	if ((hwt906 != NULL) && (hwt906->angle.yaw_init_flag != 0u))
	{
		if ((chassis_move_update->chassis_yaw_motor != NULL) && (chassis_move_update->chassis_yaw_motor->angle_offset_init != 0u) && (toe_is_error(YAW_GIMBAL_MOTOR_TOE) == 0U))
		{
			/* 云台绝对航向减大 yaw 关节角，得到底盘航向。 */
			yaw_relative_angle = chassis_move_update->chassis_yaw_motor->relative_angle;
			if ((chassis_move_update->chassis_mode == CHASSIS_VECTOR_SPIN) && (chassis_move_update->chassis_yaw_motor->relative_angle_unwrap_init != 0u))
			{
				yaw_relative_angle = chassis_move_update->chassis_yaw_motor->relative_angle_unwrapped;
			}
			chassis_move_update->chassis_yaw = hwt906->angle.yaw_total_rad + CHASSIS_GIMBAL_YAW_FEEDBACK_SIGN * yaw_relative_angle;
			/* 速度反馈必须沿用相对角的位置符号链。 */
			chassis_move_update->chassis_yaw_rate = CHASSIS_YAW_RATE_FEEDBACK_SIGN * hwt906->gyro.radps[HWT_AXIS_YAW] + CHASSIS_GIMBAL_YAW_FEEDBACK_SIGN * GIMBAL_YAW_FEEDBACK_SIGN * MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].fdb.vel;
		}
		chassis_move_update->chassis_pitch = rad_format(hwt906->angle.rad[HWT_AXIS_PITCH]);
		chassis_move_update->chassis_roll = rad_format(hwt906->angle.rad[HWT_AXIS_ROLL]);
	}
	first_order_filter_cali(&chassis_move_update->chassis_yaw_rate_filter, chassis_move_update->chassis_yaw_rate);
	chassis_move_update->chassis_yaw_rate_filtered = chassis_move_update->chassis_yaw_rate_filter.out;

	if ((chassis_move_update->chassis_yaw_motor != NULL) && (chassis_move_update->chassis_yaw_motor->angle_offset_init != 0u) && (toe_is_error(YAW_GIMBAL_MOTOR_TOE) == 0U))
	{
		chassis_move_update->chassis_relative_angle = CHASSIS_GIMBAL_YAW_FEEDBACK_SIGN * rad_format(chassis_move_update->chassis_yaw_motor->relative_angle);
		chassis_move_update->chassis_relative_angle_vel = CHASSIS_GIMBAL_YAW_FEEDBACK_SIGN * GIMBAL_YAW_FEEDBACK_SIGN * MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].fdb.vel;
	}
	else
	{
		chassis_move_update->chassis_relative_angle_vel = 0.0f;
	}
}

/**
  * @brief          初始化底盘控制结构体
  * @retval         none
  */
__attribute__((used)) void chassis_init(chassis_move_t *chassis_move_init)
{
	if (chassis_move_init == NULL) return;

	chassis_move_init->chassis_mode = CHASSIS_VECTOR_NO_MOVE;
	chassis_move_init->spin_start_tick = 0U;
	chassis_move_init->spin_wave_active = 0U;
	chassis_move_init->spin_exit_transition_active = 0U;
	chassis_move_init->chassis_RC = get_remote_control_point();
	chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();

	for(uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move_init->chassis_3508[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
		chassis_move_init->chassis_3508[i].speed_filtered = 0.0f;
		chassis_move_init->model_3508_out[i] = 0.0f;
		chassis_move_init->model_accel[i] = 0.0f;
		chassis_move_init->model_last_speed_set[i] = 0.0f;
		chassis_move_init->speed_pi_iout[i] = 0.0f;
		chassis_move_init->speed_pid_last_error[i] = 0.0f;
		chassis_move_init->stop_brake_active[i] = 0u;
		chassis_move_init->motor_current_limit_a[i] = CHASSIS_CURRENT_BASE_LIMIT_A;
		chassis_move_init->last_motor_current_limit_a[i] = CHASSIS_CURRENT_BASE_LIMIT_A;
		first_order_filter_init(&chassis_move_init->wheel_speed_filter[i], CHASSIS_CONTROL_TIME, &chassis_wheel_speed_filter_tau);
	}
	PID_init(&chassis_move_init->chassis_angle_pid, PID_USUAL, chassis_yaw_pid_param, CHASSIS_ANGLE_PD_MAX_OUT, 0.0f);

	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, &chassis_x_order_filter);
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, &chassis_y_order_filter);

	chassis_move_init->vx_max_speed =  NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init->vy_max_speed =  NORMAL_MAX_CHASSIS_SPEED_Y;
	chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
	chassis_move_init->vx = 0.0f;
	chassis_move_init->vy = 0.0f;
	chassis_move_init->wz = 0.0f;
	chassis_move_init->vx_plan = 0.0f;
	chassis_move_init->vy_plan = 0.0f;
	chassis_move_init->wz_plan = 0.0f;
	chassis_move_init->vx_plan_accel = 0.0f;
	chassis_move_init->vy_plan_accel = 0.0f;
	chassis_move_init->wz_plan_accel = 0.0f;
	chassis_move_init->chassis_relative_angle = 0.0f;
	chassis_move_init->chassis_relative_angle_vel = 0.0f;
	chassis_move_init->chassis_relative_angle_target = 0.0f;
	chassis_move_init->chassis_relative_angle_set = 0.0f;
	chassis_move_init->chassis_relative_angle_set_vel = 0.0f;
	chassis_move_init->chassis_yaw_target = 0.0f;
	chassis_move_init->chassis_yaw_set = 0.0f;
	chassis_move_init->chassis_yaw_set_vel = 0.0f;
	chassis_move_init->chassis_yaw_set_vel_filtered = 0.0f;
	chassis_body_feedforward_clear(chassis_move_init);
	chassis_move_init->chassis_yaw_rate = 0.0f;
	chassis_move_init->chassis_yaw_rate_filtered = 0.0f;
	first_order_filter_init(&chassis_move_init->chassis_yaw_rate_filter, CHASSIS_CONTROL_TIME, &chassis_yaw_rate_filter_tau);
	first_order_filter_init(&chassis_move_init->chassis_yaw_set_vel_filter, CHASSIS_CONTROL_TIME, &chassis_yaw_set_vel_filter_tau);
	chassis_move_init->wz_speed_pi_iout = 0.0f;

	chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          刷新底盘控制模式
  * @retval         none
  */
__attribute__((used)) void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
	if(chassis_move_mode == NULL) return;
	chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief          处理底盘模式切换过渡
  * @retval         none
  */
__attribute__((used)) void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
	uint8_t i;

	if(chassis_move_transit == NULL) return;

	if((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_MOVE) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_MOVE)
	{
		chassis_move_transit->spin_exit_transition_active = 0U;
		chassis_move_transit->chassis_relative_angle_target = 0.0f;
		chassis_move_transit->chassis_relative_angle_set = 0.0f;
		chassis_move_transit->chassis_relative_angle_set_vel = 0.0f;
		chassis_move_transit->chassis_yaw_target = chassis_move_transit->chassis_yaw;
		chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
		chassis_move_transit->chassis_yaw_set_vel = 0.0f;
		chassis_move_transit->wz_set = 0.0f;
		chassis_move_transit->wz_plan = 0.0f;
		chassis_move_transit->wz_plan_accel = 0.0f;
		PID_clear(&chassis_move_transit->chassis_angle_pid);
		chassis_wz_speed_pi_clear(chassis_move_transit);
		chassis_yaw_set_vel_filter_clear(chassis_move_transit);
		chassis_body_feedforward_clear(chassis_move_transit);
		chassis_zero_force_clear(chassis_move_transit);
	}
	else if((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
	{
		if (chassis_move_transit->last_chassis_mode == CHASSIS_VECTOR_SPIN)
		{
			/* 退出小陀螺时从当前实测角速度开始，避免轮速目标瞬间反向。 */
			chassis_move_transit->spin_exit_transition_active = 1U;
			chassis_move_transit->wz_plan = chassis_move_transit->wz;
			chassis_move_transit->wz_plan_accel = 0.0f;
			for (i = 0U; i < CHASSIS_MODULE_NUM; i++)
			{
				chassis_move_transit->speed_pi_iout[i] = 0.0f;
				chassis_move_transit->speed_pid_last_error[i] = chassis_move_transit->chassis_3508[i].speed_filtered;
				chassis_move_transit->model_accel[i] = 0.0f;
				chassis_move_transit->stop_brake_active[i] = 0U;
			}
		}
		else
		{
			chassis_move_transit->spin_exit_transition_active = 0U;
		}
		chassis_move_transit->chassis_relative_angle_target = 0.0f;
		chassis_move_transit->chassis_relative_angle_set = 0.0f;
		chassis_move_transit->chassis_relative_angle_set_vel = 0.0f;
		PID_clear(&chassis_move_transit->chassis_angle_pid);
		chassis_wz_speed_pi_clear(chassis_move_transit);
		chassis_yaw_set_vel_filter_clear(chassis_move_transit);
		chassis_body_feedforward_clear(chassis_move_transit);
	}
	else if((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_YAW_HOLD) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_YAW_HOLD)
	{
		chassis_move_transit->spin_exit_transition_active = 0U;
		chassis_move_transit->chassis_yaw_target = chassis_move_transit->chassis_yaw;
		chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
		chassis_move_transit->chassis_yaw_set_vel = 0.0f;
		PID_clear(&chassis_move_transit->chassis_angle_pid);
		chassis_yaw_set_vel_filter_clear(chassis_move_transit);
		chassis_body_feedforward_clear(chassis_move_transit);
	}
	else if((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_SPIN) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_SPIN)
	{
		chassis_move_transit->spin_exit_transition_active = 0U;
		chassis_move_transit->chassis_relative_angle_target = chassis_move_transit->chassis_yaw;
		chassis_move_transit->chassis_relative_angle_set = chassis_move_transit->chassis_yaw;
		chassis_move_transit->chassis_relative_angle_set_vel = 0.0f;
		PID_clear(&chassis_move_transit->chassis_angle_pid);
		chassis_yaw_set_vel_filter_clear(chassis_move_transit);
		chassis_body_feedforward_clear(chassis_move_transit);
	}

	chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief          计算底盘目标速度并写入控制结构体
  * @retval         none
  */
__attribute__((used)) void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
	fp32 vector[2];

	if (chassis_move_control == NULL) return;

	fp32 vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;

	// 由行为层生成基础速度指令
	chassis_behaviour_control_set(&vx_set, &vy_set, &wz_set, chassis_move_control);

	if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_MOVE)
	{
		chassis_move_control->vx_set = vx_set = 0.0f;
		chassis_move_control->vy_set = vy_set = 0.0f;
		chassis_move_control->wz_set = wz_set = 0.0f;
	}
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
	{
		chassis_move_control->wz_set = chassis_follow_yaw_control(chassis_move_control, wz_set);

		chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	}
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_YAW_HOLD)
	{
		chassis_move_control->wz_set = chassis_yaw_hold_control(chassis_move_control, wz_set);

		chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	}
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_SPIN)
	{
		const hwt_imu_info_t *hwt906 = hwt906_get_info();
		if ((hwt906 == NULL) || (hwt906->angle.yaw_init_flag == 0u) || (chassis_move_control->chassis_yaw_motor == NULL) || (chassis_move_control->chassis_yaw_motor->angle_offset_init == 0u) || (toe_is_error(YAW_GIMBAL_MOTOR_TOE) != 0U) || (MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].fdb.last_fdb_time == 0U))
		{
			chassis_move_control->vx_set = 0.0f;
			chassis_move_control->vy_set = 0.0f;
			chassis_move_control->wz_set = 0.0f;
			return;
		}

		/* 遥控平移以 HWT906 航向为参考，换算到底盘自身坐标。 */
		vector[0] = vx_set;
		vector[1] = vy_set;
		/* 使用 HWT906 解算的底盘航向，将航向坐标系速度换算到底盘坐标系。 */
		vector_rotate(hwt906->angle.yaw_total_rad - chassis_move_control->chassis_yaw, vector);
		chassis_move_control->vx_set = fp32_constrain(vector[0], chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(vector[1], chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
		chassis_move_control->wz_set = wz_set;
	}
}

/**
  * @brief          底盘控制循环
  * @note           无力模式或在线门控失败时直接清控制状态并返回，正常零输入仍保留零速速度环。
  * @param[out]     chassis_move_control_loop: 底盘控制结构体指针
  * @retval         none
  */
__attribute__((used)) void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	uint8_t zero_speed_cmd;

	if (chassis_move_control_loop == NULL) return;

	if ((chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_NO_MOVE) || (chassis_control_online() == 0u))
	{
		/* 无力模式和离线保护只清状态并输出 0，不进入零速速度环。 */
		chassis_zero_force_clear(chassis_move_control_loop);
		return;
	}

	zero_speed_cmd = ((fabsf(chassis_move_control_loop->vx_set) < 0.0001f) && (fabsf(chassis_move_control_loop->vy_set) < 0.0001f) && (fabsf(chassis_move_control_loop->wz_set) < 0.0001f)) ? 1u : 0u;

	if ((zero_speed_cmd != 0u) && (chassis_move_control_loop->spin_exit_transition_active == 0U))
	{
		chassis_move_control_loop->vx_plan = 0.0f;
		chassis_move_control_loop->vy_plan = 0.0f;
		chassis_move_control_loop->wz_plan = 0.0f;
		chassis_move_control_loop->vx_plan_accel = 0.0f;
		chassis_move_control_loop->vy_plan_accel = 0.0f;
		chassis_move_control_loop->wz_plan_accel = 0.0f;
		chassis_body_feedforward_clear(chassis_move_control_loop);
	}
	else
	{
		chassis_move_control_loop->vx_plan = chassis_s_curve_update(chassis_move_control_loop->vx_set, &chassis_move_control_loop->vx_plan, &chassis_move_control_loop->vx_plan_accel, chassis_move_control_loop->vx_max_speed, CHASSIS_MAX_ACCEL, CHASSIS_MAX_JERK, CHASSIS_STOP_DECEL, CHASSIS_STOP_JERK);
		chassis_move_control_loop->vy_plan = chassis_s_curve_update(chassis_move_control_loop->vy_set, &chassis_move_control_loop->vy_plan, &chassis_move_control_loop->vy_plan_accel, chassis_move_control_loop->vy_max_speed, CHASSIS_MAX_ACCEL, CHASSIS_MAX_JERK, CHASSIS_STOP_DECEL, CHASSIS_STOP_JERK);
		if ((chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) || (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_YAW_HOLD))
		{
			if (chassis_move_control_loop->spin_exit_transition_active != 0U)
			{
				const fp32 target_wz_plan = fp32_constrain(chassis_move_control_loop->wz_set, -CHASSIS_WZ_MAX_SPEED, CHASSIS_WZ_MAX_SPEED);
				fp32 last_wz_plan = chassis_move_control_loop->wz_plan;

				chassis_move_control_loop->wz_plan += chassis_limit_abs( (target_wz_plan - last_wz_plan) * CHASSIS_SPIN_EXIT_WZ_FILTER_ALPHA, CHASSIS_WZ_MAX_ACCEL * CHASSIS_CONTROL_TIME);
				chassis_move_control_loop->wz_plan_accel = (chassis_move_control_loop->wz_plan - last_wz_plan) / CHASSIS_CONTROL_TIME;
				if ((fabsf(chassis_move_control_loop->wz_plan - target_wz_plan) <= CHASSIS_SPIN_EXIT_WZ_EPS) && (fabsf(chassis_move_control_loop->wz - chassis_move_control_loop->wz_plan) <= CHASSIS_SPIN_EXIT_WZ_EPS))
				{
					chassis_move_control_loop->wz_plan = target_wz_plan;
					chassis_move_control_loop->wz_plan_accel = 0.0f;
					chassis_move_control_loop->spin_exit_transition_active = 0U;
				}
			}
			else
			{
				chassis_move_control_loop->wz_plan = fp32_constrain(chassis_move_control_loop->wz_set, -CHASSIS_WZ_MAX_SPEED, CHASSIS_WZ_MAX_SPEED);
				chassis_move_control_loop->wz_plan_accel = 0.0f;
			}
		}
		else
		{
			chassis_move_control_loop->wz_plan = chassis_s_curve_update(chassis_move_control_loop->wz_set, &chassis_move_control_loop->wz_plan, &chassis_move_control_loop->wz_plan_accel, CHASSIS_WZ_MAX_SPEED, CHASSIS_WZ_MAX_ACCEL, CHASSIS_WZ_MAX_JERK, CHASSIS_WZ_MAX_ACCEL, CHASSIS_WZ_MAX_JERK);
		}
		if (chassis_move_control_loop->spin_exit_transition_active == 0U)
		{
			chassis_move_control_loop->wz_plan = chassis_limit_wz_by_lateral_accel(chassis_move_control_loop->vx_plan, chassis_move_control_loop->vy_plan, chassis_move_control_loop->wz_plan);
		}
		chassis_body_feedforward_update(chassis_move_control_loop);
	}
	chassis_body_velocity_brake_update(chassis_move_control_loop);
	chassis_wheel_control_update(chassis_move_control_loop);
}

/**
  * @brief          发送底盘电流命令
  * @note           无力模式、DBUS 离线或任一底盘电机离线时发送 0 电流。
  * @param[in]      chassis_move_send: 底盘控制结构体指针
  * @retval         none
  */
__attribute__((used)) void chassis_send_cmd(chassis_move_t *chassis_move_send)
{
	if (chassis_move_send == NULL)
	{
		return;
	}

	if ((chassis_move_send->chassis_mode == CHASSIS_VECTOR_NO_MOVE) || (chassis_control_online() == 0u))
	{
		CAN_cmd_CHASSIS_ALL(0, 0, 0, 0);
		return;
	}

	CAN_cmd_CHASSIS_ALL(chassis_move_send->chassis_3508[0].give_current, chassis_move_send->chassis_3508[1].give_current, chassis_move_send->chassis_3508[2].give_current, chassis_move_send->chassis_3508[3].give_current);
}
#endif
