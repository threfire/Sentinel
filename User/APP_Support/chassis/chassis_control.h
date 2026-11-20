/**
  * @file       chassis_control.h
  * @brief      全向底盘公共控制运算接口
  * @note       承载反馈辅助、航向闭环、速度规划、轮速控制和电流限幅。
  */
#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "chassis_task.h"

/** @brief 清除整车前馈与制动状态。 */
void chassis_body_feedforward_clear(chassis_move_t *chassis_move_ff);
/** @brief 清除底盘控制历史并置零轮端命令。 */
void chassis_zero_force_clear(chassis_move_t *chassis_move_zero);
/** @brief 计算云台跟随模式底盘角速度给定。 */
fp32 chassis_follow_yaw_control(chassis_move_t *chassis_move_follow, fp32 manual_wz);
/** @brief 计算航向保持模式底盘角速度给定。 */
fp32 chassis_yaw_hold_control(chassis_move_t *chassis_move_yaw_hold, fp32 manual_wz);
/** @brief 清除整车角速度 PI 状态。 */
void chassis_wz_speed_pi_clear(chassis_move_t *chassis_move_clear);
/** @brief 清除角速度目标滤波器状态。 */
void chassis_yaw_set_vel_filter_clear(chassis_move_t *chassis_move_clear);
/** @brief 检查遥控器和四个底盘电机在线状态。 */
uint8_t chassis_control_online(void);
/** @brief 对输入值执行对称限幅。 */
fp32 chassis_limit_abs(fp32 value, fp32 max_abs);
/** @brief 按速度、加速度和 jerk 约束更新规划速度。 */
fp32 chassis_s_curve_update(fp32 cmd, fp32 *plan, fp32 *accel, fp32 max_speed, fp32 max_accel, fp32 max_jerk, fp32 stop_accel, fp32 stop_jerk);
/** @brief 根据车体加速度更新轮端前馈电流。 */
void chassis_body_feedforward_update(chassis_move_t *chassis_move_ff);
/** @brief 根据车体速度误差更新制动补偿。 */
void chassis_body_velocity_brake_update(chassis_move_t *chassis_move_brake);
/** @brief 完成逆解、速度环、动态限流和功率控制。 */
void chassis_wheel_control_update(chassis_move_t *chassis_move_control_loop);

#endif
