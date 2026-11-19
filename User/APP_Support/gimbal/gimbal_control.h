/**
  * @file       gimbal_control.h
  * @brief      四轴云台阶段接口
  * @note       声明初始化、模式、过渡、反馈、目标、控制和发送七个阶段入口。
  */
#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include "gimbal_task.h"

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
  * @brief          处理云台模式切换过渡
  * @retval         none
  */
void gimbal_mode_change_control_transit(gimbal_control_t *control);

/**
  * @brief          生成云台目标控制量
  * @retval         none
  */
void gimbal_set_control(gimbal_control_t *control);

/**
  * @brief          执行云台闭环控制
  * @retval         none
  */
void gimbal_control_loop(gimbal_control_t *control);

/**
  * @brief          发送云台电机命令
  * @retval         none
  */
void gimbal_send_cmd(gimbal_control_t *control);

#endif
