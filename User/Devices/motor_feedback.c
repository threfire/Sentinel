/**
 * @file motor_feedback.c
 * @brief MIT 与 DJI 电机反馈解包及接收时间更新。
 */
#include "motor_feedback.h"
#include "robot_param.h"
#include <stddef.h>

MITMeasure_t MIT_MOTOR_MEASURE[5];
motor_measure_t CHASSIS_MOTOR_MEASURE[4];
motor_measure_t DJI_MOTOR_MEASURE[8];

static float mit_vel_lpf[5];

static float motor_feedback_uint_to_float(int value, float min, float max, uint8_t bits)
{
    return (float)value * (max - min) / (float)((1U << bits) - 1U) + min;
}

void motor_feedback_init(void)
{
    uint8_t index;

    for (index = 0U; index < 5U; index++)
    {
        MITMeasure_t *measure = &MIT_MOTOR_MEASURE[index];
        measure->param.P_min = P_MIN;
        measure->param.P_max = P_MAX;
        measure->param.V_min = V_MIN;
        measure->param.V_max = V_MAX;
        measure->param.KP_min = KP_MIN;
        measure->param.KP_max = KP_MAX;
        measure->param.KD_min = KD_MIN;
        measure->param.KD_max = KD_MAX;
        measure->param.T_min = T_MIN;
        measure->param.T_max = T_MAX;
        mit_vel_lpf[index] = 0.0f;
    }
}

void motor_feedback_update_mit(uint8_t index, const uint8_t data[8], uint32_t time_ms)
{
    MITMeasure_t *measure;
    const float alpha = 1.0f;
    float vel_raw;

    if (index >= 5U || data == NULL)
    {
        return;
    }

    measure = &MIT_MOTOR_MEASURE[index];
    measure->fdb.id = data[0] & 0x0F;
    measure->fdb.state = data[0] >> 4;
    measure->fdb.p_int = (data[1] << 8) | data[2];
    measure->fdb.v_int = (data[3] << 4) | (data[4] >> 4);
    measure->fdb.t_int = ((data[4] & 0x0F) << 8) | data[5];
    measure->fdb.pos = motor_feedback_uint_to_float(measure->fdb.p_int, P_MIN, P_MAX, 16U);
    vel_raw = motor_feedback_uint_to_float(measure->fdb.v_int, V_MIN, V_MAX, 12U);
    mit_vel_lpf[index] += alpha * (vel_raw - mit_vel_lpf[index]);
    measure->fdb.vel = mit_vel_lpf[index];
    measure->fdb.tor = motor_feedback_uint_to_float(measure->fdb.t_int, T_MIN, T_MAX, 12U);
    measure->fdb.t_mos = (float)data[6];
    measure->fdb.t_motor = (float)data[7];
    measure->fdb.last_fdb_time = time_ms;
}

void motor_feedback_update_dji(motor_measure_t *measure, const uint8_t data[8], uint32_t time_ms)
{
    if (measure == NULL || data == NULL)
    {
        return;
    }

    measure->last_ecd = measure->ecd;
    measure->ecd = (uint16_t)((data[0] << 8) | data[1]);
    measure->speed_rpm = (int16_t)((data[2] << 8) | data[3]);
    measure->given_current = (int16_t)((data[4] << 8) | data[5]);
    measure->temperate = data[6];
    measure->last_fdb_time = time_ms;
}

motor_measure_t *get_chassis_motor_measure_point(uint8_t index)
{
    return &CHASSIS_MOTOR_MEASURE[index & 0x03U];
}
