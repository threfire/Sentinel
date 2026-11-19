/**
 * @file robot_map.h
 * @brief 本车电机总线、CAN ID 与反馈数组槽位映射。
 */
#ifndef ROBOT_MAP_H
#define ROBOT_MAP_H

/* CAN1：大 yaw MIT、四轮 3508。 */
#define DM_BIG_YAW_CAN_ID                   0x02U
#define DM_BIG_YAW_MASTER_ID                0x52U
#define DM_YAW_CAN_ID                       DM_BIG_YAW_CAN_ID
#define DM_YAW_MASTER_ID                    DM_BIG_YAW_MASTER_ID
#define GIMBAL_YAW_MIT_INDEX                1U
#define CAN_CHASSIS_ALL_ID                  0x1FFU
#define CAN_M1_3508_ID                      0x205U
#define CAN_M2_3508_ID                      0x206U
#define CAN_M3_3508_ID                      0x207U
#define CAN_M4_3508_ID                      0x208U
#define CHASSIS_MODULE_NUM                  4U
#define CHASSIS_DJI_MIRROR_BASE_INDEX       4U

/* CAN2：拨弹、pitch、roll MIT 与小 yaw GM6020。 */
#define DM_STRUM_CAN_ID                     0x01U
#define DM_STRUM_MASTER_ID                  0x51U
#define SHOOT_STRUM_MIT_INDEX               0U
#define DM_PIT_CAN_ID                       0x03U
#define DM_PIT_MASTER_ID                    0x53U
#define GIMBAL_PITCH_MIT_INDEX              3U
#define DM_ROLL_CAN_ID                      0x04U
#define DM_ROLL_MASTER_ID                   0x54U
#define GIMBAL_ROLL_MIT_INDEX               2U
#define CAN_SMALL_YAW_ID                    0x205U
#define CAN_SMALL_YAW_CURRENT_CMD_ID        0x1FEU
#define GIMBAL_SMALL_YAW_DJI_INDEX          3U

/* CAN3：三路 M2006/C610 摩擦轮。 */
#define CAN_FRIC1_ID                        0x201U
#define CAN_FRIC2_ID                        0x202U
#define CAN_FRIC3_ID                        0x203U
#define SHOOT_FRIC1_DJI_INDEX               0U
#define SHOOT_FRIC2_DJI_INDEX               1U
#define SHOOT_FRIC3_DJI_INDEX               2U

/* 底盘 3508 反馈槽位与电机正方向。 */
#define WHEEL_REAR_205                      0U
#define WHEEL_RIGHT_206                     1U
#define WHEEL_FRONT_207                     2U
#define WHEEL_LEFT_208                      3U
#define WHEEL_REAR_RIGHT_205                WHEEL_REAR_205
#define WHEEL_FRONT_RIGHT_206               WHEEL_RIGHT_206
#define WHEEL_FRONT_LEFT_207                WHEEL_FRONT_207
#define WHEEL_REAR_LEFT_208                 WHEEL_LEFT_208
#define CHASSIS_WHEEL_205_DIRECTION         1.0f
#define CHASSIS_WHEEL_206_DIRECTION        -1.0f
#define CHASSIS_WHEEL_207_DIRECTION        -1.0f
#define CHASSIS_WHEEL_208_DIRECTION         1.0f

#endif /* ROBOT_MAP_H */
