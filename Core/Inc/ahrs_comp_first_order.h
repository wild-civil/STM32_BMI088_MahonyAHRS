#ifndef AHRS_COMP_FIRST_ORDER_H
#define AHRS_COMP_FIRST_ORDER_H

#include <stdint.h>
#include <stdbool.h>
#include "sensor_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 一阶互补滤波 (First-Order Complementary Filter)
 *
 * 核心思想：
 *   - 高频段：相信陀螺仪（动态响应好，但是积分会漂移）
 *   - 低频段：相信加速度计/磁力计（无漂移，但是噪声大、怕振动）
 *
 * 核心公式（欧拉角形式）：
 *   angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle
 *
 *   其中 alpha = tau / (tau + dt), tau 为时间常数
 * ========================================================================== */

typedef struct {
    /* 滤波参数 */
    float alpha;         /* 陀螺仪权重, 典型值 0.98 (高频信陀螺) */
    float alpha_yaw;     /* 航向角权重, 典型值 0.95 (如果有磁力计则更小) */

    /* 姿态输出 (欧拉角形式), rad */
    float roll;          /* 横滚角, rad */
    float pitch;         /* 俯仰角, rad */
    float yaw;           /* 航向角, rad */

    /* 四元数形式 (可选, 用于和 Mahony 对比) */
    quaternion_t quat;

    /* 采样周期 */
    float dt;            /* s */
    float sample_freq;   /* Hz */

    /* 初始化标志 */
    bool initialized;
} ahrs_comp_first_order_t;

/* ================================ 函数声明 ================================ */

/**
 * @brief 初始化一阶互补滤波
 * @param ahrs AHRS实例指针
 * @param sample_freq 采样频率, Hz
 * @param tau 时间常数, s (推荐 0.5~1.0, 越大越相信陀螺)
 */
void ahrs_comp_first_order_init(ahrs_comp_first_order_t* ahrs,
                                float sample_freq, float tau);

/**
 * @brief 用加速度计和磁力计初始化姿态 (第一次计算时调用)
 * @param ahrs AHRS实例指针
 * @param accel 加速度计, m/s^2
 * @param mag 磁力计, uT (可为NULL, 此时yaw=0)
 */
void ahrs_comp_first_order_calibrate(ahrs_comp_first_order_t* ahrs,
                                     const vector3f_t* accel,
                                     const vector3f_t* mag);

/**
 * @brief 一阶互补滤波更新 (欧拉角形式, 最简单直观)
 * @param ahrs AHRS实例指针
 * @param gyro 陀螺仪, rad/s
 * @param accel 加速度计, m/s^2
 * @param mag 磁力计, uT (可为NULL, 此时yaw仅由陀螺积分)
 */
void ahrs_comp_first_order_update_euler(ahrs_comp_first_order_t* ahrs,
                                        const vector3f_t* gyro,
                                        const vector3f_t* accel,
                                        const vector3f_t* mag);

/**
 * @brief 一阶互补滤波更新 (四元数形式, 无万向锁)
 *        这是 Mahony 的简化版：去掉了积分项 (KI=0), 误差仅用比例 (KP)
 * @param ahrs AHRS实例指针
 * @param gyro 陀螺仪, rad/s
 * @param accel 加速度计, m/s^2
 * @param mag 磁力计, uT (可为NULL)
 */
void ahrs_comp_first_order_update_quat(ahrs_comp_first_order_t* ahrs,
                                       const vector3f_t* gyro,
                                       const vector3f_t* accel,
                                       const vector3f_t* mag);

/**
 * @brief 获取欧拉角
 */
void ahrs_comp_first_order_get_euler(const ahrs_comp_first_order_t* ahrs,
                                     euler_angle_t* euler);

/**
 * @brief 获取四元数
 */
void ahrs_comp_first_order_get_quaternion(const ahrs_comp_first_order_t* ahrs,
                                          quaternion_t* quat);

/**
 * @brief 重置
 */
void ahrs_comp_first_order_reset(ahrs_comp_first_order_t* ahrs);

#ifdef __cplusplus
}
#endif

#endif /* AHRS_COMP_FIRST_ORDER_H */
