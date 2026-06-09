#ifndef AHRS_MAHONY_H
#define AHRS_MAHONY_H

#include <stdint.h>
#include <stdbool.h>
#include "sensor_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* 滤波参数 */
    float kp;           // 比例增益
    float ki;           // 积分增益

    /* 状态变量 */
    quaternion_t quat;  // 四元数 (w, x, y, z)
    vector3f_t integral_fb; // 积分反馈项 (用于KI)

    /* 采样频率 */
    float sample_freq;  // Hz
    float dt;           // 采样周期, s

    /* 初始化标志 */
    bool initialized;
} ahrs_mahony_t;

/* ================================ 函数声明 ================================ */

/**
 * @brief 初始化Mahony AHRS
 * @param ahrs AHRS实例指针
 * @param sample_freq 采样频率, Hz
 * @param kp 比例增益 (推荐 1.0~5.0)
 * @param ki 积分增益 (推荐 0.0~0.1)
 */
void ahrs_mahony_init(ahrs_mahony_t* ahrs, float sample_freq, float kp, float ki);

/**
 * @brief 用加速度计和磁力计初始化姿态
 * @param ahrs AHRS实例指针
 * @param accel 加速度计数据 (归一化)
 * @param mag 磁力计数据 (归一化, 可以为NULL)
 */
void ahrs_mahony_calibrate(ahrs_mahony_t* ahrs, const vector3f_t* accel, const vector3f_t* mag);

/**
 * @brief 更新Mahony AHRS (带磁力计)
 * @param ahrs AHRS实例指针
 * @param gyro 陀螺仪数据, rad/s
 * @param accel 加速度计数据, m/s^2
 * @param mag 磁力计数据, uT (可以为NULL)
 */
void ahrs_mahony_update(ahrs_mahony_t* ahrs, const vector3f_t* gyro, 
                         const vector3f_t* accel, const vector3f_t* mag);

/**
 * @brief 更新Mahony AHRS (仅IMU, 无磁力计)
 * @param ahrs AHRS实例指针
 * @param gyro 陀螺仪数据, rad/s
 * @param accel 加速度计数据, m/s^2
 */
void ahrs_mahony_update_imu(ahrs_mahony_t* ahrs, const vector3f_t* gyro, 
                             const vector3f_t* accel);

/**
 * @brief 获取当前姿态四元数
 * @param ahrs AHRS实例指针
 * @param quat 输出四元数
 */
void ahrs_mahony_get_quaternion(const ahrs_mahony_t* ahrs, quaternion_t* quat);

/**
 * @brief 获取当前欧拉角 (Roll-Pitch-Yaw)
 * @param ahrs AHRS实例指针
 * @param euler 输出欧拉角, rad
 */
void ahrs_mahony_get_euler(const ahrs_mahony_t* ahrs, euler_angle_t* euler);

/**
 * @brief 获取完整姿态输出
 * @param ahrs AHRS实例指针
 * @param attitude 输出姿态结构体
 */
void ahrs_mahony_get_attitude(const ahrs_mahony_t* ahrs, attitude_t* attitude);

/**
 * @brief 重置AHRS
 * @param ahrs AHRS实例指针
 */
void ahrs_mahony_reset(ahrs_mahony_t* ahrs);

#ifdef __cplusplus
}
#endif

#endif /* AHRS_MAHONY_H */
