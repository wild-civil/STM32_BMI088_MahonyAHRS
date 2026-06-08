#ifndef SENSOR_COMMON_H
#define SENSOR_COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================ 数据类型定义 ================================ */
typedef struct {
    float x;
    float y;
    float z;
} vector3f_t;

typedef struct {
    float q0;  // w
    float q1;  // x
    float q2;  // y
    float q3;  // z
} quaternion_t;

typedef struct {
    float roll;    // 横滚角, rad
    float pitch;   // 俯仰角, rad
    float yaw;     // 航向角, rad
} euler_angle_t;

/* ================================ IMU 数据结构 ================================ */
typedef struct {
    vector3f_t gyro;      // 陀螺仪, rad/s
    vector3f_t accel;     // 加速度计, m/s^2
    float temperature;    // 温度, degC
    uint32_t timestamp;   // 时间戳, us
    bool valid;           // 数据有效性
} imu_data_t;

typedef enum {
    IMU_ID_BMI088 = 0,
    IMU_ID_ICM42688P,
    IMU_ID_IIM42652,
    IMU_ID_MAX
} imu_id_t;

typedef struct {
    imu_id_t id;
    imu_data_t data;
    float weight;         // 融合权重, 0~1
    float health_score;   // 健康度评分, 0~1
    bool healthy;         // 是否健康
} imu_instance_t;

/* ================================ 磁力计数据结构 ================================ */
typedef struct {
    vector3f_t mag;       // 磁力计, uT
    uint32_t timestamp;
    bool valid;
} mag_data_t;

typedef enum {
    MAG_ID_IST8310 = 0,
    MAG_ID_RM3100,
    MAG_ID_MAX
} mag_id_t;

typedef struct {
    mag_id_t id;
    mag_data_t data;
    float weight;
    float health_score;
    bool healthy;
} mag_instance_t;

/* ================================ 气压计数据结构 ================================ */
typedef struct {
    float pressure;       // 气压, Pa
    float temperature;    // 温度, degC
    float altitude;       // 高度, m
    uint32_t timestamp;
    bool valid;
} baro_data_t;

typedef enum {
    BARO_ID_BMP581 = 0,
    BARO_ID_MS5611,
    BARO_ID_MAX
} baro_id_t;

typedef struct {
    baro_id_t id;
    baro_data_t data;
    float weight;
    float health_score;
    bool healthy;
} baro_instance_t;

/* ================================ GNSS 数据结构 ================================ */
typedef struct {
    double latitude;      // 纬度, deg
    double longitude;     // 经度, deg
    float altitude;       // 海拔高度, m
    float velocity_n;     // 北向速度, m/s
    float velocity_e;     // 东向速度, m/s
    float velocity_d;     // 地向速度, m/s
    uint8_t fix_type;     // 定位类型: 0=no, 1=2D, 2=3D, 4=RTK float, 5=RTK fixed
    uint8_t satellites;   // 卫星数
    float hdop;           // 水平精度因子
    uint32_t timestamp;   // 时间戳, us
    bool valid;
} gnss_data_t;

/* ================================ 融合后输出 ================================ */
typedef struct {
    quaternion_t quat;    // 四元数
    euler_angle_t euler;  // 欧拉角
    vector3f_t gyro_bias; // 陀螺仪零偏, rad/s
    vector3f_t accel_bias;// 加速度计零偏, m/s^2
} attitude_t;

typedef struct {
    double latitude;
    double longitude;
    float altitude;
    vector3f_t velocity;  // NED 速度, m/s
    attitude_t attitude;
    uint32_t timestamp;
} nav_solution_t;

/* ================================ 函数声明 ================================ */

/* 向量运算 */
float vector3f_norm(const vector3f_t* v);
void vector3f_normalize(vector3f_t* v);
float vector3f_dot(const vector3f_t* a, const vector3f_t* b);
void vector3f_cross(vector3f_t* out, const vector3f_t* a, const vector3f_t* b);

/* 四元数运算 */
void quaternion_normalize(quaternion_t* q);
void quaternion_multiply(quaternion_t* out, const quaternion_t* a, const quaternion_t* b);
void quaternion_conjugate(quaternion_t* out, const quaternion_t* q);
void quaternion_from_euler(quaternion_t* q, float roll, float pitch, float yaw);
void quaternion_to_euler(const quaternion_t* q, euler_angle_t* euler);

/* 工具函数 */
float deg_to_rad(float deg);
float rad_to_deg(float rad);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_COMMON_H */
