#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include <stdint.h>
#include <stdbool.h>
#include "sensor_common.h"
#include "ahrs_mahony.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================ 配置 ================================ */

#define MAX_IMU_COUNT       3   /* 最多支持3个IMU */
#define MAX_MAG_COUNT       2   /* 最多支持2个磁力计 */
#define MAX_BARO_COUNT      2   /* 最多支持2个气压计 */

/* ================================ 健康度指标 ================================ */

typedef struct {
    float accel_noise;      /* 加速度计噪声标准差, m/s^2 */
    float gyro_noise;       /* 陀螺仪噪声标准差, rad/s */
    float consistency;      /* 与融合结果的一致性评分, 0~1 */
    float motion_level;     /* 运动水平, 0~1 (0静止, 1剧烈运动) */
    float overall_score;    /* 综合评分, 0~1 */
} imu_health_metrics_t;

/* ================================ 多IMU融合实例 ================================ */

typedef struct {
    /* IMU实例 */
    imu_instance_t imus[MAX_IMU_COUNT];
    int imu_count;
    
    /* 每个IMU独立的AHRS */
    ahrs_mahony_t ahrs[MAX_IMU_COUNT];
    
    /* 磁力计 */
    mag_instance_t mags[MAX_MAG_COUNT];
    int mag_count;
    
    /* 融合后的姿态 */
    attitude_t fused_attitude;
    
    /* 融合权重 (动态更新) */
    float imu_weights[MAX_IMU_COUNT];
    
    /* 统计窗口 */
    float accel_history[MAX_IMU_COUNT][100][3];  /* 用于计算噪声 */
    int history_index;
    int history_count;
    
    /* 采样频率 */
    float sample_freq;
    bool initialized;
} imu_fusion_t;

/* ================================ 函数声明 ================================ */

/**
 * @brief 初始化多IMU融合
 * @param fusion 融合实例指针
 * @param sample_freq 采样频率, Hz
 */
void imu_fusion_init(imu_fusion_t* fusion, float sample_freq);

/**
 * @brief 注册一个IMU
 * @param fusion 融合实例指针
 * @param imu_id IMU ID
 * @return 成功返回true
 */
bool imu_fusion_register_imu(imu_fusion_t* fusion, imu_id_t imu_id);

/**
 * @brief 注册一个磁力计
 * @param fusion 融合实例指针
 * @param mag_id 磁力计ID
 * @return 成功返回true
 */
bool imu_fusion_register_mag(imu_fusion_t* fusion, mag_id_t mag_id);

/**
 * @brief 更新一个IMU的数据
 * @param fusion 融合实例指针
 * @param imu_id IMU ID
 * @param data IMU数据
 */
void imu_fusion_update_imu(imu_fusion_t* fusion, imu_id_t imu_id, const imu_data_t* data);

/**
 * @brief 更新一个磁力计的数据
 * @param fusion 融合实例指针
 * @param mag_id 磁力计ID
 * @param data 磁力计数据
 */
void imu_fusion_update_mag(imu_fusion_t* fusion, mag_id_t mag_id, const mag_data_t* data);

/**
 * @brief 执行融合更新
 * @param fusion 融合实例指针
 */
void imu_fusion_update(imu_fusion_t* fusion);

/**
 * @brief 获取融合后的姿态
 * @param fusion 融合实例指针
 * @param attitude 输出姿态
 */
void imu_fusion_get_attitude(const imu_fusion_t* fusion, attitude_t* attitude);

/**
 * @brief 获取IMU健康度指标
 * @param fusion 融合实例指针
 * @param imu_id IMU ID
 * @param metrics 输出指标
 */
void imu_fusion_get_health_metrics(const imu_fusion_t* fusion, imu_id_t imu_id, imu_health_metrics_t* metrics);

/**
 * @brief 获取最佳IMU的索引
 * @param fusion 融合实例指针
 * @return 最佳IMU索引
 */
int imu_fusion_get_best_imu_index(const imu_fusion_t* fusion);

/**
 * @brief 重置融合
 * @param fusion 融合实例指针
 */
void imu_fusion_reset(imu_fusion_t* fusion);

#ifdef __cplusplus
}
#endif

#endif /* IMU_FUSION_H */
