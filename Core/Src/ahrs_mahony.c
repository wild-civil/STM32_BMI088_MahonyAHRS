#include "ahrs_mahony.h"
#include <string.h>
#include <math.h>

/* ================================ 内部辅助函数 ================================ */

static float inv_sqrt(float x)
{
    /* 快速开方倒数, 或者直接用 1/sqrtf(x) */
    return 1.0f / sqrtf(x);
}

/* ================================ 公共函数实现 ================================ */

void ahrs_mahony_init(ahrs_mahony_t* ahrs, float sample_freq, float kp, float ki)
{
    memset(ahrs, 0, sizeof(ahrs_mahony_t));
    
    ahrs->kp = kp;
    ahrs->ki = ki;
    ahrs->sample_freq = sample_freq;
    ahrs->dt = 1.0f / sample_freq;
    
    ahrs->quat.q0 = 1.0f;
    ahrs->quat.q1 = 0.0f;
    ahrs->quat.q2 = 0.0f;
    ahrs->quat.q3 = 0.0f;
    
    ahrs->integral_fb.x = 0.0f;
    ahrs->integral_fb.y = 0.0f;
    ahrs->integral_fb.z = 0.0f;
    
    ahrs->initialized = false;
}

void ahrs_mahony_calibrate(ahrs_mahony_t* ahrs, const vector3f_t* accel, const vector3f_t* mag)
{
    /* 先用加速度计初始化 Roll/Pitch */
    vector3f_t a_norm = *accel;
    vector3f_normalize(&a_norm);
    
    float pitch = asinf(-a_norm.x);
    float roll = atan2f(a_norm.y, a_norm.z);
    float yaw = 0.0f;
    
    if (mag != NULL) {
        vector3f_t m_norm = *mag;
        vector3f_normalize(&m_norm);
        
        /* 把磁力计转到水平面 */
        float cos_roll = cosf(roll);
        float sin_roll = sinf(roll);
        float cos_pitch = cosf(pitch);
        float sin_pitch = sinf(pitch);
        
        float mx = m_norm.x * cos_pitch + m_norm.y * sin_roll * sin_pitch + m_norm.z * cos_roll * sin_pitch;
        float my = m_norm.y * cos_roll - m_norm.z * sin_roll;
        
        yaw = atan2f(-my, mx);
    }
    
    /* 从欧拉角初始化四元数 */
    quaternion_from_euler(&ahrs->quat, roll, pitch, yaw);
    ahrs->initialized = true;
}

void ahrs_mahony_update(ahrs_mahony_t* ahrs, const vector3f_t* gyro, 
                         const vector3f_t* accel, const vector3f_t* mag)
{
    float recip_norm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    float gx = gyro->x, gy = gyro->y, gz = gyro->z;
    float ax = accel->x, ay = accel->y, az = accel->z;

    /* 如果没有初始化, 先初始化 */
    if (!ahrs->initialized) {
        ahrs_mahony_calibrate(ahrs, accel, mag);
        return;
    }

    /* 只在加速度计有效时计算反馈 */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        /* 归一化加速度 */
        recip_norm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        /* 辅助变量 */
        q0q0 = ahrs->quat.q0 * ahrs->quat.q0;
        q0q1 = ahrs->quat.q0 * ahrs->quat.q1;
        q0q2 = ahrs->quat.q0 * ahrs->quat.q2;
        q0q3 = ahrs->quat.q0 * ahrs->quat.q3;
        q1q1 = ahrs->quat.q1 * ahrs->quat.q1;
        q1q2 = ahrs->quat.q1 * ahrs->quat.q2;
        q1q3 = ahrs->quat.q1 * ahrs->quat.q3;
        q2q2 = ahrs->quat.q2 * ahrs->quat.q2;
        q2q3 = ahrs->quat.q2 * ahrs->quat.q3;
        q3q3 = ahrs->quat.q3 * ahrs->quat.q3;

        /* 估计的重力方向 */
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;

        if (mag != NULL && !((mag->x == 0.0f) && (mag->y == 0.0f) && (mag->z == 0.0f))) {
            float mx = mag->x, my = mag->y, mz = mag->z;
            
            /* 归一化磁力计 */
            recip_norm = inv_sqrt(mx * mx + my * my + mz * mz);
            mx *= recip_norm;
            my *= recip_norm;
            mz *= recip_norm;

            /* 参考地磁场方向 (水平面) */
            hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            bx = sqrtf(hx * hx + hy * hy);
            bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

            /* 估计的磁场方向 */
            halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
            halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
            halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

            /* 误差是估计值和测量值的叉积 */
            halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
            halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
            halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
        } else {
            /* 只用加速度计, 没有磁力计 */
            halfex = ay * halfvz - az * halfvy;
            halfey = az * halfvx - ax * halfvz;
            halfez = ax * halfvy - ay * halfvx;
        }

        /* 积分反馈 */
        if (ahrs->ki > 0.0f) {
            ahrs->integral_fb.x += ahrs->ki * halfex * ahrs->dt;
            ahrs->integral_fb.y += ahrs->ki * halfey * ahrs->dt;
            ahrs->integral_fb.z += ahrs->ki * halfez * ahrs->dt;
            gx += ahrs->integral_fb.x;
            gy += ahrs->integral_fb.y;
            gz += ahrs->integral_fb.z;
        } else {
            ahrs->integral_fb.x = 0.0f;
            ahrs->integral_fb.y = 0.0f;
            ahrs->integral_fb.z = 0.0f;
        }

        /* 比例反馈 */
        gx += ahrs->kp * halfex;
        gy += ahrs->kp * halfey;
        gz += ahrs->kp * halfez;
    }

    /* 积分四元数变化率 */
    gx *= 0.5f * ahrs->dt;
    gy *= 0.5f * ahrs->dt;
    gz *= 0.5f * ahrs->dt;
    qa = ahrs->quat.q0;
    qb = ahrs->quat.q1;
    qc = ahrs->quat.q2;

    ahrs->quat.q0 += (-qb * gx - qc * gy - ahrs->quat.q3 * gz);
    ahrs->quat.q1 += (qa * gx + qc * gz - ahrs->quat.q3 * gy);
    ahrs->quat.q2 += (qa * gy - qb * gz + ahrs->quat.q3 * gx);
    ahrs->quat.q3 += (qa * gz + qb * gy - qc * gx);

    /* 归一化四元数 */
    recip_norm = inv_sqrt(ahrs->quat.q0 * ahrs->quat.q0 + 
                          ahrs->quat.q1 * ahrs->quat.q1 + 
                          ahrs->quat.q2 * ahrs->quat.q2 + 
                          ahrs->quat.q3 * ahrs->quat.q3);
    ahrs->quat.q0 *= recip_norm;
    ahrs->quat.q1 *= recip_norm;
    ahrs->quat.q2 *= recip_norm;
    ahrs->quat.q3 *= recip_norm;
}

void ahrs_mahony_update_imu(ahrs_mahony_t* ahrs, const vector3f_t* gyro, 
                             const vector3f_t* accel)
{
    /* 调用带磁力计的版本, 但磁力计传NULL */
    ahrs_mahony_update(ahrs, gyro, accel, NULL);
}

void ahrs_mahony_get_quaternion(const ahrs_mahony_t* ahrs, quaternion_t* quat)
{
    *quat = ahrs->quat;
}

void ahrs_mahony_get_euler(const ahrs_mahony_t* ahrs, euler_angle_t* euler)
{
    quaternion_to_euler(&ahrs->quat, euler);
}

void ahrs_mahony_get_attitude(const ahrs_mahony_t* ahrs, attitude_t* attitude)
{
    attitude->quat = ahrs->quat;
    quaternion_to_euler(&ahrs->quat, &attitude->euler);
    /* 这里的零偏估计可以用更复杂的方法, 暂时设为0 */
    attitude->gyro_bias = ahrs->integral_fb;
    attitude->accel_bias.x = 0.0f;
    attitude->accel_bias.y = 0.0f;
    attitude->accel_bias.z = 0.0f;
}

void ahrs_mahony_reset(ahrs_mahony_t* ahrs)
{
    ahrs->quat.q0 = 1.0f;
    ahrs->quat.q1 = 0.0f;
    ahrs->quat.q2 = 0.0f;
    ahrs->quat.q3 = 0.0f;
    
    ahrs->integral_fb.x = 0.0f;
    ahrs->integral_fb.y = 0.0f;
    ahrs->integral_fb.z = 0.0f;
    
    ahrs->initialized = false;
}
