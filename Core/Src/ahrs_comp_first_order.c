#include "ahrs_comp_first_order.h"
#include <string.h>
#include <math.h>

/* ================================ 内部辅助宏 ================================ */

#define PI_F  3.14159265358979323846f

static float inv_sqrt_f(float x)
{
    return 1.0f / sqrtf(x);
}

/* 将角度限制到 [-pi, pi] */
static float wrap_pi(float angle)
{
    while (angle >  PI_F) angle -= 2.0f * PI_F;
    while (angle < -PI_F) angle += 2.0f * PI_F;
    return angle;
}

/* ================================ 公共函数实现 ================================ */

void ahrs_comp_first_order_init(ahrs_comp_first_order_t* ahrs,
                                float sample_freq, float tau)
{
    memset(ahrs, 0, sizeof(ahrs_comp_first_order_t));

    ahrs->sample_freq = sample_freq;
    ahrs->dt          = 1.0f / sample_freq;

    /*
     * alpha = tau / (tau + dt)
     *   tau 越大 -> alpha 越接近 1 -> 越相信陀螺仪
     *   tau = 0.5s, dt=0.01s (100Hz) -> alpha = 0.5/0.51 ≈ 0.98
     */
    ahrs->alpha     = tau / (tau + ahrs->dt);
    ahrs->alpha_yaw = tau / (tau + ahrs->dt);  /* 如需磁力计修正可减小此值 */

    /* 初始姿态：假设水平 */
    ahrs->roll  = 0.0f;
    ahrs->pitch = 0.0f;
    ahrs->yaw   = 0.0f;

    ahrs->quat.q0 = 1.0f;
    ahrs->quat.q1 = 0.0f;
    ahrs->quat.q2 = 0.0f;
    ahrs->quat.q3 = 0.0f;

    ahrs->initialized = false;
}

void ahrs_comp_first_order_calibrate(ahrs_comp_first_order_t* ahrs,
                                     const vector3f_t* accel,
                                     const vector3f_t* mag)
{
    /* -------- 1. 由加速度计计算 roll/pitch (静态假设) -------- */
    vector3f_t a = *accel;
    float a_norm = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
    if (a_norm < 1e-6f) {
        /* 加速度计无效 */
        ahrs->roll  = 0.0f;
        ahrs->pitch = 0.0f;
    } else {
        a.x /= a_norm;
        a.y /= a_norm;
        a.z /= a_norm;

        /*
         * 在 FRD 坐标系下 (X前,Y右,Z地), 重力方向为 +Z
         * 静止时:
         *   pitch = asin(-a_x)        （X轴向前加速时 a_x 为正, 此时抬头）
         *   roll  = atan2(a_y, a_z)   （Y轴向右时右滚）
         */
        ahrs->pitch = asinf(-a.x);
        ahrs->roll  = atan2f(a.y, a.z);
    }

    /* -------- 2. 由磁力计计算 yaw (如果有) -------- */
    if (mag != NULL) {
        vector3f_t m = *mag;
        float m_norm = sqrtf(m.x * m.x + m.y * m.y + m.z * m.z);
        if (m_norm > 1e-6f) {
            m.x /= m_norm;
            m.y /= m_norm;
            m.z /= m_norm;

            /* 将磁力计投影到水平面（使用当前 roll/pitch） */
            float cr = cosf(ahrs->roll);
            float sr = sinf(ahrs->roll);
            float cp = cosf(ahrs->pitch);
            float sp = sinf(ahrs->pitch);

            float mx_h = m.x * cp + m.y * sr * sp + m.z * cr * sp;
            float my_h = m.y * cr - m.z * sr;

            /* 在 NED 系下, 地磁北方向, 所以 yaw = atan2(-my_h, mx_h) */
            ahrs->yaw = atan2f(-my_h, mx_h);
        } else {
            ahrs->yaw = 0.0f;
        }
    } else {
        ahrs->yaw = 0.0f;
    }

    /* 同时更新四元数 */
    quaternion_from_euler(&ahrs->quat, ahrs->roll, ahrs->pitch, ahrs->yaw);

    ahrs->initialized = true;
}

/* ----------------------------------------------------------------
 * 版本 1：欧拉角形式的一阶互补滤波
 *   最直观、最容易理解
 *   缺点：pitch ≈ ±90° 时有万向锁
 * ---------------------------------------------------------------- */
void ahrs_comp_first_order_update_euler(ahrs_comp_first_order_t* ahrs,
                                        const vector3f_t* gyro,
                                        const vector3f_t* accel,
                                        const vector3f_t* mag)
{
    float gx = gyro->x;
    float gy = gyro->y;
    float gz = gyro->z;
    float ax = accel->x;
    float ay = accel->y;
    float az = accel->z;
    float dt = ahrs->dt;

    /* 未初始化则先用加速度计/磁力计得到初始姿态 */
    if (!ahrs->initialized) {
        ahrs_comp_first_order_calibrate(ahrs, accel, mag);
        return;
    }

    /* =================================================================
     * 步骤 1：陀螺仪积分 -> 得到"预测"姿态 (高频准确)
     *
     *   gyro_predict = angle_prev + omega * dt
     *
     * 注意：此处直接用 body 角速度近似欧拉角速率
     *   (严格来说需要乘姿态矩阵, 但小角度下近似成立)
     * ================================================================= */
    float gyro_roll  = ahrs->roll  + gx * dt;
    float gyro_pitch = ahrs->pitch + gy * dt;
    float gyro_yaw   = ahrs->yaw   + gz * dt;

    /* =================================================================
     * 步骤 2：加速度计 -> 得到"测量"姿态 (低频无漂移)
     *
     *   accel_roll  = atan2(a_y, a_z)
     *   accel_pitch = asin(-a_x / |a|)
     * ================================================================= */
    float accel_roll  = 0.0f;
    float accel_pitch = 0.0f;
    bool  accel_valid = false;

    float a_norm_sq = ax * ax + ay * ay + az * az;
    if (a_norm_sq > 1e-6f) {
        float a_norm = sqrtf(a_norm_sq);
        float nx = ax / a_norm;
        float ny = ay / a_norm;
        float nz = az / a_norm;

        accel_roll  = atan2f(ny, nz);
        accel_pitch = asinf(-nx);
        accel_valid = true;
    }

    /* =================================================================
     * 步骤 3：磁力计 -> 得到"测量"航向 (如果有)
     * ================================================================= */
    float mag_yaw    = gyro_yaw;   /* 默认用陀螺仪积分值 */
    bool  mag_valid  = false;

    if (mag != NULL) {
        float mx = mag->x;
        float my = mag->y;
        float mz = mag->z;
        float m_norm_sq = mx * mx + my * my + mz * mz;

        if (m_norm_sq > 1e-6f) {
            /* 归一化 */
            float inv_n = inv_sqrt_f(m_norm_sq);
            mx *= inv_n;
            my *= inv_n;
            mz *= inv_n;

            /* 把磁力计转到水平面（用当前姿态近似） */
            float cr = cosf(gyro_roll);
            float sr = sinf(gyro_roll);
            float cp = cosf(gyro_pitch);
            float sp = sinf(gyro_pitch);

            float mx_h = mx * cp + my * sr * sp + mz * cr * sp;
            float my_h = my * cr - mz * sr;

            mag_yaw   = atan2f(-my_h, mx_h);
            mag_valid = true;
        }
    }

    /* =================================================================
     * 步骤 4：互补融合
     *
     *   angle = alpha * gyro_predict + (1 - alpha) * accel_measure
     *
     * 理解：
     *   - alpha 接近 1 -> 高通滤波 (保留陀螺的快速变化)
     *   - (1-alpha)   -> 低通滤波 (保留加速度计的长期稳态)
     *   两者相加就是"全通"，得到互补滤波
     * ================================================================= */
    if (accel_valid) {
        /* 处理 roll 过零点跳变: 让 accel 测量值与当前 gyro 值在同一个周期 */
        float d_roll  = accel_roll  - gyro_roll;
        float d_pitch = accel_pitch - gyro_pitch;
        d_roll  = wrap_pi(d_roll);
        d_pitch = wrap_pi(d_pitch);

        ahrs->roll  = gyro_roll  + (1.0f - ahrs->alpha) * d_roll;
        ahrs->pitch = gyro_pitch + (1.0f - ahrs->alpha) * d_pitch;
    } else {
        /* 加速度计无效, 纯陀螺积分 */
        ahrs->roll  = gyro_roll;
        ahrs->pitch = gyro_pitch;
    }

    if (mag_valid) {
        /* yaw 同样处理过零点 */
        float d_yaw = mag_yaw - gyro_yaw;
        d_yaw = wrap_pi(d_yaw);
        ahrs->yaw = gyro_yaw + (1.0f - ahrs->alpha_yaw) * d_yaw;
    } else {
        /* 没有磁力计, yaw 纯陀螺积分, 会漂移 */
        ahrs->yaw = gyro_yaw;
    }

    /* 归一化到 [-pi, pi] */
    ahrs->roll  = wrap_pi(ahrs->roll);
    ahrs->pitch = wrap_pi(ahrs->pitch);
    ahrs->yaw   = wrap_pi(ahrs->yaw);

    /* 同步更新四元数（用于和 Mahony 对比输出） */
    quaternion_from_euler(&ahrs->quat, ahrs->roll, ahrs->pitch, ahrs->yaw);
}

/* ----------------------------------------------------------------
 * 版本 2：四元数形式的一阶互补滤波
 *   和 Mahony 结构完全一致，只是去掉了 KI 项
 *   -> 本质上是 Mahony 的 KP-only 版本
 *
 *   这样做的好处：
 *     1) 无万向锁
 *     2) 可以直接和 Mahony 对比 "加不加积分项" 的区别
 * ---------------------------------------------------------------- */
void ahrs_comp_first_order_update_quat(ahrs_comp_first_order_t* ahrs,
                                       const vector3f_t* gyro,
                                       const vector3f_t* accel,
                                       const vector3f_t* mag)
{
    float recip_norm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    float gx = gyro->x, gy = gyro->y, gz = gyro->z;
    float ax = accel->x, ay = accel->y, az = accel->z;
    float dt = ahrs->dt;

    /* 将 alpha 映射为等效 KP:
     *   在小角度误差下, angle += (1-alpha) * error * (2/dt)?
     *   这里我们简化：用一个固定 kp = (1-alpha)/dt, 让比例增益随时间常数变化
     *   tau=0.5, dt=0.01 -> alpha=0.98 -> kp ≈ (1-0.98)/0.01 = 2.0
     */
    float kp = (1.0f - ahrs->alpha) / dt * 0.5f;  /* 乘以 0.5 是因为后面用 half 误差 */

    if (!ahrs->initialized) {
        ahrs_comp_first_order_calibrate(ahrs, accel, mag);
        return;
    }

    /* 仅在加速度计有效时计算修正 */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        /* 归一化加速度 */
        recip_norm = inv_sqrt_f(ax * ax + ay * ay + az * az);
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
            recip_norm = inv_sqrt_f(mx * mx + my * my + mz * mz);
            mx *= recip_norm;
            my *= recip_norm;
            mz *= recip_norm;

            /* 参考地磁场方向 */
            hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            bx = sqrtf(hx * hx + hy * hy);
            bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

            /* 估计的磁场方向 */
            halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
            halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
            halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

            /* 误差 = 叉积 */
            halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
            halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
            halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
        } else {
            halfex = ay * halfvz - az * halfvy;
            halfey = az * halfvx - ax * halfvz;
            halfez = ax * halfvy - ay * halfvx;
        }

        /* ----------------------------------------------------------------
         * 关键区别（和 Mahony 对比）：
         *   Mahony:   omega_corrected = omega + Kp * e + Ki * integral(e)
         *   一阶互补: omega_corrected = omega + Kp * e     (没有积分项!)
         * ---------------------------------------------------------------- */
        gx += kp * halfex;
        gy += kp * halfey;
        gz += kp * halfez;
    }

    /* 积分四元数变化率 (和 Mahony 完全一样) */
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = ahrs->quat.q0;
    qb = ahrs->quat.q1;
    qc = ahrs->quat.q2;

    ahrs->quat.q0 += (-qb * gx - qc * gy - ahrs->quat.q3 * gz);
    ahrs->quat.q1 += (qa * gx + qc * gz - ahrs->quat.q3 * gy);
    ahrs->quat.q2 += (qa * gy - qb * gz + ahrs->quat.q3 * gx);
    ahrs->quat.q3 += (qa * gz + qb * gy - qc * gx);

    /* 归一化 */
    recip_norm = inv_sqrt_f(ahrs->quat.q0 * ahrs->quat.q0 +
                            ahrs->quat.q1 * ahrs->quat.q1 +
                            ahrs->quat.q2 * ahrs->quat.q2 +
                            ahrs->quat.q3 * ahrs->quat.q3);
    ahrs->quat.q0 *= recip_norm;
    ahrs->quat.q1 *= recip_norm;
    ahrs->quat.q2 *= recip_norm;
    ahrs->quat.q3 *= recip_norm;

    /* 同步更新欧拉角 */
    euler_angle_t e;
    quaternion_to_euler(&ahrs->quat, &e);
    ahrs->roll  = e.roll;
    ahrs->pitch = e.pitch;
    ahrs->yaw   = e.yaw;
}

void ahrs_comp_first_order_get_euler(const ahrs_comp_first_order_t* ahrs,
                                     euler_angle_t* euler)
{
    euler->roll  = ahrs->roll;
    euler->pitch = ahrs->pitch;
    euler->yaw   = ahrs->yaw;
}

void ahrs_comp_first_order_get_quaternion(const ahrs_comp_first_order_t* ahrs,
                                          quaternion_t* quat)
{
    *quat = ahrs->quat;
}

void ahrs_comp_first_order_reset(ahrs_comp_first_order_t* ahrs)
{
    ahrs->roll  = 0.0f;
    ahrs->pitch = 0.0f;
    ahrs->yaw   = 0.0f;

    ahrs->quat.q0 = 1.0f;
    ahrs->quat.q1 = 0.0f;
    ahrs->quat.q2 = 0.0f;
    ahrs->quat.q3 = 0.0f;

    ahrs->initialized = false;
}
