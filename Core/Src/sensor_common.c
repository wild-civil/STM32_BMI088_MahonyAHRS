#include "sensor_common.h"

/* ================================ 向量运算 ================================ */

float vector3f_norm(const vector3f_t* v)
{
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

void vector3f_normalize(vector3f_t* v)
{
    float norm = vector3f_norm(v);
    if (norm > 1e-6f) {
        v->x /= norm;
        v->y /= norm;
        v->z /= norm;
    }
}

float vector3f_dot(const vector3f_t* a, const vector3f_t* b)
{
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

void vector3f_cross(vector3f_t* out, const vector3f_t* a, const vector3f_t* b)
{
    out->x = a->y * b->z - a->z * b->y;
    out->y = a->z * b->x - a->x * b->z;
    out->z = a->x * b->y - a->y * b->x;
}

/* ================================ 四元数运算 ================================ */

void quaternion_normalize(quaternion_t* q)
{
    float norm = sqrtf(q->q0 * q->q0 + q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3);
    if (norm > 1e-6f) {
        q->q0 /= norm;
        q->q1 /= norm;
        q->q2 /= norm;
        q->q3 /= norm;
    }
}

void quaternion_multiply(quaternion_t* out, const quaternion_t* a, const quaternion_t* b)
{
    float w = a->q0 * b->q0 - a->q1 * b->q1 - a->q2 * b->q2 - a->q3 * b->q3;
    float x = a->q0 * b->q1 + a->q1 * b->q0 + a->q2 * b->q3 - a->q3 * b->q2;
    float y = a->q0 * b->q2 - a->q1 * b->q3 + a->q2 * b->q0 + a->q3 * b->q1;
    float z = a->q0 * b->q3 + a->q1 * b->q2 - a->q2 * b->q1 + a->q3 * b->q0;
    
    out->q0 = w;
    out->q1 = x;
    out->q2 = y;
    out->q3 = z;
}

void quaternion_conjugate(quaternion_t* out, const quaternion_t* q)
{
    out->q0 = q->q0;
    out->q1 = -q->q1;
    out->q2 = -q->q2;
    out->q3 = -q->q3;
}

void quaternion_from_euler(quaternion_t* q, float roll, float pitch, float yaw)
{
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    q->q0 = cr * cp * cy + sr * sp * sy;
    q->q1 = sr * cp * cy - cr * sp * sy;
    q->q2 = cr * sp * cy + sr * cp * sy;
    q->q3 = cr * cp * sy - sr * sp * cy;
}

void quaternion_to_euler(const quaternion_t* q, euler_angle_t* euler)
{
    /* 旋转顺序: Z-Y-X (Yaw-Pitch-Roll), NED坐标系 */
    
    /* Roll (x-axis rotation) */
    float sinr_cosp = 2.0f * (q->q0 * q->q1 + q->q2 * q->q3);
    float cosr_cosp = 1.0f - 2.0f * (q->q1 * q->q1 + q->q2 * q->q2);
    euler->roll = atan2f(sinr_cosp, cosr_cosp);

    /* Pitch (y-axis rotation) */
    float sinp = 2.0f * (q->q0 * q->q2 - q->q3 * q->q1);
    if (fabsf(sinp) >= 1.0f) {
        euler->pitch = copysignf(3.1415926f / 2.0f, sinp);
    } else {
        euler->pitch = asinf(sinp);
    }

    /* Yaw (z-axis rotation) */
    float siny_cosp = 2.0f * (q->q0 * q->q3 + q->q1 * q->q2);
    float cosy_cosp = 1.0f - 2.0f * (q->q2 * q->q2 + q->q3 * q->q3);
    euler->yaw = atan2f(siny_cosp, cosy_cosp);
}

/* ================================ 工具函数 ================================ */

float deg_to_rad(float deg)
{
    return deg * 3.1415926f / 180.0f;
}

float rad_to_deg(float rad)
{
    return rad * 180.0f / 3.1415926f;
}
