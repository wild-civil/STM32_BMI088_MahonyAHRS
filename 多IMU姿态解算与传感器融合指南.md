# 多IMU姿态解算与传感器融合指南

## 目录

1. [概述](#概述)
2. [传感器配置分析](#传感器配置分析)
3. [姿态解算数学基础](#姿态解算数学基础)
4. [Mahony互补滤波：公式与代码对应](#mahony互补滤波公式与代码对应)
5. [Madgwick梯度下降滤波](#madgwick梯度下降滤波)
6. [多IMU融合架构](#多imu融合架构)
7. [开源项目分析](#开源项目分析)
8. [实现建议](#实现建议)

---

## 概述

### 多IMU系统的优势

与单IMU系统相比，多IMU（3个IMU：ICM-42688P + BMI088 + IIM-42652）融合具有以下优势：

1. **冗余性**：一个IMU故障不影响系统工作
2. **精度提升**：通过传感器融合算法降低噪声
3. **动态性能与静态性能平衡**：不同IMU在不同频段表现不同
4. **故障检测与隔离**：通过交叉验证检测异常传感器

### 完整传感器配置

| 传感器类型 | 型号 | 功能 |
|-----------|------|------|
| IMU 1 | ICM-42688P | 6轴（Gyro + Accel） |
| IMU 2 | BMI088 | 6轴（Gyro + Accel） |
| IMU 3 | IIM-42652 | 6轴（Gyro + Accel） |
| 磁力计 1 | RM3100 | 3轴磁强计 |
| 磁力计 2 | IST8310 | 3轴磁强计 |
| 气压计 1 | BMP581 | 压力/高度测量 |
| 气压计 2 | MS5611 | 压力/高度测量 |
| GNSS | UM982 | RTK定位 |

---

## 姿态解算数学基础

### 四元数基本概念

**四元数定义**：
$$ \mathbf{q} = \begin{bmatrix} q_w \\ q_x \\ q_y \\ q_z \end{bmatrix}, \quad \|\mathbf{q}\| = 1 $$

**四元数乘法（⊗）**：
$$ \mathbf{q}_1 \otimes \mathbf{q}_2 = \begin{bmatrix} 
q_{1w}q_{2w} - q_{1x}q_{2x} - q_{1y}q_{2y} - q_{1z}q_{2z} \\
q_{1w}q_{2x} + q_{1x}q_{2w} + q_{1y}q_{2z} - q_{1z}q_{2y} \\
q_{1w}q_{2y} - q_{1x}q_{2z} + q_{1y}q_{2w} + q_{1z}q_{2x} \\
q_{1w}q_{2z} + q_{1x}q_{2y} - q_{1y}q_{2x} + q_{1z}q_{2w} 
\end{bmatrix} $$

### 四元数运动学方程

**核心方程**（将角速度转换为四元数变化率）：
$$ \dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \boldsymbol{\Omega}_b $$
其中：
$$ \boldsymbol{\Omega}_b = \begin{bmatrix} 0 \\ \omega_x \\ \omega_y \\ \omega_z \end{bmatrix} $$

**离散化（前向欧拉）**：
$$ \mathbf{q}_{t+\Delta t} \approx \mathbf{q}_t + \dot{\mathbf{q}} \cdot \Delta t $$
然后归一化：
$$ \mathbf{q}_{t+\Delta t} = \frac{\mathbf{q}_{t+\Delta t}}{\|\mathbf{q}_{t+\Delta t}\|} $$

### 四元数转欧拉角

$$
\begin{cases}
\text{yaw} = \arctan2\left(2(q_w q_z + q_x q_y), 1 - 2(q_y^2 + q_z^2)\right) \\
\text{pitch} = \arcsin\left(2(q_w q_y - q_z q_x)\right) \\
\text{roll} = \arctan2\left(2(q_w q_x + q_y q_z), 1 - 2(q_x^2 + q_y^2)\right)
\end{cases}
$$

---

## Mahony互补滤波：公式与代码对应

### Mahony滤波原理

Mahony滤波是一种互补滤波，利用：
- **陀螺仪**：提供快速的动态响应，但存在漂移
- **加速度计**：提供静态姿态参考，但受运动加速度干扰
- **磁力计**：提供航向参考

### 核心数学公式

#### 1. 误差计算

**重力向量估计（由当前姿态预测）**：
$$ \hat{\mathbf{g}} = \begin{bmatrix} 
2(q_x q_z - q_w q_y) \\
2(q_w q_x + q_y q_z) \\
q_w^2 - q_x^2 - q_y^2 + q_z^2 
\end{bmatrix} $$

**重力误差（叉积）**：
$$ \mathbf{e}_a = \mathbf{a}_{\text{meas}} \times \hat{\mathbf{g}} $$

**磁场误差（类似）**：
$$ \mathbf{e}_m = \mathbf{m}_{\text{meas}} \times \hat{\mathbf{m}} $$

**总误差**：
$$ \mathbf{e} = \mathbf{e}_a + \mathbf{e}_m $$

#### 2. PI控制器

**比例项**：$ \delta\omega_p = K_p \cdot \mathbf{e} $

**积分项**：$ \mathbf{e}_i = \mathbf{e}_i + \mathbf{e} \cdot \Delta t $，$ \delta\omega_i = K_i \cdot \mathbf{e}_i $

**修正后的角速度**：
$$ \omega_{\text{corr}} = \omega_{\text{meas}} + \delta\omega_p + \delta\omega_i $$

#### 3. 四元数更新

使用修正后的角速度进行积分：
$$ \dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \begin{bmatrix} 0 \\ \omega_{\text{corr},x} \\ \omega_{\text{corr},y} \\ \omega_{\text{corr},z} \end{bmatrix} $$

### 代码实现（来自 MahonyAHRS.c）

#### 完整的 MahonyAHRSupdate 函数

```c
void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, 
                      float ax, float ay, float az, 
                      float mx, float my, float mz)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 使用IMU模式（无磁力计）
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MahonyAHRSupdateIMU(q, gx, gy, gz, ax, ay, az);
        return;
    }

    // 只有在加速度计有效时计算反馈
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // 1. 归一化加速度计测量
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 2. 归一化磁力计测量
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // 辅助变量（避免重复计算）
        q0q0 = q[0] * q[0];
        q0q1 = q[0] * q[1];
        q0q2 = q[0] * q[2];
        q0q3 = q[0] * q[3];
        q1q1 = q[1] * q[1];
        q1q2 = q[1] * q[2];
        q1q3 = q[1] * q[3];
        q2q2 = q[2] * q[2];
        q2q3 = q[2] * q[3];
        q3q3 = q[3] * q[3];

        // 3. 参考地球磁场方向
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // 4. 估计重力和磁场方向
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;  // 重力向量估计 (公式)
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // 5. 误差是估计方向和测量方向的叉积
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // 6. 应用积分反馈（如果启用）
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq);
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx;  // 应用积分反馈
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;  // 防止积分饱和
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // 7. 应用比例反馈
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // 8. 积分四元数变化率
    gx *= (0.5f * (1.0f / sampleFreq));  // 预处理公共因子
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);  // 四元数积分
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);

    // 9. 归一化四元数
    recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;
}
```

#### 代码与公式的对应关系

| 步骤 | 公式 | 代码 |
|------|------|------|
| 归一化 | $ \mathbf{a}_{\text{norm}} = \frac{\mathbf{a}}{\|\mathbf{a}\|} $ | `recipNorm = invSqrt(ax*ax+...); ax *= recipNorm;` |
| 重力向量估计 | $ \hat{\mathbf{g}} = [2(q_x q_z - q_w q_y),\ 2(q_w q_x + q_y q_z),\ q_w^2 - q_x^2 - q_y^2 + q_z^2]^T $ | `halfvx = q1q3 - q0q2; halfvy = q0q1 + q2q3; halfvz = q0q0 - 0.5f + q3q3;` |
| 误差计算（叉积） | $ \mathbf{e} = \mathbf{a}_{\text{meas}} \times \hat{\mathbf{g}} $ | `halfex = ay*halfvz - az*halfvy; ...` |
| PI控制器 | $ \omega_{\text{corr}} = \omega + K_p \mathbf{e} + K_i \int \mathbf{e} dt $ | `gx += twoKp * halfex + integralFBx;` |
| 四元数更新 | $ \dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \boldsymbol{\Omega} $ | `q[0] += (-qb*gx - qc*gy - q[3]*gz); ...` |
| 归一化 | $ \mathbf{q}_{\text{norm}} = \frac{\mathbf{q}}{\|\mathbf{q}\|} $ | `recipNorm = invSqrt(...); q[0] *= recipNorm; ...` |

#### 快速逆平方根函数

```c
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);  // 神奇数字
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));  // 牛顿迭代
    return y;
}
```

---

## Madgwick梯度下降滤波

### 原理

Madgwick滤波使用**梯度下降**来最小化代价函数，该代价函数表示：
- 测量的重力向量与姿态预测的重力向量之间的误差
- 测量的磁场向量与姿态预测的磁场向量之间的误差

### 核心公式

**代价函数**：
$$ f(\mathbf{q}) = \begin{bmatrix} 
2(q_x q_z - q_w q_y) - a_x \\
2(q_w q_x + q_y q_z) - a_y \\
2(\frac{1}{2} - q_x^2 - q_y^2) - a_z \\
2b_x(\frac{1}{2} - q_y^2 - q_z^2) + 2b_z(q_x q_z - q_w q_y) - m_x \\
2b_x(q_x q_y - q_w q_z) + 2b_z(q_w q_x + q_y q_z) - m_y \\
2b_x(q_w q_y + q_x q_z) + 2b_z(\frac{1}{2} - q_x^2 - q_y^2) - m_z 
\end{bmatrix} $$

**梯度**：
$$ \nabla f = J_f^T f(\mathbf{q}) $$
其中 $ J_f $ 是雅可比矩阵。

**梯度下降更新**：
$$ \mathbf{q}_{k+1} = \mathbf{q}_k - \beta \frac{\nabla f}{\|\nabla f\|} $$

### 简化代码框架

```c
void MadgwickAHRSupdate(float q[4], float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz, float dt)
{
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    
    // 1. 梯度下降计算修正量
    gradientDescentStep(q, ax, ay, az, mx, my, mz, &s0, &s1, &s2, &s3);
    
    // 2. 计算四元数变化率（陀螺仪 + 梯度修正）
    qDot1 = 0.5f * (-q[1]*gx - q[2]*gy - q[3]*gz) - beta * s0;
    qDot2 = 0.5f * (q[0]*gx + q[2]*gz - q[3]*gy) - beta * s1;
    qDot3 = 0.5f * (q[0]*gy - q[1]*gz + q[3]*gx) - beta * s2;
    qDot4 = 0.5f * (q[0]*gz + q[1]*gy - q[2]*gx) - beta * s3;
    
    // 3. 积分并归一化
    q[0] += qDot1 * dt;
    q[1] += qDot2 * dt;
    q[2] += qDot3 * dt;
    q[3] += qDot4 * dt;
    normalizeQuaternion(q);
}
```

---

## 多IMU融合架构

### 方案1：传感器级融合（加权平均）

在进入姿态解算前，先融合多个IMU的原始数据：

```
IMU1 (Gyro+Accel) ──┐
IMU2 (Gyro+Accel) ──┼──> 加权融合 ──> Mahony/Madgwick ──> 姿态
IMU3 (Gyro+Accel) ──┘
                    ↑
              基于噪声协方差
```

**实现代码**：
```c
typedef struct {
    float gyro[3];   // 角速度
    float accel[3];  // 加速度
    float weight;    // 权重
    float cov[6];    // 协方差 (gyro_x,y,z, accel_x,y,z)
} IMUData;

void fuseMultiIMU(IMUData imus[], int imu_count, 
                  float fused_gyro[], float fused_accel[])
{
    float total_weight = 0.0f;
    
    // 1. 计算总权重
    for (int i = 0; i < imu_count; i++) {
        total_weight += imus[i].weight;
    }
    
    // 2. 加权平均
    for (int axis = 0; axis < 3; axis++) {
        fused_gyro[axis] = 0.0f;
        fused_accel[axis] = 0.0f;
        
        for (int i = 0; i < imu_count; i++) {
            fused_gyro[axis] += imus[i].gyro[axis] * imus[i].weight / total_weight;
            fused_accel[axis] += imus[i].accel[axis] * imus[i].weight / total_weight;
        }
    }
}
```

### 方案2：状态估计融合（基于卡尔曼滤波）

每个IMU独立运行姿态解算，然后融合姿态结果：

```
IMU1 ──> Mahony ──> q1 ──┐
IMU2 ──> Mahony ──> q2 ──┼──> EKF/UKF ──> 融合姿态 q
IMU3 ──> Mahony ──> q3 ──┘
```

**ESKF实现框架**（参考 imu_x_fusion）：

```cpp
// 状态向量: [p, v, q, ba, bg]
struct State {
    Eigen::Vector3d p;     // 位置
    Eigen::Vector3d v;     // 速度
    Eigen::Quaterniond q;  // 姿态
    Eigen::Vector3d ba;    // 加速度计零偏
    Eigen::Vector3d bg;    // 陀螺仪零偏
};

class ESKF {
public:
    // 预测步骤（使用IMU）
    void predict(const IMUData& imu, double dt) {
        // 1. 名义状态传播
        propagateNominalState(imu, dt);
        
        // 2. 误差状态协方差传播
        propagateErrorCovariance(imu, dt);
    }
    
    // 更新步骤（使用多个IMU作为观测）
    void updateWithMultiIMU(const std::vector<IMUData>& imus) {
        for (const auto& imu : imus) {
            // 计算观测残差
            Eigen::VectorXd residual = computeResidual(imu);
            
            // 计算卡尔曼增益
            Eigen::MatrixXd K = computeKalmanGain();
            
            // 更新误差状态
            updateErrorState(K * residual);
            
            // 修正名义状态
            correctNominalState();
            
            // 重置误差状态
            resetErrorState();
        }
    }
};
```

### 方案3：联邦滤波

```
                      ┌──> 局部滤波器1 (IMU1 + Mag1) ──> x1, P1
                      │
 全局参考 ────────────┼──> 局部滤波器2 (IMU2 + Mag2) ──> x2, P2
                      │
                      └──> 局部滤波器3 (IMU3 + GNSS) ──> x3, P3
                                             
                                             ↓ 融合
                                      主滤波器 ──> 全局状态
```

---

## 开源项目分析

### 1. BMI088 + IST8310 单IMU项目（当前工作区）

**路径**：[`/workspace/Core/`](file:///workspace/Core/)

**架构**：
```
INS_task.c:
  ├─ IMU温度控制（PID控制BMI088温度）
  ├─ DMA数据读取（陀螺仪、加速度计、磁力计）
  ├─ MahonyAHRSupdate() 姿态解算
  └─ get_angle() 四元数转欧拉角
```

**关键代码片段（INS_task.c）**：
```c
// INS任务主循环
void INS_task(void)
{
    // 读取各传感器数据
    if(gyro_update_flag & (1 << IMU_NOTIFY_SHFITS)) {
        BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, 
                             bmi088_real_data.gyro);
    }
    
    // Mahony姿态解算
    AHRS_update(INS_quat, 0.001f, bmi088_real_data.gyro, 
               bmi088_real_data.accel, ist8310_real_data.mag);
    
    // 转换为欧拉角
    get_angle(INS_quat, &INS_angle[0], &INS_angle[1], &INS_angle[2]);
}
```

### 2. imu_x_fusion 项目

**路径**：[`/workspace/imu_x_fusion/`](file:///workspace/imu_x_fusion/)

**特点**：
- 支持 ESKF、IEKF、UKF 等多种滤波算法
- 支持 IMU + GNSS 松耦合融合
- 支持 IMU + 6DoF里程计融合

**核心文件**：
- [`include/estimator/ekf.hpp`](file:///workspace/imu_x_fusion/include/estimator/ekf.hpp) - EKF/ESKF 实现
- [`include/sensor/imu.hpp`](file:///workspace/imu_x_fusion/include/sensor/imu.hpp) - IMU模型
- [`src/imu_gnss_ekf.cpp`](file:///workspace/imu_x_fusion/src/imu_gnss_ekf.cpp) - IMU+GNSS融合示例

### 3. OB_GINS 项目

**路径**：[`/workspace/OB_GINS/`](file:///workspace/OB_GINS/)

**特点**：
- 基于图优化（滑动窗口）的GNSS/INS融合
- 支持多种IMU预积分方式（考虑地球自转等）
- 支持边缘化（Marginalization）

**核心文件**：
- [`src/preintegration/preintegration.h`](file:///workspace/OB_GINS/src/preintegration/preintegration.h) - IMU预积分
- [`src/factors/gnss_factor.h`](file:///workspace/OB_GINS/src/factors/gnss_factor.h) - GNSS观测因子

---

## 实现建议

### 阶段1：传感器驱动与校准

1. **实现所有传感器驱动**
   - ICM-42688P (SPI/I2C)
   - BMI088 (SPI，已有参考代码)
   - IIM-42652 (SPI/I2C)
   - RM3100 (I2C/SPI)
   - IST8310 (I2C，已有参考代码)
   - BMP581 (I2C/SPI)
   - MS5611 (I2C/SPI)
   - UM982 (UART/USB)

2. **传感器校准**
   - **加速度计**：六面校准，估计零偏和尺度因子
   - **陀螺仪**：静置估计零偏（Allan方差分析）
   - **磁力计**：椭球拟合（软铁/硬铁校正）
   - **多IMU外参标定**：估计各IMU之间的相对旋转和位移

### 阶段2：单IMU姿态解算

为每个IMU独立实现姿态解算：
```
┌─────────────────┐
│  IMU数据采集    │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  传感器校准     │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Mahony/Madgwick │
│   姿态解算      │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  欧拉角输出     │
└─────────────────┘
```

### 阶段3：多IMU融合（传感器级）

1. **实时噪声估计**
   ```c
   // 使用滑动窗口估计IMU噪声
   void estimateIMUNoise(IMUData* imu, const float new_data[6], int window_size) {
       // 更新窗口数据
       // 计算均值和方差
       // 更新权重：噪声小 -> 权重大
       imu->weight = 1.0f / (computeTotalVariance(imu) + eps);
   }
   ```

2. **故障检测**
   ```c
   bool detectIMUFault(const IMUData& imu, const IMUData& reference) {
       float gyro_diff = vectorNorm(imu.gyro, reference.gyro);
       float accel_diff = vectorNorm(imu.accel, reference.accel);
       
       return (gyro_diff > GYRO_FAULT_THRESHOLD) || 
              (accel_diff > ACCEL_FAULT_THRESHOLD);
   }
   ```

### 阶段4：多传感器松耦合融合（ESKF）

```
状态向量 X = [p, v, q, ba1, bg1, ba2, bg2, ba3, bg3]^T
           (位置, 速度, 姿态, 
            IMU1零偏, IMU2零偏, IMU3零偏)

观测：
  ├─ 3个IMU的Gyro+Accel（预测/更新）
  ├─ 2个磁力计（观测更新）
  ├─ 2个气压计（高度观测）
  └─ GNSS RTK（位置/速度观测）
```

### 阶段5：完整导航系统

整合所有传感器，实现完整的导航方案：
- 姿态（3DOF）
- 位置（3DOF，GNSS + 气压高度）
- 速度（3DOF）

---

## 附录

### A. 有用的参考资料

- [Mahony原始论文](https://www.researchgate.net/publication/245448317_Nonlinear_Complementary_Filters_on_the_Special_Orthogonal_Group)
- [Madgwick论文](https://www.sciencedirect.com/science/article/pii/S0921880711001691)
- [i2Nav的OB_GINS](https://github.com/i2Nav-WHU/OB_GINS)
- [imu_x_fusion](https://github.com/cggos/imu_x_fusion)

### B. 参数调优建议

| 参数 | 含义 | 推荐范围 |
|------|------|----------|
| Mahony Kp | 比例增益 | 0.5 ~ 2.0 |
| Mahony Ki | 积分增益 | 0.0 ~ 0.1 |
| Madgwick beta | 梯度下降步长 | 0.03 ~ 0.1 |
| ESKF过程噪声 | IMU噪声协方差 | 需要Allan方差标定 |
