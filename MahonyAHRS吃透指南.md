# MahonyAHRS.c 彻底吃透指南

> **目标**：把每一行代码都搞明白，公式和代码一一对应

---

## 📋 文件概览

| 函数 | 行号 | 作用 |
|------|------|------|
| `MahonyAHRSupdate` | 36-84 | 主函数：带磁力计的完整姿态更新 |
| `MahonyAHRSupdateIMU` | 86-128 | 仅IMU模式（无磁力计）|
| `invSqrt` | 131-138 | 快速逆平方根 |

---

## 🔄 核心流程总览

```
┌──────────────────────────────────────────────────────────────────┐
│                      MahonyAHRSupdate()                          │
├──────────────────────────────────────────────────────────────────┤
│                                                                   │
│  1. 读取输入:                                                      │
│     ├─ q[4]     : 当前四元数 (姿态)                               │
│     ├─ gx,gy,gz : 陀螺仪角速度 (rad/s)                            │
│     ├─ ax,ay,az : 加速度计 (任意单位)                             │
│     └─ mx,my,mz : 磁力计 (任意单位, 可为0)                        │
│                                                                   │
│  2. 如果没有磁力计数据 → 调用 MahonyAHRSupdateIMU()              │
│                                                                   │
│  3. 归一化加速度和磁力计                                            │
│                                                                   │
│  4. 计算辅助变量 (避免重复计算)                                     │
│                                                                   │
│  5. 计算参考磁场方向 (hx, hy, bx, bz)                             │
│                                                                   │
│  6. 估计重力和磁场方向 (halfvx, halfvy, halfvz, ...)             │
│                                                                   │
│  7. 计算误差 (叉积)                                                │
│                                                                   │
│  8. PI控制器修正                                                   │
│     ├─ 积分项: 消除零偏                                            │
│     └─ 比例项: 即时修正                                            │
│                                                                   │
│  9. 积分四元数变化率                                               │
│                                                                   │
│  10. 归一化四元数                                                  │
│                                                                   │
│  11. 更新 q[4] 输出                                               │
└──────────────────────────────────────────────────────────────────┘
```

---

## 📖 第一部分：定义和初始化

### 1.1 宏定义（第22-25行）

```c
#define sampleFreq   1000.0f    // 采样频率 1000Hz
#define twoKpDef     (2.0f * 0.5f)  // 2 * 比例增益
#define twoKiDef     (2.0f * 0.0f)  // 2 * 积分增益
```

**公式对应**：

| 宏 | 值 | 实际增益 |
|----|-----|---------|
| `twoKpDef` | 2 × 0.5 | Kp = 0.5 |
| `twoKiDef` | 2 × 0.0 | Ki = 0.0 |

**为什么要乘2？**

这是代码优化技巧，后面计算误差时用的是 `halfex`（误差的一半），所以：
```c
twoKp * halfex = (2*Kp) * (ex/2) = Kp * ex
```
乘2和除2抵消，少算一次乘法！

### 1.2 全局变量（第27-32行）

```c
volatile float twoKp = twoKpDef;           // 比例增益 (可动态调整)
volatile float twoKi = twoKiDef;           // 积分增益 (可动态调整)
volatile float integralFBx = 0.0f;        // 积分反馈 X
volatile float integralFBy = 0.0f;        // 积分反馈 Y
volatile float integralFBz = 0.0f;        // 积分反馈 Z
```

**volatile 的作用**：
- 防止编译器优化
- 允许在中断或主循环外修改这些值

---

## 📖 第二部分：快速逆平方根 `invSqrt()`

### 2.1 代码（第131-138行）

```c
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;           // 把float的位模式当成long
    i = 0x5f3759df - (i>>1);      // 魔数！快速近似
    y = *(float*)&i;               // 把long的位模式当成float
    y = y * (1.5f - (halfx * y * y));  // 牛顿迭代一次
    return y;
}
```

### 2.2 数学原理

**目的**：计算 $ \frac{1}{\sqrt{x}} $

**传统方法**：
```c
y = 1.0f / sqrtf(x);  // 慢，需要硬件支持
```

**invSqrt 的魔法**：
1. **位操作近似**：$ y \approx \frac{1}{\sqrt{x}} $
2. **牛顿迭代**：$ y_{n+1} = y_n(1.5 - \frac{x \cdot y_n^2}{2}) $

### 2.3 直观理解

**类比**：就像用"猜数字"的方法
1. 先猜一个大概的值（魔数）
2. 验证并微调（牛顿迭代）
3. 几次后就非常准确了

**精度**：单次牛顿迭代后精度 ~0.001%，足够工程使用！

---

## 📖 第三部分：`MahonyAHRSupdate()` 主函数

### 3.1 函数签名（第36行）

```c
void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, 
                     float ax, float ay, float az, 
                     float mx, float my, float mz)
```

**参数说明**：

| 参数 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `q[4]` | in/out | - | 四元数 [q0,q1,q2,q3] |
| `gx,gy,gz` | input | rad/s | 陀螺仪角速度 |
| `ax,ay,az` | input | 任意 | 加速度计 |
| `mx,my,mz` | input | 任意 | 磁力计（可为0）|

---

### 3.2 步骤1：磁力计检查（第41-44行）

```c
// Use IMU algorithm if magnetometer measurement invalid
if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    MahonyAHRSupdateIMU(q, gx, gy, gz, ax, ay, az);
    return;
}
```

**逻辑**：
- 如果磁力计数据全为0，说明没有磁力计或数据无效
- 就调用**仅IMU版本**的函数（不用磁力计修正航向）
- 这样姿态解算照样工作，只是Yaw会漂移

---

### 3.3 步骤2：加速度计有效性检查（第47-53行）

```c
// Compute feedback only if accelerometer measurement valid
if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // ... 核心计算 ...
}
```

**为什么要检查**：
- 避免除以0（归一化时）
- 避免 NaN 出现
- 加速度计饱和或故障时跳过修正

---

### 3.4 步骤3：归一化（第49-53行）

```c
// Normalise accelerometer measurement
recipNorm = invSqrt(ax * ax + ay * ay + az * az);
ax *= recipNorm;
ay *= recipNorm;
az *= recipNorm;
```

**公式**：

$$ \mathbf{a}_{\text{norm}} = \frac{\mathbf{a}}{\|\mathbf{a}\|} = \frac{\mathbf{a}}{\sqrt{a_x^2 + a_y^2 + a_z^2}} $$

**为什么归一化？**
- 只需要**方向**，不需要**大小**
- 静态时加速度应该是重力向量（长度=1g）
- 归一化后不管IMU怎么放，方向都是一致的

---

### 3.5 步骤4：辅助变量（第61-70行）

```c
// Auxiliary variables to avoid repeated arithmetic
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
```

**作用**：
- 预计算四元数的乘积
- 后面会多次用到这些值
- **减少重复乘法**，提升性能

---

### 3.6 步骤5：参考磁场方向（第72-74行）

```c
// Reference direction of Earth's magnetic field
hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
bx = sqrt(hx * hx + hy * hy);
bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
```

**这段代码在干什么？**

这是把**机体系磁场** $ \mathbf{m}^b $ 转到**导航系** $ \mathbf{m}^n $：

$$ \mathbf{m}^n = \mathbf{R}(q) \cdot \mathbf{m}^b $$

展开后就是上面那一堆公式！

**为什么要算 bx, bz？**
- `bx, bz`：地磁场在导航系的估计方向
- **只保留水平分量**（去掉磁倾角）
- 后面用这个来修正航向

---

### 3.7 步骤6：估计重力和磁场方向（第76-81行）

```c
// Estimated direction of gravity and magnetic field
halfvx = q1q3 - q0q2;              // 重力向量 X 的一半
halfvy = q0q1 + q2q3;              // 重力向量 Y 的一半
halfvz = q0q0 - 0.5f + q3q3;       // 重力向量 Z 的一半

halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
```

**重力向量估计（核心公式！）**

用当前姿态 $ q $ 把**导航系重力** $ [0, 0, 1] $ 转回**机体系**：

$$ \hat{\mathbf{g}}^b = (q^*) \otimes [0, 0, 1] \otimes q $$

展开后：

$$ \hat{\mathbf{g}}^b = \begin{bmatrix} 2(q_1 q_3 - q_0 q_2) \\ 2(q_0 q_1 + q_2 q_3) \\ q_0^2 - q_1^2 - q_2^2 + q_3^2 \end{bmatrix} = 2 \begin{bmatrix} \text{halfvx} \\ \text{halfvy} \\ \text{halfvz} \end{bmatrix} $$

**磁场向量估计**：

类似的，把导航系的水平磁场 $ [b_x, 0, b_z] $ 转回机体系。

---

### 3.8 步骤7：计算误差（叉积！）（第83-86行）

```c
// Error is sum of cross product between estimated and measured direction
halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
```

**这是整个算法的核心！**

**公式**：

$$ \mathbf{e} = \underbrace{\mathbf{a}_{\text{meas}} \times \hat{\mathbf{g}}}_{\text{加速度误差}} + \underbrace{\mathbf{m}_{\text{meas}} \times \hat{\mathbf{m}}}_{\text{磁场误差}} $$

展开后就是上面那一行代码！

**叉积的物理意义**：

| 叉积结果 | 含义 |
|---------|------|
| 方向 | "绕哪个轴旋转"能让测量值和估计值对齐 |
| 大小 | 误差的大小（与 sin(夹角) 成正比）|

**举例**：
- 假设 IMU 歪了，测量的重力方向和估计的不一致
- 叉积会告诉你"需要绕哪个轴转多少度"才能对齐

---

### 3.9 步骤8：PI 控制器（第88-101行）

```c
// Compute and apply integral feedback if enabled
if(twoKi > 0.0f) {
    // 积分项：累积过去的误差
    integralFBx += twoKi * halfex * (1.0f / sampleFreq);
    integralFBy += twoKi * halfey * (1.0f / sampleFreq);
    integralFBz += twoKi * halfez * (1.0f / sampleFreq);
    
    // 应用积分反馈
    gx += integralFBx;
    gy += integralFBy;
    gz += integralFBz;
}
else {
    // 清零，防止积分饱和
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
}

// Apply proportional feedback
gx += twoKp * halfex;
gy += twoKp * halfey;
gz += twoKp * halfez;
```

**P（比例）项**：

$$ \delta\omega_P = K_p \cdot \mathbf{e} $$

- "现在错了多少，就改多少"
- 误差大 → 修正力度大
- 响应快，但会有稳态误差

**I（积分）项**：

$$ \delta\omega_I = K_i \int \mathbf{e} \cdot dt $$

- "过去一直在错，累积起来修正"
- 消除陀螺仪零偏导致的漂移
- 但可能过度修正导致震荡

**为什么要除以 sampleFreq？**

离散化时：$ \int e \cdot dt \approx e \cdot \Delta t = \frac{e}{f_{\text{sample}}} $

---

### 3.10 步骤9：积分四元数变化率（第103-112行）

```c
// Integrate rate of change of quaternion
gx *= (0.5f * (1.0f / sampleFreq));  // 预处理：dt/2
gy *= (0.5f * (1.0f / sampleFreq));
gz *= (0.5f * (1.0f / sampleFreq));

qa = q[0];
qb = q[1];
qc = q[2];

q[0] += (-qb * gx - qc * gy - q[3] * gz);
q[1] += (qa * gx + qc * gz - q[3] * gy);
q[2] += (qa * gy - qb * gz + q[3] * gx);
q[3] += (qa * gz + qb * gy - qc * gx);
```

**核心公式（四元数运动学）**：

$$ \dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \boldsymbol{\Omega} $$

其中：
$$ \boldsymbol{\Omega} = \begin{bmatrix} 0 \\ \omega_x \\ \omega_y \\ \omega_z \end{bmatrix} $$

**离散化**：

$$ \mathbf{q}_{t+1} = \mathbf{q}_t + \dot{\mathbf{q}}_t \cdot \Delta t $$

**为什么要乘以 0.5？**

因为四元数变化率公式本身就有 $ \frac{1}{2} $，而 $ \Delta t = \frac{1}{f_{\text{sample}}} $

---

### 3.11 步骤10：归一化四元数（第114-119行）

```c
// Normalise quaternion
recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
q[0] *= recipNorm;
q[1] *= recipNorm;
q[2] *= recipNorm;
q[3] *= recipNorm;
```

**公式**：

$$ \mathbf{q}_{\text{norm}} = \frac{\mathbf{q}}{\|\mathbf{q}\|} = \frac{\mathbf{q}}{\sqrt{q_0^2 + q_1^2 + q_2^2 + q_3^2}} $$

**为什么必须归一化？**
- 四元数表示旋转时必须是单位四元数
- 数值积分会累积误差，导致模长变化
- 每次更新后必须归一化！

---

## 📖 第四部分：`MahonyAHRSupdateIMU()` 仅IMU版本

### 4.1 与完整版的区别

| 功能 | 完整版 | IMU版 |
|------|--------|-------|
| 磁力计 | ✅ 用 | ❌ 不用 |
| Yaw修正 | ✅ 有 | ❌ 没有 |
| 计算量 | 多 | 少 |
| Yaw漂移 | 小 | 有 |

### 4.2 核心差异（第76-79行）

```c
// IMU版：只用加速度计
halfex = (ay * halfvz - az * halfvy);
halfey = (az * halfvx - ax * halfvz);
halfez = (ax * halfvy - ay * halfvx);

// 完整版：加速度计 + 磁力计
halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
```

**去掉的就是磁力计的修正项！**

---

## 🎯 实战例子：手动走一遍

### 场景
IMU 静止在水平面上，初始姿态。

### 输入

```c
// 陀螺仪（假设完美静止）
gx = 0.0f;  gy = 0.0f;  gz = 0.0f;

// 加速度计（测到重力）
ax = 0.0f;  ay = 0.0f;  az = 1.0f;

// 四元数（初始值，单位四元数 = 无旋转）
q[0] = 1.0f;  q[1] = 0.0f;  q[2] = 0.0f;  q[3] = 0.0f;
```

### 执行步骤

**步骤1**：归一化加速度（已经是1了，不需要变）

**步骤2**：辅助变量
```
q0q0 = 1.0f
其他都是 0
```

**步骤3**：估计重力方向
```
halfvx = 0 - 0 = 0
halfvy = 0 + 0 = 0
halfvz = 1.0 - 0.5 + 0 = 0.5  // [0, 0, 1]的一半
```

**步骤4**：计算误差
```
halfex = 0 * 0.5 - 1.0 * 0 = 0
halfey = 1.0 * 0 - 0 * 0.5 = 0
halfez = 0 * 0 - 0 * 0 = 0
```

**结果**：没有误差！姿态正确，不需要修正！

### 如果IMU歪了10度？

假设 IMU 歪了，测量到的加速度：
```
ax = sin(10°) ≈ 0.174
ay = 0
az = cos(10°) ≈ 0.985
```

**估计重力**（不变）：
```
halfvx = 0
halfvy = 0
halfvz = 0.5
```

**计算误差**：
```
halfex = 0 * 0.5 - 0.985 * 0 = 0
halfey = 0.985 * 0 - 0.174 * 0.5 = -0.087
halfez = 0.174 * 0 - 0 * 0.5 = 0
```

**PI修正**（假设 Kp=1, Ki=0）：
```
gy += 1.0 * (-0.087) = -0.087  // 往正方向修正
```

**结果**：角速度被修正，系统知道"需要往回转"，会慢慢修正姿态！

---

## 🐛 常见问题排查

### 问题1：姿态抖动
**原因**：Kp太大
**解决**：减小 twoKpDef，例如改成 `(2.0f * 0.3f)`

### 问题2：Yaw持续漂移
**原因**：没有磁力计或Ki太小
**解决**：
1. 确保磁力计数据有效
2. 适当增大 Ki，例如改成 `(2.0f * 0.01f)`

### 问题3：积分饱和（震荡）
**原因**：Ki太大，积分项累积太多
**解决**：
1. 减小 Ki
2. 或者限制积分项的幅值

### 问题4：初始化时跳变
**原因**：初始四元数和实际姿态差太大
**解决**：静置IMU几秒再开始解算

---

## 📊 调参速查表

| 参数 | 默认值 | 调大 | 调小 |
|------|--------|------|------|
| `twoKp` | 1.0 (2×0.5) | 响应快但抖 | 响应慢但平滑 |
| `twoKi` | 0.0 (2×0.0) | 消除漂移但可能震荡 | 无漂移抑制 |

**推荐配置**：

| 应用场景 | Kp | Ki |
|---------|----|----|
| 无人机（快速响应）| 0.5~1.0 | 0.0~0.005 |
| 平衡小车（需要平滑）| 0.3~0.5 | 0.01~0.02 |
| 初始调试 | 1.0~2.0 | 0.0 |

---

## 🔗 延伸学习

### 推荐阅读顺序

1. **四元数基础**：
   - [维基百科：四元数与空间旋转](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation)
   - [3Blue1Brown 四元数视频](https://www.youtube.com/watch?v=zjMuIxRvygQ)

2. **互补滤波原理**：
   - [原版 Mahony 论文](https://ahrs.readthedocs.io/en/latest/filters/mahony.html)

3. **代码实践**：
   - PX4 的实现：[PX4-ECL](https://github.com/PX4/PX4-ECL)
   - ROS imu_filter_madgwick

---

## ✅ 总结：Mahony 滤波的核心思想

1. **陀螺仪积分**：给出快速动态的姿态变化
2. **加速度计/磁力计修正**：通过叉积误差，纠正陀螺仪的漂移
3. **PI控制器**：平衡响应速度和稳定性
4. **四元数积分**：把角速度变成姿态

**一句话概括**：
> "用陀螺仪开车（积分），用加速度计/磁力计看路牌（修正）"

---

**下一步**：把四元数转欧拉角也吃透吗？
