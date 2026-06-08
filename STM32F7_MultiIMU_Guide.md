# STM32F7 多IMU导航系统 - 快速入门

## 📁 项目结构

```
/workspace/
├── Core/
│   ├── Inc/
│   │   ├── sensor_common.h          # 统一数据结构 (新!)
│   │   ├── ahrs_mahony.h            # Mahony姿态解算 (新!)
│   │   ├── imu_fusion.h             # 多IMU融合 (新!)
│   │   ├── BMI088Middleware.h       # 现有BMI088驱动
│   │   ├── ist8310driver.h          # 现有IST8310驱动
│   │   └── ...
│   └── Src/
│       ├── sensor_common.c          # 向量/四元数运算 (新!)
│       ├── ahrs_mahony.c            # Mahony滤波实现 (新!)
│       ├── imu_fusion.c             # 多IMU融合 (预留)
│       └── ...
├── 阶段规划.md                      # 完整5阶段规划
├── 多IMU姿态解算与传感器融合指南.md # 理论详解
└── STM32F7_MultiIMU_Guide.md        # 本文 (快速入门)
```

---

## 🚀 阶段 1: 工程搭建 (当前阶段)

### 1.1 硬件引脚映射

根据您的需求，我建议的引脚分配（可修改）：

| 功能 | 芯片 | 引脚 | 说明 |
|------|------|------|------|
| **SPI1** | BMI088 | PA5/PA6/PA7 | SCK/MISO/MOSI |
| | CS_Accel | PE3 | |
| | CS_Gyro | PE4 | |
| | INT_Accel | PE0 | |
| | INT_Gyro | PE1 | |
| **SPI2** | ICM-42688P | PB10/PC2/PC3 | 独立SPI |
| | CS | PB9 | |
| | INT | PB8 | |
| **SPI3** | IIM-42652 | PC10/PC11/PC12 | 独立SPI |
| | CS | PA15 | |
| | INT | PA8 | |
| **I2C1** | IST8310 | PB8/PB9 | 磁力计1 |
| **I2C2** | RM3100 | PH4/PH5 | 磁力计2 |
| **I2C3** | BMP581 | PH7/PH8 | 气压计1 |
| **I2C4** | MS5611 | PF14/PF15 | 气压计2 |
| **UART4** | UM982 GNSS | PA0/PA1 | 高速UART |
| **UART1** | Debug | PA9/PA10 | 调试输出 |
| **CAN1** | CAN | PB8/PB9 | 预留 |
| **TIM1** | PWM | PE9 | BMI088温度控制 |

### 1.2 STM32CubeMX 配置步骤

1. **创建新项目**
   - 选择 `STM32F743VIH6`
   - 时钟配置: 外部晶振 -> 216MHz

2. **SPI 配置** (每个IMU用独立SPI或分时复用)
   - SPI1: 主模式, Prescaler=8 (27MHz)
   - SPI2: 主模式
   - SPI3: 主模式
   - 开启 DMA: RX + TX

3. **I2C 配置**
   - I2C1/2/3/4: 快速模式 (400kHz)

4. **UART 配置**
   - UART1: 115200 (调试)
   - UART4: 921600 (GNSS)
   - 开启 DMA RX + IDLE 中断

5. **GPIO 配置**
   - CS引脚: 推挽输出, 初始高电平
   - INT引脚: 外部中断, 下降/上升沿

6. **生成代码**
   - 生成 Keil MDK-ARM 工程

---

## 🔧 阶段 2: 单IMU姿态解算

### 2.1 快速开始代码

基于您现有的 BMI088 代码，使用新的统一框架：

```c
/* main.c */
#include "sensor_common.h"
#include "ahrs_mahony.h"
#include "BMI088driver.h"
#include "ist8310driver.h"

/* 全局实例 */
ahrs_mahony_t ahrs;
imu_data_t bmi088_data;
mag_data_t ist8310_data;

int main(void) {
    /* STM32初始化... */
    HAL_Init();
    SystemClock_Config();

    /* 初始化AHRS: 1kHz采样, Kp=2.0, Ki=0.005 */
    ahrs_mahony_init(&ahrs, 1000.0f, 2.0f, 0.005f);
    
    /* 初始化传感器 */
    BMI088_init();
    ist8310_init();

    while (1) {
        /* 1. 读取传感器数据 */
        BMI088_read(bmi088_data.gyro, bmi088_data.accel, &bmi088_data.temp);
        ist8310_read_mag(&ist8310_data.mag);
        
        /* 2. 更新AHRS */
        ahrs_mahony_update(&ahrs, &bmi088_data.gyro, 
                           &bmi088_data.accel, &ist8310_data.mag);
        
        /* 3. 获取欧拉角 (deg) */
        euler_angle_t euler;
        ahrs_mahony_get_euler(&ahrs, &euler);
        
        float roll_deg = rad_to_deg(euler.roll);
        float pitch_deg = rad_to_deg(euler.pitch);
        float yaw_deg = rad_to_deg(euler.yaw);
        
        /* 4. 调试输出 */
        printf("Roll: %.1f, Pitch: %.1f, Yaw: %.1f\n", 
               roll_deg, pitch_deg, yaw_deg);
        
        HAL_Delay(1);
    }
}
```

### 2.2 调参指南

| 参数 | 推荐值 | 效果 |
|------|--------|------|
| `kp` | 1.0~5.0 | 越大收敛越快, 但噪声大 |
| `ki` | 0.0~0.1 | 越大零偏抑制越好, 但可能震荡 |
| **建议** | 先调 `kp`, 稳定后加小 `ki` |

---

## 🔄 阶段 3: 多IMU融合

### 3.1 融合架构 (推荐方案)

```
  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐
  │   BMI088     │  │ ICM-42688P   │  │ IIM-42652    │
  │  (AHRS #1)   │  │  (AHRS #2)   │  │  (AHRS #3)   │
  └───────┬──────┘  └───────┬──────┘  └───────┬──────┘
          │                 │                 │
          └─────────────────┼─────────────────┘
                            │
                    ┌───────▼───────┐
                    │ 健康度评估    │
                    │ • 噪声        │
                    │ • 一致性      │
                    │ • 运动合理性  │
                    └───────┬───────┘
                            │
                    ┌───────▼───────┐
                    │ 择优切换      │
                    │ 或加权融合    │
                    └───────┬───────┘
                            │
                    ┌───────▼───────┐
                    │ 最终姿态输出  │
                    └───────────────┘
```

### 3.2 健康度评估

1. **噪声评估** - 滑动窗口计算标准差
2. **一致性评估** - 与其它IMU的姿态差
3. **运动合理性** - 加速度/角速度突变检测

---

## 📊 数据结构速查表

### 统一数据类型

```c
/* 3D向量 */
typedef struct { float x, y, z; } vector3f_t;

/* 四元数 (w,x,y,z) */
typedef struct { float q0, q1, q2, q3; } quaternion_t;

/* 欧拉角 (rad) */
typedef struct { float roll, pitch, yaw; } euler_angle_t;

/* IMU数据 */
typedef struct {
    vector3f_t gyro;   // rad/s
    vector3f_t accel;  // m/s^2
    float temp;        // degC
} imu_data_t;
```

### 常用函数

```c
/* 向量运算 */
vector3f_normalize(&v);             // 归一化
vector3f_cross(&out, &a, &b);       // 叉积

/* 四元数运算 */
quaternion_from_euler(&q, roll, pitch, yaw);  // 欧拉角->四元数
quaternion_to_euler(&q, &euler);              // 四元数->欧拉角

/* 工具 */
deg_to_rad(deg);    // 度转弧度
rad_to_deg(rad);    // 弧度转度
```

---

## 🛠️ 现有驱动迁移指南

您的现有代码可以无缝集成到新框架：

### BMI088
```c
/* 现有代码读取 */
float gyro[3], accel[3], temp;
BMI088_read(gyro, accel, &temp);

/* 复制到新结构 */
imu_data_t data;
data.gyro.x = gyro[0];  // 注意轴对齐和单位!
data.gyro.y = gyro[1];
data.gyro.z = gyro[2];
data.accel.x = accel[0];
data.accel.y = accel[1];
data.accel.z = accel[2];
data.temp = temp;
```

### IST8310
```c
/* 现有代码读取 */
float mag[3];
ist8310_read_mag(mag);

/* 复制到新结构 */
mag_data_t data;
data.mag.x = mag[0];
data.mag.y = mag[1];
data.mag.z = mag[2];
```

---

## 📖 参考文档

1. **理论详解** - 查看 `多IMU姿态解算与传感器融合指南.md`
2. **阶段规划** - 查看 `阶段规划.md`
3. **现有代码** - `Core/Src/BMI088driver.c`, `Core/Src/ist8310driver.c`

---

## ❓ 下一步

请告诉我：
1. 引脚分配是否需要调整？
2. 是否先从单IMU开始（BMI088+IST8310）？
3. 是否需要我帮您移植现有代码？
4. 是否需要生成CubeMX配置模板？
