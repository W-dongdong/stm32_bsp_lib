# MPU6050 驱动库使用说明

## 依赖

- `i2c_drv.h` — I2C 设备抽象层（提供 `I2CDevice`、`MemRead`、`MemWrite`）
- `Mahony.h` — 6 轴 Mahony AHRS 姿态解算

## 快速开始

```cpp
#include "MPU6050.h"

// 1. 初始化 I2C（以 STM32 HAL 为例）
I2CDevice i2c2(&hi2c2);

// 2. 配置 MPU6050
MPU6050Config cfg = {
    .DLPF          = MPU6050_DLPF_44HZ,       // 低通滤波 44Hz
    .GyroRange     = MPU6050_GYRO_2000DEG,    // 陀螺 ±2000°/s
    .AccelRange    = MPU6050_ACCEL_16G,       // 加速度 ±16g
    .SampleRateDiv = 9,                       // 采样率 = 1000/(1+9) = 100Hz
    .Kp            = 0.5f,                    // Mahony 比例增益
    .Ki            = 0.1f                     // Mahony 积分增益
};

// 3. 构造（自动完成硬件初始化和采样率计算）
MPU6050 imu(&i2c2, cfg);

// 4. 陀螺零偏校准（IMU 必须静止）
Delay(3000);
imu.CalibrateGyro();

// 5. 配置定时器中断，频率 = 采样率（本例 100Hz → 10ms 周期）
// 在定时器中断回调中调用 GetData()

// ---- 定时器中断回调（以 STM32 HAL 为例）----
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        imu.GetData();  // 100Hz 精确调用，与 m_detT 匹配
    }
}

// 主循环中可以随时读取姿态角
while (1) {
    printf("Roll: %.2f  Pitch: %.2f  Yaw: %.2f\n",
           imu.m_Roll, imu.m_Pitch, imu.m_Yaw);
    HAL_Delay(100);  // 打印频率无精度要求，随意
}
```

## 配置参数详解

### `MPU6050Config`

| 字段 | 类型 | 说明 |
|------|------|------|
| `DLPF` | `uint8_t` | DLPF 带宽，见下方 DLPF 选项表 |
| `GyroRange` | `uint8_t` | 陀螺仪量程，见下方陀螺量程表 |
| `AccelRange` | `uint8_t` | 加速度计量程，见下方加计量程表 |
| `SampleRateDiv` | `uint8_t` | 采样率分频器（0~255），公式见下文 |
| `Kp` | `float` | Mahony 比例增益（典型值 0.5） |
| `Ki` | `float` | Mahony 积分增益（典型值 0.1） |

### DLPF 选项

| 宏 | 加速度带宽 | 陀螺带宽 | 陀螺输出率 |
|----|-----------|---------|-----------|
| `MPU6050_DLPF_260HZ` | 260 Hz | 256 Hz | 8 kHz |
| `MPU6050_DLPF_184HZ` | 184 Hz | 188 Hz | 1 kHz |
| `MPU6050_DLPF_94HZ`  | 94 Hz  | 98 Hz  | 1 kHz |
| `MPU6050_DLPF_44HZ`  | 44 Hz  | 42 Hz  | 1 kHz |
| `MPU6050_DLPF_21HZ`  | 21 Hz  | 20 Hz  | 1 kHz |
| `MPU6050_DLPF_10HZ`  | 10 Hz  | 10 Hz  | 1 kHz |
| `MPU6050_DLPF_5HZ`   | 5 Hz   | 5 Hz   | 1 kHz |

### 陀螺量程

| 宏 | 满量程 |
|----|--------|
| `MPU6050_GYRO_250DEG` | ±250 °/s |
| `MPU6050_GYRO_500DEG` | ±500 °/s |
| `MPU6050_GYRO_1000DEG` | ±1000 °/s |
| `MPU6050_GYRO_2000DEG` | ±2000 °/s |

### 加速度量程

| 宏 | 满量程 |
|----|--------|
| `MPU6050_ACCEL_2G` | ±2 g |
| `MPU6050_ACCEL_4G` | ±4 g |
| `MPU6050_ACCEL_8G` | ±8 g |
| `MPU6050_ACCEL_16G` | ±16 g |

### 采样率计算

采样率根据 `DLPF` 和 `SampleRateDiv` 自动计算，无需用户手动填入。

- DLPF 启用（1~6）时，陀螺输出率 = 1 kHz：
  ```
  采样率 = 1000 / (1 + SampleRateDiv)  [Hz]
  ```
- DLPF 禁用（DLPF_CFG = 0）时，陀螺输出率 = 8 kHz：
  ```
  采样率 = 8000 / (1 + SampleRateDiv)  [Hz]
  ```

采样率自动传给 Mahony 滤波器作为积分时间步长，用户只需按计算出的周期调用 `GetData()`。

## 公开成员（可读）

### 原始数据（LSB）

| 成员 | 类型 | 说明 |
|------|------|------|
| `m_RawAccX` | `int16_t` | X 轴加速度原始值 |
| `m_RawAccY` | `int16_t` | Y 轴加速度原始值 |
| `m_RawAccZ` | `int16_t` | Z 轴加速度原始值 |
| `m_RawGyroX` | `int16_t` | X 轴陀螺原始值 |
| `m_RawGyroY` | `int16_t` | Y 轴陀螺原始值 |
| `m_RawGyroZ` | `int16_t` | Z 轴陀螺原始值 |
| `m_Temp` | `int16_t` | 温度传感器原始值 |

### 物理量（经标度换算）

| 成员 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `m_AccX` | `float` | g | X 轴加速度 |
| `m_AccY` | `float` | g | Y 轴加速度 |
| `m_AccZ` | `float` | g | Z 轴加速度 |
| `m_GyroX` | `float` | rad/s | X 轴角速度（已减零偏） |
| `m_GyroY` | `float` | rad/s | Y 轴角速度（已减零偏） |
| `m_GyroZ` | `float` | rad/s | Z 轴角速度（已减零偏） |

### 姿态角（Mahony 输出）

| 成员 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `m_Roll` | `float` | 度 (°) | 横滚角 |
| `m_Pitch` | `float` | 度 (°) | 俯仰角 |
| `m_Yaw` | `float` | 度 (°) | 偏航角（无磁力计时会漂移） |

## 函数接口

### 构造函数

```cpp
MPU6050(I2CDevice *i2c, const MPU6050Config &config)
```

- 创建对象时自动完成：
  1. 根据 DLPF 和 SampleRateDiv 计算采样率并初始化 Mahony 滤波器
  2. 计算陀螺/加速度标度因子
  3. 唤醒 MPU6050 并设置时钟源为 PLL Gyro X
  4. 配置采样率、DLPF、陀螺量程、加速度量程
- `i2c`：指向已初始化的 `I2CDevice` 指针，不可为 `NULL`

### GetData

```cpp
uint8_t GetData()
```

读取传感器数据，运行 Mahony AHRS，更新欧拉角。**应通过定时器中断以固定频率调用**，频率必须与构造函数自动计算的采样率一致，否则 Mahony 积分时间（`m_detT`）与实际不符，姿态会漂移。

| 返回值 | 含义 |
|--------|------|
| `1` | 成功：I2C 读取成功，Mahony 更新完成，`m_Roll`/`m_Pitch`/`m_Yaw` 等已更新 |
| `0` | 失败：I2C 读取失败，**本轮不更新任何数据**，姿态保持上一帧值 |

### CalibrateGyro

```cpp
uint8_t CalibrateGyro()
```

陀螺仪零偏校准，读取 1000 帧静止数据，取平均值作为零偏存入 `m_OffsetGyroX/Y/Z`。后续每次 `GetData()` 自动减去零偏。

| 返回值 | 含义 |
|--------|------|
| `1` | 成功：零偏已更新 |
| `0` | 失败：有效帧数不足（< 500），零偏保持原值 |

**调用条件**：IMU 必须完全静止，建议在 PLL 稳定后调用。

```cpp
MPU6050 imu(&i2c2, cfg);
HAL_Delay(3000);           // 等待 PLL 稳定
imu.CalibrateGyro();      // 零偏校准
```

### WriteReg

```cpp
uint8_t WriteReg(uint8_t RegAddress, uint8_t Data)
```

向 MPU6050 寄存器写入一个字节。

| 返回值 | 含义 |
|--------|------|
| 非 0 | I2C 写入成功 |
| `0` | I2C 写入失败或 `m_i2c` 为 `NULL` |

### ReadReg

```cpp
uint8_t ReadReg(uint8_t RegAddress)
```

从 MPU6050 寄存器读取一个字节。

| 返回值 | 含义 |
|--------|------|
| 寄存器值 | I2C 读取成功 |
| `0` | I2C 读取失败或 `m_i2c` 为 `NULL` |

## 完整示例

```cpp
#include "MPU6050.h"

I2CDevice i2c2(&hi2c2);
MPU6050 imu(&i2c2, cfg);  // 全局或静态，供 ISR 访问

/* ---- 定时器中断回调（100Hz = 10ms）---- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        imu.GetData();  // 精确按采样率调用，I2C 失败自动跳过本帧
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_I2C2_Init();
    MX_TIM2_Init();  // 配置 TIM2 为 100Hz 中断

    // ---- 配置 ----
    MPU6050Config cfg;
    cfg.DLPF          = MPU6050_DLPF_44HZ;
    cfg.GyroRange     = MPU6050_GYRO_2000DEG;
    cfg.AccelRange    = MPU6050_ACCEL_16G;
    cfg.SampleRateDiv = 9;         // 1000/(1+9) = 100Hz
    cfg.Kp            = 0.5f;
    cfg.Ki            = 0.1f;

    // ---- 初始化 ----
    imu = MPU6050(&i2c2, cfg);     // 构造后 PLL 自动选定、寄存器已配置

    // ---- 零偏校准（IMU 必须静止）----
    HAL_Delay(3000);
    if (!imu.CalibrateGyro()) {
        // 有效帧 < 500，校准失败
    }

    // ---- 启动定时器 ----
    HAL_TIM_Base_Start_IT(&htim2);  // 开始以 100Hz 调用 GetData()

    // ---- 主循环：随时读取姿态 ----
    while (1) {
        float roll  = imu.m_Roll;    // 度
        float pitch = imu.m_Pitch;
        float yaw   = imu.m_Yaw;     // 无磁力计时会漂移
        HAL_Delay(100);
    }
}
```

## 时钟源

库固定使用 **PLL with X-axis gyroscope reference**（CLKSEL = 001），这是 MPU6050 数据手册推荐的时钟源，精度和稳定性最优。用户无需配置。
