# BSP 驱动库文档

STM32 HAL 外设的 C++ 类封装。所有 BSP 驱动共享以下设计模式：

- **构造函数注册：** 创建实例时自动注册到内部静态数组
- **中断回调分发：** HAL 中断回调中遍历所有已注册实例，匹配句柄后设置标志位
- **多实例支持：** 一个外设类型可同时使用多个实例

---

## 一、I2CDevice — I2C 驱动

### 依赖
- STM32 HAL: `i2c.h`

### 最大实例数
3

### 使用示例

#### 例 1：读取 I2C 传感器的寄存器（如 MPU6050 WHO_AM_I）

```cpp
#include "i2c_drv.h"

I2CDevice i2c1(&hi2c1);

// 先确认设备在线
if (!i2c1.IsDeviceReady(0xD0, 3, 100)) {
    // 设备无应答，检查接线和地址
    return;
}

// 单寄存器读：WHO_AM_I (0x75)
uint8_t whoami = 0;
if (i2c1.MemRead(0xD0, 0x75, I2C_MEMADD_SIZE_8BIT, &whoami, 1, 10)) {
    // 成功: whoami == 0x68 说明 MPU6050 正常
    if (whoami == 0x68) { /* 设备识别成功 */ }
} else {
    // 读取超时或失败
}
```

#### 例 2：写多个寄存器（MPU6050 初始化）

```cpp
// 连续写多个寄存器
uint8_t config_data;

config_data = 0x00;
i2c1.MemWrite(0xD0, 0x6B, I2C_MEMADD_SIZE_8BIT, &config_data, 1, 10); // 唤醒
config_data = 0x09;
i2c1.MemWrite(0xD0, 0x19, I2C_MEMADD_SIZE_8BIT, &config_data, 1, 10); // 采样率分频
config_data = 0x18;
i2c1.MemWrite(0xD0, 0x1B, I2C_MEMADD_SIZE_8BIT, &config_data, 1, 10); // 陀螺仪量程
```

#### 例 3：批量读取传感器数据

```cpp
// 读取 MPU6050 14 字节加速度 + 温度 + 陀螺仪数据
uint8_t raw[14];
if (i2c1.MemRead(0xD0, 0x3B, I2C_MEMADD_SIZE_8BIT, raw, 14, 10)) {
    int16_t accX  = (raw[0]  << 8) | raw[1];
    int16_t accY  = (raw[2]  << 8) | raw[3];
    int16_t accZ  = (raw[4]  << 8) | raw[5];
    int16_t temp  = (raw[6]  << 8) | raw[7];
    int16_t gyroX = (raw[8]  << 8) | raw[9];
    int16_t gyroY = (raw[10] << 8) | raw[11];
    int16_t gyroZ = (raw[12] << 8) | raw[13];
    // 6 轴数据就绪
}
```

#### 例 4：中断方式接收（配合 MPU6050 等传感器）

```cpp
// 发起中断接收（需 CubeMX 中使能 I2C 中断）
// 注意：本库未封装 _IT 函数，需直接调用 HAL
HAL_I2C_Mem_Read_IT(i2c1.m_hi2c, 0xD0, 0x3B, I2C_MEMADD_SIZE_8BIT, rx_buf, 14);

// 在 main 循环或回调中检测
if (i2c1.m_RxFlag) {
    i2c1.m_RxFlag = 0;
    // rx_buf 中数据就绪，非阻塞方式拿到结果
}
```

### 构造函数

```cpp
I2CDevice(I2C_HandleTypeDef *hi2c);
```
- 传入 HAL I2C 句柄（`&hi2c1`、`&hi2c2`）
- `NULL` 不注册，实例无用

### API

#### IsDeviceReady
```cpp
uint8_t IsDeviceReady(uint16_t DevAddress, uint32_t Trials, uint32_t Timeout);
```
- 轮询设备应答，内部调用 `HAL_I2C_IsDeviceReady()`
- **返回:** `1` 在线 / `0` 超时

#### MasterTransmit / MasterReceive
```cpp
uint8_t MasterTransmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
uint8_t MasterReceive(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
```
- 无寄存器寻址的原始数据收发（适合 OLED 等不需要寄存器地址的器件）
- 内部调用 `HAL_I2C_Master_Transmit()` / `HAL_I2C_Master_Receive()`
- **返回:** `1` 成功 / `0` 超时或错误

#### MemWrite / MemRead
```cpp
uint8_t MemWrite(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
                 uint8_t *pData, uint16_t Size, uint32_t Timeout);
uint8_t MemRead (uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
                 uint8_t *pData, uint16_t Size, uint32_t Timeout);
```
- 带寄存器寻址的读写（适合 MPU6050、EEPROM 等有寄存器映射的器件）
- `MemAddSize`: `I2C_MEMADD_SIZE_8BIT` 或 `I2C_MEMADD_SIZE_16BIT`
- 内部调用 `HAL_I2C_Mem_Write()` / `HAL_I2C_Mem_Read()`
- **返回:** `1` 成功 / `0` 超时或错误

#### SlaveTransmit / SlaveReceive
```cpp
uint8_t SlaveTransmit(uint8_t *pData, uint16_t Size, uint32_t Timeout);
uint8_t SlaveReceive(uint8_t *pData, uint16_t Size, uint32_t Timeout);
```
- 从机模式收发
- **返回:** `1` 成功 / `0` 超时或错误

### 中断回调

| HAL 回调 | 设置的标志 |
|---|---|
| `HAL_I2C_MasterRxCpltCallback` | `m_RxFlag = 1` |
| `HAL_I2C_MemRxCpltCallback` | `m_RxFlag = 1` |
| `HAL_I2C_ErrorCallback` | 用户自行实现 |

> 发送方法全部是阻塞的。

---

## 二、USARTDevice — 串口驱动

### 依赖
- STM32 HAL: `usart.h`
- 标准库: `<stdarg.h>`、`<stdio.h>`

### 最大实例数
6

### 使用示例

#### 例 1：调试打印（阻塞 printf）

```cpp
#include "usart_drv.h"

USARTDevice uart1(&huart1);

// 基础打印
uart1.printf("System boot OK\r\n");
uart1.printf("Voltage: %d.%02d V\r\n", 12, 34);        // "Voltage: 12.34 V"
uart1.printf("Pos: %d, Vel: %d, Cur: %.2f\r\n", pos, vel, cur);
```

#### 例 2：非阻塞 DMA 打印（高频发送不卡主循环）

```cpp
// CubeMX 中需配置 USART1_TX 的 DMA
USARTDevice uart1(&huart1);

// 高频日志 —— 不会卡住主循环
while (1) {
    uart1.DMA_printf("timestamp=%lu\r\n", HAL_GetTick());
    // DMA 忙时 printf 会自动丢弃，不影响控制周期
    HAL_Delay(1);
}
```

#### 例 3：IDLE + DMA 不定长接收（最常用）

```cpp
uint8_t rx_buf[256];
USARTDevice uart2(&huart2);

// 启动 DMA + IDLE 接收
uart2.IdleRx_DMA(rx_buf, sizeof(rx_buf));

while (1) {
    if (uart2.m_RxFlag) {
        uint16_t len = uart2.m_RxLen;   // 实际收到字节数
        uart2.m_RxFlag = 0;

        // 处理收到的数据
        // 例如：协议解析、命令匹配
        if (len >= 8 && rx_buf[0] == 0xAA) {
            // 帧头 0xAA，后面是数据
        }

        // ★ 必须重新启动接收
        uart2.IdleRx_DMA(rx_buf, sizeof(rx_buf));
    }
}
```

#### 例 4：发送二进制数据（传感器原始值、协议帧）

```cpp
// 发送单个 uint16_t 小端序
uint16_t encoder = 4096;
uart1.Send_U16(encoder, 10);

// 发送自定义协议帧
uint8_t frame[] = {0xAA, 0x01, 0x02, 0x03, 0x55};
uart1.Send_Block(frame, sizeof(frame), 10);

// 发送单个字节应答
uart1.Send_U8(0x06, 5);  // ACK
```

#### 例 5：阻塞接收单字节（简单指令交互）

```cpp
uint8_t cmd;
if (uart1.Read_U8(&cmd, 100)) {  // 等 100ms
    switch (cmd) {
        case 'A': /* 执行 A 指令 */ break;
        case 'B': /* 执行 B 指令 */ break;
    }
} else {
    // 超时，没有收到数据
}
```

### 构造函数

```cpp
USARTDevice(UART_HandleTypeDef *huart);
```
- 传入 HAL UART 句柄

### API

#### Send_U8 / Send_U16 / Send_U32
```cpp
uint8_t Send_U8 (uint8_t  byte,     uint32_t timeout);
uint8_t Send_U16(uint16_t U16_Data, uint32_t timeout);
uint8_t Send_U32(uint32_t U32_Data, uint32_t timeout);
```
- 发送 8/16/32 位数据（小端序），内部调用 `HAL_UART_Transmit()`
- **返回:** `1` 成功 / `0` 超时或 `m_huart == NULL / timeout == 0`

#### Send_Block
```cpp
uint8_t Send_Block(uint8_t *pData, uint16_t len, uint32_t timeout);
```
- 发送指定长度数据块，内部调用 `HAL_UART_Transmit()`
- `pData == NULL` 或 `len == 0` 也返回 0
- **返回:** `1` 成功 / `0` 失败

#### Read_U8
```cpp
uint8_t Read_U8(uint8_t *pData, uint32_t timeout);
```
- 阻塞接收 1 字节，内部调用 `HAL_UART_Receive()`
- **返回:** `1` 成功（数据在 `*pData`） / `0` 超时或参数无效

#### DMA_Start / DMA_Stop
```cpp
uint8_t DMA_Start(uint8_t *usart_rx_buf, uint16_t USART_RX_BUF_LEN);
uint8_t DMA_Stop();
```
- 启动/停止 DMA 接收，内部分别调用 `HAL_UART_Receive_DMA()` / `HAL_UART_DMAStop()`
- **返回:** `1` 成功 / `0` 失败

#### IdleRx_DMA
```cpp
uint8_t IdleRx_DMA(uint8_t *pData, uint16_t Size);
```
- 启动 IDLE 中断 + DMA 不定长接收，需在 CubeMX 中使能 UART 全局中断
- 内部调用 `HAL_UARTEx_ReceiveToIdle_DMA()`
- 收到一帧后 `m_RxFlag = 1`，`m_RxLen = 实际长度`
- **每次接收完成后必须重新调用以继续接收**

#### printf
```cpp
uint8_t printf(const char *format, ...);
```
- 阻塞格式化打印，内部缓冲区 `m_TxBuf[128]`
- 内部逻辑: `vsnprintf()` → `Send_Block()`，超时固定 15ms
- 超长自动截断到 127 字符
- **返回:** `1` 成功 / `0` 格式化失败

#### DMA_printf
```cpp
uint8_t DMA_printf(const char *format, ...);
```
- 非阻塞 DMA 打印，需 CubeMX 配置 UART TX DMA
- DMA 忙时直接丢弃（`gState != READY` 返回 0）
- 内部逻辑: 检查 `hdmatx != NULL` 且空闲 → 格式化 → `HAL_UART_Transmit_DMA()`
- **返回:** `1` 已排队 / `0` DMA 未配置、忙、或失败

> `printf` 和 `DMA_printf` 共享 `m_TxBuf`，**不可重入**

### 中断回调

| 回调 | 行为 |
|---|---|
| `HAL_UARTEx_RxEventCallback` | `m_RxLen = Size`，`m_RxFlag = 1` |
| `HAL_UART_ErrorCallback` | 遍历匹配实例，用户自行处理 |

---

## 三、CANDevice — CAN 总线驱动

### 依赖
- STM32 HAL: `can.h`

### 最大实例数
3

### CanMsg 结构体

```cpp
typedef struct {
    uint32_t ID;       // STD: 0x000~0x7FF / EXT: 0x000~0x1FFFFFFF
    uint8_t  IDE;      // CAN_ID_STD 或 CAN_ID_EXT
    uint8_t  RTR;      // CAN_RTR_DATA 或 CAN_RTR_REMOTE
    uint8_t  DLC;      // 数据长度 0~8
    uint8_t  Data[8];  // 固定 8 字节
} CanMsg;
```

### 使用示例

#### 例 1：完整收发流程（RoboMaster 电机）

```cpp
#include "can_drv.h"

CANDevice can1(&hcan1);

// 配置过滤器：接收电机反馈 (ID 0x201~0x204)
can1.Filter_Config(0, CAN_RX_FIFO0, 0x201);
can1.Filter_Config(1, CAN_RX_FIFO0, 0x202);
can1.Filter_Config(2, CAN_RX_FIFO0, 0x203);
can1.Filter_Config(3, CAN_RX_FIFO0, 0x204);

// 使能中断 + 启动
can1.IT_Config(CAN_IT_RX_FIFO0_MSG_PENDING);
can1.Start();

// 发送电机控制帧 (ID 0x200, 8 字节)
CanMsg tx;
tx.ID   = 0x200;
tx.IDE  = CAN_ID_STD;
tx.RTR  = CAN_RTR_DATA;
tx.DLC  = 8;
tx.Data[0] = 0x00; tx.Data[1] = 0x00;  // 电机1控制值: 0
tx.Data[2] = 0x10; tx.Data[3] = 0x00;  // 电机2控制值: 4096
tx.Data[4] = 0x00; tx.Data[5] = 0x00;  // 电机3
tx.Data[6] = 0x00; tx.Data[7] = 0x00;  // 电机4

if (can1.Send_Msg(&tx, 5)) {
    // 已排入发送邮箱
} else {
    // 发送失败：参数错 / 邮箱满超时
}

// 中断接收：回调自动把数据存入 can1.m_RxMsg
while (1) {
    if (can1.m_RxFlag) {
        can1.m_RxFlag = 0;

        uint32_t id  = can1.m_RxMsg.ID;      // 电机 ID 号
        uint8_t  dlc = can1.m_RxMsg.DLC;     // 数据长度

        // 解析电机反馈（M3508/M2006 格式）
        uint16_t encoder = (can1.m_RxMsg.Data[0] << 8) | can1.m_RxMsg.Data[1];
        int16_t  vel     = (can1.m_RxMsg.Data[2] << 8) | can1.m_RxMsg.Data[3];
        int16_t  current = (can1.m_RxMsg.Data[4] << 8) | can1.m_RxMsg.Data[5];
        uint8_t  temp    = can1.m_RxMsg.Data[6];

        // 按 ID 分发处理
        if (id == 0x201) { /* 电机1 反馈 */ }
        if (id == 0x202) { /* 电机2 反馈 */ }
    }
}
```

#### 例 2：发送自定义协议帧

```cpp
// 发送速度指令到 ID=0x300 的设备
CanMsg cmd;
cmd.ID   = 0x300;
cmd.IDE  = CAN_ID_STD;
cmd.RTR  = CAN_RTR_DATA;
cmd.DLC  = 4;
// 小端序写入 float 速度
float speed = 10.5f;
memcpy(cmd.Data, &speed, 4);

can1.Send_Msg(&cmd, 5);
```

#### 例 3：多 CAN 总线同时工作

```cpp
CANDevice can1(&hcan1);  // bus_idx = 0
CANDevice can2(&hcan2);  // bus_idx = 1

// 各自独立配置
can1.Filter_Config(0, CAN_RX_FIFO0, 0x201);
can1.IT_Config(CAN_IT_RX_FIFO0_MSG_PENDING);
can1.Start();

can2.Filter_Config(14, CAN_RX_FIFO0, 0x201);  // CAN2 从 Bank 14 开始
can2.IT_Config(CAN_IT_RX_FIFO0_MSG_PENDING);
can2.Start();

// 各自独立收发，通过 m_bus_idx 自动区分
if (can1.m_RxFlag) { /* can1 收到消息 */ }
if (can2.m_RxFlag) { /* can2 收到消息 */ }
```

### 构造函数

```cpp
CANDevice(CAN_HandleTypeDef *hcan);
```
- `m_bus_idx` 按创建顺序自动分配（0, 1, 2）
- 构造时清零 `m_RxMsg`

### API

#### Filter_Config
```cpp
uint8_t Filter_Config(uint8_t FilterBank, uint32_t FilterFIFO, uint16_t ID);
```
| 参数 | 说明 |
|---|---|
| `FilterBank` | Bank 号（CAN1: 0~13, CAN2: 14~27） |
| `FilterFIFO` | `CAN_RX_FIFO0` 或 `CAN_RX_FIFO1` |
| `ID` | 11 位标准 ID |

- 内部逻辑: 32-bit ID Mask 模式，`FilterIdHigh = (ID & 0x7FF) << 5`，Mask = `0x7FF << 5`，仅匹配 ID
- **返回:** `1` 成功 / `0` 失败

#### IT_Config
```cpp
uint8_t IT_Config(uint32_t ActiveITs);
```
- `ActiveITs`: 中断源，如 `CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING`
- 内部调用 `HAL_CAN_ActivateNotification()`
- **返回:** `1` 成功 / `0` 失败

#### Start
```cpp
uint8_t Start();
```
- 启动 CAN 总线，内部调用 `HAL_CAN_Start()`
- **返回:** `1` 成功 / `0` 失败

#### Send_Msg
```cpp
uint8_t Send_Msg(CanMsg *TxMsg, uint32_t time_out);
```
- 支持标准帧和扩展帧（由 `TxMsg->IDE` 决定）
- **内部逻辑:**
  1. 参数校验: NULL、DLC > 8、IDE 合法性、ID 范围
  2. 填充 `CAN_TxHeader`
  3. **轮询等待空闲邮箱**: 检查 `CAN_FLAG_TME0/1/2`，超时则返回 0
  4. `HAL_CAN_AddTxMessage()` 排入邮箱
- **返回:** `1` 已排入 / `0` 参数错误、超时或 HAL 错误
- 阻塞调用，会等待 `time_out` ms

#### Read_Msg
```cpp
uint8_t Read_Msg(uint32_t RxFIFO, CanMsg *RxMsg);
```
- `RxFIFO`: `CAN_RX_FIFO0` / `CAN_RX_FIFO1`，`RxMsg` 为输出参数
- 内部逻辑: `HAL_CAN_GetRxMessage()` → 提取 DLC/IDE/RTR → 根据 IDE 赋值 StdId/ExtId
- **返回:** `1` 成功 / `0` 失败

### 中断回调

| 回调 | 行为 |
|---|---|
| `HAL_CAN_RxFifo0MsgPendingCallback` | `Read_Msg(FIFO0, &m_RxMsg)` → `m_RxFlag = 1` |
| `HAL_CAN_RxFifo1MsgPendingCallback` | `Read_Msg(FIFO1, &m_RxMsg)` → `m_RxFlag = 1` |

回调通过遍历已注册实例匹配 `m_hcan`，无需硬编码 CAN1/CAN2 判断。

---

## 四、TIMDevice — 定时器驱动

### 依赖
- STM32 HAL: `tim.h`

### 最大实例数
14

### 使用示例

#### 例 1：控制循环定时器（1kHz）

```cpp
// CubeMX: TIM6 配置 Prescaler=83, Counter Period=999 → 1kHz (84MHz 时钟)
#include "timer_drv.h"

TIMDevice tim6(&htim6);
tim6.Start();

while (1) {
    if (tim6.m_Flag) {
        tim6.m_Flag = 0;

        // 1kHz 执行：PID 计算、传感器读取、状态机
        pid_calc();
        sensor_update();
        state_machine();
    }
}
```

#### 例 2：多个定时器不同频率

```cpp
TIMDevice tim6(&htim6);  // 1kHz
TIMDevice tim7(&htim7);  // 100Hz

tim6.Start();
tim7.Start();

while (1) {
    if (tim6.m_Flag) {
        tim6.m_Flag = 0;
        // 1kHz 快循环：电机控制
    }
    if (tim7.m_Flag) {
        tim7.m_Flag = 0;
        // 100Hz 慢循环：状态上报、OLED 刷新
    }
}
```

#### 例 3：精确延时计数

```cpp
TIMDevice tim6(&htim6);  // 1ms 周期
tim6.Start();

uint32_t elapsed_ms = 0;
while (elapsed_ms < 500) {   // 等待 500ms
    if (tim6.m_Flag) {
        tim6.m_Flag = 0;
        elapsed_ms++;
    }
}
uart1.printf("500ms elapsed\r\n");
```

### 构造函数

```cpp
TIMDevice(TIM_HandleTypeDef *htim);
```

### API

#### Start
```cpp
uint8_t Start();
```
- 启动时基中断，内部调用 `HAL_TIM_Base_Start_IT()`
- **前置条件:** CubeMX 中配置好 Prescaler、Counter Period、使能 NVIC
- **返回:** `1` 成功 / `0` 失败

### 中断回调

| 回调 | 行为 |
|---|---|
| `HAL_TIM_PeriodElapsedCallback` | 遍历已注册实例匹配 `htim`，设置 `m_Flag = 1` |

> 这是最简封装，仅提供 Period Elapsed 中断。PWM/编码器模式还未加入。
