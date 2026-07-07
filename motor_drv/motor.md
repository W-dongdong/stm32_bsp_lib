# M2006 / M3508 — 电机控制库

> M2006 和 M3508 接口一致，以下用 `Motor` 泛指。差异仅在于默认减速比和 M3508 独有的 ActiveRecovery（本文档不涉及）。

## 依赖
- `can_drv.h` — CAN 总线驱动
- `pid_drv.h` — PID 控制器

## 使用方式

三步走：**① 创建电机** → **② CAN 中断收反馈接收消息** → **③ 定时器发送控制电机**
循环通路打通之后，然后就可以在其他地方设置电机模式

### ① 创建电机并设置模式

```cpp
#include "can_drv.h"
#include "M2006_Lib.h"

// 初始化 CAN 总线
CANDevice can1(&hcan1);
can1.Filter_Config(0, CAN_RX_FIFO0, 0x201);
can1.IT_Config(CAN_IT_RX_FIFO0_MSG_PENDING);
can1.Start();

// 创建电机（CAN 总线, ID 1~8）
M2006 motor1(&can1, 1);
M2006 motor2(&can1, 2);

// ★ 调 PID 参数（必须！默认值只是占位，不调电机不动或不稳）
motor1.PID_Config(
    PID(20, 0, 5, 1500, 2000),            // 位置环: Kp, Ki, Kd, 积分限幅, 输出限幅
    PID(0.7, 0.01, 0.1, 2000, 15000)      // 速度环: Kp, Ki, Kd, 积分限幅, 输出限幅
);
motor2.PID_Config(
    PID(20, 0, 5, 1500, 2000),
    PID(0.7, 0.01, 0.1, 2000, 15000)
);

// 调完后设置控制模式
motor1.SpeedMode(3000);                    // 速度模式: 3000 rpm
motor2.PosSpeedMode(90.0f, 2000);         // 位置模式: 转到 90°, 限速 2000 rpm
```

### ② CAN 中断中解析反馈

CAN 中断触发后，`can1.m_RxMsg` 和 `can1.m_RxFlag` 自动就绪。在 **main 循环**中检测并调用 `ControlLoopUpdate`（不要在中断里调——里面会做 PID 计算和 Buffer 填充，不阻塞但也不该放 ISR）：

```cpp
// HAL_CAN_RxFifo0MsgPendingCallback 自动把数据写入 can1.m_RxMsg，m_RxFlag = 1
// 以下在 main 循环中处理：
if (can1.m_RxFlag) {
    can1.m_RxFlag = 0;

    // 解析反馈 + PID 计算 + 填入 TX Buffer（一行搞定）
    M2006::ControlLoopUpdate(&can1);

    // 此时 motor1.m_Vel、motor1.m_abs_Pos 等反馈变量已更新
}
```

`ControlLoopUpdate` 内部做了什么：
1. 根据 `can1.m_RxMsg.ID`（0x201~0x208）找到对应电机
2. `ParseFeedback()` — 解析编码器、转速、扭矩、温度
3. `pid_calc()` — 根据 `m_Mode` 执行对应 PID/扭矩/阻抗计算
4. `MsgAppend()` — 将控制值填入 TX Buffer

### ③ 定时器中发送控制帧

TX Buffer 填好后，在定时器中断（或 main 循环）中统一发出：

```cpp
// 定时器 1kHz 中断 → main 循环检测
if (tim6.m_Flag) {
    tim6.m_Flag = 0;

    // 发送所有电机的控制帧
    M2006::SendGroup(&can1, 0x200);  // ID 1~4
    M2006::SendGroup(&can2, 0x1FF);  // ID 5~8
}
```

> `SendGroup` 内部调用 `can->Send_Msg()`，会阻塞等待 CAN 邮箱（< 5ms默认值）。每帧发送后 TX Buffer 自动清零。
一般不建议一根总线上电机超过6个，CAN总线会满载并且触发大量的错误帧。
---

## 完整时序图

```
设置目标
  └→ motor1.SpeedMode(3000);                    // 速度模式: 3000 rpm
  └→ motor2.PosSpeedMode(90.0f, 2000);         // 位置模式: 转到 90°, 限速 2000


定时器 1kHz
  └→ SendGroup(0x200)  ──→ CAN 总线 ──→ 电机收到指令，动作
  └→ SendGroup(0x1FF)
                                    │
                                    ▼
CAN 中断 (电机反馈 0x201~0x208)
  └→ can1.m_RxMsg / m_RxFlag 就绪
       │
       ▼
main 循环检测 m_RxFlag
  └→ ControlLoopUpdate(&can1)
       ├─ ParseFeedback → 更新 m_Vel, m_abs_Pos, m_Torque_Curr...
       ├─ pid_calc()    → 算出控制值
       └─ MsgAppend()   → 填入 TX Buffer（下个定时器周期发出）
```

---

## PID 调参指南

**不调参电机无法正常工作。** 构造函数里的默认值是通用起点，必须根据实际负载调整。

### 速度环（SpeedMode）

```
PID(Kp, Ki, Kd, 积分限幅, 输出限幅)
```

| 参数 | 作用 | 调法 |
|---|---|---|
| `Kp` | 响应快慢 | 从小往大加，直到目标速度有明显超调再回调 20% |
| `Ki` | 消除稳态误差 | 先设为 0，Kp 调好后逐步加，能跟上目标即可 |
| `Kd` | 抑制震荡 | 出现抖动时加大，平稳后尽量小 |
| `积分限幅` | 防止积分饱和 | 设为输出限幅的 10%~20% |
| `输出限幅` | 最大输出电流值 | M2006 设 10000，M3508 设 15000 |

### 位置环（PosSpeedMode）

```
PID(Kp, Ki, Kd, 积分限幅, 输出限幅)
```

> 位置环输出的是**速度目标值**，所以 `输出限幅` = 最大允许转速 (rpm)。

| 参数 | 作用 | 调法 |
|---|---|---|
| `Kp` | 位置刚度 | 从小往大加，到达目标有轻微超调即可 |
| `Ki` | 消除位置静差 | 通常设很小以抵消摩擦力，除非有静态误差 |
| `Kd` | 抑制过冲 | 到达目标前"刹车"，超调大就加 |
| `积分限幅` | 同上 | 设小值（位置不需要大积分） |
| `输出限幅` | 最大转速 rpm | 就是你传的 `Speed_Limit` 值 |

### 建议调参顺序

```
① 先调速度环 Kp → ② 加 Ki（如果需要）→ ③ 加 Kd（如果有震荡）
→ ④ 再调位置环 Kp → ⑤ 加 Kd（如果过冲大）
不建议盲目增大Kd，速度噪音会被放大导致系统抖动
```


## 模式一览

```cpp
motor.SpeedMode(3000);                       // 速度模式: 目标转速 rpm
motor.PosSpeedMode(90.0f, 2000);             // 位置模式: 角度(°)+限速 rpm
motor.TorqueConstant_Config(0.3f);           // 扭矩常数 Nm/A（先配）
motor.TorqueMode(1.5f);                      // 扭矩模式: 目标扭矩 Nm
motor.MITMode(0.0f, 0.0f, 10.0f, 2.0f, 0);  // 阻抗模式: 角度,速度,kp,kd,ff
```

## 反馈变量

`ControlLoopUpdate` 后可直接读取：

| 变量 | 类型 | 含义 |
|---|---|---|
| `m_Vel` | `int16_t` | 转速 (rpm, 电机轴) |
| `m_abs_Pos` | `int32_t` | 绝对位置 (编码器脉冲) |
| `m_Torque_Curr` | `float` | 扭矩电流 (-20~20 A) |
| `m_Temp` | `uint8_t` | 温度 (°C) |
| `m_Mode` | `uint8_t` | 当前模式 (0~4) |

## 注意事项
- 目前 **MIT** 模式和 **扭矩模式** 的扭矩常数还没有测，请谨慎使用
- **不要在 ISR 中调 `SendGroup`**，它阻塞等邮箱。在 main 循环或定时器 flag 分支中调
- **不要在 ISR 中调 `ControlLoopUpdate`**，虽然不阻塞但含浮点 PID 运算，放 main 循环更安全
- M2006（C610 电调）控制值 ±10000，M3508（C620）±16384
- 程序上，每 CAN 总线最多 8 个电机，多总线独立工作
