# 智能家居控制系统 - 代码架构分析

## 一、系统概述

这是一个基于STM32F103的智能家居环境监测与控制系统，实现了：
- 环境监测（温湿度、光照、空气质量）
- 智能窗户/窗帘控制（基于光照）
- 空气质量自动通风（开窗+风扇）
- 蓝牙通信（HC-05）

---

## 二、硬件架构

### 2.1 MCU配置
- **型号**: STM32F103
- **系统时钟**: 72MHz (HSE 8MHz × PLL9)
- **外设配置**:
  - ADC1: MQ135空气质量传感器
  - I2C1: BH1750光照传感器
  - TIM2: PWM输出（风扇调速）
  - USART1: HC-05蓝牙模块
  - DMA: ADC、UART数据传输

### 2.2 GPIO引脚分配

#### 传感器输入
| 引脚 | 功能 | 说明 |
|------|------|------|
| PA1 | ADC1_CH1 | MQ135空气质量传感器 |
| PB6 | I2C1_SCL | BH1750光照传感器 |
| PB7 | I2C1_SDA | BH1750光照传感器 |
| PB12 | DHT22_Data | DHT22温湿度传感器 |

#### 执行器输出
| 引脚组 | 功能 | 驱动器 | 说明 |
|--------|------|--------|------|
| PA2-PA5 | Motor1 | ULN2003 | 窗户电机（相对模式）|
| PA8-PA11 | Motor2 | ULN2003 | 窗帘电机（位置模式）|
| PA0 | Fan | GPIO/PWM | 风扇控制 |

#### 通信接口
| 引脚 | 功能 | 说明 |
|------|------|------|
| PA9 | USART1_TX | HC-05蓝牙发送 |
| PA10 | USART1_RX | HC-05蓝牙接收 |

---

## 三、软件模块架构

```
┌─────────────────────────────────────────────────────────────┐
│                         主程序 (main.c)                      │
│                    ├── 初始化                                │
│                    ├── 主循环                                │
│                    └── 外设配置                              │
└─────────────────────────────────────────────────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
   ┌────▼────┐         ┌──────▼─────┐       ┌──────▼─────┐
   │ 传感器层 │         │  控制逻辑层 │       │  执行器层   │
   └─────────┘         └────────────┘       └────────────┘
        │                     │                     │
   ┌────┴────┐         ┌──────┴─────┐       ┌──────┴─────┐
   │         │         │            │       │            │
BH1750   MQ135      光照控制   空气质量    窗户电机   风扇
DHT22                       控制
```

---

## 四、核心模块详解

### 4.1 传感器模块

#### 4.1.1 BH1750光照传感器 (gy_30.c/h)
**功能**: 测量环境光照强度，生成窗户/窗帘控制标志
- **接口**: I2C通信
- **测量范围**: 0-65535 lux
- **读取频率**: 500ms/次

**控制逻辑 - 三态控制**:
```
窗户控制:
  - 强光 (>2000 lux)    → 关窗
  - 适中 (50-2000 lux)  → 开窗
  - 弱光 (<50 lux)      → 关窗

窗帘控制:
  - 强光 (>1500 lux)    → 全关
  - 适中 (300-1500 lux) → 半开
  - 弱光 (<300 lux)     → 全开
```

**关键特性**:
- 滞后控制（10 lux）：防止频繁切换
- 标志位机制：传感器设置标志，电机响应标志
- 状态跟踪：记录当前窗户/窗帘状态

#### 4.1.2 MQ135空气质量传感器 (MQ135.c/h)
**功能**: 检测空气中有害气体浓度
- **接口**: ADC1_CH1 (DMA模式)
- **采样率**: 连续采样
- **判断阈值**: ADC原始值 > 2000 = 空气质量差

**核心函数**:
```c
uint8_t air_quality_is_bad()
  - 返回 1: 空气质量差（需要通风）
  - 返回 0: 空气质量正常
```

#### 4.1.3 DHT22温湿度传感器 (dht.c/h)
**功能**: 测量温度和湿度
- **接口**: 单总线协议 (PB12)
- **测量范围**:
  - 温度: -40~80°C (精度±0.5°C)
  - 湿度: 0~100%RH (精度±2%RH)

---

### 4.2 执行器模块

#### 4.2.1 ULN2003步进电机驱动 (motor_uln2003.c/h)

**驱动方式**: 4相8拍
**电机型号**: 28BYJ-48
**步进参数**:
- 1圈 = 2048步
- 步进延时: 2ms
- 驱动逻辑: 低电平导通（ULN2003反向驱动）

**双模式控制**:

**模式1: 相对模式 (MOTOR_MODE_RELATIVE)**
```c
用途: 窗户控制 (Motor1)
特点: 每次移动固定步数
操作:
  - Open():  正转 relative_steps 步
  - Close(): 反转 relative_steps 步
  - Half():  正转 relative_steps/2 步
```

**模式2: 位置模式 (MOTOR_MODE_POSITION)**
```c
用途: 窗帘控制 (Motor2)
特点: 移动到绝对位置（带位置跟踪）
操作:
  - MoveToOpen():  移动到位置0（全开）
  - MoveToHalf():  移动到位置max/2（半开）
  - MoveToClose(): 移动到位置max（全关）
位置跟踪: 自动记录当前位置，避免重复移动
```

**相位表** (4相8拍):
```
Phase 0: 0001 (A)
Phase 1: 0011 (AB)
Phase 2: 0010 (B)
Phase 3: 0110 (BC)
Phase 4: 0100 (C)
Phase 5: 1100 (CD)
Phase 6: 1000 (D)
Phase 7: 1001 (DA)
```

#### 4.2.2 风扇驱动模块 (fan.c/h)

**控制引脚**: PA0
**驱动原理**: 低电平恒速转 / PWM调速

**双模式控制**:

**模式1: 恒速模式 (FAN_MODE_CONSTANT)**
```c
实现: GPIO输出低电平
特点: 最大转速，简单可靠
用途: 空气质量通风
```

**模式2: PWM调速模式 (FAN_MODE_PWM)**
```c
实现: TIM2_CH1 PWM输出
占空比: 0-100%
PWM频率: ~1kHz
用途: 精确速度控制
```

**模式切换**: 自动处理GPIO/PWM复用配置

---

### 4.3 控制逻辑层

#### 4.3.1 光照自动控制逻辑 (main.c: 170-226行)

**流程**:
```
1. LightSensor_Process() 读取光照，设置标志位
2. 获取窗户控制标志 (window_flag)
   ├─ WINDOW_FLAG_OPEN  → Motor1 Open()
   └─ WINDOW_FLAG_CLOSE → Motor1 Close()
3. 获取窗帘控制标志 (curtain_flag)
   ├─ CURTAIN_FLAG_OPEN  → Motor2 MoveToOpen()
   ├─ CURTAIN_FLAG_HALF  → Motor2 MoveToHalf()
   └─ CURTAIN_FLAG_CLOSE → Motor2 MoveToClose()
4. Motor_ULN2003_Process() 执行步进
5. 更新状态反馈给光照模块
```

**防重复执行机制**:
```c
// 检查电机状态，避免重复触发
if (wflag == WINDOW_FLAG_OPEN && motor1.state != MOTOR_ULN2003_OPENING)
if (motor2.state != MOTOR_ULN2003_MOVING && motor2.target_position != 0)
```

#### 4.3.2 空气质量自动通风控制 (air_quality_control.c/h)

**控制策略**:
```
检测 → 判断 → 执行
   │      │      └→ 开窗 + 启动风扇
   │      └→ 空气质量是否差？
   └→ 调用 air_quality_is_bad()

停止条件:
  - 空气质量变好 → 立即停止风扇
  - 手动控制介入 → 立即停止自动通风
```

**优先级管理**:
```
PRIORITY_MANUAL (最高)          手动控制
    ↓
PRIORITY_AIR_QUALITY (高)       空气质量控制
    ↓
PRIORITY_LIGHT (低)             光照控制
    ↓
PRIORITY_NONE (无)              空闲
```

**智能窗户管理**:
```c
StartVentilation():
  1. 检查窗户是否已开
  2. 如果窗户关闭 → 打开窗户
  3. 启动风扇恒速运转
  4. 标记 window_opened_by_air_ctrl = 1

StopVentilation():
  1. 停止风扇
  2. 窗户保持当前状态（不自动关闭）
  3. 清除标记
```

---

### 4.4 通信模块

#### 4.4.1 HC-05蓝牙模块 (HC05.c/h)

**配置**:
```c
波特率: 9600
数据位: 8位
停止位: 1位
校验位: 无
接口: USART1 + DMA
```

**数据传输** (main.c: 229-241行):
```c
发送频率: 2秒/次
数据格式: "T:25.3,H:60.2,L:500,A:95\r\n"
说明:
  - T: 温度 (°C)
  - H: 湿度 (%RH)
  - L: 光照 (lux)
  - A: 空气质量
```

---

## 五、主循环执行流程

```c
while(1) {
    // 1. 读取DHT22温湿度
    DHT22_update();
    temp = get_temperature();
    hum = get_humidity();

    // 2. 空气质量自动控制（高优先级）
    AirQualityControl_Process(&hAirCtrl, PRIORITY_NONE);
    Control_Priority air_priority = AirQualityControl_GetPriority(&hAirCtrl);

    // 3. 光照传感器处理（设置标志位）
    LightSensor_Process(&hlsensor);
    g_light_lux = LightSensor_GetLux(&hlsensor);

    // 4. 光照控制窗户（带优先级判断）
    if (air_priority < PRIORITY_AIR_QUALITY) {
        // 空气质量控制未激活，执行光照控制
        wflag = LightSensor_GetWindowFlag(&hlsensor);
        if (wflag == WINDOW_FLAG_OPEN && motor1.state != MOTOR_ULN2003_OPENING) {
            Motor_ULN2003_Open(&motor1);
            LightSensor_ClearWindowFlag(&hlsensor);
        }
        else if (wflag == WINDOW_FLAG_CLOSE && motor1.state != MOTOR_ULN2003_CLOSING) {
            Motor_ULN2003_Close(&motor1);
            LightSensor_ClearWindowFlag(&hlsensor);
        }
    }
    else {
        // 空气质量控制激活中，光照控制被抑制
        LightSensor_ClearWindowFlag(&hlsensor);
    }

    // 5. 光照控制窗帘（不受空气质量影响）
    cflag = LightSensor_GetCurtainFlag(&hlsensor);
    if (cflag == CURTAIN_FLAG_OPEN && motor2.target_position != 0) {
        Motor_ULN2003_MoveToOpen(&motor2);
        LightSensor_ClearCurtainFlag(&hlsensor);
    }
    else if (cflag == CURTAIN_FLAG_HALF && motor2.target_position != max/2) {
        Motor_ULN2003_MoveToHalf(&motor2);
        LightSensor_ClearCurtainFlag(&hlsensor);
    }
    else if (cflag == CURTAIN_FLAG_CLOSE && motor2.target_position != max) {
        Motor_ULN2003_MoveToClose(&motor2);
        LightSensor_ClearCurtainFlag(&hlsensor);
    }

    // 6. 执行电机步进
    Motor_ULN2003_Process(&motor1);  // 窗户
    Motor_ULN2003_Process(&motor2);  // 窗帘

    // 7. 更新窗户状态反馈
    if (Motor_ULN2003_GetState(&motor1) == MOTOR_ULN2003_OPEN) {
        LightSensor_UpdateWindowState(&hlsensor, WINDOW_FLAG_OPEN);
    }

    // 8. 蓝牙数据发送（每2秒）
    if (HAL_GetTick() - last_send_time >= 2000) {
        HC05_SendString(buffer);
        last_send_time = HAL_GetTick();
    }
}
```

---

## 六、空气质量控制集成完成

### 6.1 集成状态
✅ 已完成集成:
- `control_priority.h` - 系统级优先级定义
- `air_quality_control.c/h` - 空气质量控制逻辑
- `fan.c/h` - 风扇驱动（双模式）
- `MQ135.c/h` - 空气质量传感器
- `main.c` - 主循环集成完成

### 6.2 已添加的代码

**头文件引用**:
```c
#include "fan.h"
#include "air_quality_control.h"
#include "control_priority.h"
```

**全局变量声明**:
```c
Fan_HandleTypeDef hfan;
Air_Quality_Control_HandleTypeDef hAirCtrl;
```

**初始化代码**:
```c
// 初始化风扇 (PA0, TIM2_CH1)
Fan_Init(&hfan, GPIOA, GPIO_PIN_0, &htim2, TIM_CHANNEL_1);

// 初始化空气质量控制模块
AirQualityControl_Init(&hAirCtrl, &motor1, &hfan);
```

**主循环集成**:
```c
// 空气质量自动控制（高优先级）
AirQualityControl_Process(&hAirCtrl, PRIORITY_NONE);
Control_Priority air_priority = AirQualityControl_GetPriority(&hAirCtrl);

// 光照控制带优先级判断
if (air_priority < PRIORITY_AIR_QUALITY) {
    // 执行光照控制窗户
}
else {
    // 光照控制被抑制
    LightSensor_ClearWindowFlag(&hlsensor);
}
```

### 6.3 优先级协调机制

**工作流程**:
```
空气质量差 (MQ135 > 2000)
    ↓
优先级提升: PRIORITY_AIR_QUALITY
    ↓
打开窗户 + 启动风扇
    ↓
光照控制被抑制
    ↓
持续监测
    ↓
空气质量恢复 (MQ135 ≤ 2000)
    ↓
优先级降低: PRIORITY_NONE
    ↓
立即停止风扇
    ↓
光照控制恢复工作
```

---

## 七、架构改进

### 7.1 优先级定义解耦

**重构前**:
```
air_quality_control.h
└── 定义 Control_Priority (❌ 耦合)
```

**重构后**:
```
control_priority.h (系统级公共定义)
├── Control_Priority 枚举
├── 优先级比较宏
└── 调试辅助函数

air_quality_control.h
└── #include "control_priority.h" (✅ 解耦)

gy_30.h (未来扩展)
└── #include "control_priority.h" (✅ 解耦)
```

---

## 八、关键设计模式

### 8.1 标志位机制
```
传感器模块 → 设置标志位 → 执行器模块响应标志位 → 清除标志位
优点: 解耦传感器和执行器，便于扩展
```

### 8.2 状态机模式
```c
电机状态: IDLE → OPENING/CLOSING/MOVING → OPEN/CLOSED/AT_POSITION
风扇状态: STOPPED → RUNNING
控制状态: IDLE → DETECTING → VENTILATING
```

### 8.3 双模式设计
```
电机: 相对模式 / 位置模式
风扇: 恒速模式 / PWM调速模式
优点: 同一硬件支持不同应用场景
```

### 8.4 滞后控制
```c
阈值 + 滞后量 / 阈值 - 滞后量
作用: 防止边界振荡，提高稳定性
```

### 8.5 优先级协调
```
动态优先级调整 + 外部优先级查询 + 主循环判断
作用: 实现多控制源的智能协调
```

---

## 九、系统工作场景示例

### 场景A: 清晨 (光照逐渐增强)
```
1. 光照 < 50 lux (夜晚)
   - 窗户: 关闭
   - 窗帘: 全开

2. 光照 50 → 500 lux (清晨)
   - 窗户: 打开 ✓
   - 窗帘: 半开 ✓

3. 光照 > 2000 lux (强光)
   - 窗户: 关闭 ✓
   - 窗帘: 全关 ✓
```

### 场景B: 空气质量异常
```
1. 检测到MQ135 > 2000 (空气质量差)
   - 优先级提升: PRIORITY_AIR_QUALITY
   - 检查窗户状态

2. 如果窗户关闭
   - 打开窗户

3. 启动风扇恒速运转
   - PA0输出低电平

4. 持续监测
   - MQ135 ≤ 2000 → 立即停止风扇
```

### 场景C: 冲突处理 (强光 + 空气质量差)
```
执行顺序:
1. 空气质量控制优先
   - 打开窗户（通风需要）
   - 启动风扇

2. 光照控制被抑制
   - 不会因为强光关闭窗户

3. 空气质量恢复后
   - 停止风扇
   - 光照控制恢复
```

---

## 十、总结

这是一个**设计良好且已完整集成**的智能家居控制系统，具有清晰的模块化架构和合理的控制逻辑。

### 主要特色

1. **双电机系统**: 窗户用相对模式，窗帘用位置模式，各司其职
2. **智能光照控制**: 三态控制 + 滞后机制，稳定可靠
3. **空气质量监测**: 自动通风系统，健康保障
4. **智能优先级管理**: 空气质量 > 光照控制，动态协调
5. **蓝牙通信**: 实时数据传输，便于监控
6. **解耦架构**: 优先级定义独立，易于扩展

### 系统完整性

✅ **已完成功能**:
- 环境监测（温湿度、光照、空气质量）
- 智能窗户/窗帘控制（基于光照）
- 空气质量自动通风（开窗+风扇）
- 优先级协调机制
- 蓝牙数据传输
- 模块化架构重构

### 技术亮点

1. **优先级定义解耦**: 创建`control_priority.h`作为系统级公共定义
2. **动态优先级调整**: 根据传感器数据实时调整控制优先级
3. **非侵入式集成**: 通过外部优先级判断实现控制协调
4. **双模式支持**: 电机、风扇均支持两种工作模式
5. **滞后控制**: 防止阈值边界频繁切换

### 下一步建议

1. **功能增强**:
   - 实现蓝牙命令接收（手动控制模式）
   - 添加MQ135实时数据到蓝牙传输
   - 增加系统状态LED指示

2. **性能优化**:
   - 添加主循环延时（避免CPU占用过高）
   - 优化传感器读取频率
   - 实现低功耗模式

3. **可靠性提升**:
   - 添加错误处理和异常恢复
   - 实现看门狗保护
   - 添加传感器数据有效性检查

4. **扩展功能**:
   - 增加温度控制（空调/加热器）
   - 增加湿度控制（加湿器/除湿机）
   - 实现场景模式（在家/外出/睡眠）

---

## 附录：文件清单

### 核心驱动文件
- `Application/Inc/motor_uln2003.h/c` - 双模式电机驱动
- `Application/Inc/fan.h/c` - 双模式风扇驱动
- `Application/Inc/gy_30.h/c` - 光照传感器模块
- `Application/Inc/MQ135.h/c` - 空气质量传感器
- `Application/Inc/dht.h/c` - 温湿度传感器
- `Application/Inc/HC05.h/c` - 蓝牙通信模块

### 控制逻辑文件
- `Application/Inc/control_priority.h` - 系统级优先级定义 ⭐
- `Application/Inc/air_quality_control.h/c` - 空气质量控制 ⭐

### 主程序文件
- `Core/Src/main.c` - 主程序及控制协调 ⭐

### 配置文件
- `CMakeLists.txt` - 项目构建配置

⭐ 标记为本次修改/新增的关键文件

---

**系统状态**: ✅ 完整集成，可投入测试
**架构评级**: ⭐⭐⭐⭐⭐ 优秀
**代码质量**: 模块化、解耦、易维护

