/**
 * @file    gy_30.c
 * @brief   GY-30 光照控制模块
 * @note    根据光照强度自动控制窗户/窗帘开关
 *          使用 motor1 (PA2-PA5) 控制窗帘电机
 */

#include "gy_30.h"

/* 电机控制引脚定义 (使用 motor1: PA2, PA3, PA4, PA5) */
#define CURTAIN_MOTOR_PORT      GPIOA
#define CURTAIN_IN1_PIN         motor1_Pin      // PA2
#define CURTAIN_IN2_PIN         motor1A3_Pin    // PA3
#define CURTAIN_IN3_PIN         motor1A4_Pin    // PA4
#define CURTAIN_IN4_PIN         motor1A5_Pin    // PA5

/* 步进电机相序 (四相八拍) */
static const uint8_t step_sequence[8] = {
    0x01,  // 0001
    0x03,  // 0011
    0x02,  // 0010
    0x06,  // 0110
    0x04,  // 0100
    0x0C,  // 1100
    0x08,  // 1000
    0x09   // 1001
};

static uint8_t current_step = 0;

/**
 * @brief  设置步进电机相位
 * @param  phase: 相位值 (0-7)
 */
static void Curtain_SetPhase(uint8_t phase)
{
    uint8_t seq = step_sequence[phase & 0x07];

    HAL_GPIO_WritePin(CURTAIN_MOTOR_PORT, CURTAIN_IN1_PIN, (seq & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CURTAIN_MOTOR_PORT, CURTAIN_IN2_PIN, (seq & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CURTAIN_MOTOR_PORT, CURTAIN_IN3_PIN, (seq & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CURTAIN_MOTOR_PORT, CURTAIN_IN4_PIN, (seq & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  步进电机正转一步 (打开窗帘)
 */
static void Curtain_StepForward(void)
{
    current_step++;
    if (current_step >= 8) {
        current_step = 0;
    }
    Curtain_SetPhase(current_step);
}

/**
 * @brief  步进电机反转一步 (关闭窗帘)
 */
static void Curtain_StepBackward(void)
{
    if (current_step == 0) {
        current_step = 7;
    } else {
        current_step--;
    }
    Curtain_SetPhase(current_step);
}

/**
 * @brief  停止电机 (所有引脚置低)
 */
static void Curtain_Stop(void)
{
    HAL_GPIO_WritePin(CURTAIN_MOTOR_PORT, CURTAIN_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CURTAIN_MOTOR_PORT, CURTAIN_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CURTAIN_MOTOR_PORT, CURTAIN_IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CURTAIN_MOTOR_PORT, CURTAIN_IN4_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  初始化光照控制模块
 */
void LightCtrl_Init(LightCtrl_HandleTypeDef *hlctrl, BH1750_HandleTypeDef *hbh1750,
                    float threshold_high, float threshold_low)
{
    hlctrl->hbh1750 = hbh1750;
    hlctrl->lux_threshold_high = threshold_high;
    hlctrl->lux_threshold_low = threshold_low;
    hlctrl->current_lux = 0;
    hlctrl->state = LIGHT_CTRL_IDLE;
    hlctrl->mode = LIGHT_MODE_AUTO;
    hlctrl->hysteresis = 50;  // 默认滞后量 50 lx

    /* 初始化电机引脚为停止状态 */
    Curtain_Stop();
}

/**
 * @brief  光照控制处理函数
 */
void LightCtrl_Process(LightCtrl_HandleTypeDef *hlctrl)
{
    /* 读取当前光照值 */
    if (BH1750_ReadLight(hlctrl->hbh1750, &hlctrl->current_lux) != HAL_OK) {
        return;  // 读取失败，跳过本次处理
    }

    /* 手动模式下不自动控制 */
    if (hlctrl->mode == LIGHT_MODE_MANUAL) {
        return;
    }

    /* 自动控制逻辑 */
    switch (hlctrl->state) {
        case LIGHT_CTRL_IDLE:
        case LIGHT_CTRL_OPEN:
            /* 光照过强，需要关闭窗帘 */
            if (hlctrl->current_lux > hlctrl->lux_threshold_high + hlctrl->hysteresis) {
                hlctrl->state = LIGHT_CTRL_CLOSING;
            }
            break;

        case LIGHT_CTRL_CLOSED:
            /* 光照过弱，需要打开窗帘 */
            if (hlctrl->current_lux < hlctrl->lux_threshold_low - hlctrl->hysteresis) {
                hlctrl->state = LIGHT_CTRL_OPENING;
            }
            break;

        case LIGHT_CTRL_OPENING:
            /* 执行打开动作 - 步进电机正转 */
            Curtain_StepForward();
            HAL_Delay(2);  // 步进延时

            /* 检查是否达到目标 (这里简化处理，实际应使用限位开关) */
            if (hlctrl->current_lux >= hlctrl->lux_threshold_low) {
                Curtain_Stop();
                hlctrl->state = LIGHT_CTRL_OPEN;
            }
            break;

        case LIGHT_CTRL_CLOSING:
            /* 执行关闭动作 - 步进电机反转 */
            Curtain_StepBackward();
            HAL_Delay(2);  // 步进延时

            /* 检查是否达到目标 */
            if (hlctrl->current_lux <= hlctrl->lux_threshold_high) {
                Curtain_Stop();
                hlctrl->state = LIGHT_CTRL_CLOSED;
            }
            break;
    }
}

/**
 * @brief  设置光照阈值
 */
void LightCtrl_SetThreshold(LightCtrl_HandleTypeDef *hlctrl,
                            float threshold_high, float threshold_low)
{
    hlctrl->lux_threshold_high = threshold_high;
    hlctrl->lux_threshold_low = threshold_low;
}

/**
 * @brief  设置控制模式
 */
void LightCtrl_SetMode(LightCtrl_HandleTypeDef *hlctrl, LightCtrl_Mode mode)
{
    hlctrl->mode = mode;
    if (mode == LIGHT_MODE_MANUAL) {
        Curtain_Stop();  // 切换到手动模式时停止电机
    }
}

/**
 * @brief  手动打开窗帘
 */
void LightCtrl_OpenCurtain(LightCtrl_HandleTypeDef *hlctrl)
{
    hlctrl->mode = LIGHT_MODE_MANUAL;
    hlctrl->state = LIGHT_CTRL_OPENING;

    /* 执行若干步 */
    for (int i = 0; i < 512; i++) {  // 约1/4圈
        Curtain_StepForward();
        HAL_Delay(2);
    }

    Curtain_Stop();
    hlctrl->state = LIGHT_CTRL_OPEN;
}

/**
 * @brief  手动关闭窗帘
 */
void LightCtrl_CloseCurtain(LightCtrl_HandleTypeDef *hlctrl)
{
    hlctrl->mode = LIGHT_MODE_MANUAL;
    hlctrl->state = LIGHT_CTRL_CLOSING;

    /* 执行若干步 */
    for (int i = 0; i < 512; i++) {  // 约1/4圈
        Curtain_StepBackward();
        HAL_Delay(2);
    }

    Curtain_Stop();
    hlctrl->state = LIGHT_CTRL_CLOSED;
}

/**
 * @brief  停止窗帘运动
 */
void LightCtrl_StopCurtain(LightCtrl_HandleTypeDef *hlctrl)
{
    Curtain_Stop();
    hlctrl->state = LIGHT_CTRL_IDLE;
}

/**
 * @brief  获取当前光照值
 */
float LightCtrl_GetLux(LightCtrl_HandleTypeDef *hlctrl)
{
    return hlctrl->current_lux;
}

/**
 * @brief  获取当前状态
 */
LightCtrl_State LightCtrl_GetState(LightCtrl_HandleTypeDef *hlctrl)
{
    return hlctrl->state;
}
