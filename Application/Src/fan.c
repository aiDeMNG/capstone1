/**
 * @file    fan.c
 * @brief   风扇驱动模块
 * @note    控制PA0引脚驱动风扇
 *          支持两种控制模式:
 *          1. 恒速模式 - 低电平恒速转动
 *          2. PWM调速模式 - 通过PWM占空比调节转速
 */

#include "fan.h"

/* ==================== 私有函数声明 ==================== */

static void Fan_ConfigGPIO(Fan_HandleTypeDef *hfan);
static void Fan_ConfigPWM(Fan_HandleTypeDef *hfan);

/* ==================== 公共函数实现 ==================== */

/**
 * @brief  初始化风扇
 */
void Fan_Init(Fan_HandleTypeDef *hfan,
              GPIO_TypeDef *gpio_port,
              uint16_t gpio_pin,
              TIM_HandleTypeDef *htim,
              uint32_t tim_channel)
{
    if (hfan == NULL)
    {
        return;
    }

    hfan->mode = FAN_MODE_CONSTANT;
    hfan->state = FAN_STATE_STOPPED;
    hfan->gpio_port = gpio_port;
    hfan->gpio_pin = gpio_pin;
    hfan->htim = htim;
    hfan->tim_channel = tim_channel;
    hfan->duty_cycle = 0;

    /* 确保 PWM 停止（如果之前被启动了） */
    if (hfan->htim != NULL)
    {
        HAL_TIM_PWM_Stop(hfan->htim, hfan->tim_channel);
    }

    /* 强制配置为GPIO模式，引脚设置为高电平（风扇停止） */
    Fan_ConfigGPIO(hfan);
    HAL_GPIO_WritePin(hfan->gpio_port, hfan->gpio_pin, GPIO_PIN_SET);
}

/**
 * @brief  启动风扇 - 恒速模式
 */
void Fan_StartConstant(Fan_HandleTypeDef *hfan)
{
    if (hfan == NULL)
    {
        return;
    }

    /* 如果之前是PWM模式，先停止PWM */
    if (hfan->mode == FAN_MODE_PWM && hfan->htim != NULL)
    {
        HAL_TIM_PWM_Stop(hfan->htim, hfan->tim_channel);
    }

    /* 切换到GPIO模式 */
    hfan->mode = FAN_MODE_CONSTANT;
    Fan_ConfigGPIO(hfan);

    /* 设置PA0为低电平，风扇恒速运行 */
    HAL_GPIO_WritePin(hfan->gpio_port, hfan->gpio_pin, GPIO_PIN_RESET);
    hfan->state = FAN_STATE_RUNNING;
}

/**
 * @brief  启动风扇 - PWM调速模式
 */
void Fan_StartPWM(Fan_HandleTypeDef *hfan, uint8_t duty_cycle)
{
    if (hfan == NULL || hfan->htim == NULL)
    {
        return;
    }

    /* 限制占空比范围 */
    if (duty_cycle > 100)
    {
        duty_cycle = 100;
    }

    /* 切换到PWM模式 */
    hfan->mode = FAN_MODE_PWM;
    hfan->duty_cycle = duty_cycle;
    Fan_ConfigPWM(hfan);

    /* 设置PWM占空比并启动 */
    uint32_t pulse = (hfan->htim->Init.Period + 1) * duty_cycle / 100;
    __HAL_TIM_SET_COMPARE(hfan->htim, hfan->tim_channel, pulse);
    HAL_TIM_PWM_Start(hfan->htim, hfan->tim_channel);

    hfan->state = (duty_cycle > 0) ? FAN_STATE_RUNNING : FAN_STATE_STOPPED;
}

/**
 * @brief  设置PWM占空比
 */
void Fan_SetDuty(Fan_HandleTypeDef *hfan, uint8_t duty_cycle)
{
    if (hfan == NULL || hfan->htim == NULL || hfan->mode != FAN_MODE_PWM)
    {
        return;
    }

    /* 限制占空比范围 */
    if (duty_cycle > 100)
    {
        duty_cycle = 100;
    }

    hfan->duty_cycle = duty_cycle;

    /* 更新PWM占空比 */
    uint32_t pulse = (hfan->htim->Init.Period + 1) * duty_cycle / 100;
    __HAL_TIM_SET_COMPARE(hfan->htim, hfan->tim_channel, pulse);

    hfan->state = (duty_cycle > 0) ? FAN_STATE_RUNNING : FAN_STATE_STOPPED;
}

/**
 * @brief  停止风扇
 */
void Fan_Stop(Fan_HandleTypeDef *hfan)
{
    if (hfan == NULL)
    {
        return;
    }

    if (hfan->mode == FAN_MODE_CONSTANT)
    {
        /* GPIO模式：设置为高电平停止风扇 */
        HAL_GPIO_WritePin(hfan->gpio_port, hfan->gpio_pin, GPIO_PIN_SET);
    }
    else if (hfan->mode == FAN_MODE_PWM && hfan->htim != NULL)
    {
        /* PWM模式：停止PWM输出 */
        HAL_TIM_PWM_Stop(hfan->htim, hfan->tim_channel);
        hfan->duty_cycle = 0;
    }

    hfan->state = FAN_STATE_STOPPED;
}

/**
 * @brief  获取风扇状态
 */
Fan_State Fan_GetState(Fan_HandleTypeDef *hfan)
{
    if (hfan == NULL)
    {
        return FAN_STATE_STOPPED;
    }
    return hfan->state;
}

/**
 * @brief  获取风扇模式
 */
Fan_Mode Fan_GetMode(Fan_HandleTypeDef *hfan)
{
    if (hfan == NULL)
    {
        return FAN_MODE_CONSTANT;
    }
    return hfan->mode;
}

/**
 * @brief  获取PWM占空比
 */
uint8_t Fan_GetDuty(Fan_HandleTypeDef *hfan)
{
    if (hfan == NULL)
    {
        return 0;
    }
    return hfan->duty_cycle;
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief  配置GPIO模式
 * @note   将PA0配置为普通GPIO输出
 */
static void Fan_ConfigGPIO(Fan_HandleTypeDef *hfan)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 配置GPIO为推挽输出 */
    GPIO_InitStruct.Pin = hfan->gpio_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(hfan->gpio_port, &GPIO_InitStruct);
}

/**
 * @brief  配置PWM模式
 * @note   将PA0配置为定时器PWM输出
 *         需要在CubeMX中预先配置TIM2_CH1复用功能
 */
static void Fan_ConfigPWM(Fan_HandleTypeDef *hfan)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 配置GPIO为复用推挽输出 (PWM) */
    GPIO_InitStruct.Pin = hfan->gpio_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(hfan->gpio_port, &GPIO_InitStruct);
}
