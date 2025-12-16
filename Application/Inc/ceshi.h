/**
 * @file    ceshi.h
 * @brief   电机测试程序
 * @note    测试两个步进电机:
 *          - ULN2003 (窗户电机): PA2, PA3, PA4, PA5 - 转2圈后反转2圈，循环
 *          - A4988 (窗帘电机): PA8(STEP), PA9(DIR), PA10(MS1), PA11(MS2), PA12(MS3) - 转10圈后反转10圈，循环
 *          EN引脚接地(常使能)
 *
 *          A4988细分模式: MS1=H, MS2=H, MS3=H -> 1/16步进 (3200步/圈)
 */

#ifndef __CESHI_H
#define __CESHI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ==================== 测试参数配置 ==================== */

/* ULN2003参数 (28BYJ-48电机) */
#define CESHI_ULN2003_STEPS_PER_REV     2048    // 每圈步数
#define CESHI_ULN2003_REVS              2       // 转动圈数
#define CESHI_ULN2003_STEP_DELAY_MS     2       // 步进延时 (ms)

/* A4988参数 (42步进电机, 1/16细分) */
#define CESHI_A4988_STEPS_PER_REV       3200    // 每圈步数 (200*16)
#define CESHI_A4988_REVS                10      // 转动圈数
#define CESHI_A4988_STEP_DELAY_US       500     // 步进延时 (us)
#define CESHI_A4988_PULSE_US            10      // 脉冲宽度 (us)

/* ==================== 函数声明 ==================== */

/**
 * @brief  测试程序初始化
 * @note   初始化GPIO引脚状态
 */
void Ceshi_Init(void);

/**
 * @brief  测试程序主循环
 * @note   在main函数的while(1)中调用
 *         ULN2003: 正转2圈 -> 反转2圈 -> 循环
 *         A4988: 正转10圈 -> 反转10圈 -> 循环
 */
void Ceshi_Run(void);

/**
 * @brief  仅测试ULN2003电机
 * @note   正转2圈后反转2圈，循环
 */
void Ceshi_ULN2003_Test(void);

/**
 * @brief  仅测试A4988电机
 * @note   正转10圈后反转10圈，循环
 */
void Ceshi_A4988_Test(void);

#ifdef __cplusplus
}
#endif

#endif /* __CESHI_H */
