/**
 * @file    control_priority.h
 * @brief   系统控制优先级定义
 * @note    所有自动控制模块的公共优先级定义
 *          用于协调多个控制源（光照、空气质量、手动等）
 */

#ifndef __CONTROL_PRIORITY_H
#define __CONTROL_PRIORITY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ==================== 控制优先级定义 ==================== */

/**
 * @brief  系统控制优先级枚举
 * @note   数值越大，优先级越高
 *         高优先级控制可以抑制低优先级控制
 */
typedef enum {
    PRIORITY_NONE = 0,          // 无优先级（空闲状态）
    PRIORITY_LIGHT,             // 光照控制优先级（低）
    PRIORITY_AIR_QUALITY,       // 空气质量控制优先级（高）
    PRIORITY_MANUAL             // 手动控制优先级（最高）
} Control_Priority;

/**
 * @brief  优先级比较宏
 * @note   用于判断优先级高低
 */
#define PRIORITY_IS_HIGHER(p1, p2)  ((p1) > (p2))
#define PRIORITY_IS_LOWER(p1, p2)   ((p1) < (p2))
#define PRIORITY_IS_EQUAL(p1, p2)   ((p1) == (p2))

/**
 * @brief  获取优先级名称（调试用）
 * @param  priority: 优先级
 * @return 优先级名称字符串
 */
static inline const char* Priority_GetName(Control_Priority priority)
{
    switch(priority) {
        case PRIORITY_NONE:         return "NONE";
        case PRIORITY_LIGHT:        return "LIGHT";
        case PRIORITY_AIR_QUALITY:  return "AIR_QUALITY";
        case PRIORITY_MANUAL:       return "MANUAL";
        default:                    return "UNKNOWN";
    }
}

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_PRIORITY_H */
