#ifndef MAIN_H
#define MAIN_H

/*
 *  main.h — 对外暴露 HAL 句柄，让 drivers/proto 层的桩函数能访问
 *  不依赖具体 CubeMX 配置
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 业务层需要的 HAL 句柄（main.c 里定义）*/
extern void *htim_pwm_left;
extern void *htim_pwm_right;
extern void *htim_ctrl;
extern void *huart1;

/* CubeMX 生成函数原型（保留占位）*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART1_UART_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);

#ifdef __cplusplus
}
#endif

#endif
