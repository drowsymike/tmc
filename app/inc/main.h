#ifndef STM32F401xE
#define STM32F401xE
#endif

#define HAL_RCC_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define set_temperature_cmd "set\0"
#define turn_off_cmd "sdn\0"

void MX_USART2_UART_Init(void);
void i2c_init(void);
void MX_GPIO_Init(void);
void Error_Handler(char* msg);
void find_cmd(uint8_t* msg);

int set_pwm(uint8_t filling);

void HAL_MspInit(void);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base);
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c);
void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc);
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc);
void HAL_UART_MspInit(UART_HandleTypeDef* huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

extern bool set_temperature_cmd_flag;
extern bool turn_off_cmd_flag;
extern bool info_cmd_flag;
extern bool data_incoming;

#ifdef __cplusplus
}
#endif

#endif
