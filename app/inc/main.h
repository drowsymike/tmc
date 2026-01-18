#ifndef STM32F401xE
#define STM32F401xE
#endif

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define set_temperature_cmd "set"
#define turn_off_cmd "sd"
#define info_cmd "--h"

void MX_USART2_UART_Init(void);
void i2c_init(void);
void MX_GPIO_Init(void);
void Error_Handler(char* msg);

#ifdef __cplusplus
}
#endif

#endif
