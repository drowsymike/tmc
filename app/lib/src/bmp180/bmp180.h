#ifndef BMP180_H
#define BMP180_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sensor.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

/*--Defines------------------------------------------------------------------*/

#define bmp180_addr 0x77 << (uint16_t)1
#define bmp180_start_measurement_flag_register 0xF4
#define bmp180_start_measurement_size 1
#define bmp180_start_measurement_flag_register_size 1
#define bmp180_init_success_msg "BMP180 init success"
#define bmp180_init_failure_msg "BMP180 init failure"
#define bmp180_oss 0

/*--Extern objects-----------------------------------------------------------*/

extern struct sensor_t bmp180;
extern I2C_HandleTypeDef i2c;
extern UART_HandleTypeDef huart2;

/*--Functions----------------------------------------------------------------*/

/**
 * @brief 
 * 
 * @return void 
 */
void bmp180_struct_init(void);

/**
 * @brief 
 * 
 * @return void 
 */
void bmp180_init(void);

/**
 * @brief 
 * 
 * @return void 
 */
uint32_t bmp180_get_data(void);

/**
 * @brief 
 * 
 */
void bmp180_get_global_coefficients(void);

/**
 * @brief 
 * 
 * @return float 
 */
float bmp180_get_temp(void);

#ifdef __cplusplus
}
#endif

#endif