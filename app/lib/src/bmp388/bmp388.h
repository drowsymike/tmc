#ifndef BMP388_H
#define BMP388_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sensor.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

/*--Defines------------------------------------------------------------------*/

#define bmp388_sleep_mode_code  0
#define bmp388_forced_mode_code 1
#define bmp388_normal_mode_code 2

#define bmp388_cmd_register_addr 0x7E
#define bmp388_config_register_addr 0x7D
#define bmp388_odr_register_addr 0x20
#define bmp388_osr_register_addr 0x1F
#define bmp388_pwr_ctrl_register_addr 0x1B
#define bmp388_if_conf_register_addr 
#define bmp388_int_ctrl_register_addr
#define bmp388_fifo_config_2_register_addr
#define bmp388_fifo_config_1_register_addr
#define bmp388_fifo_wtm_1_register_addr
#define bmp388_fifo_data_register_addr
#define bmp388_fifo_length_1_register_addr
#define bmp388_fifo_length_0_register_addr
#define bmp388_int_status_register_addr
#define bmp388_event_register_addr
#define bmp388_sensortime2_register_addr
#define bmp388_sensortime1_register_addr
#define bmp388_sensortime0_register_addr
#define bmp388_data_5_register_addr
#define bmp388_data_4_register_addr
#define bmp388_data_3_register_addr
#define bmp388_data_2_register_addr
#define bmp388_data_1_register_addr
#define bmp388_data_0_register_addr
#define bmp388_status_register_addr
#define bmp388_err_reg_register_addr
#define bmp388_chip_id_register_addr


/*--Extern objects-----------------------------------------------------------*/

extern struct sensor_t bmp388;

/*--Functions----------------------------------------------------------------*/

/**
 * @brief 
 * 
 * @return void 
 */
void bmp388_struct_init(void);

/**
 * @brief 
 * 
 */
void bmp388_init(void);

/**
 * @brief 
 * 
 */
void bmp388_get_data(void);

/**
 * @brief 
 * 
 */
void bmp388_irr_filter_enable(void);

/**
 * @brief 
 * 
 */
void bmp388_irr_filter_init(void);

/**
 * @brief 
 * 
 */
__WEAK void bmp388_int_line_interrupt_handler(void);

/**
 * @brief 
 * 
 */
void bmp388_set_power_mode(uint8_t mode);
/*---------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif