#ifdef __cplusplus
extern "C" {
#endif

#include "bmp180.h"
#include "string.h"
#include "stdio.h"

/*--Global stuff for calcs or a thing-----------------------------------------*/

struct sensor_t bmp180;
int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
uint16_t AC4, AC5, AC6;
int32_t X1, X2, B5;
uint8_t bmp180_calib_data[22] = {0};
uint8_t bmp180_pressure_buffer[3] = {0};

/*--Commands------------------------------------------------------------------*/

uint8_t bmp180_start_measurement = 0x2E;

/*--Functions-----------------------------------------------------------------*/

/**
 * @brief 
 * 
 */
void bmp180_init(void) {
    if (HAL_I2C_Mem_Write(&i2c, (bmp180_addr), bmp180_start_measurement_flag_register, bmp180_start_measurement_flag_register_size, &bmp180_start_measurement, 1, HAL_MAX_DELAY) == HAL_OK) {
        HAL_UART_Transmit(&huart2, (const uint8_t*)bmp180_init_success_msg, strlen((char *)bmp180_init_success_msg), HAL_MAX_DELAY);
	} else {
        HAL_UART_Transmit(&huart2, (const uint8_t*)bmp180_init_failure_msg, strlen((char *)bmp180_init_failure_msg), HAL_MAX_DELAY);
    }
    bmp180_get_global_coefficients();
}

/**
 * @brief 
 * 
 * @return uint32_t 
 */
uint32_t bmp180_get_data(void) {
    HAL_I2C_Mem_Read(&i2c, (bmp180_addr), 0xF6, 1, bmp180_pressure_buffer, 3, HAL_MAX_DELAY);
	long UP = (((long)bmp180_pressure_buffer[0] << 16) | ((long)bmp180_pressure_buffer[1] << 8) | (long)bmp180_pressure_buffer[2]) >> 8;
	long B6 = B5 - 4000;
	X1 = (B2 * ((B6 * B6) >> 12)) >> 11;
	X2 = (AC2 * B6) >> 11;
	long X3 = X1 + X2;
	long B3 = ((((long)AC1 * 4 + X3) << bmp180_oss) + 2) >> 2;
	X1 = (AC3 * B6) >> 13;
	X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	unsigned long B4 = (AC4 * (unsigned long)(X3 + 32768)) >> 15;
	unsigned long B7 = ((unsigned long)UP - B3) * (50000UL >> bmp180_oss);
	long p;
	if (B7 < 0x80000000)
		p = (B7 << 1) / B4;
	else
		p = (B7 / B4) << 1;

	X1 = (p >> 8) * (p >> 8);
	X1 = (X1 * 3038L) >> 16;
	X2 = (-7357L * p) >> 16;
	p = p + ((X1 + X2 + 3791L) >> 4);
	return p;
}

float bmp180_get_temp(void) {
    uint8_t temp_data[2];
    uint8_t start_temp_cmd = 0x2E;
    HAL_I2C_Mem_Write(&i2c, bmp180_addr, 0xF4, 1, &start_temp_cmd, 1, 500);
	HAL_Delay(5);
    HAL_I2C_Mem_Read(&i2c, bmp180_addr, 0xF6, 1, temp_data, 2, HAL_MAX_DELAY);
    long UT = (temp_data[0] << 8) | temp_data[1];
    X1 = (UT - (long)AC6) * (long)AC5 >> 15;
    X2 = ((long)MC << 11) / (X1 + MD);
    B5 = X1 + X2; 
    float T = (B5 + 8) >> 4;
    return T / 10.0;
}

/**
 * @brief 
 * 
 */
void bmp180_struct_init(void) {
    bmp180.address = bmp180_addr;
    bmp180.init_fn = bmp180_init;
    bmp180.get_data_fn = bmp180_get_data;
}

/*--Peripherial functions----------------------------------------------------*/

/**
 * @brief 
 * 
 */
void bmp180_get_global_coefficients(void) {
	HAL_I2C_Mem_Read(&i2c, bmp180_addr, 0xAA, 1, bmp180_calib_data, 22, HAL_MAX_DELAY);
	AC1 = (int16_t)((bmp180_calib_data[0] << 8) | bmp180_calib_data[1]);
	AC2 = (int16_t)((bmp180_calib_data[2] << 8) | bmp180_calib_data[3]);
	AC3 = (int16_t)((bmp180_calib_data[4] << 8) | bmp180_calib_data[5]);
	AC4 = (uint16_t)((bmp180_calib_data[6] << 8) | bmp180_calib_data[7]);
	AC5 = (uint16_t)((bmp180_calib_data[8] << 8) | bmp180_calib_data[9]);
	AC6 = (uint16_t)((bmp180_calib_data[10] << 8) | bmp180_calib_data[11]);
	B1  = (int16_t)((bmp180_calib_data[12] << 8) | bmp180_calib_data[13]);
	B2  = (int16_t)((bmp180_calib_data[14] << 8) | bmp180_calib_data[15]);
	MB  = (int16_t)((bmp180_calib_data[16] << 8) | bmp180_calib_data[17]);
	MC  = (int16_t)((bmp180_calib_data[18] << 8) | bmp180_calib_data[19]);
	MD  = (int16_t)((bmp180_calib_data[20] << 8) | bmp180_calib_data[21]);
}

#ifdef __cplusplus
}
#endif