#ifdef __cplusplus
extern "C" {
#endif

#include "bmp180.h"
#include "string.h"
#include "stdio.h"

/*--Global stuff for calcs or a thing----------------------------------------*/

struct sensor_t bmp388;
enum sensor_measurement_accuracy {
    temperature,
    pressure_ultra_low_power,
    pressure_low_power,
    pressure_standart_resolution,
    pressure_high_resolution,
    pressure_ultra_high_resolution,
    pressure_highest_resolution
};




#ifdef __cplusplus
}
#endif