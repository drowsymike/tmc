#ifndef SENSOR_H
#define SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "common.h"

/*---------------------------------------------------------------------------*/

typedef uint8_t sensor_address;

struct sensor_t {
    sensor_address address;
    void* config_data;
    void (*init_fn)(void);
    uint32_t (*get_data_fn)(void);
};

/*---------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif