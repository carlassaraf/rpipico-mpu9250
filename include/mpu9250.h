#ifndef _MPU9250_H_
#define _MPU9250_H_

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "mpu9250_registers.h"

#include <math.h>

// Direccion del MPU9250 cuando AD0 = 0
#define MPU9250_ADDRESS     0x68
// Direccion del AK8963
#define AK8963_ADDRESS      0x0C   

/**
 * @brief Estructura de datos para configurar el MPU9250
*/
typedef struct {
    uint8_t scl_gpio;       // GPIO para el SCL
    uint8_t sda_gpio;       // GPIO para el SDA
    i2c_inst_t *i2c;        // Puntero al I2C
    uint32_t baudrate;      // Velocidad de transmision
    uint8_t mpu_address;    // Direccion del MPU
    uint8_t ak_address;     // Direccion del magnetometro
} mpu9250_t;

/**
 * @brief Status codes para funciones
*/
typedef enum {
    MPU9250_OK,
    MPU9250_NO_AK8963,
    MPU9250_ERR,
} mpu9250_status_t;

/**
 * @brief Devuelve una estructura por defecto para usar el MPU
 * @return estructura tipo mpu9250_t
*/
static inline mpu9250_t mpu9250_get_default_config() {
    return (mpu9250_t) {
        .scl_gpio = 5,
        .sda_gpio = 4,
        .i2c = i2c0,
        .baudrate = 400000,
        .mpu_address = MPU9250_ADDRESS,
        .ak_address = AK8963_ADDRESS
    };
}

// Prototipos

mpu9250_status_t mpu9250_init(mpu9250_t mpu);
void mpu9250_self_test(float *gyro_st_result, float *accel_st_result);

#endif