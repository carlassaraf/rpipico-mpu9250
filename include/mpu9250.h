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

typedef enum {
    A2G,
    A4G,
    A8G,
    A16G
} mpu9250_accel_fs_sel_t;

typedef enum {
    G250DPS,
    G500DPS,
    G1000DPS,
    G2000DPS
} mpu9250_gyro_fs_sel_t;

typedef enum {
    M14BITS,
    M16BITS
} mpu9250_mag_output_bits_t;

typedef enum {
    SMPL_1000HZ,
    SMPL_500HZ,
    SMPL_333HZ,
    SMPL_250HZ,
    SMPL_200HZ,
    SMPL_167HZ,
    SMPL_143HZ,
    SMPL_125HZ
} mpu9250_fifo_sample_rate_t;

typedef enum {
    GYRO_DLPF_250HZ,
    GYRO_DLPF_184HZ,
    GYRO_DLPF_92HZ,
    GYRO_DLPF_41HZ,
    GYRO_DLPF_20HZ,
    GYRO_DLPF_10HZ,
    GYRO_DLPF_5HZ,
    GYRO_DLPF_3600HZ
} mpu9250_gyro_dlpf_cfg_t;

typedef enum {
    ACCEL_DLPF_218HZ_0,
    ACCEL_DLPF_218HZ_1,
    ACCEL_DLPF_99HZ,
    ACCEL_DLPF_45HZ,
    ACCEL_DLPF_21HZ,
    ACCEL_DLPF_10HZ,
    ACCEL_DLPF_5HZ,
    ACCEL_DPLF_420HZ
} mpu9250_accel_dlpf_cfg_t;

typedef struct {
    mpu9250_accel_fs_sel_t accel_fs_sel;
    mpu9250_gyro_fs_sel_t gyro_fs_sel;
    mpu9250_mag_output_bits_t mag_output_bits;
    mpu9250_fifo_sample_rate_t fifo_sample_sate;
    uint8_t gyro_fchoice;
    mpu9250_gyro_dlpf_cfg_t gyro_dlpf_cfg;
    uint8_t accel_fchoice;
    mpu9250_accel_dlpf_cfg_t accel_dlpf_cfg;
} mpu9250_settings_t;

/**
 * @brief Estructura de datos para configurar el MPU9250
*/
typedef struct {
    uint8_t scl_gpio;               // GPIO para el SCL
    uint8_t sda_gpio;               // GPIO para el SDA
    i2c_inst_t *i2c;                // Puntero al I2C
    uint32_t baudrate;              // Velocidad de transmision
    uint8_t mpu_address;            // Direccion del MPU
    uint8_t ak_address;             // Direccion del magnetometro
    mpu9250_settings_t settings;    // Configuracion del MPU9250
} mpu9250_t;

/**
 * @brief Status codes para funciones
*/
typedef enum {
    MPU9250_OK,
    MPU9250_NO_AK8963,
    MPU9250_ERR,
} mpu9250_status_t;

// Prototipos

mpu9250_t mpu9250_get_default_config(void);
mpu9250_status_t mpu9250_init(mpu9250_t mpu);
void mpu9250_self_test(float *gyro_st_result, float *accel_st_result);
float mpu9250_read_temperature(void);
void mpu9250_read_accel(float *dst);
void mpu9250_read_gyro(float *dst);

#endif