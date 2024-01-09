#include "mpu9250.h"

// Estructura global para el MPU9250
static mpu9250_t _mpu;

/**
 * @brief Escribe un byte por I2C
 * @param addr direccion del dispositivo
 * @param reg direccion del registro a escribir
 * @param val valor a escribir en el registro
*/
static inline void mpu9250_write_byte(uint8_t addr, uint8_t reg, uint8_t val) {
    // Buffer a enviar
    uint8_t buff[] = { reg, val };
    // Envio bytes
    i2c_write_blocking(_mpu.i2c, addr, buff, 2, false);
}

/**
 * @brief Lee un byte por I2C
 * @param addr direccion del dispositivo
 * @param reg registro a leer
 * @return valor del registro
*/
static inline uint8_t mpu9250_read_byte(uint8_t addr, uint8_t reg) {
    // Apunto al registro que quiero leer
    i2c_write_blocking(_mpu.i2c, addr, &reg, 1, true);
    // Variable auxiliar para guardar el dato
    uint8_t aux;
    // Leo el registro
    i2c_read_blocking(_mpu.i2c, addr, &aux, 1, false);
    // Devuelvo el valor
    return aux;
}

/**
 * @brief Lee una cantidad de bytes por I2C
 * @param addr direccion del dispositivo
 * @param dst puntero a donde guardar los bytes
 * @param len cantidad de bytes a leer
*/
static inline void mpu9250_read_bytes(uint8_t addr, uint8_t reg, uint8_t *dst, uint8_t len) {
    // Apunto al registro que quiero leer
    i2c_write_blocking(_mpu.i2c, addr, &reg, 1, true);
    // Leo el registro
    i2c_read_blocking(_mpu.i2c, addr, dst, len, false);
}

static inline mpu9250_status_t ak8963_is_available(void) {
    // Leo el WHO_AM_I para verificar que la comunicacion esta OK
    volatile uint8_t ak_who_am_i = mpu9250_read_byte(_mpu.ak_address, WHO_AM_I_AK8963);
    // Verifico que el valor este dentro de lo esperado
    if(ak_who_am_i == 0x48) {
        // La comunicacion esta bien
        return MPU9250_OK;
    }

    return MPU9250_ERR;
}

static inline mpu9250_status_t mpu9250_is_available(void) {
    // Leo el WHO_AM_I para verificar que la comunicacion esta OK
    volatile uint8_t mpu_who_am_i = mpu9250_read_byte(_mpu.mpu_address, WHO_AM_I_MPU9250);
    // Verifico que el valor este dentro de lo esperado
    if(mpu_who_am_i == 0x71 || mpu_who_am_i == 0x73) {
        // La comunicacion esta bien
        return MPU9250_OK;
    }

    return MPU9250_ERR;
}

/**
 * @brief Efectua una cantidad de lecturas del acelerometro y giroscopo y las promedia
 * @param accel_av puntero donde guardar los valores promediados del acelerometro (X, Y, Z)
 * @param gyro_av puntero donde guardar los valores promediados del giroscopo (X, Y, Z)
 * @param n_readings cantidad de lecturas
*/
static inline void mpu9250_read_and_average(int16_t *accel_av, int16_t *gyro_av, uint8_t n_readings) {
    // Tomo 200 lecturas
    for(uint8_t i = 0; i < n_readings; i++) {
        // Variable para lectura de los 3 ejes
        uint8_t raw[6] = { 0 };
        // Leo los datos del acelerometro en X, Y, Z
        mpu9250_read_bytes(_mpu.mpu_address, ACCEL_XOUT_H, raw, 6);
        // Acumulo el valor de 16 bits leido (primero parte alta, luego baja)
        accel_av[0] += (int16_t)((int16_t)(raw[0] << 8) | raw[1]);
        accel_av[1] += (int16_t)((int16_t)(raw[2] << 8) | raw[3]);
        accel_av[2] += (int16_t)((int16_t)(raw[4] << 8) | raw[5]);
        // Leo los datos del giroscopo en X, Y, Z
        mpu9250_read_bytes(_mpu.mpu_address, GYRO_XOUT_H, raw, 6);
        // Acumulo el valor de 16 bits leido (primero parte alta, luego baja)
        gyro_av[0] += (int16_t)((int16_t)(raw[0] << 8) | raw[1]);
        gyro_av[1] += (int16_t)((int16_t)(raw[2] << 8) | raw[3]);
        gyro_av[2] += (int16_t)((int16_t)(raw[4] << 8) | raw[5]);
    }

    // Hago los promedios de las 200 lecturas
    for(uint8_t i = 0; i < 3; i++) {
        accel_av[i] /= 200;
        gyro_av[i] /= 200;
    }
}

/**
 * @brief Inicializa el MPU9250
 * @param mpu estructura de configuracion para el MPU
 * @return estado de la inicializacion. Devuelve OK si salio bien
*/
mpu9250_status_t mpu9250_init(mpu9250_t mpu) {
    // Copio la estructura de datos del MPU
    _mpu = mpu;
    // Inicializo I2C
    i2c_init(_mpu.i2c, _mpu.baudrate);
    // Configuro los GPIO
    gpio_set_function(_mpu.scl_gpio, GPIO_FUNC_I2C);
    gpio_set_function(_mpu.sda_gpio, GPIO_FUNC_I2C);
    // Configuro pull-ups
    gpio_pull_up(_mpu.scl_gpio);
    gpio_pull_up(_mpu.sda_gpio);
  	
    // Verifico que el MPU9250 y el AK8963 esten en el bus
    if(mpu9250_is_available() == MPU9250_OK && ak8963_is_available() == MPU9250_OK) {
        return MPU9250_OK;
    }

    // El dispositivo no esta bien conectado
    return MPU9250_ERR;
}

/**
 * @brief Efectua un self test para verificar la integridad del sensor
 * @param gyro_st_result puntero donde guardar el resultado porcentual del
 * self test de los tres ejes del giroscopo
 * @param accel_st_result puntero donde guardar el resultado porcentual del
 * self test de los tres ejes del acelerometro
*/
void mpu9250_self_test(float *gyro_st_result, float *accel_st_result) {
    // Variable para guardar promedios de lecturas reales y de fabrica (ST)
    int16_t accel_av[3] = { 0 }, gyro_av[3] = { 0 }, accel_st_av[3] = { 0 }, gyro_st_av[3] = { 0 };
    // Macro para resetear el fondo de escala del giroscopo
    #define GYRO_FS_SEL_RST_MASK (~(0b11 << 3))
    // Macro para resetear el fondo de escala del acelerometro
    #define ACCEL_FS_SEL_RST_MASK (~(0b11 << 3))
    
    // DLPF (digital lower pass filter) BW a 92Hz, sampling rate 1KHz, filter delay 3.9ms
    mpu9250_write_byte(_mpu.mpu_address, CONFIG, 0x02);
    // DLPF BW a 92Hz, sampling rate 1KHz, filter delay 7.8ms
    mpu9250_write_byte(_mpu.mpu_address, ACCEL_CONFIG2, 0x02);
    // Leo la vieja configuracion de fondo de escala del giroscopo y reseteo los bits de GYRO_FS_SEL (FS 250dps)
    uint8_t gyro_config_old = mpu9250_read_byte(_mpu.mpu_address, GYRO_CONFIG);
    uint8_t gyro_config = gyro_config_old & GYRO_FS_SEL_RST_MASK;
    // Escribo la nueva configuracion
    mpu9250_write_byte(_mpu.mpu_address, GYRO_CONFIG, gyro_config);
    // Leo la vieja configuracion de fondo de escala del acelerometro y reseteo los bits de ACCEL_FS_SEL (FS 2g)
    uint8_t accel_config_old = mpu9250_read_byte(_mpu.mpu_address, ACCEL_CONFIG);
    uint8_t accel_config = accel_config_old & ACCEL_FS_SEL_RST_MASK;
    // Escribo la nueva configuracion
    mpu9250_write_byte(_mpu.mpu_address, ACCEL_CONFIG, accel_config);

    // Hago 200 lecturas y promedio
    mpu9250_read_and_average(accel_av, gyro_av, 200);

    // Habilito self test

    // Seteo los tres bits mas significativos de las configuraciones del gyro y accel para habilitar self test
    gyro_config |= 0xe0;
    accel_config |= 0xe0;
    // Escribo configuraciones
    mpu9250_write_byte(_mpu.mpu_address, GYRO_CONFIG, gyro_config);
    mpu9250_write_byte(_mpu.mpu_address, ACCEL_CONFIG, accel_config);
    // Espero 20ms para que se estabilicen las oscilaciones
    sleep_ms(20);

    // Hago 200 lecturas y promedio con self test
    mpu9250_read_and_average(accel_st_av, gyro_st_av, 200);
    
    // Limpio la configuracion anterior
    mpu9250_write_byte(_mpu.mpu_address, GYRO_CONFIG, gyro_config & ~0xe0);
    mpu9250_write_byte(_mpu.mpu_address, ACCEL_CONFIG, accel_config & ~0xe0);
    // Espero 20 ms
    sleep_ms(20);
    // Vuelvo a la operacion anterior
    mpu9250_write_byte(_mpu.mpu_address, GYRO_CONFIG, gyro_config_old);
    mpu9250_write_byte(_mpu.mpu_address, ACCEL_CONFIG, accel_config_old);

    // Obtengo los valores de self test para cada eje
    uint8_t accel_st_code[3] = { 0 }, gyro_st_code[3] = { 0 };
    mpu9250_read_bytes(_mpu.mpu_address, SELF_TEST_X_GYRO, gyro_st_code, 3);
    mpu9250_read_bytes(_mpu.mpu_address, SELF_TEST_X_ACCEL, accel_st_code, 3);

    // Obtengo los valores de self test en base al self test code anterior
    float st_value;
    for(uint8_t i = 0; i < 3; i++) {
        // Valor del self test para el giroscopo
        st_value = 2620.0 * pow(1.01, (gyro_st_code[i] - 1.0));
        // Guardo valor final porcentual
        gyro_st_result[i] = 100.0 * (gyro_st_av[i] - gyro_av[i]) / st_value;
        // Valor del self test para el acelerometro
        st_value = 2620.0 * pow(1.01, (accel_st_code[i] - 1.0));
        // Guardo valor final porcentual
        accel_st_result[i] = 100.0 * (accel_st_av[i] - accel_av[i]) / st_value;
    }
}