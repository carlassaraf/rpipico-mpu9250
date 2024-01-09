#include "mpu9250.h"

// Estructura global para el MPU9250
static mpu9250_t _mpu;

// Estructura de configuracion del MPU9250
static mpu9250_settings_t _settings = {
    .accel_fs_sel = A16G,
    .gyro_fs_sel = G2000DPS,
    .mag_output_bits = M16BITS,
    .fifo_sample_sate = SMPL_200HZ,
    .gyro_fchoice = 0x03,
    .gyro_dlpf_cfg = GYRO_DLPF_41HZ,
    .accel_fchoice = 0x01,
    .accel_dlpf_cfg = ACCEL_DLPF_45HZ
};

// Prototipos de funciones privadas

static inline void _mpu9250_write_byte(uint8_t addr, uint8_t reg, uint8_t val);
static inline uint8_t _mpu9250_read_byte(uint8_t addr, uint8_t reg);
static inline void _mpu9250_read_bytes(uint8_t addr, uint8_t reg, uint8_t *dst, uint8_t len);
static inline void _mpu9250_read_and_average(int16_t *accel_av, int16_t *gyro_av, uint8_t n_readings);
static float _mpu9250_get_accel_resolution(mpu9250_accel_fs_sel_t accel_fs_sel);
static float _mpu9250_get_gyro_resolution(mpu9250_gyro_fs_sel_t gyro_fs_sel);
static void _mpu9250_init(void);
static void _mpu9250_read_raw_data(int16_t *dst, uint8_t reg, uint8_t len);
static inline void _mpu9250_read_raw_accel(int16_t *dst);
static inline void _mpu9250_read_raw_temp(int16_t *dst);
static inline void _mpu9250_read_raw_gyro(int16_t *dst);


// Funciones publicas

/**
 * @brief Verifica que el AK8963 este en el bus
 * @return status de la operacion. MPU9250_OK si esta el dispositivo
*/
static inline mpu9250_status_t ak8963_is_available(void) {
    // Leo el WHO_AM_I para verificar que la comunicacion esta OK
    volatile uint8_t ak_who_am_i = _mpu9250_read_byte(_mpu.ak_address, WHO_AM_I_AK8963);
    // Verifico que el valor este dentro de lo esperado
    if(ak_who_am_i == 0x48) {
        // La comunicacion esta bien
        return MPU9250_OK;
    }
    return MPU9250_ERR;
}

/**
 * @brief Verifica que el MPU9250 este en el bus
 * @return status de la operacion. MPU9250_OK si esta el dispositivo
*/
static inline mpu9250_status_t mpu9250_is_available(void) {
    // Leo el WHO_AM_I para verificar que la comunicacion esta OK
    volatile uint8_t mpu_who_am_i = _mpu9250_read_byte(_mpu.mpu_address, WHO_AM_I_MPU9250);
    // Verifico que el valor este dentro de lo esperado
    if(mpu_who_am_i == 0x71 || mpu_who_am_i == 0x73) {
        // La comunicacion esta bien
        return MPU9250_OK;
    }
    return MPU9250_ERR;
}

/**
 * @brief Devuelve una estructura por defecto para usar el MPU
 * @return estructura tipo mpu9250_t
*/
mpu9250_t mpu9250_get_default_config(void) {
    return (mpu9250_t) {
        .scl_gpio = 5,
        .sda_gpio = 4,
        .i2c = i2c0,
        .baudrate = 400000,
        .mpu_address = MPU9250_ADDRESS,
        .ak_address = AK8963_ADDRESS,
        .settings = _settings,
    };
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
  	
    // Verifico que el MPU9250 este en el bus
    if(mpu9250_is_available() == MPU9250_OK) {
        // Inicializo el MPU9250
        _mpu9250_init();
        // Verifico que el AK8963 este en el bus
        if(ak8963_is_available() == MPU9250_OK) {
            return MPU9250_OK;
        }
        else {
            return MPU9250_NO_AK8963;
        }
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
    _mpu9250_write_byte(_mpu.mpu_address, CONFIG, 0x02);
    // DLPF BW a 92Hz, sampling rate 1KHz, filter delay 7.8ms
    _mpu9250_write_byte(_mpu.mpu_address, ACCEL_CONFIG2, 0x02);
    // Leo la vieja configuracion de fondo de escala del giroscopo y reseteo los bits de GYRO_FS_SEL (FS 250dps)
    uint8_t gyro_config_old = _mpu9250_read_byte(_mpu.mpu_address, GYRO_CONFIG);
    uint8_t gyro_config = gyro_config_old & GYRO_FS_SEL_RST_MASK;
    // Escribo la nueva configuracion
    _mpu9250_write_byte(_mpu.mpu_address, GYRO_CONFIG, gyro_config);
    // Leo la vieja configuracion de fondo de escala del acelerometro y reseteo los bits de ACCEL_FS_SEL (FS 2g)
    uint8_t accel_config_old = _mpu9250_read_byte(_mpu.mpu_address, ACCEL_CONFIG);
    uint8_t accel_config = accel_config_old & ACCEL_FS_SEL_RST_MASK;
    // Escribo la nueva configuracion
    _mpu9250_write_byte(_mpu.mpu_address, ACCEL_CONFIG, accel_config);

    // Hago 200 lecturas y promedio
    _mpu9250_read_and_average(accel_av, gyro_av, 200);

    // Habilito self test

    // Seteo los tres bits mas significativos de las configuraciones del gyro y accel para habilitar self test
    gyro_config |= 0xe0;
    accel_config |= 0xe0;
    // Escribo configuraciones
    _mpu9250_write_byte(_mpu.mpu_address, GYRO_CONFIG, gyro_config);
    _mpu9250_write_byte(_mpu.mpu_address, ACCEL_CONFIG, accel_config);
    // Espero 20ms para que se estabilicen las oscilaciones
    sleep_ms(20);

    // Hago 200 lecturas y promedio con self test
    _mpu9250_read_and_average(accel_st_av, gyro_st_av, 200);
    
    // Limpio la configuracion anterior
    _mpu9250_write_byte(_mpu.mpu_address, GYRO_CONFIG, gyro_config & ~0xe0);
    _mpu9250_write_byte(_mpu.mpu_address, ACCEL_CONFIG, accel_config & ~0xe0);
    // Espero 20 ms
    sleep_ms(20);
    // Vuelvo a la operacion anterior
    _mpu9250_write_byte(_mpu.mpu_address, GYRO_CONFIG, gyro_config_old);
    _mpu9250_write_byte(_mpu.mpu_address, ACCEL_CONFIG, accel_config_old);

    // Obtengo los valores de self test para cada eje
    uint8_t accel_st_code[3] = { 0 }, gyro_st_code[3] = { 0 };
    _mpu9250_read_bytes(_mpu.mpu_address, SELF_TEST_X_GYRO, gyro_st_code, 3);
    _mpu9250_read_bytes(_mpu.mpu_address, SELF_TEST_X_ACCEL, accel_st_code, 3);

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

/**
 * @brief Lee la temperatura corregida
 * @return devuelve temperatura corregida
*/
float mpu9250_read_temperature(void) {
    // Variable para guardar la temperatura
    int16_t temp;
    // Leo la temperatura sin ajustar
    _mpu9250_read_raw_temp(&temp);
    // Devuelvo temperatura corregida
    return temp / 333.87 + 21.0;
}

// Funciones privadas

/**
 * @brief Escribe un byte por I2C
 * @param addr direccion del dispositivo
 * @param reg direccion del registro a escribir
 * @param val valor a escribir en el registro
*/
static inline void _mpu9250_write_byte(uint8_t addr, uint8_t reg, uint8_t val) {
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
static inline uint8_t _mpu9250_read_byte(uint8_t addr, uint8_t reg) {
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
static inline void _mpu9250_read_bytes(uint8_t addr, uint8_t reg, uint8_t *dst, uint8_t len) {
    // Apunto al registro que quiero leer
    i2c_write_blocking(_mpu.i2c, addr, &reg, 1, true);
    // Leo el registro
    i2c_read_blocking(_mpu.i2c, addr, dst, len, false);
}

/**
 * @brief Efectua una cantidad de lecturas del acelerometro y giroscopo y las promedia
 * @param accel_av puntero donde guardar los valores promediados del acelerometro (X, Y, Z)
 * @param gyro_av puntero donde guardar los valores promediados del giroscopo (X, Y, Z)
 * @param n_readings cantidad de lecturas
*/
static inline void _mpu9250_read_and_average(int16_t *accel_av, int16_t *gyro_av, uint8_t n_readings) {
    // Tomo 200 lecturas
    for(uint8_t i = 0; i < n_readings; i++) {
        // Variable para lectura de los 3 ejes
        uint8_t raw[6] = { 0 };
        // Leo los datos del acelerometro en X, Y, Z
        _mpu9250_read_bytes(_mpu.mpu_address, ACCEL_XOUT_H, raw, 6);
        // Acumulo el valor de 16 bits leido (primero parte alta, luego baja)
        accel_av[0] += (int16_t)((int16_t)(raw[0] << 8) | raw[1]);
        accel_av[1] += (int16_t)((int16_t)(raw[2] << 8) | raw[3]);
        accel_av[2] += (int16_t)((int16_t)(raw[4] << 8) | raw[5]);
        // Leo los datos del giroscopo en X, Y, Z
        _mpu9250_read_bytes(_mpu.mpu_address, GYRO_XOUT_H, raw, 6);
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
 * @brief Obtiene la resolucion del acelerometro a partir 
 * de la configuracion elegida
 * @param accel_fs_sel valor elegido para el registro
 * @return valor de resolucion
*/
static float _mpu9250_get_accel_resolution(mpu9250_accel_fs_sel_t accel_fs_sel) {
    // Posibles escalas del acelerometro
    switch (accel_fs_sel) {
        // Se calculan como la resolucion a fondo de escala FS sobre
        // la resolucion del ADC (15 bits)
        case A2G:
            return 2.0 / 32768.0;
        case A4G:
            return 4.0 / 32768.0;
        case A8G:
            return 8.0 / 32768.0;
        case A16G:
            return 16.0 / 32768.0;
        default:
            return 0.0;
    }
}

/**
 * @brief Obtiene la resolucion del giroscopo a partir 
 * de la configuracion elegida
 * @param gyro_fs_sel valor elegido para el registro
 valor elegido para el registro
 * @return valor de resolucion
*/
static float _mpu9250_get_gyro_resolution(mpu9250_gyro_fs_sel_t gyro_fs_sel) {
    // Posibles escalas del giroscopo
    switch (gyro_fs_sel) {
        // Se calculan como la resolucion a fondo de escala FS sobre
        // la resolucion del ADC (15 bits)
        case G250DPS:
            return 250.0 / 32768.0;
        case G500DPS:
            return 500.0 / 32768.0;
        case G1000DPS:
            return 1000.0 / 32768.0;
        case G2000DPS:
            return 2000.0 / 32768.0;
        default:
            return 0.0;
    }
}

/**
 * @brief Inicializacion interna del MPU9250
*/
static void _mpu9250_init(void) {
    // Variables auxiliares
    uint8_t addr = _mpu.mpu_address;
    mpu9250_settings_t settings = _mpu.settings;
    // Reseteo dispositivo seteando el bit 7 de reset
    _mpu9250_write_byte(addr, PWR_MGMT_1, 0x80);
    sleep_ms(100);
    // Wake up limpiando el bit 6 de sleep, habilito todos los sensores
    _mpu9250_write_byte(addr, PWR_MGMT_1, 0x00);
    sleep_ms(100);
    // Selecciono fuente de clock al PLL del giroscopo como referencia
    _mpu9250_write_byte(addr, PWR_MGMT_1, 0x01);
    sleep_ms(200);
    // Configuro giroscopo y termometro
    // Deshabilito FSYNC y seteo BW del termometro y gyro a 41Hz y 42Hz
    // delay minimo de 5.9 ms
    _mpu9250_write_byte(addr, CONFIG, (uint8_t) settings.gyro_dlpf_cfg);

    // Sample rate = gyro output rate / (1 + SMPLRT_DIV) va a ser de 200Hz
    _mpu9250_write_byte(addr, SMPLRT_DIV, (uint8_t) settings.fifo_sample_sate);

    // Configuro rango a fondo de escala de giroscopo
    uint8_t gyro_config = ((uint8_t)(settings.gyro_fs_sel) << 3) | ((uint8_t)(~settings.gyro_fchoice) & 0x03);
    _mpu9250_write_byte(addr, GYRO_CONFIG, gyro_config);

    // Configuro rango a fondo de escala del acelerometro
    uint8_t accel_config = (uint8_t)(settings.accel_fs_sel) << 3;
    _mpu9250_write_byte(addr, ACCEL_CONFIG, accel_config);

    // Configuro sample rate del acelerometro
    accel_config = (~(settings.accel_fchoice << 3) & 0x08) | (((uint8_t)settings.accel_dlpf_cfg) & 0x07);
    _mpu9250_write_byte(addr, ACCEL_CONFIG2, accel_config);

    // Configuro interrupciones y bypass enable
    _mpu9250_write_byte(addr, INT_PIN_CFG, 0x22);
    _mpu9250_write_byte(addr, INT_ENABLE, 0x01);
    sleep_ms(100);
}

/**
 * @brief Lee una cantidad de bytes y arma valores de 16 bits
 * @param dst puntero a destino de los datos
 * @param reg registro a partir del cual leer
 * @param len cantidad de bytes a leer
*/
static void _mpu9250_read_raw_data(int16_t *dst, uint8_t reg, uint8_t len) {
    // Array auxiliar
    uint8_t raw[len];
    // Leo la n cantidad de datos
    _mpu9250_read_bytes(_mpu.mpu_address, reg, raw, len);
    // Armo los datos de 16 bits
    for(uint8_t i = 0; i < len / 2; i++) {
        dst[i] = ((int16_t)raw[2 * i] << 8) | (int16_t)raw[2 * i + 1];        
    }
}

/**
 * @brief Lee datos del acelerometro
 * @param dst puntero a destino donde guardar los datos
*/
static inline void _mpu9250_read_raw_accel(int16_t *dst) {
    // Lee los datos del acelerometro
    _mpu9250_read_raw_data(dst, ACCEL_XOUT_H, 6);
}

/**
 * @brief Lee datos del sensor de temperatura
 * @param dst puntero a destino donde guardar los datos
*/
static inline void _mpu9250_read_raw_temp(int16_t *dst) {
    // Lee los datos del sensor de temperatura
    _mpu9250_read_raw_data(dst, TEMP_OUT_H, 2);
}

/**
 * @brief Lee datos del giroscopo
 * @param dst puntero a destino donde guardar los datos
*/
static inline void _mpu9250_read_raw_gyro(int16_t *dst) {
    // Lee los datos del giroscopo
    _mpu9250_read_raw_data(dst, GYRO_XOUT_H, 6);
}