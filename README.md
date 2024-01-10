# rpipico-mpu9250

Biblioteca para usar el MPU9250 con la Raspberry Pi Pico y su SDK. Puede agregarse de forma particular a un proyecto o agregarse a un proyecto de PlatformIO agregando la dependencia en el `platformio.ini`:

```
lib_deps = https://github.com/carlassaraf/rpipico-mpu9250.git
```

## Nota sobre implementacion

De momento, la biblioteca no tiene soporte para el magnetometro (AK8963). Solo es posible leer el acelerometro, giroscopo y sensor de temperatura por I2C.
