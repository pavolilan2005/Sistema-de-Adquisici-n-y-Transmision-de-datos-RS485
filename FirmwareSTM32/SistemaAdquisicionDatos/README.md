# Nodo 1 — STM32F411CEU6

Firmware de adquisición del sistema ASTRA I. Proyecto generado/configurado con STM32CubeMX y compilado con Keil MDK-ARM. Versión de aplicación: **1.0.4**.

## Función

- Lee el acelerómetro y el giroscopio del MPU6050 y las magnitudes ambientales del BME680 por I²C1.
- Convierte las lecturas a unidades físicas y a campos de punto fijo para el protocolo binario común.
- Transmite una trama de 32 bytes por USART1 y MAX485 cada segundo.
- Espera un ACK binario de seis bytes de nodo 2; valida sincronía, tipo, secuencia y CRC-8.
- Cambia PC13 cada 200 ms para indicar que el lazo de aplicación sigue activo.
- Tolera que el BME680 no esté conectado: actualiza su estado y reintenta inicializarlo sin detener el MPU6050.

El profesor autorizó para este proyecto el uso de dos sensores I²C (MPU6050 y BME680) en el STM32. El bus SPI del sistema lo utiliza el radio SX1278 del nodo 2; esta aprobación queda documentada como excepción al requisito original de sensor SPI en el STM32.

## Pines y buses

| Función | Pin | Parámetros |
|---|---|---|
| I²C1 SCL / SDA | PB6 / PB7 | 100 kHz; bus compartido por MPU6050 y BME680 |
| USART1 TX / RX | PA9 / PA10 | 115200 bit/s, 8N1 |
| MAX485 DE / `/RE` | PA2 / PA1 | Alto para transmitir, bajo para recibir |
| LED TX RS-485 | PB13 | Pulso durante transmisión |
| LED RX / ACK | PB12 | Pulso de 100 ms al validar el ACK esperado |
| Heartbeat | PC13 | Cambio de estado cada 200 ms |

El LED TX/RX quedó invertido en la primera PCB. La asignación de firmware se ajustó al mapa actual PB13=TX y PB12=RX; verifica la continuidad en la placa antes de reproducirla.

## Sensores

### MPU6050

Dirección I²C de 7 bits `0x68` (AD0 conectado a GND; la HAL recibe la dirección desplazada). Configuración: acelerómetro ±2 g y giroscopio ±250 °/s. El firmware expone aceleración en `g`, giro en `°/s` y temperatura interna en `°C`.

### BME680

Se comunica por I²C y detecta dirección `0x76` o `0x77`. Usa la Sensor API de Bosch, modo forzado y compensación de temperatura, presión, humedad y gas. Una lectura solo se marca válida cuando el sensor reporta datos nuevos; gas tiene un indicador de validez separado que comprueba calentador estable. La medición usa objetivo de calentador de 300 °C y duración de 100 ms.

Si el sensor no está disponible, no interpretes ceros como una medición: revisa `watch_bme_ok`, `watch_bme_data_valid` y el bit BME de la trama.

## Protocolo y unidades

La UART transmite el paquete binario de 32 bytes sin delimitadores ASCII. Incluye cabecera `A5 5A`, versión, tipo, secuencia, flags de sensores, mediciones en punto fijo big-endian y CRC-8/ATM. En RS-485 el gateway responde un ACK binario independiente; el nodo no espera un ACK LoRa directamente.

La tabla de offsets, escalas, unidades, resolución y parámetros completos del CRC está en [`../../FirmwareESP32/PROTOCOLO_BINARIO.md`](../../FirmwareESP32/PROTOCOLO_BINARIO.md) y en la copia de referencia del repositorio (`docs/Protocolo-y-Keil-Watch.md`).

## Variables recomendadas para Keil Watch

| Grupo | Variables | Comportamiento esperado |
|---|---|---|
| Ejecución | `watch_heartbeat_count`, `watch_sensor_count` | Aumentan; heartbeat aproximadamente cada 200 ms y sensor cada 1 s. |
| Estado MPU | `watch_mpu_ok`, `watch_sensor_error_count` | MPU=1 cuando la identidad y lectura son válidas; errores estables en cero. |
| Estado BME | `watch_bme_ok`, `watch_bme_data_valid`, `watch_bme_gas_valid`, `watch_bme_retry_count` | Flags según presencia, nueva conversión y estabilidad del calentador. |
| Aceleración | `accel_x_g`, `accel_y_g`, `accel_z_g` | Valores en g; en reposo, la magnitud total suele aproximarse a 1 g. |
| Giro y temperatura | `gyro_x_dps`, `gyro_y_dps`, `gyro_z_dps`, `mpu_temperature_c`, `bme_temperature_c` | °/s y °C; el giro debe acercarse a cero con la placa quieta. |
| BME680 | `bme_pressure_pa_float`, `bme_humidity_pct`, `bme_gas_ohms` | Pa, %RH y Ω; compara solo cuando las flags de validez sean 1. |
| Trama y secuencia | `sequence_number`, `watch_last_sequence`, `watch_last_frame_flags`, `watch_last_crc`, `watch_tx_frame[0..31]` | Secuencia modulo 256; bytes A5 5A al inicio y CRC en offset 31. |
| ACK | `watch_last_ack_ok`, `watch_ack_count`, `watch_timeout_count`, `watch_bad_ack_count`, `watch_rx_frame[0..5]` | El contador de ACK aumenta cuando se valida secuencia y CRC. |
| Prueba de integridad | `watch_corrupt_next_frame`, `watch_corrupt_frame_count` | Al poner el primero en 1 se altera una trama después de calcular su CRC; el segundo aumenta. |

Keil Watch pertenece al flujo STM32. Para ESP-IDF usa el monitor serial, las vistas TFT/web y los datos de ThingSpeak.

## Compilar y programar

1. Revisa periféricos en `SistemaAdquisicionDatos.ioc` con STM32CubeMX.
2. Abre `MDK-ARM/SistemaAdquisicionDatos.uvprojx` en Keil MDK-ARM.
3. Compila el target `SistemaAdquisicionDatos` y programa el STM32F411CEU6 con ST-Link.

No incluyas en commits los objetos, archivos `.axf`/`.hex`, mapas, logs ni opciones personales de Keil; se regeneran localmente. Los archivos HAL/CMSIS y sus licencias permanecen en el proyecto.

## Prueba de trama corrupta

Activa `watch_corrupt_next_frame` para que una sola trama se altere después del cálculo del CRC. El nodo 2 debe detectar el CRC distinto, incrementar su contador de tramas corruptas, descartar esa muestra y no enviar ACK RS-485 para ella. La secuencia siguiente debe volver a validarse normalmente. Esta prueba ya se ejecutó y se observó su contador en ThingSpeak.

## Licencia de terceros

El controlador BME68x Sensor API de Bosch y la licencia BSD-3-Clause están incluidos en `Core/Inc`. Conserva ese aviso y las licencias de STM32 HAL/CMSIS cuando redistribuyas el repositorio.

## Doxygen

Desde la raíz de este proyecto (donde se encuentra `Doxyfile`), genera la referencia HTML con:

```powershell
doxygen Doxyfile
```

La salida se crea en `docs/doxygen/html`. Los archivos HTML generados se mantienen locales y no se versionan; comparte `Doxyfile`, este README y los comentarios fuente para regenerarla.
