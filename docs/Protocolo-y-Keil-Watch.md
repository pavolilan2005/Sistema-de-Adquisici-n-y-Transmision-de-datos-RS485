# Protocolo binario y variables de Keil Watch

Esta guía describe el enlace entre el nodo 1 (STM32F411CEU6) y el nodo 2 (ESP32). Los campos y escalas completos también están definidos en [`FirmwareESP32/PROTOCOLO_BINARIO.md`](../FirmwareESP32/PROTOCOLO_BINARIO.md) y en `FirmwareSTM32/SistemaAdquisicionDatos/Core/Inc/telemetry_protocol.h`.

## Paquete STM32 → gateway

El STM32 envía **32 bytes binarios** por USART1 a 115200 8N1. No se trata de texto CSV: no contiene `$`, comas, `*`, CR ni LF. Los dos extremos usan el mismo formato y CRC-8/ATM.

| Offset | Longitud | Campo | Representación |
|---:|---:|---|---|
| 0–1 | 2 | Sincronía | `A5 5A` |
| 2 | 1 | Versión | `0x01` |
| 3 | 1 | Tipo | `0x01` datos |
| 4 | 1 | Secuencia | `uint8`, wrap 255→0 |
| 5 | 1 | Flags | bit 0 MPU válido; bit 1 BME válido |
| 6–17 | 12 | Aceleración XYZ y giro XYZ | `int16` big-endian, escalas Q14/Q7 |
| 18–21 | 4 | Temperatura MPU y BME | `int16` Q8.8, °C = entero / 256 |
| 22–24 | 3 | Presión BME | `uint24` big-endian, Pa enteros |
| 25–26 | 2 | Humedad BME | `uint16` Q8.8, %RH = entero / 256 |
| 27–30 | 4 | Resistencia de gas | `uint32` big-endian, Ω |
| 31 | 1 | CRC-8/ATM | cubre offsets 2–30 |

La aceleración se recupera como entero con signo / 16384 para obtener g. El giro se recupera como entero con signo / 128 para obtener °/s. Los sensores no válidos se identifican con flags; sus ceros no son mediciones.

### Parámetros CRC-8/ATM

- Polinomio normal: `0x07`.
- Valor inicial: `0x00`.
- RefIn: no; RefOut: no.
- XOR final: `0x00`.
- Trama de datos: bytes 2 a 30 inclusive; se excluyen sincronía y byte CRC.
- ACK: bytes 2 a 4 inclusive.

## ACK gateway → STM32

El ACK ocupa seis bytes: `A5 5A 01 80 secuencia crc`. El tipo `0x80` identifica ACK. El STM32 solo lo toma como válido si la sincronía, tipo, secuencia y CRC corresponden a la trama transmitida. Timeout: 300 ms. PB12 parpadea al reconocer el ACK; PB13 indica transmisión.

El ACK RS-485 confirma únicamente la recepción/validación del gateway. El ACK LoRa que devuelve el nodo 3 es otro mensaje y confirma el siguiente salto.

## Revisión en Keil Watch

Añade estas variables, manteniendo el target en Run:

- Funcionamiento: `watch_heartbeat_count`, `watch_sensor_count`, `watch_sensor_error_count`.
- Estado de sensores: `watch_mpu_ok`, `watch_bme_ok`, `watch_bme_data_valid`, `watch_bme_gas_valid`, `watch_bme_retry_count`.
- Valores físicos: `accel_x_g`, `accel_y_g`, `accel_z_g`, `gyro_x_dps`, `gyro_y_dps`, `gyro_z_dps`, `mpu_temperature_c`, `bme_temperature_c`, `bme_pressure_pa_float`, `bme_humidity_pct`, `bme_gas_ohms`.
- Enlace: `sequence_number`, `watch_last_sequence`, `watch_last_ack_ok`, `watch_ack_count`, `watch_timeout_count`, `watch_bad_ack_count`.
- Tramas: `watch_last_frame_flags`, `watch_last_crc`, `watch_tx_frame`, `watch_rx_frame`.
- Corrupción controlada: `watch_corrupt_next_frame`, `watch_corrupt_frame_count`.

La prueba de corrupción altera un bit después de calcular el CRC. El gateway debe descartarla, no confirmar esa secuencia y aumentar el contador de errores; el siguiente paquete válido debe continuar el flujo. La prueba se verificó con publicación activa a ThingSpeak.

Para monitorear ESP32 usa el monitor serial de ESP-IDF y los dos dashboards de ThingSpeak; Keil Watch no inspecciona las variables de esos nodos.
