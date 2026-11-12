# Protocolo y Keil Watch

## Trama RS485 del STM32

El STM32 forma primero el cuerpo sin delimitadores:

```text
D,seq,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps,mpu_temp_centiC,pres_Pa,bme_temp_centiC,hum_milliPct,gas_ohm
```

Despues calcula XOR byte a byte sobre ese cuerpo y transmite:

```text
$D,seq,...,gas_ohm*XX\r\n
```

`XX` es hexadecimal de dos digitos. El nodo 2 descarta la trama si no inicia con `$`, no contiene `*`, el XOR no coincide o la secuencia no cabe en un byte.

| Campo | Unidad transmitida | Conversión |
|---|---|---|
| `seq` | 0–255 | secuencia de 8 bits, wrap a cero |
| `ax_mg`, `ay_mg`, `az_mg` | mg | dividir entre 1000 para obtener g |
| `gx_mdps`, `gy_mdps`, `gz_mdps` | mdps | dividir entre 1000 para obtener grados/s |
| `mpu_temp_centiC` | centi-°C | dividir entre 100 para obtener °C |
| `pres_Pa` | Pa | reservada, actualmente cero |
| `bme_temp_centiC` | centi-°C | reservada, actualmente cero |
| `hum_milliPct` | milésimas de % | reservada, actualmente cero |
| `gas_ohm` | ohmios | reservada, actualmente cero |

## ACK

El gateway responde al STM32 inmediatamente después de validar la trama:

```text
$A,seq,OK\r\n
```

El STM32 espera hasta 300 ms y compara la secuencia recibida con la transmitida. El LED RX PB12 pulsa 100 ms solo ante un ACK válido de esa secuencia.

El gateway posteriormente retransmite la muestra por LoRa. El ACK RS485 confirma la recepción en el gateway; el ACK LoRa confirma el salto a la estación. La tarea LoRa acepta `$A,seq,OK` o `ACK,seq,OK` cuando la secuencia coincide.

## Variables para Keil Watch

| Variable | Qué observar | Valor esperado |
|---|---|---|
| `watch_heartbeat_count` | actividad de PC13 | aumenta aproximadamente cada 200 ms |
| `watch_mpu_ok` | resultado de WHO_AM_I | `1` si el ID del MPU6050 es `0x68` |
| `watch_sensor_count` | ciclo de adquisición | aumenta aproximadamente cada segundo |
| `watch_sensor_error_count` | fallos de MPU6050 | permanece en cero en operación normal |
| `accel_x_mg`, `accel_y_mg`, `accel_z_mg` | aceleración procesada | alrededor de ±1000 mg por eje según orientación |
| `gyro_x_mdps`, `gyro_y_mdps`, `gyro_z_mdps` | giroscopio procesado | cerca de cero con la placa quieta |
| `mpu_temp_centi_c` | temperatura MPU | centi-°C, dividir entre 100 |
| `sequence_number`, `watch_last_sequence` | contador transmitido | incrementa modulo 256 tras una adquisición válida |
| `watch_tx_frame` | última trama TX | inicia `$D,` y contiene `*XX` y CR/LF |
| `watch_last_ack_ok` | ACK de la secuencia actual | `1` solo si secuencia y texto coinciden |
| `watch_ack_count` | ACK válidos acumulados | incrementa por cada respuesta correcta |
| `watch_timeout_count` | errores TX/timeout ACK | estable si RS485 responde normalmente |
| `watch_rx_line` | última respuesta completa | `$A,<seq>,OK` |
| `bme_pressure_pa`, `bme_temperature_centi_c`, `bme_humidity_milli_pct`, `bme_gas_ohms` | campos BME reservados | permanecen en cero en este firmware |

`watch_tx_frame` y `watch_rx_line` son arreglos `volatile`; Keil puede mostrarlos como bytes o caracteres. Mantén el target en Run para que los contadores se actualicen.

## Estado del ESP32 nodo 2

En el monitor serie se esperan los mensajes `SX1278 VERSION=0x12`, `Trama STM32 válida, secuencia=N`, el envío LoRa y el resultado del ACK del nodo 3.

Contadores de `main.c`: `rs485_valid_frames`, `rs485_ack_sent`, `rs485_ack_errors`, `lora_frames_replaced`, `lora_ack_received`, `lora_ack_timeouts` y `lora_tx_errors`.
