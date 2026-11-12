# Sistema de adquisición y transmisión de datos

Documentación de los dos firmwares incluidos en esta carpeta. Versiones documentales actuales: STM32 **1.0.1** y gateway ESP32 **1.0.1**.

## Arquitectura

```text
MPU6050 ─I2C1─┐
              ├─ STM32F411 ─ USART1 ─ MAX485 ═ RS485 ═ MAX485 ─ UART2 ─ ESP32 nodo 2
BME680 (pendiente)┘                                              │
                                                                  └─ SPI2 ─ SX1278 ))) LoRa ))) estación nodo 3
```

El STM32 mide el MPU6050 y envía una trama por RS485. El gateway responde el ACK RS485 al recibir una trama válida, la conserva en una cola de una muestra y después la retransmite por LoRa. El gateway espera un ACK de la estación terrena antes de dar por confirmado el salto LoRa. En esta carpeta no está incluido el firmware de la estación nodo 3.

## Firmware STM32F411

Proyecto STM32CubeMX/Keil: `FirmwareSTM32/SistemaAdquisicionDatos/SistemaAdquisicionDatos.ioc` y `MDK-ARM/SistemaAdquisicionDatos.uvprojx`.

### Pines y periféricos

| Función | Pin STM32 |
|---|---|
| I2C1 SCL / SDA | PB6 / PB7 |
| USART1 TX / RX hacia MAX485 | PA9 / PA10 |
| MAX485 DE / `/RE` | PA2 / PA1 |
| LED RX de ACK | PB12 |
| LED TX RS485 | PB13 |
| Heartbeat | PC13 |

I2C1 trabaja a 100 kHz; USART1 trabaja a 115200, 8N1. El MPU6050 está configurado con AD0 a GND (dirección `0x68`), rango de acelerómetro ±2 g y giroscopio ±250 °/s.

**BME680:** este firmware aún no lo inicializa ni lo lee. Los campos de presión, temperatura BME, humedad y gas permanecen en cero en la trama.

### Funcionamiento

- El LED PC13 cambia de estado cada 200 ms como indicador de ejecución.
- Cada segundo se lee el MPU6050, se convierten los valores a mg, mdps y centi-°C, y se transmite la trama.
- El LED TX PB13 se activa durante la transmisión RS485.
- Si llega el ACK correspondiente, RX PB12 se enciende durante 100 ms.
- El timeout de ACK es 300 ms. Si no hay ACK, aumenta `watch_timeout_count`.

## Firmware ESP32 nodo 2

Proyecto ESP-IDF 6.1: `FirmwareESP32`.

### Pines

| Función | GPIO ESP32 |
|---|---:|
| UART2 TX / RX al MAX485 | 17 / 16 |
| MAX485 DE / `/RE` | 33 / 25 |
| LED TX LoRa / LED RX LoRa | 27 / 32 |
| LED integrado | 2 |
| SX1278 SCK / MISO / MOSI | 18 / 19 / 23 |
| SX1278 NSS / RESET / DIO0 | 5 / 4 / 26 |

UART2 también está a 115200, 8N1. En RS485 conecta A con A, B con B y GND común; usa el MAX485 para los dos extremos, no conectes UART directamente entre placas. Revisa terminación y polarización según longitud/topología del bus.

El SX1278 opera a 433 MHz, SF7, BW125 kHz, CR 4/5, CRC habilitado y sync word `0x12`. La banda y el perfil deben coincidir en los otros nodos.

### Tareas FreeRTOS

- `rs485_task`: recibe bytes de UART2, separa líneas por LF y procesa las tramas completas.
- `lora_task`: consume la muestra pendiente, transmite por LoRa y espera el acuse del nodo 3.
- La cola LoRa tiene longitud 1. Si radio está ocupado, una trama nueva sustituye la muestra aún pendiente; aumenta `lora_frames_replaced`.
- La respuesta RS485 se envía inmediatamente después de validar la trama, sin esperar el resultado del enlace LoRa.

## Protocolo

- STM32 → nodo 2: `$D,seq,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps,mpu_temp_centiC,pres_Pa,bme_temp_centiC,hum_milliPct,gas_ohm*XX\r\n`
- `XX`: XOR hexadecimal de todos los caracteres entre `$` y `*`, sin incluir delimitadores.
- Nodo 2 → STM32: `$A,seq,OK\r\n`; se envía solo si trama, checksum y secuencia son válidos.
- Nodo 2 → nodo 3: vuelve a enviar la trama válida terminada en CR/LF por LoRa.
- Nodo 3 → nodo 2: ACK aceptado como `$A,seq,OK` o `ACK,seq,OK`, con secuencia igual a la transmitida.

La secuencia es de 8 bits, de 0 a 255, y vuelve a 0 al desbordarse. Los campos BME en cero indican que la lectura aún no está implementada en el firmware STM32; no representan una medición ambiental real.

## Compilar y cargar

### STM32

1. Abre el archivo `.ioc` con STM32CubeMX/CubeIDE para revisar la configuración de periféricos.
2. Abre `MDK-ARM/SistemaAdquisicionDatos.uvprojx` en Keil MDK.
3. Compila el target `SistemaAdquisicionDatos` y programa el STM32F411CEU6 mediante ST-Link.

### ESP32

Desde PowerShell con ESP-IDF 6.1 cargado:

```powershell
cd FirmwareESP32
idf.py set-target esp32
idf.py build
idf.py -p COMx flash monitor
```

Sustituye `COMx` por el puerto del ESP32. El log informa inicio del nodo, recepción de tramas RS485, transmisión LoRa y recepción/timeouts de ACK.

## Versionado

El número de versión de cada fuente está en su comentario inicial. Al modificar código, incrementa el PATCH del firmware correspondiente y actualiza el `project(... VERSION ...)` del CMake de ESP-IDF y esta documentación.

## Archivos de apoyo

- [Mapa de trama, unidades y variables Keil Watch](docs/Protocolo-y-Keil-Watch.md)
- [README del gateway ESP32](FirmwareESP32/README.md)
