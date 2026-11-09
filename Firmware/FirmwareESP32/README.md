# Transmisor LoRa ESP32 + SX1278 + MPU6050 + BME680

Proyecto ESP-IDF 6.1 para leer un MPU6050 y, cuando está conectado, un BME680 por I2C y enviar sus lecturas en texto ASCII por LoRa sin pausa fija entre paquetes. El LED en D2/GPIO2 parpadea mientras se transmite. El BME680 es opcional durante las pruebas: si no está presente, el transmisor manda los campos del MPU6050 y `SEQ`.

## Conexiones I2C de los sensores

Los dos sensores comparten SDA, SCL, alimentación y tierra:

| Sensor | Pin del módulo | ESP32 clásico |
|---|---|---:|
| MPU6050 | SDA | GPIO21 |
| BME680 | SDA | GPIO21 |
| MPU6050 | SCL | GPIO22 |
| BME680 | SCL | GPIO22 |
| Ambos | GND | GND |
| Ambos | VCC | 3.3 V o según la placa del módulo |

Direcciones predeterminadas del firmware: MPU6050 `0x68` (AD0 a GND) y BME680 `0x76` (SDO a GND). Si los pines AD0/SDO están altos, cambia las direcciones a `0x69`/`0x77` al inicio de `main/main.c`. Usa módulos con niveles I2C compatibles con 3.3 V; agrega pull-ups SDA/SCL si la placa no los incorpora.

## Conexiones del SX1278

| SX1278 | ESP32 clásico |
|---|---:|
| SCK | GPIO18 |
| MISO | GPIO19 |
| MOSI | GPIO23 |
| NSS / CS | GPIO5 |
| RESET | GPIO14 |
| DIO0 | Sin conectar (el transmisor consulta TX_DONE por SPI) |
| GND | GND |
| VCC | Según la placa del módulo |
| Antena | Antena apropiada para 433 MHz |

El LED de transmisión usa GPIO2 (D2 en muchas placas). Para un LED externo, conecta GPIO2 a la resistencia y al ánodo del LED; conecta el cátodo a GND.

## Paquete LoRa

El firmware envía un paquete separado por espacios, por ejemplo:

```text
SEQ:25 TEMP:22.4 PRES:1012.3 HUM:48.5 GAS:85432 AX:0.02 AY:-0.01 AZ:0.99 GX:1.2 GY:-0.7 GZ:0.4 TM:23.1
```

`TEMP`, `PRES`, `HUM` y `GAS` vienen del BME680; presión en hPa y gas en ohmios. `AX/AY/AZ` son g y `GX/GY/GZ` grados por segundo. `TM` es la temperatura opcional del MPU6050. `SEQ` aumenta cuando se transmite correctamente. El código no envía paquetes si falla una lectura del sensor y comprueba que el payload quede por debajo de 128 bytes (el receptor reserva un byte para el terminador de texto).

Parámetros de radio idénticos al receptor `receptor_lora_tft`: 433 MHz, SF7, BW 125 kHz, CR 4/5, encabezado explícito, CRC habilitado, IQ normal y sync word `0x12`.

## Compilar, grabar y monitorizar

Abre el entorno de ESP-IDF 6.1, entra a la carpeta del proyecto y ejecuta:

```powershell
idf.py set-target esp32
idf.py build
idf.py -p COMx flash monitor
```

El gestor de componentes descargará las dependencias indicadas en `main/idf_component.yml` durante la configuración/compilación. Cambia `COMx` por el puerto de la placa. El log muestra el número de secuencia, tamaño y contenido del paquete enviado.

## Documentación Doxygen

Las funciones, estructuras, registros de radio, constantes y flujo de ejecución están documentados en el código fuente con comentarios Doxygen en español. Para generar la referencia HTML desde la raíz del proyecto, ejecuta:

```powershell
doxygen .\Doxyfile
```

En Windows, si Doxygen no está agregado al `PATH`, usa:

```powershell
& "C:\Program Files\doxygen\bin\doxygen.exe" .\Doxyfile
```

Abre `docs/api/index.html` para consultar la documentación navegable, incluidos los enlaces al código fuente.

