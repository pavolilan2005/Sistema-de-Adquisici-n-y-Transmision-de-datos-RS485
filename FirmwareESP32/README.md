# Gateway ESP32 nodo 2 — Firmware v1.0.3

Gateway RS-485 a LoRa con ESP-IDF 6.1 y FreeRTOS. Recibe la trama binaria del STM32, valida su CRC-8/ATM, devuelve el ACK RS-485 y reenvía por SX1278 exactamente los mismos 32 bytes. Una tarea independiente publica las mediciones y diagnóstico a ThingSpeak.

## Ajustes de enlace

- RS-485: UART2, 115200 8N1; TX17, RX16; DE33, /RE25.
- LoRa SX1278: SCK18, MISO19, MOSI23, NSS5, RESET4, DIO0 26.
- Radio: 433 MHz, SF7, BW125 kHz, CR4/5, CRC de radio y sync word 0x12.
- LEDs: LoRa TX GPIO27, LoRa RX GPIO32, indicador integrado GPIO2.

Las tareas RS485 y LoRa se mantienen separadas. El ACK RS-485 sale inmediatamente después de validar la trama; el envío LoRa y la espera de ACK de la estación no bloquean la recepción serial.

## Protocolo

Los tres nodos usan un paquete binario fijo de 32 bytes con cabecera A5 5A, versión 1, secuencia, banderas de sensores, campos fijos big-endian y CRC-8/ATM. El ACK usa seis bytes y el mismo CRC. La especificación completa y la tabla de campos están en main/telemetry_protocol.h.

## ThingSpeak

ThingSpeak se activa si se configuran SSID Wi-Fi y dos Write API Keys. En este proyecto el nodo 2 se conecta a una red Wi-Fi en modo estación y actualiza dos canales cada 16 segundos:

- Canal 1: aceleración, giro y temperaturas.
- Canal 2: presión, humedad, gas, secuencia, CRC recibido, CRC calculado, contador de tramas corruptas y máscara de estado (bit 0 trama más reciente válida, bit 1 MPU, bit 2 BME, bit 3 se ha detectado CRC inválido). Los CRC quedan enclavados en la última discrepancia para que la actualización más lenta de la nube no oculte una trama corrupta.

Distribución exacta de campos:

| Canal | Campo | Variable | Unidad / contenido |
|---|---:|---|---|
| 1 | 1–3 | Aceleración X/Y/Z | g |
| 1 | 4–6 | Giro X/Y/Z | °/s |
| 1 | 7 | Temperatura MPU6050 | °C |
| 1 | 8 | Temperatura BME680 | °C |
| 2 | 1 | Presión BME680 | Pa |
| 2 | 2 | Humedad BME680 | %RH |
| 2 | 3 | Resistencia de gas BME680 | Ω |
| 2 | 4 | Secuencia | 0–255 |
| 2 | 5 | CRC recibido | byte decimal (0–255) |
| 2 | 6 | CRC calculado | byte decimal (0–255) |
| 2 | 7 | Tramas corruptas acumuladas | contador desde el arranque |
| 2 | 8 | Máscara de estado | bit 0 trama reciente válida; bit 1 MPU válido; bit 2 BME válido; bit 3 CRC inválido detectado |

Copia main/cloud_secrets.h.example como main/cloud_secrets.h y completa las cuatro macros. cloud_secrets.h está excluido de Git; no publiques las claves. En ThingSpeak configura ocho campos por canal con nombres y unidades acordes con la lista anterior. Si el archivo contiene cadenas vacías, el gateway conserva RS-485 y LoRa y omite la conexión Wi-Fi.

La tasa de 16 s respeta el intervalo mínimo de 15 s por canal de la cuenta gratuita de ThingSpeak. Los datos de radio siguen avanzando a la frecuencia de muestreo, aunque la nube publique más lentamente.

## Compilar y cargar

Abre este directorio con la extensión ESP-IDF de VS Code o ejecuta:
idf.py set-target esp32
idf.py build
idf.py -p COMx flash monitor

Cambia COMx por el puerto serie real. No se guardan credenciales en el firmware ni se requiere una cuenta para compilar el resto del nodo.

## Versionado

Cada cambio del firmware incrementa el PATCH en el comentario inicial del código y en CMakeLists.txt / Doxyfile.

## Doxygen

Desde la raíz de `FirmwareESP32`, genera la referencia API ejecutando:

```powershell
doxygen Doxyfile
```

La configuración del proyecto está en `Doxyfile`; conserva los comentarios Doxygen de `main.c`, `cloud.c` y los headers al modificar el firmware.
