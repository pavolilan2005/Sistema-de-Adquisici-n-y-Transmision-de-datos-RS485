# Gateway ESP32 nodo 2

Gateway RS485 a LoRa implementado con ESP-IDF 6.1 y FreeRTOS. La documentación integral del conjunto STM32 + ESP32 está en [README del firmware](../README.md); el formato de trama y la lista Keil Watch están en [Protocolo y Keil Watch](../docs/Protocolo-y-Keil-Watch.md).

## Ajustes de enlace

- RS485: UART2, 115200 8N1; TX17, RX16; DE33, `/RE`25.
- LoRa SX1278: SCK18, MISO19, MOSI23, NSS5, RESET4, DIO0 26.
- Radio: 433 MHz, SF7, BW125 kHz, CR4/5, CRC, sync word `0x12`.
- LEDs: LoRa TX GPIO27, LoRa RX GPIO32, indicador integrado GPIO2.

El nodo responde por RS485 al validar XOR y secuencia; luego encola la trama más reciente para LoRa y espera ACK de la estación. El UART de este código es 115200 8N1.
