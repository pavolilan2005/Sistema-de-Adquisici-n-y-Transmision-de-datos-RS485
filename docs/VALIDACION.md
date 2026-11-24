# Matriz de verificación de extremo a extremo

Esta matriz registra las pruebas que el autor ha reportado como ejecutadas durante la integración. No reemplaza mediciones de laboratorio ni representa resultados cuantitativos de alcance/rendimiento.

| Requisito / etapa | Método de comprobación | Evidencia observada | Estado |
|---|---|---|---|
| Heartbeat del nodo 1 | Mantener STM32 en Run y observar PC13/`watch_heartbeat_count`. | PC13 cambia cada 200 ms; el proyecto compiló sin errores reportados. | Verificado |
| Sensores del nodo 1 | Revisar variables de Keil Watch y flags MPU/BME. | MPU6050 y BME680 entregaron variables procesadas; los flags distinguen validez. | Verificado por el autor |
| Integridad STM32 → nodo 2 | Comparar cabecera, tipo, longitud y CRC-8 de la trama binaria. | El nodo 2 registra tramas RS-485 válidas y contadores de secuencia. | Verificado |
| ACK RS-485 | Enviar trama válida y observar ACK/LED del STM32. | El ACK corresponde a la secuencia; los contadores Watch permiten observar ACK y timeout. | Verificado |
| Rechazo de trama corrupta | Activar `watch_corrupt_next_frame` una vez; revisar log/contador del gateway. | El nodo 2 reportó CRC incorrecto y descartó esa trama. | Verificado |
| Reenvío LoRa y ACK | Mantener nodos 2 y 3 activos y revisar secuencia/ACK en sus monitores. | Se observaron ACK LoRa válidos y continuidad de secuencia. | Verificado |
| Recepción y visualización nodo 3 | Revisar TFT y dashboard local. | Telemetría, estado del enlace y panel web activos durante la integración. | Verificado por el autor |
| ThingSpeak | Revisar las gráficas de los dos canales; repetir la corrupción controlada. | Ambos canales mostraron datos; el campo de tramas corruptas aumentó y los CRC se distinguieron en ThingSpeak. | Verificado |

## Límites conocidos

- La prueba de sensores SPI fue sustituida por dos sensores I²C en el nodo 1 con aprobación personal del profesor; la aprobación se registra en los READMEs y en la memoria técnica.
- La métrica de CPU de la estación terrena es una estimación ligera, elegida después de que la instrumentación global de FreeRTOS afectara la recepción LoRa.
- No se reporta aquí un porcentaje cuantitativo de pérdida, alcance máximo, autonomía, ni comparación estadística; esas mediciones requieren datos de ensayo registrados.
- La PCB compartida de nodos 1/2 y la PCB de la estación tuvieron correcciones manuales de señales; consúltese cada README de hardware antes de fabricar otra revisión.

## Registro que conviene conservar para la demostración

1. Captura de Keil Watch con sensores, flags, secuencia y ACK.
2. Log serie del nodo 2 con recepción RS-485 válida y ACK LoRa válido.
3. Log de una trama corrupta y capturas del Canal 2 de ThingSpeak con CRC recibido/calculado y contador.
4. Captura de la TFT con valores, gráfica y estado de enlace.
5. Foto del sistema con los tres nodos conectados.

Las claves Wi-Fi y Write API de ThingSpeak no deben incluirse en estas capturas ni en el repositorio.
