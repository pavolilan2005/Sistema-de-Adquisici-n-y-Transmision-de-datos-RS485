# Protocolo binario ASTRA-I — versión 1

Los nodos 1, 2 y 3 usan este mismo formato de protocolo (versión 1). RS-485 y LoRa transportan los 32 bytes sin convertirlos a texto. El nodo 2 acepta una trama solamente si sincronía, versión, tipo y CRC-8 coinciden.

## Trama de telemetría (32 bytes)

| Offset | Bytes | Campo | Conversión |
|---:|---:|---|---|
| 0 | 1 | Sincronía | A5 |
| 1 | 1 | Sincronía | 5A |
| 2 | 1 | Versión | 01 |
| 3 | 1 | Tipo | 01 = datos |
| 4 | 1 | Secuencia | uint8, incrementa y envuelve de 255 a 0 |
| 5 | 1 | Banderas | bit 0 MPU válido; bit 1 BME válido |
| 6 | 2 | Aceleración X | int16 C2 Q1.14; g = valor / 16384 |
| 8 | 2 | Aceleración Y | int16 C2 Q1.14; g = valor / 16384 |
| 10 | 2 | Aceleración Z | int16 C2 Q1.14; g = valor / 16384 |
| 12 | 2 | Giro X | int16 C2 Q9.7; °/s = valor / 128 |
| 14 | 2 | Giro Y | int16 C2 Q9.7; °/s = valor / 128 |
| 16 | 2 | Giro Z | int16 C2 Q9.7; °/s = valor / 128 |
| 18 | 2 | Temperatura MPU | int16 C2 Q8.8; °C = valor / 256 |
| 20 | 2 | Temperatura BME | int16 C2 Q8.8; °C = valor / 256 |
| 22 | 3 | Presión BME | uint24 entero, Pa |
| 25 | 2 | Humedad BME | uint16 Q8.8; %RH = valor / 256 |
| 27 | 4 | Resistencia de gas | uint32 entero, Ω |
| 31 | 1 | CRC | CRC-8/ATM de bytes 2–30 |

Todos los enteros de varios bytes se envían big-endian. Los campos con signo usan complemento a dos. La representación de aceleración tiene 14 bits fraccionarios; giro, 7; temperaturas y humedad, 8. La presión usa Pa enteros, dentro del rango de 24 bits. Valores sin sensor se acompañan de su bandera válida en cero; no se interpretan como mediciones.

## CRC

Parámetros CRC-8/ATM (CRC-8/ITU): polinomio normal 0x07, valor inicial 0x00, RefIn=false, RefOut=false, XorOut=0x00. El cálculo comienza en el byte de versión (offset 2), cubre hasta el último byte de gas (offset 30) e ignora sincronía y byte CRC.

El ACK tiene seis bytes: A5 5A 01 80 secuencia CRC. 0x80 identifica el ACK y el CRC se calcula sobre offsets 2–4. El nodo 2 responde el ACK RS-485 inmediatamente después de validar una trama; el ACK LoRa solo llega después de que el nodo 3 valide la misma trama. Son confirmaciones distintas.

## Escalas acordadas

- MPU6050 a ±2 g: 16384 cuentas por g; aceleración Q14 coincide con la lectura cruda del acelerómetro.
- MPU6050 a ±250 °/s: 131 cuentas por °/s; el firmware convierte a Q7 con redondeo.
- MPU6050: °C = raw/340 + 36.53; se codifica en Q8.
- BME680: API oficial Bosch compensa temperatura, presión, humedad y gas antes de codificar los valores.

## Cálculo y justificación de bits

La trama usa punto fijo para evitar transmitir `float` dependiente de la arquitectura. Cada escala se elige para cubrir el rango del sensor y mantener una resolución de cuantización más fina que la precisión física del sensor. Los bits de signo se interpretan como complemento a dos; las fracciones se obtienen dividiendo el entero recibido por la escala indicada.

| Magnitud | Rango que debe cubrir | Formato transmitido | Bits fraccionarios | Resolución de cuantización | Rango representable |
|---|---:|---|---:|---:|---:|
| Aceleración por eje | −2 a +2 g (MPU6050 configurado a ±2 g) | `int16`, escala 16384 cuentas/g | 14 | 1/16384 g = 0.0000610 g | −2 a +1.999939 g |
| Velocidad angular por eje | −250 a +250 °/s (MPU6050 configurado a ±250 °/s) | `int16`, escala 128 cuentas/(°/s) | 7 | 1/128 °/s = 0.0078125 °/s | −256 a +255.9921875 °/s |
| Temperatura MPU6050 | −40 a +85 °C | `int16`, escala 256 cuentas/°C | 8 | 1/256 °C = 0.00390625 °C | −128 a +127.99609375 °C |
| Temperatura BME680 | −40 a +85 °C | `int16`, escala 256 cuentas/°C | 8 | 1/256 °C = 0.00390625 °C | −128 a +127.99609375 °C |
| Presión BME680 | 0 a 16777215 Pa en el campo; cubre el intervalo atmosférico del sensor | `uint24`, Pa enteros | 0 | 1 Pa | 0 a 16777215 Pa |
| Humedad relativa BME680 | 0 a 100 %RH | `uint16`, escala 256 cuentas/%RH | 8 | 1/256 %RH = 0.00390625 %RH | 0 a 255.99609375 %RH |
| Resistencia de gas BME680 | 0 a 4294967295 Ω en el campo | `uint32`, Ω enteros | 0 | 1 Ω | 0 a 4294967295 Ω |

En los campos `int16` se reservan 16 bits totales y el bit más significativo participa en el signo. Por ejemplo, para aceleración se transmite `round(g × 16384)`; para recuperar el valor se calcula `int16_recibido / 16384`. Para giro se usa `round((°/s) × 128)` y se divide entre 128 al decodificar. Para temperaturas se multiplica/divide entre 256; para humedad también, pero el campo es sin signo. Presión y gas se redondean a la unidad entera indicada. La conversión STM32 limita valores fuera del rango representable antes de escribir el campo.

La resolución indicada es la granularidad digital de la trama, no la exactitud del sensor. La incertidumbre real depende de la tolerancia, configuración, calibración y condiciones de operación del MPU6050 y BME680.

## Prueba controlada de CRC

watch_corrupt_next_frame del nodo 1 altera un bit del payload después de formar el CRC. El nodo 2 registra la trama corrupta, incrementa su contador y no envía ACK RS-485 ni la reenvía por LoRa. En ThingSpeak, el par de CRC queda enclavado con la discrepancia y la máscara de estado conserva el bit 3 mientras el contador total refleja cada trama corrupta. El nodo 1 registra timeout. Desactivar la variable no requiere reinicio.

## Aprobación de interfaces

El PDF del proyecto pide al menos un sensor SPI y uno I²C en el STM32. El alumno informa que el profesor aprobó personalmente usar dos sensores I²C (MPU6050 y BME680) en el nodo 1 y mantener SPI para el radio LoRa SX1278 del nodo 2. La decisión se registra como autorización del docente; no se presenta como cumplimiento literal del enunciado PDF.
