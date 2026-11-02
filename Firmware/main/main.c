/**
 * @file main.c
 * @brief Firmware transmisor LoRa para ESP32 clásico, SX1278 y sensores I2C.
 *
 * @details
 * El módulo inicializa el bus I2C, lee aceleración, velocidad angular y
 * temperatura del MPU6050, y añade temperatura, presión, humedad y resistencia
 * de gas del BME680 cuando este sensor está disponible. El paquete ASCII se
 * transmite mediante el SX1278 con el perfil compatible con `receptor_lora_tft`.
 *
 * @section flujo Flujo de ejecución
 * `app_main()` configura el LED, inicializa sensores y radio y entra en un
 * ciclo que toma muestras, construye el payload, lo transmite y aumenta `SEQ`
 * únicamente si el envío terminó correctamente.
 *
 * @section dependencias Dependencias
 * La API Bosch adaptada para ESP-IDF realiza la compensación del BME680; el
 * componente `i2c_bus` administra el bus compartido con el MPU6050.
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "bme68x_i2c_esp_idf.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"

/** @name Pines de hardware
 *  Pines sugeridos para ESP32 clásico; se cambian aquí si el cableado difiere.
 *  @{
 */
#define LORA_SPI_HOST       SPI2_HOST
#define LORA_PIN_SCK        18
#define LORA_PIN_MISO       19
#define LORA_PIN_MOSI       23
#define LORA_PIN_CS          5
#define LORA_PIN_RESET      14
#define LED_PIN               2 /* D2 en muchas placas ESP32 = GPIO2 */
/** @} */

/** @name Perfil de radio
 *  Valores que deben coincidir con `lora_sx127x.c` del receptor.
 *  @{
 */
#define LORA_FREQUENCY_HZ   433000000UL
#define LORA_SPREADING_FACTOR 7
#define LORA_BANDWIDTH_CODE 0x07 /* 125 kHz */
#define LORA_CODING_RATE    1    /* 4/5 */
#define LORA_TX_POWER_DBM   17
#define TX_PERIOD_MS        0 /* Sin pausa fija entre paquetes */
#define LORA_MAX_PAYLOAD    127
/** @} */

/** @name Configuración I2C y sensores
 *  El MPU6050 y el BME680 comparten SDA/SCL. Las direcciones pueden variar
 *  según el nivel de AD0 y SDO en los módulos.
 *  @{
 */
#define I2C_PORT             I2C_NUM_0
#define I2C_PIN_SDA          21
#define I2C_PIN_SCL          22
#define I2C_FREQUENCY_HZ     400000
#define MPU6050_ADDRESS      0x68 /* AD0 bajo; 0x69 si AD0 está alto */
#define BME680_ADDRESS       0x76 /* SDO bajo; 0x77 si SDO está alto */
/** @} */

static const char *TAG = "lora_tx";
/** Handle del dispositivo SX1278 agregado al bus SPI. */
static spi_device_handle_t s_spi;
/** Handle del bus I2C compartido por ambos sensores. */
static i2c_bus_handle_t s_i2c_bus;
/** Handle del dispositivo MPU6050 dentro del bus I2C. */
static i2c_bus_device_handle_t s_mpu;
/** Handle devuelto por la API Bosch para el BME680. */
static bme68x_handle_t s_bme;
/** `true` cuando el BME680 fue detectado y configurado correctamente. */
static bool s_bme_ready;
/** Configuración de sobremuestreo y filtro aplicada al BME680. */
static struct bme68x_conf s_bme_conf;
/** Perfil de calentador usado para obtener la resistencia del gas del BME680. */
static struct bme68x_heatr_conf s_bme_heater;

/** @brief Direcciones de registros usados del SX1276/SX1278. */
enum {
    REG_FIFO                 = 0x00, /**< Puerto FIFO para escritura de payload. */
    REG_OP_MODE              = 0x01, /**< Modo operativo y selección LoRa. */
    REG_FRF_MSB              = 0x06, /**< Byte más significativo de frecuencia. */
    REG_FRF_MID              = 0x07, /**< Byte intermedio de frecuencia. */
    REG_FRF_LSB              = 0x08, /**< Byte menos significativo de frecuencia. */
    REG_PA_CONFIG            = 0x09, /**< Selección y potencia de PA. */
    REG_LNA                  = 0x0C, /**< Configuración del amplificador de recepción. */
    REG_FIFO_ADDR_PTR        = 0x0D, /**< Puntero actual de FIFO. */
    REG_FIFO_TX_BASE_ADDR    = 0x0E, /**< Dirección inicial de FIFO de transmisión. */
    REG_IRQ_FLAGS            = 0x12, /**< Banderas de interrupción LoRa. */
    REG_MODEM_CONFIG_1       = 0x1D, /**< BW, coding rate y tipo de encabezado. */
    REG_MODEM_CONFIG_2       = 0x1E, /**< Spreading factor y CRC de payload. */
    REG_PREAMBLE_MSB         = 0x20, /**< Byte alto de longitud de preámbulo. */
    REG_PREAMBLE_LSB         = 0x21, /**< Byte bajo de longitud de preámbulo. */
    REG_PAYLOAD_LENGTH       = 0x22, /**< Longitud del paquete en bytes. */
    REG_MODEM_CONFIG_3       = 0x26, /**< AGC y optimización de tasa de datos. */
    REG_DETECTION_OPTIMIZE   = 0x31, /**< Optimización de detección LoRa. */
    REG_INVERT_IQ            = 0x33, /**< Polaridad IQ normal/invertida. */
    REG_DETECTION_THRESHOLD  = 0x37, /**< Umbral de detección LoRa. */
    REG_SYNC_WORD            = 0x39, /**< Sync word del enlace. */
    REG_VERSION              = 0x42, /**< Identificador de la familia SX127x. */
};

/** @brief Valores de modo LoRa y bandera de fin de transmisión. */
enum {
    MODE_LONG_RANGE = 0x80, /**< Bit que activa el módem LoRa. */
    MODE_SLEEP      = 0x00, /**< Modo de bajo consumo antes de configurar el radio. */
    MODE_STDBY      = 0x01, /**< Standby: permite configurar FIFO y registros. */
    MODE_TX         = 0x03, /**< Transmisión LoRa. */
    IRQ_TX_DONE     = 0x08, /**< Máscara de interrupción TxDone. */
};

/**
 * @struct mpu6050_raw_t
 * @brief Lectura cruda de los siete canales consecutivos del MPU6050.
 *
 * Los valores son muestras signed de 16 bits antes de convertirlas a g, DPS
 * y grados Celsius. La escala configurada es +/-2 g, +/-250 DPS y temperatura
 * interna del MPU6050.
 */
typedef struct {
    int16_t ax;          /**< Aceleración X cruda, LSB a 16384 LSB/g. */
    int16_t ay;          /**< Aceleración Y cruda, LSB a 16384 LSB/g. */
    int16_t az;          /**< Aceleración Z cruda, LSB a 16384 LSB/g. */
    int16_t temperature; /**< Temperatura interna cruda, 340 LSB/°C. */
    int16_t gx;          /**< Velocidad angular X cruda, 131 LSB/(°/s). */
    int16_t gy;          /**< Velocidad angular Y cruda, 131 LSB/(°/s). */
    int16_t gz;          /**< Velocidad angular Z cruda, 131 LSB/(°/s). */
} mpu6050_raw_t;

/**
 * @brief Escribe un byte en un registro del SX127x.
 * @param address Dirección de registro de 7 bits.
 * @param value Byte que se escribirá.
 * @return `ESP_OK` si SPI completó la transacción; en otro caso, el error SPI.
 * @details El bit 7 del primer byte se activa para indicar operación de escritura.
 */
static esp_err_t reg_write(uint8_t address, uint8_t value)
{
    uint8_t tx[2] = {(uint8_t)(address | 0x80), value};
    spi_transaction_t transaction = {
        .length = sizeof(tx) * 8,
        .tx_buffer = tx,
    };
    return spi_device_transmit(s_spi, &transaction);
}

/**
 * @brief Lee un byte de un registro del SX127x.
 * @param address Dirección de registro de 7 bits.
 * @param[out] value Recibe el byte leído cuando la transacción termina bien.
 * @return `ESP_OK` si SPI completó la transacción; en otro caso, el error SPI.
 * @details El bit 7 de la dirección se limpia para indicar una lectura.
 */
static esp_err_t reg_read(uint8_t address, uint8_t *value)
{
    uint8_t tx[2] = {(uint8_t)(address & 0x7F), 0x00};
    uint8_t rx[2] = {0};
    spi_transaction_t transaction = {
        .length = sizeof(tx) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_transmit(s_spi, &transaction);
    if (err == ESP_OK) {
        *value = rx[1];
    }
    return err;
}

/**
 * @brief Copia un payload en la FIFO de transmisión del SX127x.
 * @param data Bytes que se enviarán.
 * @param length Cantidad de bytes; debe estar entre 1 y `LORA_MAX_PAYLOAD`.
 * @return `ESP_OK`, `ESP_ERR_INVALID_SIZE` o el error del controlador SPI.
 * @note El primer byte de la transacción selecciona el registro FIFO para escritura.
 */
static esp_err_t fifo_write(const uint8_t *data, size_t length)
{
    if (length == 0 || length > LORA_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_SIZE;
    }
    uint8_t tx[LORA_MAX_PAYLOAD + 1];
    tx[0] = REG_FIFO | 0x80;
    memcpy(&tx[1], data, length);
    spi_transaction_t transaction = {
        .length = (length + 1) * 8,
        .tx_buffer = tx,
    };
    return spi_device_transmit(s_spi, &transaction);
}

/**
 * @brief Cambia el SX127x a un modo LoRa.
 * @param mode Código de modo, por ejemplo `MODE_STDBY` o `MODE_TX`.
 * @return Estado de la escritura SPI del registro `REG_OP_MODE`.
 */
static esp_err_t radio_set_mode(uint8_t mode)
{
    return reg_write(REG_OP_MODE, MODE_LONG_RANGE | mode);
}

/**
 * @brief Inicializa el bus SPI y configura el SX1278 para el receptor asociado.
 * @return `ESP_OK` si detecta `REG_VERSION == 0x12` y programa el perfil completo;
 *         devuelve el error del controlador o `ESP_ERR_NOT_FOUND` si el radio no responde.
 * @details Configura 433 MHz, SF7, BW125 kHz, CR4/5, encabezado explícito,
 * CRC, preámbulo de 8 símbolos, sync word `0x12` e IQ normal.
 */
static esp_err_t radio_init(void)
{
    spi_bus_config_t bus_config = {
        .sclk_io_num = LORA_PIN_SCK,
        .mosi_io_num = LORA_PIN_MOSI,
        .miso_io_num = LORA_PIN_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(LORA_SPI_HOST, &bus_config, SPI_DMA_DISABLED), TAG, "No se pudo iniciar SPI");

    spi_device_interface_config_t device_config = {
        .clock_speed_hz = 4 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = LORA_PIN_CS,
        .queue_size = 1,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(LORA_SPI_HOST, &device_config, &s_spi), TAG, "No se pudo agregar SX1278 al bus SPI");

    gpio_config_t reset_config = {
        .pin_bit_mask = 1ULL << LORA_PIN_RESET,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&reset_config), TAG, "No se pudo configurar RESET");
    gpio_set_level(LORA_PIN_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_PIN_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t version = 0;
    ESP_RETURN_ON_ERROR(reg_read(REG_VERSION, &version), TAG, "No se pudo leer VERSION del radio");
    if (version != 0x12) {
        ESP_LOGE(TAG, "SX127x no detectado: VERSION=0x%02X", version);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_RETURN_ON_ERROR(radio_set_mode(MODE_SLEEP), TAG, "No se pudo poner el radio en sleep");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(radio_set_mode(MODE_STDBY), TAG, "No se pudo poner el radio en standby");

    const uint32_t frf = (uint32_t)(((uint64_t)LORA_FREQUENCY_HZ << 19) / 32000000ULL);
    ESP_RETURN_ON_ERROR(reg_write(REG_FRF_MSB, (frf >> 16) & 0xFF), TAG, "Error configurando frecuencia");
    ESP_RETURN_ON_ERROR(reg_write(REG_FRF_MID, (frf >> 8) & 0xFF), TAG, "Error configurando frecuencia");
    ESP_RETURN_ON_ERROR(reg_write(REG_FRF_LSB, frf & 0xFF), TAG, "Error configurando frecuencia");

    ESP_RETURN_ON_ERROR(reg_write(REG_PA_CONFIG, 0x80 | (LORA_TX_POWER_DBM - 2)), TAG, "Error configurando potencia");
    ESP_RETURN_ON_ERROR(reg_write(REG_LNA, 0x23), TAG, "Error configurando LNA");
    ESP_RETURN_ON_ERROR(reg_write(REG_FIFO_TX_BASE_ADDR, 0x00), TAG, "Error configurando FIFO");
    ESP_RETURN_ON_ERROR(reg_write(REG_FIFO_ADDR_PTR, 0x00), TAG, "Error configurando FIFO");
    ESP_RETURN_ON_ERROR(reg_write(REG_MODEM_CONFIG_1, (LORA_BANDWIDTH_CODE << 4) | (LORA_CODING_RATE << 1)), TAG, "Error configurando módem");
    ESP_RETURN_ON_ERROR(reg_write(REG_MODEM_CONFIG_2, (LORA_SPREADING_FACTOR << 4) | 0x04), TAG, "Error configurando módem");
    ESP_RETURN_ON_ERROR(reg_write(REG_MODEM_CONFIG_3, 0x04), TAG, "Error configurando módem");
    ESP_RETURN_ON_ERROR(reg_write(REG_PREAMBLE_MSB, 0x00), TAG, "Error configurando preámbulo");
    ESP_RETURN_ON_ERROR(reg_write(REG_PREAMBLE_LSB, 0x08), TAG, "Error configurando preámbulo");
    ESP_RETURN_ON_ERROR(reg_write(REG_SYNC_WORD, 0x12), TAG, "Error configurando sync word");
    ESP_RETURN_ON_ERROR(reg_write(REG_DETECTION_OPTIMIZE, 0xC3), TAG, "Error configurando SF7");
    ESP_RETURN_ON_ERROR(reg_write(REG_DETECTION_THRESHOLD, 0x0A), TAG, "Error configurando SF7");
    ESP_RETURN_ON_ERROR(reg_write(REG_INVERT_IQ, 0x27), TAG, "Error configurando IQ");
    ESP_RETURN_ON_ERROR(reg_write(REG_IRQ_FLAGS, 0xFF), TAG, "Error limpiando IRQ");

    ESP_LOGI(TAG, "SX1278 listo: 433 MHz, SF7, BW125, CR4/5, CRC, sync 0x12");
    return ESP_OK;
}

/**
 * @brief Inicializa I2C, configura el MPU6050 y detecta/configura el BME680.
 * @return `ESP_OK` si el bus y el MPU6050 están listos; devuelve error si falla
 *         el bus, el MPU o su configuración.
 * @details El BME680 es opcional: si no está presente o no puede configurarse,
 * `s_bme_ready` queda en `false` y el transmisor continúa con datos del MPU6050.
 * El MPU usa dirección 0x68, filtro DLPF, escala +/-2 g y +/-250 DPS.
 */
static esp_err_t i2c_sensors_init(void)
{
    const i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_PIN_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_PIN_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQUENCY_HZ,
    };
    s_i2c_bus = i2c_bus_create(I2C_PORT, &i2c_config);
    if (s_i2c_bus == NULL) {
        ESP_LOGE(TAG, "No se pudo crear el bus I2C");
        return ESP_FAIL;
    }

    s_mpu = i2c_bus_device_create(s_i2c_bus, MPU6050_ADDRESS, I2C_FREQUENCY_HZ);
    if (s_mpu == NULL) {
        ESP_LOGE(TAG, "No se pudo agregar MPU6050 al bus I2C");
        return ESP_FAIL;
    }

    uint8_t who_am_i = 0;
    ESP_RETURN_ON_ERROR(i2c_bus_read_byte(s_mpu, 0x75, &who_am_i), TAG, "No se pudo leer WHO_AM_I del MPU6050");
    if (who_am_i != 0x68) {
        ESP_LOGE(TAG, "MPU6050 no detectado: WHO_AM_I=0x%02X", who_am_i);
        return ESP_ERR_NOT_FOUND;
    }

    /* Despertar con PLL del eje X, filtro DLPF, escalas por defecto: +/-2 g y +/-250 DPS. */
    ESP_RETURN_ON_ERROR(i2c_bus_write_byte(s_mpu, 0x6B, 0x01), TAG, "No se pudo despertar MPU6050");
    ESP_RETURN_ON_ERROR(i2c_bus_write_byte(s_mpu, 0x1A, 0x03), TAG, "No se pudo configurar filtro MPU6050");
    ESP_RETURN_ON_ERROR(i2c_bus_write_byte(s_mpu, 0x1B, 0x00), TAG, "No se pudo configurar giroscopio");
    ESP_RETURN_ON_ERROR(i2c_bus_write_byte(s_mpu, 0x1C, 0x00), TAG, "No se pudo configurar acelerómetro");
    vTaskDelay(pdMS_TO_TICKS(100));

    bme68x_i2c_config_t bme_config = {
        .i2c_handle = s_i2c_bus,
        .i2c_addr = BME680_ADDRESS,
    };
    esp_err_t bme_err = bme68x_sensor_create(&bme_config, &s_bme);
    if (bme_err != ESP_OK) {
        s_bme_ready = false;
        ESP_LOGW(TAG, "BME680 no detectado en 0x%02X; se enviaran solo datos del MPU6050", BME680_ADDRESS);
    } else {
        s_bme_conf.filter = BME68X_FILTER_OFF;
        s_bme_conf.odr = BME68X_ODR_NONE;
        s_bme_conf.os_hum = BME68X_OS_2X;
        s_bme_conf.os_pres = BME68X_OS_1X;
        s_bme_conf.os_temp = BME68X_OS_2X;
        int8_t bme_status = bme68x_set_conf(&s_bme_conf, s_bme);
        if (bme_status == BME68X_OK) {
            s_bme_heater.enable = BME68X_ENABLE;
            s_bme_heater.heatr_temp = 300;
            s_bme_heater.heatr_dur = 100;
            bme_status = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &s_bme_heater, s_bme);
        }
        if (bme_status == BME68X_OK) {
            s_bme_ready = true;
            ESP_LOGI(TAG, "BME680 listo en 0x%02X", BME680_ADDRESS);
        } else {
            s_bme_ready = false;
            ESP_LOGW(TAG, "BME680 no pudo configurarse (estado=%d); se enviaran solo datos del MPU6050", bme_status);
        }
    }

    ESP_LOGI(TAG, "MPU6050 listo en 0x%02X; I2C SDA=%d SCL=%d",
             MPU6050_ADDRESS, I2C_PIN_SDA, I2C_PIN_SCL);
    return ESP_OK;
}

/**
 * @brief Combina dos bytes I2C en una muestra signed de 16 bits.
 * @param msb Byte más significativo.
 * @param lsb Byte menos significativo.
 * @return Valor de complemento a dos interpretado como `int16_t`.
 */
static int16_t bytes_to_i16(uint8_t msb, uint8_t lsb)
{
    return (int16_t)(((uint16_t)msb << 8) | lsb);
}

/**
 * @brief Lee acelerómetro, temperatura interna y giroscopio del MPU6050.
 * @param[out] sample Estructura que recibirá las siete muestras crudas.
 * @return `ESP_OK` si la lectura de 14 bytes desde `ACCEL_XOUT_H` termina bien;
 *         en otro caso devuelve el error I2C.
 * @pre `i2c_sensors_init()` terminó correctamente.
 */
static esp_err_t mpu6050_read(mpu6050_raw_t *sample)
{
    uint8_t bytes[14];
    ESP_RETURN_ON_ERROR(i2c_bus_read_bytes(s_mpu, 0x3B, sizeof(bytes), bytes), TAG, "Error leyendo MPU6050");
    sample->ax = bytes_to_i16(bytes[0], bytes[1]);
    sample->ay = bytes_to_i16(bytes[2], bytes[3]);
    sample->az = bytes_to_i16(bytes[4], bytes[5]);
    sample->temperature = bytes_to_i16(bytes[6], bytes[7]);
    sample->gx = bytes_to_i16(bytes[8], bytes[9]);
    sample->gy = bytes_to_i16(bytes[10], bytes[11]);
    sample->gz = bytes_to_i16(bytes[12], bytes[13]);
    return ESP_OK;
}

/**
 * @brief Divide enteros con redondeo al entero más cercano, conservando el signo.
 * @param numerator Numerador signed.
 * @param denominator Denominador positivo distinto de cero.
 * @return Cociente redondeado al entero más cercano.
 * @details Se usa para convertir las cuentas crudas del MPU a centésimas de g
 * y décimas de grados por segundo sin depender de formato flotante en printf.
 */
static int32_t divide_round_signed(int32_t numerator, int32_t denominator)
{
    if (numerator >= 0) {
        return (numerator + denominator / 2) / denominator;
    }
    return -((-numerator + denominator / 2) / denominator);
}

/**
 * @brief Convierte un valor de punto flotante a décimas con redondeo.
 * @param value Valor de entrada en su unidad original.
 * @return Valor signed escalado por diez; por ejemplo, 22.4 pasa a 224.
 */
static int32_t float_to_tenths(float value)
{
    const float scaled = value * 10.0f;
    return (int32_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

/**
 * @brief Obtiene el valor absoluto de una magnitud signed dentro del rango usado.
 * @param value Valor escalado que se va a mostrar.
 * @return Magnitud no negativa como `int` para los especificadores `%d`.
 * @note Las entradas de telemetría están limitadas al rango físico del sensor,
 * por lo que no se recibe `INT32_MIN`.
 */
static int magnitude(int32_t value)
{
    return (int)(value < 0 ? -value : value);
}

/**
 * @brief Ejecuta una medición forzada del BME680 y devuelve los valores compensados.
 * @param[out] sample Estructura Bosch que recibe temperatura (°C), presión (Pa),
 * humedad (%) y resistencia de gas (ohmios).
 * @return `ESP_OK` si existe al menos un campo de medición válido; de lo contrario,
 *         `ESP_FAIL`.
 * @details Espera el tiempo de conversión y calentador redondeado hacia arriba a
 * la resolución del tick de FreeRTOS, antes de leer los resultados.
 * @pre `s_bme_ready` debe ser `true` y el sensor ya debe estar configurado.
 */
static esp_err_t bme680_read(struct bme68x_data *sample)
{
    int8_t status = bme68x_set_op_mode(BME68X_FORCED_MODE, s_bme);
    if (status != BME68X_OK) {
        ESP_LOGE(TAG, "No se pudo iniciar medicion BME680: %d", status);
        return ESP_FAIL;
    }

    const uint32_t measurement_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &s_bme_conf, s_bme)
                                   + ((uint32_t)s_bme_heater.heatr_dur * 1000U);
    const uint32_t tick_us = (uint32_t)portTICK_PERIOD_MS * 1000U;
    const TickType_t measurement_ticks = (measurement_us + tick_us - 1U) / tick_us;
    vTaskDelay(measurement_ticks);

    uint8_t fields = 0;
    status = bme68x_get_data(BME68X_FORCED_MODE, sample, &fields, s_bme);
    if (status != BME68X_OK || fields == 0) {
        ESP_LOGE(TAG, "No se obtuvo una lectura BME680 valida (estado=%d, campos=%u)", status, fields);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief Construye el payload ASCII de telemetría dentro del límite del receptor.
 * @param sequence Número que identifica el paquete; aumenta tras cada TX exitosa.
 * @param[out] payload Buffer de salida para texto terminado en NUL.
 * @param capacity Capacidad total del buffer, incluido el terminador.
 * @param[out] length Cantidad de bytes de texto sin contar el NUL.
 * @return `ESP_OK` si ambas lecturas requeridas y el formato caben en el buffer;
 *         devuelve error I2C o `ESP_ERR_INVALID_SIZE` en caso contrario.
 * @details Siempre incluye `SEQ`, `AX/AY/AZ` en g, `GX/GY/GZ` en grados/s y
 * `TM` en °C. Si hay BME680 añade `TEMP` (°C), `PRES` (hPa), `HUM` (%) y
 * `GAS` (ohmios). No supera `LORA_MAX_PAYLOAD` (127 bytes).
 */
static esp_err_t build_payload(uint32_t sequence, char *payload, size_t capacity, size_t *length)
{
    mpu6050_raw_t mpu = {0};
    struct bme68x_data bme = {0};
    ESP_RETURN_ON_ERROR(mpu6050_read(&mpu), TAG, "No se pudo leer MPU6050");

    bool include_bme = s_bme_ready;
    if (include_bme && bme680_read(&bme) != ESP_OK) {
        ESP_LOGW(TAG, "Se perdio comunicacion con BME680; se continuara solo con MPU6050");
        s_bme_ready = false;
        include_bme = false;
    }

    const int32_t ax_hundredths = divide_round_signed((int32_t)mpu.ax * 100, 16384);
    const int32_t ay_hundredths = divide_round_signed((int32_t)mpu.ay * 100, 16384);
    const int32_t az_hundredths = divide_round_signed((int32_t)mpu.az * 100, 16384);
    const int32_t gx_tenths = divide_round_signed((int32_t)mpu.gx * 10, 131);
    const int32_t gy_tenths = divide_round_signed((int32_t)mpu.gy * 10, 131);
    const int32_t gz_tenths = divide_round_signed((int32_t)mpu.gz * 10, 131);
    const float mpu_temperature_c = ((float)mpu.temperature / 340.0f) + 36.53f;
    const int32_t mpu_temp_tenths = float_to_tenths(mpu_temperature_c);
    int written;

    if (include_bme) {
        const int32_t temp_tenths = float_to_tenths(bme.temperature);
        const int32_t pressure_tenths_hpa = float_to_tenths(bme.pressure / 100.0f);
        const int32_t humidity_tenths = float_to_tenths(bme.humidity);
        const uint32_t gas_ohms = (uint32_t)(bme.gas_resistance + 0.5f);
        written = snprintf(payload, capacity,
            "SEQ:%lu TEMP:%s%d.%d PRES:%s%d.%d HUM:%s%d.%d GAS:%lu AX:%s%d.%02d AY:%s%d.%02d AZ:%s%d.%02d GX:%s%d.%d GY:%s%d.%d GZ:%s%d.%d TM:%s%d.%d",
            (unsigned long)sequence,
            temp_tenths < 0 ? "-" : "", magnitude(temp_tenths) / 10, magnitude(temp_tenths) % 10,
            pressure_tenths_hpa < 0 ? "-" : "", magnitude(pressure_tenths_hpa) / 10, magnitude(pressure_tenths_hpa) % 10,
            humidity_tenths < 0 ? "-" : "", magnitude(humidity_tenths) / 10, magnitude(humidity_tenths) % 10,
            (unsigned long)gas_ohms,
            ax_hundredths < 0 ? "-" : "", magnitude(ax_hundredths) / 100, magnitude(ax_hundredths) % 100,
            ay_hundredths < 0 ? "-" : "", magnitude(ay_hundredths) / 100, magnitude(ay_hundredths) % 100,
            az_hundredths < 0 ? "-" : "", magnitude(az_hundredths) / 100, magnitude(az_hundredths) % 100,
            gx_tenths < 0 ? "-" : "", magnitude(gx_tenths) / 10, magnitude(gx_tenths) % 10,
            gy_tenths < 0 ? "-" : "", magnitude(gy_tenths) / 10, magnitude(gy_tenths) % 10,
            gz_tenths < 0 ? "-" : "", magnitude(gz_tenths) / 10, magnitude(gz_tenths) % 10,
            mpu_temp_tenths < 0 ? "-" : "", magnitude(mpu_temp_tenths) / 10, magnitude(mpu_temp_tenths) % 10);
    } else {
        written = snprintf(payload, capacity,
            "SEQ:%lu AX:%s%d.%02d AY:%s%d.%02d AZ:%s%d.%02d GX:%s%d.%d GY:%s%d.%d GZ:%s%d.%d TM:%s%d.%d",
            (unsigned long)sequence,
            ax_hundredths < 0 ? "-" : "", magnitude(ax_hundredths) / 100, magnitude(ax_hundredths) % 100,
            ay_hundredths < 0 ? "-" : "", magnitude(ay_hundredths) / 100, magnitude(ay_hundredths) % 100,
            az_hundredths < 0 ? "-" : "", magnitude(az_hundredths) / 100, magnitude(az_hundredths) % 100,
            gx_tenths < 0 ? "-" : "", magnitude(gx_tenths) / 10, magnitude(gx_tenths) % 10,
            gy_tenths < 0 ? "-" : "", magnitude(gy_tenths) / 10, magnitude(gy_tenths) % 10,
            gz_tenths < 0 ? "-" : "", magnitude(gz_tenths) / 10, magnitude(gz_tenths) % 10,
            mpu_temp_tenths < 0 ? "-" : "", magnitude(mpu_temp_tenths) / 10, magnitude(mpu_temp_tenths) % 10);
    }

    if (written < 0 || (size_t)written >= capacity || written > LORA_MAX_PAYLOAD) {
        ESP_LOGE(TAG, "Payload demasiado largo (%d bytes; limite %d)", written, LORA_MAX_PAYLOAD);
        return ESP_ERR_INVALID_SIZE;
    }
    *length = (size_t)written;
    return ESP_OK;
}

/**
 * @brief Transmite un paquete LoRa y espera a que el SX1278 indique TxDone.
 * @param data Bytes del paquete.
 * @param length Longitud entre 1 y `LORA_MAX_PAYLOAD` bytes.
 * @return `ESP_OK` al detectar `IRQ_TX_DONE`; `ESP_ERR_INVALID_SIZE` si el
 *         paquete no cabe; o el error SPI/`ESP_ERR_TIMEOUT` si no termina a tiempo.
 * @details Limpia IRQ anteriores, posiciona la FIFO, escribe la longitud y cambia
 * el radio a `MODE_TX`. El fin se consulta por SPI; no requiere cable DIO0.
 */
static esp_err_t send_packet(const uint8_t *data, size_t length)
{
    if (length == 0 || length > LORA_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_SIZE;
    }
    ESP_RETURN_ON_ERROR(radio_set_mode(MODE_STDBY), TAG, "No se pudo entrar a standby");
    ESP_RETURN_ON_ERROR(reg_write(REG_IRQ_FLAGS, 0xFF), TAG, "No se pudieron limpiar IRQ");
    ESP_RETURN_ON_ERROR(reg_write(REG_FIFO_ADDR_PTR, 0x00), TAG, "No se pudo posicionar FIFO");
    ESP_RETURN_ON_ERROR(fifo_write(data, length), TAG, "No se pudo escribir el paquete");
    ESP_RETURN_ON_ERROR(reg_write(REG_PAYLOAD_LENGTH, (uint8_t)length), TAG, "No se pudo establecer longitud");
    ESP_RETURN_ON_ERROR(radio_set_mode(MODE_TX), TAG, "No se pudo iniciar TX");

    for (int i = 0; i < 500; ++i) {
        uint8_t irq = 0;
        ESP_RETURN_ON_ERROR(reg_read(REG_IRQ_FLAGS, &irq), TAG, "No se pudo leer IRQ");
        if (irq & IRQ_TX_DONE) {
            ESP_RETURN_ON_ERROR(reg_write(REG_IRQ_FLAGS, IRQ_TX_DONE), TAG, "No se pudo limpiar TX_DONE");
            ESP_RETURN_ON_ERROR(radio_set_mode(MODE_STDBY), TAG, "No se pudo regresar a standby");
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    (void)radio_set_mode(MODE_STDBY);
    ESP_LOGE(TAG, "Tiempo agotado esperando TX_DONE");
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Punto de entrada de ESP-IDF para el firmware transmisor.
 * @details Configura el LED de actividad, inicializa sensores y radio y repite
 * lectura, serialización y envío. Con `TX_PERIOD_MS == 0` no añade una pausa
 * artificial entre paquetes. El LED permanece encendido durante cada TX y `SEQ`
 * solo se incrementa después de una transmisión confirmada.
 *
 * @note El MPU6050 es obligatorio para transmitir; el BME680 es opcional.
 */
void app_main(void)
{
    gpio_config_t led_config = {
        .pin_bit_mask = 1ULL << LED_PIN,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&led_config));
    gpio_set_level(LED_PIN, 0);

    ESP_ERROR_CHECK(i2c_sensors_init());
    ESP_ERROR_CHECK(radio_init());

    uint32_t sequence = 0;
    while (true) {
        char payload[LORA_MAX_PAYLOAD + 1];
        size_t payload_length = 0;
        esp_err_t err = build_payload(sequence, payload, sizeof(payload), &payload_length);
        if (err == ESP_OK) {
            gpio_set_level(LED_PIN, 1);
            err = send_packet((const uint8_t *)payload, payload_length);
            gpio_set_level(LED_PIN, 0);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "TX #%lu (%u bytes): %s", (unsigned long)sequence,
                         (unsigned)payload_length, payload);
                ++sequence;
            } else {
                ESP_LOGE(TAG, "Fallo la transmision: %s", esp_err_to_name(err));
            }
        } else {
            gpio_set_level(LED_PIN, 0);
            ESP_LOGW(TAG, "Se omite el paquete por error de sensor");
        }
#if TX_PERIOD_MS > 0
        vTaskDelay(pdMS_TO_TICKS(TX_PERIOD_MS));
#endif
    }
}




