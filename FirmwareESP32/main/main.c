/**
 * @file main.c
 * @brief Gateway ESP32 entre RS485 del STM32 y radio LoRa al nodo 3.
 * @version 1.0.1
 *
 * Recibe y valida tramas por UART2, devuelve el ACK RS485 inmediatamente y
 * retransmite por SX1278 desde una tarea FreeRTOS independiente. Conserva la
 * muestra mas reciente pendiente para evitar retrasos si LoRa esta ocupado.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_master.h"

#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"

/* UART2 del ESP32 hacia el MAX485 */
#define RS485_UART                 UART_NUM_2
#define RS485_TX_GPIO              GPIO_NUM_17
#define RS485_RX_GPIO              GPIO_NUM_16
#define RS485_DE_GPIO              GPIO_NUM_33
#define RS485_RE_GPIO              GPIO_NUM_25

/* Indicadores del nodo 2 */
#define LED_LORA_TX_GPIO           GPIO_NUM_27
#define LED_LORA_RX_GPIO           GPIO_NUM_32
#define LED_ONBOARD_GPIO           GPIO_NUM_2

/* SX1278: SPI2 */
#define LORA_SPI_HOST              SPI2_HOST
#define LORA_SCK_GPIO              GPIO_NUM_18
#define LORA_MISO_GPIO             GPIO_NUM_19
#define LORA_MOSI_GPIO             GPIO_NUM_23
#define LORA_NSS_GPIO              GPIO_NUM_5
#define LORA_RESET_GPIO            GPIO_NUM_4
#define LORA_DIO0_GPIO             GPIO_NUM_26

/* Deben coincidir con los del nodo 3 y con la banda del módulo */
#define LORA_FREQUENCY_HZ          433000000UL
#define LORA_SPI_CLOCK_HZ          8000000
#define LORA_TX_TIMEOUT_MS         2500U
#define LORA_ACK_TIMEOUT_MS        2500U

#define UART_BAUD_RATE             115200
#define UART_RX_BUFFER_SIZE        1024

#define MAX_FRAME_LENGTH           180
#define MAX_LORA_PAYLOAD           200
#define MAX_ACK_LENGTH             40

/* Solo se conserva la trama pendiente más reciente para LoRa. */
#define LORA_QUEUE_LENGTH          1

static const char *TAG = "NODE2";
static spi_device_handle_t lora_spi;
static QueueHandle_t lora_queue;

/* Contadores visibles en el log serial */
static volatile uint32_t rs485_valid_frames = 0U;
static volatile uint32_t rs485_ack_sent = 0U;
static volatile uint32_t rs485_ack_errors = 0U;
static volatile uint32_t lora_frames_replaced = 0U;
static volatile uint32_t lora_ack_received = 0U;
static volatile uint32_t lora_ack_timeouts = 0U;
static volatile uint32_t lora_tx_errors = 0U;

/* Registros SX1278 */
#define REG_FIFO                  0x00
#define REG_OP_MODE               0x01
#define REG_FRF_MSB               0x06
#define REG_FRF_MID               0x07
#define REG_FRF_LSB               0x08
#define REG_PA_CONFIG             0x09
#define REG_LNA                   0x0C
#define REG_FIFO_ADDR_PTR         0x0D
#define REG_FIFO_TX_BASE_ADDR     0x0E
#define REG_FIFO_RX_BASE_ADDR     0x0F
#define REG_FIFO_RX_CURRENT       0x10
#define REG_IRQ_FLAGS             0x12
#define REG_RX_NB_BYTES           0x13
#define REG_MODEM_CONFIG_1        0x1D
#define REG_MODEM_CONFIG_2        0x1E
#define REG_PREAMBLE_MSB          0x20
#define REG_PREAMBLE_LSB          0x21
#define REG_PAYLOAD_LENGTH        0x22
#define REG_MODEM_CONFIG_3        0x26
#define REG_SYNC_WORD             0x39
#define REG_DIO_MAPPING_1         0x40
#define REG_VERSION               0x42

#define MODE_LONG_RANGE           0x80
#define MODE_SLEEP                0x00
#define MODE_STDBY                0x01
#define MODE_TX                   0x03
#define MODE_RX_CONTINUOUS        0x05

#define IRQ_RX_TIMEOUT            0x80
#define IRQ_RX_DONE               0x40
#define IRQ_PAYLOAD_CRC_ERROR     0x20
#define IRQ_TX_DONE               0x08

static void rs485_task(void *arg);
static void lora_task(void *arg);

/* ---------------- MAX485 ---------------- */

/** Deja el transceptor RS485 en escucha (DE bajo y /RE bajo). */
static void rs485_receive_mode(void)
{
    /* DE bajo apaga TX; /RE bajo habilita RX. */
    gpio_set_level(RS485_DE_GPIO, 0);
    gpio_set_level(RS485_RE_GPIO, 0);
}

/** Habilita transmision RS485 (DE alto y /RE alto). */
static void rs485_transmit_mode(void)
{
    /* DE alto habilita TX; /RE alto deshabilita RX. */
    gpio_set_level(RS485_RE_GPIO, 1);
    gpio_set_level(RS485_DE_GPIO, 1);
}

/** Envia una trama UART en modo TX y vuelve a RX al terminar. */
static esp_err_t rs485_send(const char *data, size_t length)
{
    rs485_transmit_mode();

    /* Un tick asegura tiempo de cambio de dirección del transceptor. */
    vTaskDelay(1);

    int written = uart_write_bytes(RS485_UART, data, length);
    if (written < 0 || (size_t)written != length) {
        rs485_receive_mode();
        return ESP_FAIL;
    }

    esp_err_t err = uart_wait_tx_done(RS485_UART, pdMS_TO_TICKS(100));
    rs485_receive_mode();

    return err;
}

/* ---------------- SPI / SX1278 ---------------- */

/** Escribe un registro del SX1278 por SPI. */
static esp_err_t lora_write_register(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {(uint8_t)(reg | 0x80U), value};

    spi_transaction_t transaction = {
        .length = 16,
        .tx_buffer = tx
    };

    return spi_device_polling_transmit(lora_spi, &transaction);
}

/** Lee un registro del SX1278 por SPI. */
static esp_err_t lora_read_register(uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = {(uint8_t)(reg & 0x7FU), 0x00U};
    uint8_t rx[2] = {0};

    spi_transaction_t transaction = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx
    };

    esp_err_t err = spi_device_polling_transmit(lora_spi, &transaction);
    if (err == ESP_OK)
        *value = rx[1];

    return err;
}

/** Escribe una secuencia de bytes en registros/FIFO del SX1278. */
static esp_err_t lora_write_burst(uint8_t reg,
                                  const uint8_t *data,
                                  size_t length)
{
    uint8_t tx[MAX_LORA_PAYLOAD + 1];

    if (length > MAX_LORA_PAYLOAD)
        return ESP_ERR_INVALID_SIZE;

    tx[0] = (uint8_t)(reg | 0x80U);
    memcpy(&tx[1], data, length);

    spi_transaction_t transaction = {
        .length = (length + 1U) * 8U,
        .tx_buffer = tx
    };

    return spi_device_polling_transmit(lora_spi, &transaction);
}

/** Lee una secuencia de bytes desde registros/FIFO del SX1278. */
static esp_err_t lora_read_burst(uint8_t reg,
                                 uint8_t *data,
                                 size_t length)
{
    uint8_t tx[MAX_LORA_PAYLOAD + 1] = {0};
    uint8_t rx[MAX_LORA_PAYLOAD + 1] = {0};

    if (length > MAX_LORA_PAYLOAD)
        return ESP_ERR_INVALID_SIZE;

    tx[0] = (uint8_t)(reg & 0x7FU);

    spi_transaction_t transaction = {
        .length = (length + 1U) * 8U,
        .tx_buffer = tx,
        .rx_buffer = rx
    };

    esp_err_t err = spi_device_polling_transmit(lora_spi, &transaction);
    if (err == ESP_OK)
        memcpy(data, &rx[1], length);

    return err;
}

/** Selecciona modo LoRa y modo operativo del transceptor. */
static esp_err_t lora_set_mode(uint8_t mode)
{
    return lora_write_register(REG_OP_MODE,
                               (uint8_t)(MODE_LONG_RANGE | mode));
}

/** Reinicia el SX1278 y programa frecuencia, modem, potencia y sync word. */
static esp_err_t lora_init(void)
{
    gpio_set_level(LORA_RESET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_RESET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t version = 0U;
    ESP_RETURN_ON_ERROR(lora_read_register(REG_VERSION, &version),
                        TAG, "No se pudo leer VERSION del SX1278");

    ESP_LOGI(TAG, "SX1278 VERSION=0x%02X", version);

    if (version != 0x12U) {
        ESP_LOGE(TAG, "Versión inesperada del SX1278");
        return ESP_FAIL;
    }

    ESP_RETURN_ON_ERROR(
        lora_write_register(REG_OP_MODE, MODE_LONG_RANGE | MODE_SLEEP),
        TAG, "No se pudo activar LoRa sleep");

    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_RETURN_ON_ERROR(lora_set_mode(MODE_STDBY), TAG,
                        "No se pudo poner LoRa en standby");

    /* FRF = frecuencia * 2^19 / 32 MHz */
    uint64_t frf =
        ((uint64_t)LORA_FREQUENCY_HZ << 19) / 32000000ULL;

    ESP_RETURN_ON_ERROR(
        lora_write_register(REG_FRF_MSB, (uint8_t)(frf >> 16)),
        TAG, "Error escribiendo FRF MSB");
    ESP_RETURN_ON_ERROR(
        lora_write_register(REG_FRF_MID, (uint8_t)(frf >> 8)),
        TAG, "Error escribiendo FRF MID");
    ESP_RETURN_ON_ERROR(
        lora_write_register(REG_FRF_LSB, (uint8_t)frf),
        TAG, "Error escribiendo FRF LSB");

    ESP_RETURN_ON_ERROR(
        lora_write_register(REG_FIFO_TX_BASE_ADDR, 0x00),
        TAG, "Error FIFO TX base");
    ESP_RETURN_ON_ERROR(
        lora_write_register(REG_FIFO_RX_BASE_ADDR, 0x00),
        TAG, "Error FIFO RX base");

    /* SF7, BW125 kHz, CR 4/5, explicit header, CRC y AGC habilitados. */
    ESP_RETURN_ON_ERROR(
        lora_write_register(REG_MODEM_CONFIG_1, 0x72),
        TAG, "Error modem config 1");
    ESP_RETURN_ON_ERROR(
        lora_write_register(REG_MODEM_CONFIG_2, 0x74),
        TAG, "Error modem config 2");
    ESP_RETURN_ON_ERROR(
        lora_write_register(REG_MODEM_CONFIG_3, 0x04),
        TAG, "Error modem config 3");

    ESP_RETURN_ON_ERROR(
        lora_write_register(REG_PREAMBLE_MSB, 0x00),
        TAG, "Error preámbulo MSB");
    ESP_RETURN_ON_ERROR(
        lora_write_register(REG_PREAMBLE_LSB, 0x08),
        TAG, "Error preámbulo LSB");

    ESP_RETURN_ON_ERROR(
        lora_write_register(REG_SYNC_WORD, 0x12),
        TAG, "Error sync word");

    ESP_RETURN_ON_ERROR(
        lora_write_register(REG_PA_CONFIG, 0x8C),
        TAG, "Error PA config");
    ESP_RETURN_ON_ERROR(
        lora_write_register(REG_LNA, 0x23),
        TAG, "Error LNA");

    /* DIO0: RxDone en RX y TxDone en TX. */
    ESP_RETURN_ON_ERROR(
        lora_write_register(REG_DIO_MAPPING_1, 0x00),
        TAG, "Error DIO mapping");

    ESP_RETURN_ON_ERROR(
        lora_write_register(REG_IRQ_FLAGS, 0xFF),
        TAG, "Error limpiando IRQ");

    return lora_set_mode(MODE_STDBY);
}

/** Transmite un paquete y sondea TX_DONE con espera FreeRTOS acotada. */
static esp_err_t lora_send_packet(const uint8_t *data, size_t length)
{
    esp_err_t err;

    if (length == 0U || length > MAX_LORA_PAYLOAD)
        return ESP_ERR_INVALID_SIZE;

    err = lora_set_mode(MODE_STDBY);
    if (err != ESP_OK)
        goto fail;

    err = lora_write_register(REG_IRQ_FLAGS, 0xFF);
    if (err != ESP_OK)
        goto fail;

    err = lora_write_register(REG_FIFO_ADDR_PTR, 0x00);
    if (err != ESP_OK)
        goto fail;

    err = lora_write_burst(REG_FIFO, data, length);
    if (err != ESP_OK)
        goto fail;

    err = lora_write_register(REG_PAYLOAD_LENGTH, (uint8_t)length);
    if (err != ESP_OK)
        goto fail;

    gpio_set_level(LED_LORA_TX_GPIO, 1);

    err = lora_set_mode(MODE_TX);
    if (err != ESP_OK)
        goto fail;

    TickType_t start = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(LORA_TX_TIMEOUT_MS);

    while ((xTaskGetTickCount() - start) < timeout) {
        uint8_t irq = 0U;

        err = lora_read_register(REG_IRQ_FLAGS, &irq);
        if (err != ESP_OK)
            goto fail;

        if ((irq & IRQ_TX_DONE) != 0U) {
            lora_write_register(REG_IRQ_FLAGS, IRQ_TX_DONE);
            lora_set_mode(MODE_STDBY);
            gpio_set_level(LED_LORA_TX_GPIO, 0);
            return ESP_OK;
        }

        /* Un tick completo evita que esta tarea monopolice el CPU. */
        vTaskDelay(1);
    }

    err = ESP_ERR_TIMEOUT;

fail:
    lora_set_mode(MODE_STDBY);
    gpio_set_level(LED_LORA_TX_GPIO, 0);
    return err;
}

/** Espera RX_DONE hasta timeout, valida CRC y copia el payload de la FIFO. */
static esp_err_t lora_receive_packet(uint8_t *buffer,
                                     size_t buffer_size,
                                     size_t *received_length,
                                     uint32_t timeout_ms)
{
    *received_length = 0U;

    ESP_RETURN_ON_ERROR(
        lora_write_register(REG_IRQ_FLAGS, 0xFF),
        TAG, "Error limpiando IRQ RX");

    ESP_RETURN_ON_ERROR(
        lora_set_mode(MODE_RX_CONTINUOUS),
        TAG, "No se pudo iniciar RX LoRa");

    TickType_t start = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - start) < timeout) {
        uint8_t irq = 0U;

        ESP_RETURN_ON_ERROR(
            lora_read_register(REG_IRQ_FLAGS, &irq),
            TAG, "Error leyendo IRQ RX");

        if ((irq & IRQ_RX_TIMEOUT) != 0U)
            lora_write_register(REG_IRQ_FLAGS, IRQ_RX_TIMEOUT);

        if ((irq & IRQ_PAYLOAD_CRC_ERROR) != 0U) {
            lora_write_register(REG_IRQ_FLAGS, IRQ_PAYLOAD_CRC_ERROR);
            ESP_LOGW(TAG, "Paquete LoRa con CRC incorrecto");
        }

        if ((irq & IRQ_RX_DONE) != 0U) {
            if ((irq & IRQ_PAYLOAD_CRC_ERROR) != 0U) {
                lora_write_register(REG_IRQ_FLAGS, 0xFF);
                continue;
            }

            uint8_t length = 0U;
            uint8_t fifo_addr = 0U;

            ESP_RETURN_ON_ERROR(
                lora_read_register(REG_RX_NB_BYTES, &length),
                TAG, "No se pudo leer tamaño del paquete");
            ESP_RETURN_ON_ERROR(
                lora_read_register(REG_FIFO_RX_CURRENT, &fifo_addr),
                TAG, "No se pudo leer dirección FIFO RX");

            if (length == 0U || length >= buffer_size) {
                lora_write_register(REG_IRQ_FLAGS, 0xFF);
                lora_set_mode(MODE_STDBY);
                return ESP_ERR_INVALID_SIZE;
            }

            ESP_RETURN_ON_ERROR(
                lora_write_register(REG_FIFO_ADDR_PTR, fifo_addr),
                TAG, "No se pudo fijar FIFO RX pointer");
            ESP_RETURN_ON_ERROR(
                lora_read_burst(REG_FIFO, buffer, length),
                TAG, "No se pudo leer FIFO RX");

            buffer[length] = '\0';
            *received_length = length;

            lora_write_register(REG_IRQ_FLAGS, 0xFF);
            lora_set_mode(MODE_STDBY);
            return ESP_OK;
        }

        /* Sustituye el retardo de 2 ms, que podía convertirse en 0 ticks. */
        vTaskDelay(1);
    }

    lora_set_mode(MODE_STDBY);
    return ESP_ERR_TIMEOUT;
}

/* ---------------- Validación de protocolo ---------------- */

/** Valida delimitadores, XOR y secuencia de la trama enviada por STM32. */
static bool validate_stm32_frame(const char *line,
                                 uint8_t *sequence)
{
    if (line == NULL || line[0] != '$')
        return false;

    const char *star = strchr(line, '*');
    if (star == NULL || star <= line + 1)
        return false;

    size_t body_length = (size_t)(star - (line + 1));
    if (body_length == 0U || body_length >= MAX_FRAME_LENGTH)
        return false;

    unsigned int received_checksum = 0U;
    if (sscanf(star + 1, "%2x", &received_checksum) != 1)
        return false;

    uint8_t checksum = 0U;
    for (size_t i = 0U; i < body_length; i++)
        checksum ^= (uint8_t)line[1U + i];

    if (checksum != (uint8_t)received_checksum)
        return false;

    unsigned int seq = 0U;
    if (sscanf(line + 1, "D,%u", &seq) != 1 || seq > 255U)
        return false;

    *sequence = (uint8_t)seq;
    return true;
}

/** Acepta ACK de estación solo si la secuencia coincide y el estado es OK. */
static bool validate_lora_ack(const char *ack, uint8_t expected_sequence)
{
    unsigned int sequence = 0U;
    char status[8] = {0};

    if (sscanf(ack, "$A,%u,%7[^,\r\n]", &sequence, status) == 2 ||
        sscanf(ack, "ACK,%u,%7[^,\r\n]", &sequence, status) == 2) {
        return sequence == expected_sequence &&
               strcmp(status, "OK") == 0;
    }

    return false;
}

/* ---------------- Tarea RS485: ACK inmediato ---------------- */

/** Valida una trama, responde RS485 sin esperar LoRa y encola para retransmision. */
static void handle_rs485_line(const char *line)
{
    uint8_t sequence = 0U;

    if (!validate_stm32_frame(line, &sequence)) {
        ESP_LOGW(TAG, "Trama RS485 inválida: %s", line);
        return;
    }

    rs485_valid_frames++;
    ESP_LOGI(TAG, "Trama STM32 válida, secuencia=%u", sequence);

    /*
     * El ACK sale de inmediato, antes de iniciar o esperar una operación LoRa.
     */
    char ack[24];
    int ack_length = snprintf(ack,
                              sizeof(ack),
                              "$A,%u,OK\r\n",
                              (unsigned int)sequence);

    if (ack_length <= 0 || (size_t)ack_length >= sizeof(ack)) {
        ESP_LOGE(TAG, "No se pudo construir ACK RS485");
        return;
    }

    if (rs485_send(ack, (size_t)ack_length) == ESP_OK) {
        rs485_ack_sent++;
    } else {
        rs485_ack_errors++;
        ESP_LOGE(TAG, "Error enviando ACK RS485");
    }

    /*
     * Si LoRa está ocupado y ya hay una trama pendiente, se reemplaza
     * por la muestra más reciente.
     */
    if (uxQueueMessagesWaiting(lora_queue) > 0U)
        lora_frames_replaced++;

    if (xQueueOverwrite(lora_queue, line) != pdPASS)
        ESP_LOGE(TAG, "No se pudo encolar trama LoRa");

    gpio_set_level(LED_ONBOARD_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(LED_ONBOARD_GPIO, 0);
}

/** Tarea FreeRTOS que delimita lineas UART por LF y procesa tramas RS485. */
static void rs485_task(void *arg)
{
    (void)arg;

    char line[MAX_FRAME_LENGTH];
    size_t used = 0U;
    uint8_t byte = 0U;

    while (1) {
        int count = uart_read_bytes(RS485_UART,
                                    &byte,
                                    1U,
                                    pdMS_TO_TICKS(100));

        if (count <= 0)
            continue;

        if (byte == '\r')
            continue;

        if (byte == '\n') {
            line[used] = '\0';

            if (used > 0U)
                handle_rs485_line(line);

            used = 0U;
            continue;
        }

        if (used < sizeof(line) - 1U) {
            line[used++] = (char)byte;
        } else {
            ESP_LOGW(TAG, "Trama RS485 excedió el buffer");
            used = 0U;
        }
    }
}

/* ---------------- Tarea LoRa: envío y ACK ---------------- */

/** Tarea FreeRTOS que envia muestras LoRa y espera el ACK de la estacion. */
static void lora_task(void *arg)
{
    (void)arg;

    char frame[MAX_FRAME_LENGTH];
    char payload[MAX_LORA_PAYLOAD + 1];
    uint8_t ack[MAX_ACK_LENGTH];
    size_t ack_length = 0U;

    while (1) {
        if (xQueueReceive(lora_queue,
                          frame,
                          portMAX_DELAY) != pdPASS) {
            continue;
        }

        int payload_length = snprintf(payload,
                                      sizeof(payload),
                                      "%s\r\n",
                                      frame);

        if (payload_length <= 0 ||
            (size_t)payload_length >= sizeof(payload)) {
            ESP_LOGE(TAG, "Trama demasiado larga para LoRa");
            lora_tx_errors++;
            continue;
        }

        unsigned int sent_sequence = 0U;
        if (sscanf(frame, "$D,%u", &sent_sequence) != 1 ||
            sent_sequence > 255U) {
            ESP_LOGE(TAG, "No se pudo leer la secuencia de la trama");
            continue;
        }

        ESP_LOGI(TAG, "Enviando secuencia por LoRa: %s", frame);

        esp_err_t err = lora_send_packet(
            (const uint8_t *)payload,
            (size_t)payload_length);

        if (err != ESP_OK) {
            lora_tx_errors++;
            ESP_LOGE(TAG, "Fallo TX LoRa: %s", esp_err_to_name(err));
            continue;
        }

        ack_length = 0U;
        err = lora_receive_packet(ack,
                                  sizeof(ack),
                                  &ack_length,
                                  LORA_ACK_TIMEOUT_MS);

        if (err == ESP_ERR_TIMEOUT) {
            lora_ack_timeouts++;
            ESP_LOGW(TAG, "Timeout esperando ACK LoRa");
            continue;
        }

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error recibiendo ACK LoRa: %s",
                     esp_err_to_name(err));
            continue;
        }

        ESP_LOGI(TAG, "Respuesta LoRa recibida: %s", (char *)ack);

        if (validate_lora_ack((char *)ack, (uint8_t)sent_sequence)) {
            lora_ack_received++;
            gpio_set_level(LED_LORA_RX_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_set_level(LED_LORA_RX_GPIO, 0);

            ESP_LOGI(TAG, "ACK LoRa válido para secuencia %u",
                     sent_sequence);
        } else {
            ESP_LOGW(TAG, "ACK LoRa inválido para secuencia %u",
                     sent_sequence);
        }
    }
}

/* ---------------- Inicialización de hardware ---------------- */

/** Configura GPIO de LEDs, direccion RS485, reset LoRa y DIO0. */
static void init_gpio(void)
{
    /*
     * GPIO5 (NSS) se deja fuera: el driver SPI lo controla.
     */
    gpio_config_t outputs = {
        .pin_bit_mask =
            (1ULL << RS485_DE_GPIO) |
            (1ULL << RS485_RE_GPIO) |
            (1ULL << LED_LORA_TX_GPIO) |
            (1ULL << LED_LORA_RX_GPIO) |
            (1ULL << LED_ONBOARD_GPIO) |
            (1ULL << LORA_RESET_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&outputs));

    gpio_config_t inputs = {
        .pin_bit_mask = (1ULL << LORA_DIO0_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&inputs));

    gpio_set_level(RS485_DE_GPIO, 0);
    gpio_set_level(RS485_RE_GPIO, 0);
    gpio_set_level(LED_LORA_TX_GPIO, 0);
    gpio_set_level(LED_LORA_RX_GPIO, 0);
    gpio_set_level(LED_ONBOARD_GPIO, 0);
    gpio_set_level(LORA_RESET_GPIO, 1);
}

/** Inicializa UART2 a 115200 8N1 y los pines conectados al MAX485. */
static void init_uart_rs485(void)
{
    const uart_config_t config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_driver_install(RS485_UART,
                                        UART_RX_BUFFER_SIZE,
                                        0,
                                        0,
                                        NULL,
                                        0));
    ESP_ERROR_CHECK(uart_param_config(RS485_UART, &config));
    ESP_ERROR_CHECK(uart_set_pin(RS485_UART,
                                 RS485_TX_GPIO,
                                 RS485_RX_GPIO,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_flush_input(RS485_UART));

    rs485_receive_mode();
}

/** Inicializa SPI2 y registra el SX1278 con CS/NSS en GPIO5. */
static void init_lora_spi(void)
{
    spi_bus_config_t bus_config = {
        .mosi_io_num = LORA_MOSI_GPIO,
        .miso_io_num = LORA_MISO_GPIO,
        .sclk_io_num = LORA_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = MAX_LORA_PAYLOAD + 1
    };

    ESP_ERROR_CHECK(spi_bus_initialize(LORA_SPI_HOST,
                                       &bus_config,
                                       SPI_DMA_DISABLED));

    spi_device_interface_config_t device_config = {
        .clock_speed_hz = LORA_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = LORA_NSS_GPIO,
        .queue_size = 1
    };

    ESP_ERROR_CHECK(spi_bus_add_device(LORA_SPI_HOST,
                                       &device_config,
                                       &lora_spi));
}

/* ---------------- app_main ---------------- */

/** Punto de entrada: inicializa periféricos, cola y tareas del gateway. */
void app_main(void)
{
    ESP_LOGI(TAG, "Firmware nodo 2 v1.0.1");
    init_gpio();
    init_uart_rs485();
    init_lora_spi();

    ESP_ERROR_CHECK(lora_init());

    lora_queue = xQueueCreate(LORA_QUEUE_LENGTH,
                              MAX_FRAME_LENGTH);
    if (lora_queue == NULL) {
        ESP_LOGE(TAG, "No se pudo crear la cola LoRa");
        abort();
    }

    BaseType_t rs485_created =
        xTaskCreate(rs485_task,
                    "rs485_task",
                    4096,
                    NULL,
                    12,
                    NULL);

    BaseType_t lora_created =
        xTaskCreate(lora_task,
                    "lora_task",
                    6144,
                    NULL,
                    8,
                    NULL);

    if (rs485_created != pdPASS || lora_created != pdPASS) {
        ESP_LOGE(TAG, "No se pudieron crear las tareas");
        abort();
    }

    ESP_LOGI(TAG, "Nodo 2 iniciado: DE=GPIO33, RE=GPIO25");
}
