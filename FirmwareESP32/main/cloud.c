/**
 * @file cloud.c
 * @brief Wi-Fi de estación y publicación periódica de dos canales.
 * @version 1.0.3
 *
 * Canal 1: aceleración, giro y temperaturas (ocho campos).
 * Canal 2: presión, humedad, gas, secuencia, CRC recibido/calculado,
 * contador de tramas corruptas y máscara de estado (ocho campos).
 * Cada canal se actualiza cada 16 s para respetar el límite gratuito de 15 s.
 */
#include "cloud.h"
#if defined(__has_include)
#if __has_include("cloud_secrets.h")
#include "cloud_secrets.h"
#else
#include "cloud_secrets.h.example"
#endif
#else
#include "cloud_secrets.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#define CLOUD_PERIOD_MS          16000U
#define WIFI_CONNECTED_BIT       BIT0
#define WIFI_MAX_RETRIES         12U
#define CLOUD_HTTP_TIMEOUT_MS    12000

static const char *TAG = "NODE2_CLOUD";
static EventGroupHandle_t wifi_events;
static uint8_t wifi_retries;
static portMUX_TYPE sample_mux = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
    float ax_g, ay_g, az_g;
    float gx_dps, gy_dps, gz_dps;
    float mpu_temp_c, bme_temp_c;
    uint32_t pressure_pa;
    float humidity_pct;
    uint32_t gas_ohms;
    uint8_t sequence;
    uint8_t crc_received;
    uint8_t crc_calculated;
    uint8_t mpu_valid;
    uint8_t bme_valid;
    uint8_t last_frame_valid;
    uint8_t crc_error_latched;
    uint32_t corrupt_frames;
} cloud_sample_t;

static cloud_sample_t latest_sample;
static char http_response[24];

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        (void)esp_wifi_connect();
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifi_events != NULL)
            xEventGroupClearBits(wifi_events, WIFI_CONNECTED_BIT);
        if (wifi_retries < WIFI_MAX_RETRIES)
            ++wifi_retries;
        else
            wifi_retries = 0U;
        (void)esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        wifi_retries = 0U;
        if (wifi_events != NULL)
            xEventGroupSetBits(wifi_events, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Wi-Fi conectado; ThingSpeak habilitado");
    }
}

void cloud_update_sample(const telemetry_sample_t *sample,
                         bool frame_valid,
                         uint32_t corrupt_frame_count)
{
    taskENTER_CRITICAL(&sample_mux);

    if (sample != NULL) {
        latest_sample.sequence = sample->sequence;
        bool crc_mismatch = sample->crc_received != sample->crc_calculated;
        if (latest_sample.crc_error_latched == 0U || crc_mismatch) {
            latest_sample.crc_received = sample->crc_received;
            latest_sample.crc_calculated = sample->crc_calculated;
        }
        if (crc_mismatch)
            latest_sample.crc_error_latched = 1U;

        if (frame_valid) {
            latest_sample.ax_g = (float)sample->accel_x_q14 / 16384.0f;
            latest_sample.ay_g = (float)sample->accel_y_q14 / 16384.0f;
            latest_sample.az_g = (float)sample->accel_z_q14 / 16384.0f;
            latest_sample.gx_dps = (float)sample->gyro_x_q7 / 128.0f;
            latest_sample.gy_dps = (float)sample->gyro_y_q7 / 128.0f;
            latest_sample.gz_dps = (float)sample->gyro_z_q7 / 128.0f;
            latest_sample.mpu_temp_c = (float)sample->mpu_temp_q8 / 256.0f;
            latest_sample.bme_temp_c = (float)sample->bme_temp_q8 / 256.0f;
            latest_sample.pressure_pa = sample->pressure_pa;
            latest_sample.humidity_pct = (float)sample->humidity_q8 / 256.0f;
            latest_sample.gas_ohms = sample->gas_ohms;
            latest_sample.mpu_valid =
                (sample->flags & TELEMETRY_FLAG_MPU_VALID) ? 1U : 0U;
            latest_sample.bme_valid =
                (sample->flags & TELEMETRY_FLAG_BME_VALID) ? 1U : 0U;
        }
    }

    latest_sample.last_frame_valid = frame_valid ? 1U : 0U;
    latest_sample.corrupt_frames = corrupt_frame_count;
    taskEXIT_CRITICAL(&sample_mux);
}

static esp_err_t http_event_handler(esp_http_client_event_t *event)
{
    if (event->event_id == HTTP_EVENT_ON_DATA &&
        event->data != NULL && event->data_len > 0) {
        size_t used = strlen(http_response);
        size_t available = sizeof(http_response) - used - 1U;
        size_t copy_len = (size_t)event->data_len;
        if (copy_len > available) copy_len = available;
        if (copy_len > 0U) {
            memcpy(http_response + used, event->data, copy_len);
            http_response[used + copy_len] = '\0';
        }
    }
    return ESP_OK;
}

static esp_err_t publish_fields(const char *write_key, const char *fields,
                                const char *channel_name)
{
    char url[700];
    int n = snprintf(url, sizeof(url),
                     "https://api.thingspeak.com/update?api_key=%s&%s",
                     write_key, fields);
    if (n <= 0 || (size_t)n >= sizeof(url))
        return ESP_ERR_INVALID_SIZE;

    http_response[0] = '\0';
    const esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = CLOUD_HTTP_TIMEOUT_MS,
        .event_handler = http_event_handler,
        .crt_bundle_attach = esp_crt_bundle_attach
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL)
        return ESP_FAIL;

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    long entry = strtol(http_response, NULL, 10);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s: HTTP falló: %s",
                 channel_name, esp_err_to_name(err));
        return err;
    }
    if (status != 200 || entry <= 0) {
        ESP_LOGW(TAG, "%s: ThingSpeak rechazó actualización (HTTP %d, cuerpo=%s)",
                 channel_name, status, http_response);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "%s actualizado (entrada %ld)", channel_name, entry);
    return ESP_OK;
}

static void cloud_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        EventBits_t bits = xEventGroupWaitBits(
            wifi_events, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
        if ((bits & WIFI_CONNECTED_BIT) == 0U)
            continue;

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CLOUD_PERIOD_MS));

        if ((xEventGroupGetBits(wifi_events) & WIFI_CONNECTED_BIT) == 0U)
            continue;

        cloud_sample_t sample;
        taskENTER_CRITICAL(&sample_mux);
        sample = latest_sample;
        taskEXIT_CRITICAL(&sample_mux);

        char fields1[320];
        char fields2[300];
        (void)snprintf(fields1, sizeof(fields1),
            "field1=%.5f&field2=%.5f&field3=%.5f&field4=%.5f&"
            "field5=%.5f&field6=%.5f&field7=%.2f&field8=%.2f",
            (double)sample.ax_g, (double)sample.ay_g, (double)sample.az_g,
            (double)sample.gx_dps, (double)sample.gy_dps,
            (double)sample.gz_dps, (double)sample.mpu_temp_c,
            (double)sample.bme_temp_c);
        (void)snprintf(fields2, sizeof(fields2),
            "field1=%lu&field2=%.3f&field3=%lu&field4=%u&"
            "field5=%u&field6=%u&field7=%lu&field8=%u",
            (unsigned long)sample.pressure_pa,
            (double)sample.humidity_pct,
            (unsigned long)sample.gas_ohms,
            (unsigned)sample.sequence,
            (unsigned)sample.crc_received,
            (unsigned)sample.crc_calculated,
            (unsigned long)sample.corrupt_frames,
            (unsigned)(sample.last_frame_valid | (sample.mpu_valid << 1) | (sample.bme_valid << 2) | (sample.crc_error_latched << 3)));

        (void)publish_fields(TS_CHANNEL1_WRITE_KEY, fields1, "Canal 1");
        if ((xEventGroupGetBits(wifi_events) & WIFI_CONNECTED_BIT) != 0U)
            (void)publish_fields(TS_CHANNEL2_WRITE_KEY, fields2, "Canal 2");
    }
}

esp_err_t cloud_start(void)
{
    if (CLOUD_WIFI_SSID[0] == '\0' ||
        TS_CHANNEL1_WRITE_KEY[0] == '\0' ||
        TS_CHANNEL2_WRITE_KEY[0] == '\0') {
        ESP_LOGW(TAG,
                 "Sin credenciales: completa main/cloud_secrets.h para activar ThingSpeak");
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "No se pudo limpiar NVS");
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "No se pudo inicializar NVS");

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        return err;

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        return err;

    if (esp_netif_create_default_wifi_sta() == NULL)
        return ESP_FAIL;

    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&wifi_config), TAG,
                        "No se pudo inicializar Wi-Fi");
    wifi_events = xEventGroupCreate();
    if (wifi_events == NULL)
        return ESP_ERR_NO_MEM;

    ESP_RETURN_ON_ERROR(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL),
        TAG, "No se pudo registrar eventos Wi-Fi");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL),
        TAG, "No se pudo registrar eventos IP");

    wifi_config_t station_config = {0};
    snprintf((char *)station_config.sta.ssid,
             sizeof(station_config.sta.ssid), "%s", CLOUD_WIFI_SSID);
    snprintf((char *)station_config.sta.password,
             sizeof(station_config.sta.password), "%s",
             CLOUD_WIFI_PASSWORD);
    station_config.sta.threshold.authmode =
        CLOUD_WIFI_PASSWORD[0] == '\0' ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG,
                        "No se pudo seleccionar modo estación");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &station_config),
                        TAG, "No se pudo configurar Wi-Fi");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "No se pudo iniciar Wi-Fi");

    if (xTaskCreate(cloud_task, "cloud_publish", 6144, NULL, 4, NULL) != pdPASS)
        return ESP_ERR_NO_MEM;

    ESP_LOGI(TAG, "Publicación ThingSpeak activa; periodo=%u ms",
             (unsigned)CLOUD_PERIOD_MS);
    return ESP_OK;
}
