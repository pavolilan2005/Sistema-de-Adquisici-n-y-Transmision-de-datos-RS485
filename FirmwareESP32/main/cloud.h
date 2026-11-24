/**
 * @file cloud.h
 * @brief Publicación de telemetría validada en ThingSpeak.
 * @version 1.0.3
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "telemetry_protocol.h"

esp_err_t cloud_start(void);
void cloud_update_sample(const telemetry_sample_t *sample,
                         bool frame_valid,
                         uint32_t corrupt_frame_count);
