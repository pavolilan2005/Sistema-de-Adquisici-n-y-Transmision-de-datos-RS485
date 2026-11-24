/**
 * @file main.c
 * @brief Adquisición del MPU6050 y BME680 con envío binario CRC por RS-485.
 * @version 1.0.4
 *
 * El MPU6050 y el BME680 comparten I2C1 (PB6=SCL, PB7=SDA), de acuerdo con
 * la aprobación del profesor para este proyecto. USART1 transmite una trama
 * binaria fija al nodo 2. PC13 conmuta cada 200 ms como heartbeat.
 *
 * Trama de datos: A5 5A | versión | tipo | secuencia | flags | payload | CRC-8.
 * ACK:            A5 5A | versión | 80   | secuencia | CRC-8.
 */
#include "main.h"
#include "bme68x.h"
#include "telemetry_protocol.h"

#include <stdint.h>
#include <string.h>

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

/* Direcciones I2C de 7 bits: MPU AD0=GND; BME680 detecta 0x76 o 0x77. */
#define MPU6050_ADDR_8BIT         (0x68U << 1)
#define BME680_ADDR_76_8BIT       (0x76U << 1)
#define BME680_ADDR_77_8BIT       (0x77U << 1)

/* MAX485 del nodo 1 según el esquemático. */
#define RS485_DE_PORT             GPIOA
#define RS485_DE_PIN              GPIO_PIN_2
#define RS485_RE_PORT             GPIOA
#define RS485_RE_PIN              GPIO_PIN_1

/* Mapeo lógico tras corregir la inversión TX/RX observada en la PCB. */
#define LED_TX_PORT               GPIOB
#define LED_TX_PIN                GPIO_PIN_13
#define LED_RX_PORT               GPIOB
#define LED_RX_PIN                GPIO_PIN_12
#define LED_HEARTBEAT_PORT        GPIOC
#define LED_HEARTBEAT_PIN         GPIO_PIN_13

#define HEARTBEAT_INTERVAL_MS     200U
#define RX_LED_PULSE_MS           100U
#define SENSOR_INTERVAL_MS        1000U
#define ACK_TIMEOUT_MS            300U
#define I2C_TIMEOUT_MS            100U
#define BME_RETRY_INTERVAL_MS     5000U

#define MPU_REG_ACCEL_XOUT_H      0x3BU
#define MPU_REG_PWR_MGMT_1        0x6BU
#define MPU_REG_WHO_AM_I          0x75U

#define PROTOCOL_SYNC_0           TELEMETRY_SYNC_0
#define PROTOCOL_SYNC_1           TELEMETRY_SYNC_1
#define PROTOCOL_VERSION          TELEMETRY_VERSION
#define PROTOCOL_DATA_TYPE        TELEMETRY_TYPE_DATA
#define PROTOCOL_ACK_TYPE         TELEMETRY_TYPE_ACK
#define DATA_FRAME_SIZE           TELEMETRY_DATA_LENGTH
#define ACK_FRAME_SIZE            TELEMETRY_ACK_LENGTH
#define DATA_FLAGS_MPU_VALID      TELEMETRY_FLAG_MPU_VALID
#define DATA_FLAGS_BME_VALID      TELEMETRY_FLAG_BME_VALID

#define ACCEL_LSB_PER_G           16384.0f
#define GYRO_LSB_PER_DPS          131.0f
#define BME_HEATER_TEMP_C         300U
#define BME_HEATER_DUR_MS         100U

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

static HAL_StatusTypeDef I2C_Read(uint16_t dev, uint8_t reg,
                                  uint8_t *data, uint16_t len);
static HAL_StatusTypeDef I2C_Write(uint16_t dev, uint8_t reg,
                                   uint8_t value);
static HAL_StatusTypeDef MPU6050_Init(void);
static HAL_StatusTypeDef MPU6050_ReadAndProcess(void);
static HAL_StatusTypeDef BME680_Init(void);
static HAL_StatusTypeDef BME680_StartMeasurement(uint32_t now_ms);
static void BME680_CompleteMeasurement(void);
static HAL_StatusTypeDef RS485_SendFrame(void);
static uint8_t RS485_WaitAck(uint8_t seq);
static void RS485_SetReceiveMode(void);
static void RS485_SetTransmitMode(void);
static int8_t BME680_I2C_Read(uint8_t reg, uint8_t *data,
                              uint32_t len, void *intf_ptr);
static int8_t BME680_I2C_Write(uint8_t reg, const uint8_t *data,
                               uint32_t len, void *intf_ptr);
static void BME680_DelayUs(uint32_t period_us, void *intf_ptr);
static int16_t FloatToQ(float value, float scale);
static void PutU16BE(uint8_t *dst, uint16_t value);
static void PutI16BE(uint8_t *dst, int16_t value);
static void PutU24BE(uint8_t *dst, uint32_t value);
static void PutU32BE(uint8_t *dst, uint32_t value);

/* Variables visibles en Keil Watch para validar adquisición y comunicación. */
volatile uint32_t watch_heartbeat_count = 0U;
volatile uint32_t watch_sensor_count = 0U;
volatile uint32_t watch_ack_count = 0U;
volatile uint32_t watch_timeout_count = 0U;
volatile uint32_t watch_bad_ack_count = 0U;
volatile uint32_t watch_sensor_error_count = 0U;
volatile uint32_t watch_bme_retry_count = 0U;
volatile uint32_t watch_corrupt_frame_count = 0U;
volatile uint8_t watch_last_ack_ok = 0U;
volatile uint8_t watch_mpu_ok = 0U;
volatile uint8_t watch_bme_ok = 0U;
volatile uint8_t watch_bme_data_valid = 0U;
volatile uint8_t watch_bme_gas_valid = 0U;
volatile uint8_t watch_last_frame_flags = 0U;
volatile uint8_t watch_last_sequence = 0U;
volatile uint8_t watch_last_crc = 0U;
/* Ponerlo en 1 desde Watch para corromper una trama; vuelve a 0 automáticamente. */
volatile uint8_t watch_corrupt_next_frame = 0U;

volatile int16_t accel_x_raw = 0;
volatile int16_t accel_y_raw = 0;
volatile int16_t accel_z_raw = 0;
volatile int16_t gyro_x_raw = 0;
volatile int16_t gyro_y_raw = 0;
volatile int16_t gyro_z_raw = 0;
volatile int16_t mpu_temp_raw = 0;

/* Valores físicos derivados, cómodos para inspección y documentación. */
volatile float accel_x_g = 0.0f;
volatile float accel_y_g = 0.0f;
volatile float accel_z_g = 0.0f;
volatile float gyro_x_dps = 0.0f;
volatile float gyro_y_dps = 0.0f;
volatile float gyro_z_dps = 0.0f;
volatile float mpu_temperature_c = 0.0f;
volatile int32_t mpu_temp_centi_c = 0;
volatile int32_t bme_temperature_centi_c = 0;
volatile float bme_temperature_c = 0.0f;
volatile float bme_pressure_pa_float = 0.0f;
volatile float bme_humidity_pct = 0.0f;
volatile uint32_t bme_pressure_pa = 0U;
volatile uint32_t bme_humidity_milli_pct = 0U;
volatile uint32_t bme_gas_ohms = 0U;

volatile uint8_t sequence_number = 0U;
volatile uint32_t last_heartbeat_ms = 0U;
volatile uint32_t last_sensor_ms = 0U;
volatile uint8_t watch_tx_frame[DATA_FRAME_SIZE];
volatile uint8_t watch_rx_frame[ACK_FRAME_SIZE];

static struct bme68x_dev bme_device;
static struct bme68x_conf bme_config;
static struct bme68x_heatr_conf bme_heater;
static uint8_t bme_i2c_address_7bit;
static uint8_t bme_initialized;
static uint8_t bme_measurement_pending;
static uint32_t bme_ready_at_ms;
static uint32_t bme_last_retry_ms;
static uint8_t rx_led_active;
static volatile uint8_t heartbeat_enabled;
static uint32_t rx_led_started_ms;

/* ----------------------------- I2C -------------------------------------- */

static HAL_StatusTypeDef I2C_Read(uint16_t dev, uint8_t reg,
                                  uint8_t *data, uint16_t len)
{
  return HAL_I2C_Mem_Read(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT,
                          data, len, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef I2C_Write(uint16_t dev, uint8_t reg,
                                   uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT,
                           &value, 1U, I2C_TIMEOUT_MS);
}

/* ---------------------------- MPU6050 ----------------------------------- */

static HAL_StatusTypeDef MPU6050_Init(void)
{
  uint8_t id = 0U;

  if (I2C_Read(MPU6050_ADDR_8BIT, MPU_REG_WHO_AM_I, &id, 1U) != HAL_OK ||
      id != 0x68U)
    return HAL_ERROR;

  if (I2C_Write(MPU6050_ADDR_8BIT, MPU_REG_PWR_MGMT_1, 0x00U) != HAL_OK)
    return HAL_ERROR;

  HAL_Delay(10U);

  /* DLPF=3; rango de fábrica seleccionado: ±2 g y ±250 °/s. */
  if (I2C_Write(MPU6050_ADDR_8BIT, 0x1AU, 0x03U) != HAL_OK ||
      I2C_Write(MPU6050_ADDR_8BIT, 0x1BU, 0x00U) != HAL_OK ||
      I2C_Write(MPU6050_ADDR_8BIT, 0x1CU, 0x00U) != HAL_OK)
    return HAL_ERROR;

  return HAL_OK;
}

static HAL_StatusTypeDef MPU6050_ReadAndProcess(void)
{
  uint8_t data[14];

  if (I2C_Read(MPU6050_ADDR_8BIT, MPU_REG_ACCEL_XOUT_H,
               data, sizeof(data)) != HAL_OK)
    return HAL_ERROR;

  accel_x_raw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  accel_y_raw = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  accel_z_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
  mpu_temp_raw = (int16_t)(((uint16_t)data[6] << 8) | data[7]);
  gyro_x_raw = (int16_t)(((uint16_t)data[8] << 8) | data[9]);
  gyro_y_raw = (int16_t)(((uint16_t)data[10] << 8) | data[11]);
  gyro_z_raw = (int16_t)(((uint16_t)data[12] << 8) | data[13]);

  accel_x_g = (float)accel_x_raw / ACCEL_LSB_PER_G;
  accel_y_g = (float)accel_y_raw / ACCEL_LSB_PER_G;
  accel_z_g = (float)accel_z_raw / ACCEL_LSB_PER_G;
  gyro_x_dps = (float)gyro_x_raw / GYRO_LSB_PER_DPS;
  gyro_y_dps = (float)gyro_y_raw / GYRO_LSB_PER_DPS;
  gyro_z_dps = (float)gyro_z_raw / GYRO_LSB_PER_DPS;
  mpu_temperature_c = ((float)mpu_temp_raw / 340.0f) + 36.53f;
  mpu_temp_centi_c = (int32_t)(mpu_temperature_c * 100.0f);

  return HAL_OK;
}

/* ----------------------------- BME680 ----------------------------------- */

static int8_t BME680_I2C_Read(uint8_t reg, uint8_t *data,
                              uint32_t len, void *intf_ptr)
{
  uint8_t addr = *((uint8_t *)intf_ptr);
  if (len > 0xFFFFU)
    return (int8_t)-1;

  return (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr << 1), reg,
                           I2C_MEMADD_SIZE_8BIT, data, (uint16_t)len,
                           I2C_TIMEOUT_MS) == HAL_OK) ?
          BME68X_INTF_RET_SUCCESS : (int8_t)-1;
}

static int8_t BME680_I2C_Write(uint8_t reg, const uint8_t *data,
                               uint32_t len, void *intf_ptr)
{
  uint8_t addr = *((uint8_t *)intf_ptr);
  if (len > 0xFFFFU)
    return (int8_t)-1;

  return (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(addr << 1), reg,
                            I2C_MEMADD_SIZE_8BIT, (uint8_t *)data,
                            (uint16_t)len, I2C_TIMEOUT_MS) == HAL_OK) ?
          BME68X_INTF_RET_SUCCESS : (int8_t)-1;
}

static void BME680_DelayUs(uint32_t period_us, void *intf_ptr)
{
  (void)intf_ptr;
  /* El API requiere microsegundos; redondeamos hacia arriba al tick de HAL. */
  if (period_us != 0U)
    HAL_Delay((period_us + 999U) / 1000U);
}

static HAL_StatusTypeDef BME680_Init(void)
{
  if (HAL_I2C_IsDeviceReady(&hi2c1, BME680_ADDR_76_8BIT, 2U,
                            I2C_TIMEOUT_MS) == HAL_OK) {
    bme_i2c_address_7bit = 0x76U;
  } else if (HAL_I2C_IsDeviceReady(&hi2c1, BME680_ADDR_77_8BIT, 2U,
                                   I2C_TIMEOUT_MS) == HAL_OK) {
    bme_i2c_address_7bit = 0x77U;
  } else {
    return HAL_ERROR;
  }

  memset(&bme_device, 0, sizeof(bme_device));
  bme_device.intf = BME68X_I2C_INTF;
  bme_device.intf_ptr = &bme_i2c_address_7bit;
  bme_device.read = BME680_I2C_Read;
  bme_device.write = BME680_I2C_Write;
  bme_device.delay_us = BME680_DelayUs;
  bme_device.amb_temp = 25;

  if (bme68x_init(&bme_device) != BME68X_OK)
    return HAL_ERROR;

  memset(&bme_config, 0, sizeof(bme_config));
  bme_config.os_hum = BME68X_OS_2X;
  bme_config.os_temp = BME68X_OS_2X;
  bme_config.os_pres = BME68X_OS_2X;
  bme_config.filter = BME68X_FILTER_SIZE_3;
  bme_config.odr = BME68X_ODR_NONE;

  if (bme68x_set_conf(&bme_config, &bme_device) != BME68X_OK)
    return HAL_ERROR;

  memset(&bme_heater, 0, sizeof(bme_heater));
  bme_heater.enable = BME68X_ENABLE;
  bme_heater.heatr_temp = BME_HEATER_TEMP_C;
  bme_heater.heatr_dur = BME_HEATER_DUR_MS;

  if (bme68x_set_heatr_conf(BME68X_FORCED_MODE,
                            &bme_heater,
                            &bme_device) != BME68X_OK)
    return HAL_ERROR;

  bme_initialized = 1U;
  watch_bme_ok = 1U;
  return HAL_OK;
}

/* Arranca una medición forzada sin bloquear el heartbeat de PC13. */
static HAL_StatusTypeDef BME680_StartMeasurement(uint32_t now_ms)
{
  if (!bme_initialized ||
      bme68x_set_op_mode(BME68X_FORCED_MODE, &bme_device) != BME68X_OK)
    return HAL_ERROR;

  uint32_t measure_us =
      bme68x_get_meas_dur(BME68X_FORCED_MODE, &bme_config, &bme_device);
  uint32_t wait_ms = ((measure_us + 999U) / 1000U) +
                     (uint32_t)bme_heater.heatr_dur + 5U;

  bme_ready_at_ms = now_ms + wait_ms;
  bme_measurement_pending = 1U;
  return HAL_OK;
}

static void BME680_CompleteMeasurement(void)
{
  struct bme68x_data data;
  uint8_t count = 0U;
  memset(&data, 0, sizeof(data));

  bme_measurement_pending = 0U;

  if (bme68x_get_data(BME68X_FORCED_MODE, &data, &count,
                      &bme_device) != BME68X_OK ||
      count == 0U || (data.status & BME68X_NEW_DATA_MSK) == 0U) {
    watch_bme_data_valid = 0U;
    watch_sensor_error_count++;
    return;
  }

#ifdef BME68X_USE_FPU
  /* En modo FPU, el driver entrega °C, Pa y %RH como magnitudes flotantes. */
  bme_temperature_c = data.temperature;
  bme_temperature_centi_c = (int32_t)(data.temperature * 100.0f +
      ((data.temperature >= 0.0f) ? 0.5f : -0.5f));
  bme_pressure_pa_float = data.pressure;
  bme_pressure_pa = (uint32_t)(data.pressure + 0.5f);
  bme_humidity_pct = data.humidity;
  bme_humidity_milli_pct = (uint32_t)(data.humidity * 1000.0f + 0.5f);
#else
  /* En modo entero, temperatura es centi-°C y humedad está en %RH x1000. */
  bme_temperature_c = (float)data.temperature / 100.0f;
  bme_temperature_centi_c = data.temperature;
  bme_pressure_pa = data.pressure;
  bme_pressure_pa_float = (float)data.pressure;
  bme_humidity_milli_pct = data.humidity;
  bme_humidity_pct = (float)data.humidity / 1000.0f;
#endif
  watch_bme_data_valid = 1U;

  watch_bme_gas_valid =
      ((data.status & BME68X_GASM_VALID_MSK) != 0U &&
       (data.status & BME68X_HEAT_STAB_MSK) != 0U) ? 1U : 0U;
  bme_gas_ohms = watch_bme_gas_valid ?
      (uint32_t)(data.gas_resistance +
#ifdef BME68X_USE_FPU
                 0.5f
#else
                 0U
#endif
                 ) : 0U;
}

/* ------------------------ Protocolo binario ----------------------------- */

static int16_t FloatToQ(float value, float scale)
{
  float scaled = value * scale;

  if (scaled >= 32767.0f) return INT16_MAX;
  if (scaled <= -32768.0f) return INT16_MIN;

  scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
  return (int16_t)scaled;
}

static void PutU16BE(uint8_t *dst, uint16_t value)
{
  dst[0] = (uint8_t)(value >> 8);
  dst[1] = (uint8_t)value;
}

static void PutI16BE(uint8_t *dst, int16_t value)
{
  PutU16BE(dst, (uint16_t)value);
}

static void PutU24BE(uint8_t *dst, uint32_t value)
{
  if (value > 0xFFFFFFUL) value = 0xFFFFFFUL;
  dst[0] = (uint8_t)(value >> 16);
  dst[1] = (uint8_t)(value >> 8);
  dst[2] = (uint8_t)value;
}

static void PutU32BE(uint8_t *dst, uint32_t value)
{
  dst[0] = (uint8_t)(value >> 24);
  dst[1] = (uint8_t)(value >> 16);
  dst[2] = (uint8_t)(value >> 8);
  dst[3] = (uint8_t)value;
}

static HAL_StatusTypeDef RS485_SendFrame(void)
{
  uint8_t frame[DATA_FRAME_SIZE] = {0U};
  uint8_t flags = 0U;

  frame[0] = PROTOCOL_SYNC_0;
  frame[1] = PROTOCOL_SYNC_1;
  frame[2] = PROTOCOL_VERSION;
  frame[3] = PROTOCOL_DATA_TYPE;
  frame[4] = sequence_number;

  if (watch_mpu_ok != 0U)
    flags |= DATA_FLAGS_MPU_VALID;
  if (watch_bme_data_valid != 0U)
    flags |= DATA_FLAGS_BME_VALID;
  frame[5] = flags;

  /* Aceleración Q1.14 en g: MPU crudo equivale a raw/16384 g. */
  PutI16BE(&frame[6], FloatToQ(accel_x_g, 16384.0f));
  PutI16BE(&frame[8], FloatToQ(accel_y_g, 16384.0f));
  PutI16BE(&frame[10], FloatToQ(accel_z_g, 16384.0f));

  /* Giro Q9.7 en °/s, temperatura en Q8.8 °C. */
  PutI16BE(&frame[12], FloatToQ(gyro_x_dps, 128.0f));
  PutI16BE(&frame[14], FloatToQ(gyro_y_dps, 128.0f));
  PutI16BE(&frame[16], FloatToQ(gyro_z_dps, 128.0f));
  PutI16BE(&frame[18], FloatToQ(mpu_temperature_c, 256.0f));
  PutI16BE(&frame[20], FloatToQ(bme_temperature_c, 256.0f));

  /* Presión entera en Pa (24 bits), humedad Q8.8 %RH, gas entero en ohmios. */
  PutU24BE(&frame[22], bme_pressure_pa);
  uint32_t humidity_q = (uint32_t)
      ((bme_humidity_pct * 256.0f) + 0.5f);
  if (humidity_q > 0xFFFFU) humidity_q = 0xFFFFU;
  PutU16BE(&frame[25], (uint16_t)humidity_q);
  PutU32BE(&frame[27], bme_gas_ohms);

  frame[31] = telemetry_crc8_atm(&frame[2], 29U);
  watch_last_crc = frame[31];
  watch_last_frame_flags = flags;
  watch_last_sequence = sequence_number;

  /* La corrupción controlada se aplica después del CRC para comprobar el rechazo. */
  if (watch_corrupt_next_frame != 0U) {
    frame[8] ^= 0x01U;
    watch_corrupt_next_frame = 0U;
    watch_corrupt_frame_count++;
  }

  memcpy((void *)watch_tx_frame, frame, sizeof(frame));

  RS485_SetTransmitMode();
  HAL_Delay(1U);
  HAL_GPIO_WritePin(LED_TX_PORT, LED_TX_PIN, GPIO_PIN_SET);

  HAL_StatusTypeDef tx_status =
      HAL_UART_Transmit(&huart1, frame, sizeof(frame), 100U);

  while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {}

  HAL_GPIO_WritePin(LED_TX_PORT, LED_TX_PIN, GPIO_PIN_RESET);
  RS485_SetReceiveMode();
  return tx_status;
}

static uint8_t RS485_WaitAck(uint8_t seq)
{
  uint8_t frame[ACK_FRAME_SIZE] = {0U};
  uint8_t used = 0U;
  uint8_t byte = 0U;
  uint32_t start = HAL_GetTick();

  while ((HAL_GetTick() - start) < ACK_TIMEOUT_MS) {
    if (HAL_UART_Receive(&huart1, &byte, 1U, 10U) != HAL_OK)
      continue;

    if (used == 0U) {
      if (byte == PROTOCOL_SYNC_0)
        frame[used++] = byte;
      continue;
    }

    if (used == 1U) {
      if (byte == PROTOCOL_SYNC_1)
        frame[used++] = byte;
      else
        used = (byte == PROTOCOL_SYNC_0) ? 1U : 0U;
      if (used == 1U) frame[0] = byte;
      continue;
    }

    frame[used++] = byte;
    if (used < ACK_FRAME_SIZE)
      continue;

    memcpy((void *)watch_rx_frame, frame, sizeof(frame));
    uint8_t calculated = telemetry_crc8_atm(&frame[2], 3U);
    if (frame[2] == PROTOCOL_VERSION &&
        frame[3] == PROTOCOL_ACK_TYPE &&
        frame[4] == seq &&
        frame[5] == calculated)
      return 1U;

    watch_bad_ack_count++;
    used = 0U;
  }

  memset((void *)watch_rx_frame, 0, sizeof(watch_rx_frame));
  return 0U;
}

/* ------------------------------ RS485 ----------------------------------- */

static void RS485_SetReceiveMode(void)
{
  HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RS485_RE_PORT, RS485_RE_PIN, GPIO_PIN_RESET);
}

static void RS485_SetTransmitMode(void)
{
  /* MAX485: DE=1 y /RE=1 deshabilita recepción mientras transmite. */
  HAL_GPIO_WritePin(RS485_RE_PORT, RS485_RE_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET);
}

/* -------------------------------- main ---------------------------------- */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  RS485_SetReceiveMode();

  HAL_GPIO_WritePin(LED_TX_PORT, LED_TX_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_RX_PORT, LED_RX_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_HEARTBEAT_PORT, LED_HEARTBEAT_PIN, GPIO_PIN_RESET);

  watch_mpu_ok = (MPU6050_Init() == HAL_OK) ? 1U : 0U;
  if (watch_mpu_ok == 0U)
    watch_sensor_error_count++;

  watch_bme_ok = (BME680_Init() == HAL_OK) ? 1U : 0U;
  bme_last_retry_ms = HAL_GetTick();

  last_heartbeat_ms = HAL_GetTick();
  heartbeat_enabled = 1U;
  last_sensor_ms = HAL_GetTick();

  while (1) {
    uint32_t now = HAL_GetTick();


    if (rx_led_active != 0U &&
        (uint32_t)(now - rx_led_started_ms) >= RX_LED_PULSE_MS) {
      HAL_GPIO_WritePin(LED_RX_PORT, LED_RX_PIN, GPIO_PIN_RESET);
      rx_led_active = 0U;
    }

    if (bme_measurement_pending != 0U &&
        (int32_t)(now - bme_ready_at_ms) >= 0)
      BME680_CompleteMeasurement();

    if (watch_bme_ok == 0U &&
        (uint32_t)(now - bme_last_retry_ms) >= BME_RETRY_INTERVAL_MS) {
      bme_last_retry_ms = now;
      watch_bme_retry_count++;
      if (BME680_Init() != HAL_OK)
        watch_bme_ok = 0U;
    }

    if ((uint32_t)(now - last_sensor_ms) >= SENSOR_INTERVAL_MS) {
      last_sensor_ms += SENSOR_INTERVAL_MS;
      watch_sensor_count++;
      watch_last_ack_ok = 0U;

      if (watch_mpu_ok == 0U && MPU6050_Init() == HAL_OK)
        watch_mpu_ok = 1U;

      if (watch_mpu_ok == 0U || MPU6050_ReadAndProcess() != HAL_OK) {
        watch_mpu_ok = 0U;
        watch_sensor_error_count++;
      } else {
        watch_mpu_ok = 1U;
      }

      if (watch_bme_ok != 0U &&
          BME680_StartMeasurement(now) != HAL_OK) {
        watch_bme_data_valid = 0U;
        watch_sensor_error_count++;
      }

      if (RS485_SendFrame() == HAL_OK &&
          RS485_WaitAck(sequence_number) != 0U) {
        watch_last_ack_ok = 1U;
        watch_ack_count++;
        HAL_GPIO_WritePin(LED_RX_PORT, LED_RX_PIN, GPIO_PIN_SET);
        rx_led_started_ms = HAL_GetTick();
        rx_led_active = 1U;
      } else {
        watch_timeout_count++;
      }

      sequence_number++;
    }
  }
}

/* SysTick mantiene el heartbeat incluso durante la espera de ACK por UART. */
void HAL_SYSTICK_Callback(void)
{
  if (heartbeat_enabled != 0U &&
      (uint32_t)(HAL_GetTick() - last_heartbeat_ms) >= HEARTBEAT_INTERVAL_MS) {
    last_heartbeat_ms += HEARTBEAT_INTERVAL_MS;
    HAL_GPIO_TogglePin(LED_HEARTBEAT_PORT, LED_HEARTBEAT_PIN);
    watch_heartbeat_count++;
  }
}
/* ---------------------- Reloj y periféricos ----------------------------- */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef clk = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8U;
  RCC_OscInitStruct.PLL.PLLN = 100U;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4U;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK)
    Error_Handler();
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000U;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0U;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0U;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    Error_Handler();
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200U;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK)
    Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, RS485_DE_PIN | RS485_RE_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, LED_TX_PIN | LED_RX_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, LED_HEARTBEAT_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pin = RS485_DE_PIN | RS485_RE_PIN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED_TX_PIN | LED_RX_PIN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED_HEARTBEAT_PIN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
