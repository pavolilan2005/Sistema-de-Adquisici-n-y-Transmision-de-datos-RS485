/**
 * @file main.c
 * @brief Firmware de adquisicion MPU6050 y envio de telemetria RS485.
 * @version 1.0.1
 *
 * El bucle principal mantiene el heartbeat, toma una muestra por segundo,
 * procesa unidades fisicas y espera el ACK del gateway ESP32. Los campos del
 * BME680 estan reservados en esta version y se transmiten en cero.
 */
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

/* MPU6050: AD0 a GND selecciona la direccion I2C de 7 bits 0x68. */
#define MPU6050_ADDR              (0x68U << 1)

/* MAX485 STM32: según el esquema */
#define RS485_DE_PORT             GPIOA
#define RS485_DE_PIN              GPIO_PIN_2
#define RS485_RE_PORT             GPIOA
#define RS485_RE_PIN              GPIO_PIN_1

/* LEDs */
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

#define MPU_REG_ACCEL_XOUT_H      0x3BU
#define MPU_REG_PWR_MGMT_1        0x6BU
#define MPU_REG_WHO_AM_I          0x75U

#define ACCEL_LSB_PER_G           16384L
#define GYRO_LSB_PER_DPS          131L
/** Configura SYSCLK a 100 MHz desde HSI mediante PLL. */
void SystemClock_Config(void);
/** Configura DE/RE y LEDs como salidas digitales. */
static void MX_GPIO_Init(void);
/** Configura I2C1 en PB6/PB7 a 100 kHz para los sensores. */
static void MX_I2C1_Init(void);
/** Configura USART1 en PA9/PA10 a 115200 8N1 para el MAX485. */
static void MX_USART1_UART_Init(void);

/** Lee registros I2C con un timeout acotado. */
static HAL_StatusTypeDef I2C_Read(uint16_t dev, uint8_t reg,
                                  uint8_t *data, uint16_t len);
/** Escribe un byte en un registro I2C. */
static HAL_StatusTypeDef I2C_Write(uint16_t dev, uint8_t reg,
                                   uint8_t value);
/** Comprueba WHO_AM_I y configura el MPU6050 a +/-2 g y +/-250 dps. */
static HAL_StatusTypeDef MPU6050_Init(void);
/** Lee 14 bytes y convierte aceleracion, giro y temperatura del MPU6050. */
static HAL_StatusTypeDef MPU6050_ReadAndProcess(void);
/** Coloca el MAX485 en recepcion: DE=0 y /RE=0. */
static void RS485_SetReceiveMode(void);
/** Coloca el MAX485 en transmision: DE=1 y /RE=1. */
static void RS485_SetTransmitMode(void);
/** Construye `$D,...*XX`, calcula XOR y transmite por USART1. */
static HAL_StatusTypeDef RS485_SendFrame(void);
/** Espera `$A,<seq>,OK` hasta el timeout y guarda la linea recibida. */
static uint8_t RS485_WaitAck(uint8_t seq);
/* Variables volatile para observar estado y contadores en Keil Watch. */
volatile uint32_t watch_heartbeat_count = 0U;
volatile uint32_t watch_sensor_count = 0U;
volatile uint32_t watch_ack_count = 0U;
volatile uint32_t watch_timeout_count = 0U;
volatile uint32_t watch_sensor_error_count = 0U;
volatile uint8_t watch_last_ack_ok = 0U;
volatile uint8_t watch_mpu_ok = 0U;
volatile uint8_t watch_last_sequence = 0U;

volatile int16_t accel_x_raw = 0;
volatile int16_t accel_y_raw = 0;
volatile int16_t accel_z_raw = 0;
volatile int16_t gyro_x_raw = 0;
volatile int16_t gyro_y_raw = 0;
volatile int16_t gyro_z_raw = 0;
volatile int16_t mpu_temp_raw = 0;

volatile int32_t accel_x_mg = 0;
volatile int32_t accel_y_mg = 0;
volatile int32_t accel_z_mg = 0;
volatile int32_t gyro_x_mdps = 0;
volatile int32_t gyro_y_mdps = 0;
volatile int32_t gyro_z_mdps = 0;
volatile int32_t mpu_temp_centi_c = 0;

/* Campos BME680 reservados; este firmware aun no inicializa ni lee el sensor. */
volatile uint32_t bme_pressure_pa = 0U;
volatile int32_t bme_temperature_centi_c = 0;
volatile uint32_t bme_humidity_milli_pct = 0U;
volatile uint32_t bme_gas_ohms = 0U;

volatile uint8_t sequence_number = 0U;
volatile uint32_t last_heartbeat_ms = 0U;
volatile uint32_t last_sensor_ms = 0U;

/* Estado del pulso del LED RX */
static uint8_t rx_led_active = 0U;
static uint32_t rx_led_started_ms = 0U;

/* Copias para inspección */
volatile char watch_tx_frame[180];
volatile char watch_rx_line[40];

/* ----------------------------- I2C -------------------------------------- */

/** Lee registros I2C con un timeout acotado. */
static HAL_StatusTypeDef I2C_Read(uint16_t dev, uint8_t reg,
                                  uint8_t *data, uint16_t len)
{
  return HAL_I2C_Mem_Read(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT,
                          data, len, I2C_TIMEOUT_MS);
}

/** Escribe un byte en un registro I2C. */
static HAL_StatusTypeDef I2C_Write(uint16_t dev, uint8_t reg,
                                   uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT,
                           &value, 1U, I2C_TIMEOUT_MS);
}

/* ---------------------------- MPU6050 ----------------------------------- */

/** Comprueba WHO_AM_I y configura el MPU6050 a +/-2 g y +/-250 dps. */
static HAL_StatusTypeDef MPU6050_Init(void)
{
  uint8_t id = 0U;

  if (I2C_Read(MPU6050_ADDR, MPU_REG_WHO_AM_I, &id, 1U) != HAL_OK)
    return HAL_ERROR;

  if (id != 0x68U)
    return HAL_ERROR;

  /* Despertar sensor */
  if (I2C_Write(MPU6050_ADDR, MPU_REG_PWR_MGMT_1, 0x00U) != HAL_OK)
    return HAL_ERROR;

  HAL_Delay(10U);

  /* Filtro digital */
  if (I2C_Write(MPU6050_ADDR, 0x1AU, 0x03U) != HAL_OK)
    return HAL_ERROR;

  /* Giroscopio ±250 °/s */
  if (I2C_Write(MPU6050_ADDR, 0x1BU, 0x00U) != HAL_OK)
    return HAL_ERROR;

  /* Acelerómetro ±2 g */
  if (I2C_Write(MPU6050_ADDR, 0x1CU, 0x00U) != HAL_OK)
    return HAL_ERROR;

  return HAL_OK;
}

/** Lee 14 bytes y convierte aceleracion, giro y temperatura del MPU6050. */
static HAL_StatusTypeDef MPU6050_ReadAndProcess(void)
{
  uint8_t data[14];

  if (I2C_Read(MPU6050_ADDR,
               MPU_REG_ACCEL_XOUT_H,
               data,
               sizeof(data)) != HAL_OK)
    return HAL_ERROR;

  accel_x_raw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  accel_y_raw = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  accel_z_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);

  mpu_temp_raw = (int16_t)(((uint16_t)data[6] << 8) | data[7]);

  gyro_x_raw = (int16_t)(((uint16_t)data[8] << 8) | data[9]);
  gyro_y_raw = (int16_t)(((uint16_t)data[10] << 8) | data[11]);
  gyro_z_raw = (int16_t)(((uint16_t)data[12] << 8) | data[13]);

  /* Resultado en mg: 1000 mg = 1 g. */
  accel_x_mg = ((int32_t)accel_x_raw * 1000L) / ACCEL_LSB_PER_G;
  accel_y_mg = ((int32_t)accel_y_raw * 1000L) / ACCEL_LSB_PER_G;
  accel_z_mg = ((int32_t)accel_z_raw * 1000L) / ACCEL_LSB_PER_G;

  /* Resultado en miligrados/s: 1000 mdps = 1 °/s. */
  gyro_x_mdps = ((int32_t)gyro_x_raw * 1000L) / GYRO_LSB_PER_DPS;
  gyro_y_mdps = ((int32_t)gyro_y_raw * 1000L) / GYRO_LSB_PER_DPS;
  gyro_z_mdps = ((int32_t)gyro_z_raw * 1000L) / GYRO_LSB_PER_DPS;

  /* Temperatura del MPU6050 en centésimas de °C. */
  mpu_temp_centi_c =
      3653L + (((int32_t)mpu_temp_raw * 100L) / 340L);

  return HAL_OK;
}

/* ------------------------------ RS485 ----------------------------------- */

/** Coloca el MAX485 en recepcion: DE=0 y /RE=0. */
static void RS485_SetReceiveMode(void)
{
  /* DE=0 apaga TX; /RE=0 habilita RX. */
  HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RS485_RE_PORT, RS485_RE_PIN, GPIO_PIN_RESET);
}

/** Coloca el MAX485 en transmision: DE=1 y /RE=1. */
static void RS485_SetTransmitMode(void)
{
  /* DE=1 habilita TX; /RE=1 deshabilita RX. */
  HAL_GPIO_WritePin(RS485_RE_PORT, RS485_RE_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET);
}

/** Construye `$D,...*XX`, calcula XOR y transmite por USART1. */
static HAL_StatusTypeDef RS485_SendFrame(void)
{
  char body[145];
  char frame[180];
  uint8_t checksum = 0U;
  int body_len;
  int frame_len;

  body_len = snprintf(body, sizeof(body),
      "D,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%lu,%ld,%lu,%lu",
      (unsigned int)sequence_number,
      (long)accel_x_mg,
      (long)accel_y_mg,
      (long)accel_z_mg,
      (long)gyro_x_mdps,
      (long)gyro_y_mdps,
      (long)gyro_z_mdps,
      (long)mpu_temp_centi_c,
      (unsigned long)bme_pressure_pa,
      (long)bme_temperature_centi_c,
      (unsigned long)bme_humidity_milli_pct,
      (unsigned long)bme_gas_ohms);

  if (body_len <= 0 || (size_t)body_len >= sizeof(body))
    return HAL_ERROR;

  for (int i = 0; i < body_len; i++)
    checksum ^= (uint8_t)body[i];

  frame_len = snprintf(frame, sizeof(frame), "$%s*%02X\r\n",
                       body, checksum);

  if (frame_len <= 0 || (size_t)frame_len >= sizeof(frame))
    return HAL_ERROR;

  for (uint16_t i = 0U; i < sizeof(watch_tx_frame); i++)
    watch_tx_frame[i] =
        (i < (uint16_t)frame_len) ? frame[i] : '\0';

  RS485_SetTransmitMode();
  HAL_Delay(1U);

  HAL_GPIO_WritePin(LED_TX_PORT, LED_TX_PIN, GPIO_PIN_SET);

  if (HAL_UART_Transmit(&huart1,
                        (uint8_t *)frame,
                        (uint16_t)frame_len,
                        300U) != HAL_OK) {
    HAL_GPIO_WritePin(LED_TX_PORT, LED_TX_PIN, GPIO_PIN_RESET);
    RS485_SetReceiveMode();
    return HAL_ERROR;
  }

  while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {}

  HAL_GPIO_WritePin(LED_TX_PORT, LED_TX_PIN, GPIO_PIN_RESET);
  RS485_SetReceiveMode();

  return HAL_OK;
}

/** Espera `$A,<seq>,OK` hasta el timeout y guarda la linea recibida. */
static uint8_t RS485_WaitAck(uint8_t seq)
{
  char line[40] = {0};
  char expected[20];
  uint16_t n = 0U;
  uint8_t ch;
  uint32_t start = HAL_GetTick();

  snprintf(expected, sizeof(expected), "$A,%u,OK", (unsigned int)seq);

  while ((HAL_GetTick() - start) < ACK_TIMEOUT_MS) {
    if (HAL_UART_Receive(&huart1, &ch, 1U, 10U) == HAL_OK) {
      if (ch == '\n') {
        if (n > 0U && line[n - 1U] == '\r')
          line[n - 1U] = '\0';
        else
          line[n] = '\0';

        for (uint16_t i = 0U; i < sizeof(watch_rx_line); i++)
          watch_rx_line[i] =
              (i < strlen(line)) ? line[i] : '\0';

        return (strcmp(line, expected) == 0) ? 1U : 0U;
      }

      if (n < sizeof(line) - 1U)
        line[n++] = (char)ch;
      else
        n = 0U;
    }
  }

  for (uint16_t i = 0U; i < sizeof(watch_rx_line); i++)
    watch_rx_line[i] = '\0';

  return 0U;
}

/* -------------------------------- main ---------------------------------- */

/** Inicializa HAL y ejecuta heartbeat, adquisicion, transmision y ACK. */
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
  HAL_GPIO_WritePin(LED_HEARTBEAT_PORT,
                    LED_HEARTBEAT_PIN,
                    GPIO_PIN_RESET);

  watch_mpu_ok = (MPU6050_Init() == HAL_OK) ? 1U : 0U;

  last_heartbeat_ms = HAL_GetTick();
  last_sensor_ms = HAL_GetTick();

  while (1) {
    uint32_t now = HAL_GetTick();

    /* PC13 cambia de estado cada 200 ms. */
    if ((now - last_heartbeat_ms) >= HEARTBEAT_INTERVAL_MS) {
      last_heartbeat_ms = now;
      HAL_GPIO_TogglePin(LED_HEARTBEAT_PORT, LED_HEARTBEAT_PIN);
      watch_heartbeat_count++;
    }

    /*
     * Apaga PB12 al finalizar el pulso RX.
     * La resta unsigned funciona también al desbordarse HAL_GetTick().
     */
    if (rx_led_active &&
        ((now - rx_led_started_ms) >= RX_LED_PULSE_MS)) {
      HAL_GPIO_WritePin(LED_RX_PORT, LED_RX_PIN, GPIO_PIN_RESET);
      rx_led_active = 0U;
    }

    if ((now - last_sensor_ms) >= SENSOR_INTERVAL_MS) {
      last_sensor_ms = now;
      watch_sensor_count++;
      watch_last_sequence = sequence_number;
      watch_last_ack_ok = 0U;

      if (watch_mpu_ok && MPU6050_ReadAndProcess() == HAL_OK) {
        if (RS485_SendFrame() == HAL_OK) {
          if (RS485_WaitAck(sequence_number)) {
            watch_last_ack_ok = 1U;
            watch_ack_count++;

            /* Pulso RX de 100 ms por cada ACK válido. */
            HAL_GPIO_WritePin(LED_RX_PORT, LED_RX_PIN, GPIO_PIN_SET);
            rx_led_started_ms = HAL_GetTick();
            rx_led_active = 1U;
          } else {
            watch_timeout_count++;
          }
        } else {
          watch_timeout_count++;
        }

        sequence_number++;
      } else {
        watch_sensor_error_count++;
      }
    }
  }
}

/* ---------------------- Reloj y periféricos ----------------------------- */

/** Configura SYSCLK a 100 MHz desde HSI mediante PLL. */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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

  RCC_ClkInitTypeDef clk = {0};

  clk.ClockType = RCC_CLOCKTYPE_HCLK |
                  RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 |
                  RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK)
    Error_Handler();
}

/** Configura I2C1 en PB6/PB7 a 100 kHz para los sensores. */
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

/** Configura USART1 en PA9/PA10 a 115200 8N1 para el MAX485. */
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

/** Configura DE/RE y LEDs como salidas digitales. */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA,
                    RS485_DE_PIN | RS485_RE_PIN,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,
                    LED_TX_PIN | LED_RX_PIN,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC,
                    LED_HEARTBEAT_PIN,
                    GPIO_PIN_RESET);

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

/* Declarada en main.h; debe permanecer no-static. */
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