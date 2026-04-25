/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <limits.h>
#include <stddef.h>

#include "Sensors/sensor_data.h"
#include "app_safety.h"
#include "radio/RM126x.h"
#include "sleep_manager.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  APP_SENSOR_FAMILY_SHT4X = 0,
  APP_SENSOR_FAMILY_SOIL,
  APP_SENSOR_FAMILY_SCD41,
  APP_SENSOR_FAMILY_AS7341,
  APP_SENSOR_FAMILY_COUNT
} AppSensorFamily;

typedef struct
{
  uint8_t fport;
  size_t length;
  bool is_sensor_family;
  AppSensorFamily sensor_family;
} AppTxFrame;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_SLEEP_MINUTES_PER_SLOT 10U
#define APP_INITIAL_SEND_RETRY_DELAY_MS 1000U
#define APP_SENSOR_POWER_UP_DELAY_MS 300U
#define APP_SENSOR_DELAY_CHUNK_MS 250U
#define APP_PULSE_DEBOUNCE_MS 50U
#define APP_SENSOR_TEMP_DELTA_LIMIT_MDEG_C 500
#define APP_SENSOR_HUMIDITY_DELTA_LIMIT_MPCT_RH 5000
#define APP_TX_PAYLOAD_MAX_BYTES 11U

#define APP_PAYLOAD_LEN_SHT_NORMAL 4U
#define APP_PAYLOAD_LEN_SOIL 8U
#define APP_PAYLOAD_LEN_SCD41 5U
#define APP_PAYLOAD_LEN_PULSE_COUNT 4U
#define APP_PAYLOAD_LEN_DIAGNOSTIC 9U
#define APP_PAYLOAD_LEN_SHT_MISMATCH 8U
#define APP_PAYLOAD_LEN_ERROR 1U

#define APP_ERROR_NO_SENSOR 0x01U
#define APP_ERROR_SENSOR1_FAILURE 0x02U
#define APP_ERROR_SENSOR2_FAILURE 0x03U
#define APP_ERROR_HUMIDITY_VALIDATION 0x05U

#if APP_PAYLOAD_LEN_SHT_NORMAL > APP_TX_PAYLOAD_MAX_BYTES
#error "SHT4x normal payload exceeds app payload limit"
#endif
#if APP_PAYLOAD_LEN_SOIL > APP_TX_PAYLOAD_MAX_BYTES
#error "Soil payload exceeds app payload limit"
#endif
#if APP_PAYLOAD_LEN_SCD41 > APP_TX_PAYLOAD_MAX_BYTES
#error "SCD41 payload exceeds app payload limit"
#endif
#if APP_PAYLOAD_LEN_PULSE_COUNT > APP_TX_PAYLOAD_MAX_BYTES
#error "Pulse-count payload exceeds app payload limit"
#endif
#if APP_PAYLOAD_LEN_DIAGNOSTIC > APP_TX_PAYLOAD_MAX_BYTES
#error "Diagnostic payload exceeds app payload limit"
#endif
#if APP_PAYLOAD_LEN_SHT_MISMATCH > APP_TX_PAYLOAD_MAX_BYTES
#error "SHT4x mismatch payload exceeds app payload limit"
#endif
#if APP_PAYLOAD_LEN_ERROR > APP_TX_PAYLOAD_MAX_BYTES
#error "Error payload exceeds app payload limit"
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static SensorDataSnapshot g_sensor_snapshot;
static SensorDataStatus g_sensor_status = SENSOR_DATA_STATUS_NO_SENSORS;
static RM126xHandle g_rm126x;
static const RM126xConfig g_rm126x_config = {
    .uart =
        {
            .instance = USART2,
            .tx_port = GPIOA,
            .tx_pin = GPIO_PIN_2,
            .tx_af = GPIO_AF4_USART2,
            .rx_port = GPIOA,
            .rx_pin = GPIO_PIN_3,
            .rx_af = GPIO_AF4_USART2,
            .baud_rate = RM126X_DEFAULT_BAUD_RATE,
            .kernel_clock_hz = HSI_VALUE,
        },
    .wake_timeout_ms = RM126X_DEFAULT_WAKE_TIMEOUT_MS,
    .command_timeout_ms = RM126X_DEFAULT_COMMAND_TIMEOUT_MS,
    .rx_poll_timeout_ms = RM126X_DEFAULT_RX_POLL_TIMEOUT_MS,
};
static uint8_t g_reset_reason_code = 0U;
static bool g_initial_send_pending = false;
static volatile uint32_t g_pulse_counter = 0;
static volatile bool g_pulse_uplink_pending = false;
static uint32_t g_last_pulse_tick_ms = 0U;
static bool g_pulse_debounce_armed = false;
static AppSensorFamily g_next_sensor_family = APP_SENSOR_FAMILY_SHT4X;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void) __attribute__((unused));
static void MX_IWDG_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void App_FeedWatchdog(void);
static void App_SensorDelayMs(uint32_t delay_ms);
static uint8_t App_CaptureResetReasonCode(void);
static bool App_IsSensorValid(const SensorDataSensor *sensor);
static bool App_IsShtFamilyValid(const SensorDataSnapshot *snapshot);
static bool App_IsShtTemperatureMismatch(const SensorDataSnapshot *snapshot);
static bool App_IsSoilValid(const SensorDataSnapshot *snapshot);
static bool App_IsScd41Valid(const SensorDataSnapshot *snapshot);
static bool App_IsSensorFamilyAvailable(AppSensorFamily family,
                                        const SensorDataSnapshot *snapshot);
static AppSensorFamily App_NextSensorFamily(AppSensorFamily family);
static void App_AdvanceSensorFamilyRotation(AppSensorFamily sent_family);
static bool App_SetTxFrame(AppTxFrame *frame, uint8_t fport, size_t length,
                           bool is_sensor_family,
                           AppSensorFamily sensor_family);
static void App_PackU16BE(uint8_t *buffer, uint16_t value);
static void App_PackU32BE(uint8_t *buffer, uint32_t value);
static uint16_t App_HumidityToWire(int32_t humidity_milli_pct_rh);
static int16_t App_T1TemperatureToWire(int32_t temperature_mdeg_c);
static int16_t App_T2TemperatureToWire(int32_t temperature_mdeg_c);
static void App_BuildNormalPayload(const SensorDataSnapshot *snapshot,
                                   uint8_t *payload);
static void App_BuildSoilPayload(const SensorDataSnapshot *snapshot,
                                 uint8_t *payload);
static void App_BuildScd41Payload(const SensorDataSnapshot *snapshot,
                                  uint8_t *payload);
static void App_BuildDiagnosticPayload(const SensorDataSnapshot *snapshot,
                                       uint8_t reset_reason_code,
                                       uint8_t *payload);
static void App_BuildPulseCountPayload(uint32_t pulse_count, uint8_t *payload);
static void App_BuildMismatchPayload(const SensorDataSnapshot *snapshot,
                                     uint8_t *payload);
static void App_BuildErrorPayload(uint8_t error_code, uint8_t *payload);
static bool App_BuildRotatingSensorPayload(const SensorDataSnapshot *snapshot,
                                           uint8_t *payload,
                                           AppTxFrame *frame);
static uint8_t App_SelectErrorCode(const SensorDataSnapshot *snapshot);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void App_FeedWatchdog(void)
{
  if (AppSafety_IsWatchdogActive() && (HAL_IWDG_Refresh(&hiwdg) != HAL_OK))
  {
    Error_Handler();
  }
}

static void App_SensorDelayMs(uint32_t delay_ms)
{
  uint32_t remaining_ms = delay_ms;

  App_FeedWatchdog();
  while (remaining_ms > 0U)
  {
    uint32_t chunk_ms = (remaining_ms > APP_SENSOR_DELAY_CHUNK_MS)
                            ? APP_SENSOR_DELAY_CHUNK_MS
                            : remaining_ms;

    HAL_Delay(chunk_ms);
    remaining_ms -= chunk_ms;
    App_FeedWatchdog();
  }
}

static uint8_t App_CaptureResetReasonCode(void)
{
  uint8_t reset_reason = 0U;

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  {
    reset_reason = 0x04U;
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET)
  {
    reset_reason = 0x05U;
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET)
  {
    reset_reason = 0x03U;
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != RESET)
  {
    reset_reason = 0x06U;
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET)
  {
    reset_reason = 0x01U;
  }
  else if ((__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET) ||
           (__HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST) != RESET))
  {
    reset_reason = 0x02U;
  }

  __HAL_RCC_CLEAR_RESET_FLAGS();
  return reset_reason;
}

static bool App_IsSensorValid(const SensorDataSensor *sensor)
{
  return (sensor != NULL) && sensor->present && (sensor->driver_error == 0);
}

static bool App_IsShtFamilyValid(const SensorDataSnapshot *snapshot)
{
  return (snapshot != NULL) && App_IsSensorValid(&snapshot->sensors[0]) &&
         App_IsSensorValid(&snapshot->sensors[1]);
}

static bool App_IsShtTemperatureMismatch(const SensorDataSnapshot *snapshot)
{
  int32_t temperature_delta;

  if (!App_IsShtFamilyValid(snapshot))
  {
    return false;
  }

  /* Legacy SHT4x path: only the paired SHT sensors participate in this
   * mismatch check. SCD41 temperature is intentionally excluded. */
  temperature_delta = snapshot->sensors[0].temperature_mdeg_c -
                      snapshot->sensors[1].temperature_mdeg_c;
  if (temperature_delta < 0)
  {
    temperature_delta = -temperature_delta;
  }

  return (temperature_delta > APP_SENSOR_TEMP_DELTA_LIMIT_MDEG_C);
}

static bool App_IsSoilValid(const SensorDataSnapshot *snapshot)
{
  return (snapshot != NULL) && snapshot->soil.present &&
         snapshot->soil.sample_valid && (snapshot->soil.driver_error == 0);
}

static bool App_IsScd41Valid(const SensorDataSnapshot *snapshot)
{
  return (snapshot != NULL) && snapshot->scd41.present &&
         snapshot->scd41.sample_valid && (snapshot->scd41.driver_error == 0);
}

static bool App_IsSensorFamilyAvailable(AppSensorFamily family,
                                        const SensorDataSnapshot *snapshot)
{
  switch (family)
  {
  case APP_SENSOR_FAMILY_SHT4X:
    return App_IsShtFamilyValid(snapshot);
  case APP_SENSOR_FAMILY_SOIL:
    return App_IsSoilValid(snapshot);
  case APP_SENSOR_FAMILY_SCD41:
    return App_IsScd41Valid(snapshot);
  case APP_SENSOR_FAMILY_AS7341:
    /* AS7341 reserved/future: fPort 4 exists, but this phase never emits it. */
    return false;
  default:
    return false;
  }
}

static AppSensorFamily App_NextSensorFamily(AppSensorFamily family)
{
  uint8_t next_family = (uint8_t)family;

  next_family++;

  if (next_family >= (uint8_t)APP_SENSOR_FAMILY_COUNT)
  {
    next_family = 0U;
  }

  return (AppSensorFamily)next_family;
}

static void App_AdvanceSensorFamilyRotation(AppSensorFamily sent_family)
{
  g_next_sensor_family = App_NextSensorFamily(sent_family);
}

static bool App_SetTxFrame(AppTxFrame *frame, uint8_t fport, size_t length,
                           bool is_sensor_family,
                           AppSensorFamily sensor_family)
{
  if ((frame == NULL) || (length > APP_TX_PAYLOAD_MAX_BYTES))
  {
    return false;
  }

  frame->fport = fport;
  frame->length = length;
  frame->is_sensor_family = is_sensor_family;
  frame->sensor_family = sensor_family;

  return true;
}

static void App_PackU16BE(uint8_t *buffer, uint16_t value)
{
  buffer[0] = (uint8_t)((value >> 8) & 0xFFU);
  buffer[1] = (uint8_t)(value & 0xFFU);
}

static void App_PackU32BE(uint8_t *buffer, uint32_t value)
{
  buffer[0] = (uint8_t)((value >> 24) & 0xFFU);
  buffer[1] = (uint8_t)((value >> 16) & 0xFFU);
  buffer[2] = (uint8_t)((value >> 8) & 0xFFU);
  buffer[3] = (uint8_t)(value & 0xFFU);
}

static uint16_t App_HumidityToWire(int32_t humidity_milli_pct_rh)
{
  if (humidity_milli_pct_rh < 0)
  {
    return 0U;
  }

  if (humidity_milli_pct_rh > 100000)
  {
    return 10000U;
  }

  return (uint16_t)(humidity_milli_pct_rh / 10);
}

static int16_t App_T1TemperatureToWire(int32_t temperature_mdeg_c)
{
  int32_t wire = (temperature_mdeg_c / 10) + 5500;

  if (wire < INT16_MIN)
  {
    wire = INT16_MIN;
  }
  else if (wire > INT16_MAX)
  {
    wire = INT16_MAX;
  }

  return (int16_t)wire;
}

static int16_t App_T2TemperatureToWire(int32_t temperature_mdeg_c)
{
  int32_t wire = temperature_mdeg_c / 10;

  if (wire < INT16_MIN)
  {
    wire = INT16_MIN;
  }
  else if (wire > INT16_MAX)
  {
    wire = INT16_MAX;
  }

  return (int16_t)wire;
}

static void App_BuildNormalPayload(const SensorDataSnapshot *snapshot,
                                   uint8_t *payload)
{
  const SensorDataSensor *sensor1 = &snapshot->sensors[0];
  App_PackU16BE(payload,
                (uint16_t)App_T1TemperatureToWire(sensor1->temperature_mdeg_c));
  App_PackU16BE(payload + 2,
                App_HumidityToWire(sensor1->humidity_milli_pct_rh));
}

static void App_BuildSoilPayload(const SensorDataSnapshot *snapshot,
                                 uint8_t *payload)
{
  App_PackU16BE(payload, (uint16_t)snapshot->soil.e25_x100);
  App_PackU16BE(payload + 2, snapshot->soil.ec_uS_cm);
  App_PackU16BE(payload + 4, (uint16_t)snapshot->soil.temperature_c_x100);
  App_PackU16BE(payload + 6, snapshot->soil.vwc_pct_x10);
}

static void App_BuildScd41Payload(const SensorDataSnapshot *snapshot,
                                  uint8_t *payload)
{
  /* SCD41 CO2 path: humidity is intentionally a whole percent byte. */
  App_PackU16BE(payload, snapshot->scd41.co2_ppm);
  App_PackU16BE(payload + 2,
                (uint16_t)App_T2TemperatureToWire(
                    snapshot->scd41.temperature_mdeg_c));
  payload[4] = snapshot->scd41.humidity_pct;
}

static void App_BuildDiagnosticPayload(const SensorDataSnapshot *snapshot,
                                       uint8_t reset_reason_code,
                                       uint8_t *payload)
{
  App_PackU32BE(payload, snapshot->sensors[0].serial_number);
  App_PackU32BE(payload + 4, snapshot->sensors[1].serial_number);
  payload[8] = reset_reason_code;
}

static void App_BuildPulseCountPayload(uint32_t pulse_count, uint8_t *payload)
{
  App_PackU32BE(payload, pulse_count);
}

static void App_BuildMismatchPayload(const SensorDataSnapshot *snapshot,
                                     uint8_t *payload)
{
  const SensorDataSensor *sensor1 = &snapshot->sensors[0];
  const SensorDataSensor *sensor2 = &snapshot->sensors[1];

  App_PackU16BE(payload,
                (uint16_t)App_T1TemperatureToWire(sensor1->temperature_mdeg_c));
  App_PackU16BE(payload + 2,
                App_HumidityToWire(sensor1->humidity_milli_pct_rh));
  App_PackU16BE(payload + 4,
                (uint16_t)App_T2TemperatureToWire(sensor2->temperature_mdeg_c));
  App_PackU16BE(payload + 6,
                App_HumidityToWire(sensor2->humidity_milli_pct_rh));
}

static void App_BuildErrorPayload(uint8_t error_code, uint8_t *payload)
{
  payload[0] = error_code;
}

static bool App_BuildRotatingSensorPayload(const SensorDataSnapshot *snapshot,
                                           uint8_t *payload,
                                           AppTxFrame *frame)
{
  AppSensorFamily family = g_next_sensor_family;
  uint8_t attempts;

  if ((snapshot == NULL) || (payload == NULL) || (frame == NULL))
  {
    return false;
  }

  /* Sensor family rotation: multi-sensor systems send one complete family
   * payload per uplink instead of packing combinations into an 11-byte frame. */
  for (attempts = 0U; attempts < (uint8_t)APP_SENSOR_FAMILY_COUNT; ++attempts)
  {
    if (App_IsSensorFamilyAvailable(family, snapshot))
    {
      switch (family)
      {
      case APP_SENSOR_FAMILY_SHT4X:
        /* Legacy SHT4x path: fPort 1 for normal data, fPort 11 when the
         * two SHT temperature readings disagree. */
        if (App_IsShtTemperatureMismatch(snapshot))
        {
          if (!App_SetTxFrame(frame, RM126X_FPORT_SENSOR_MISMATCH,
                              APP_PAYLOAD_LEN_SHT_MISMATCH, true, family))
          {
            return false;
          }
          App_BuildMismatchPayload(snapshot, payload);
        }
        else
        {
          if (!App_SetTxFrame(frame, RM126X_FPORT_NORMAL,
                              APP_PAYLOAD_LEN_SHT_NORMAL, true, family))
          {
            return false;
          }
          App_BuildNormalPayload(snapshot, payload);
        }
        return true;

      case APP_SENSOR_FAMILY_SOIL:
        if (!App_SetTxFrame(frame, RM126X_FPORT_SOIL, APP_PAYLOAD_LEN_SOIL,
                            true, family))
        {
          return false;
        }
        App_BuildSoilPayload(snapshot, payload);
        return true;

      case APP_SENSOR_FAMILY_SCD41:
        /* SCD41 CO2 path: standalone from SHT validation and fPort 11. */
        if (!App_SetTxFrame(frame, RM126X_FPORT_CO2, APP_PAYLOAD_LEN_SCD41,
                            true, family))
        {
          return false;
        }
        App_BuildScd41Payload(snapshot, payload);
        return true;

      case APP_SENSOR_FAMILY_AS7341:
      default:
        break;
      }
    }

    family = App_NextSensorFamily(family);
  }

  return false;
}

static uint8_t App_SelectErrorCode(const SensorDataSnapshot *snapshot)
{
  bool sensor1_valid = App_IsSensorValid(&snapshot->sensors[0]);
  bool sensor2_valid = App_IsSensorValid(&snapshot->sensors[1]);

  if (!sensor1_valid && !sensor2_valid)
  {
    return APP_ERROR_NO_SENSOR;
  }

  if (!sensor1_valid)
  {
    return APP_ERROR_SENSOR1_FAILURE;
  }

  if (!sensor2_valid)
  {
    return APP_ERROR_SENSOR2_FAILURE;
  }

  return APP_ERROR_NO_SENSOR;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == PULSE_IN_Pin)
  {
    bool bypass_debounce = SleepManager_IsStopWakeInProgress();
    uint32_t now = HAL_GetTick();

    if (!bypass_debounce && g_pulse_debounce_armed &&
        ((uint32_t)(now - g_last_pulse_tick_ms) < APP_PULSE_DEBOUNCE_MS))
    {
      return;
    }

    g_last_pulse_tick_ms = now;
    g_pulse_debounce_armed = true;
    SleepManager_HandleExternalWake();
    g_pulse_uplink_pending = true;

    if (g_pulse_counter == UINT32_MAX)
    {
      g_pulse_counter = 0U;
    }
    else
    {
      g_pulse_counter++;
    }
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */
  AppSafety_Init();
  AppSafety_SetBootStage(APP_BOOT_STAGE_HAL_INITIALIZED);
  AppSafety_PaintStack();
  g_reset_reason_code = App_CaptureResetReasonCode();
  AppSafety_SetResetReasonCode(g_reset_reason_code);
  g_initial_send_pending = true;
  MX_IWDG_Init();
  AppSafety_SetBootStage(APP_BOOT_STAGE_WATCHDOG_STARTED);
  App_FeedWatchdog();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  AppSafety_SetBootStage(APP_BOOT_STAGE_CLOCKS_CONFIGURED);

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  AppSafety_SetBootStage(APP_BOOT_STAGE_GPIO_INITIALIZED);
  App_FeedWatchdog();
  MX_I2C1_Init();
  AppSafety_SetBootStage(APP_BOOT_STAGE_I2C_INITIALIZED);
  App_FeedWatchdog();
  MX_USART2_UART_Init();
  AppSafety_SetBootStage(APP_BOOT_STAGE_UART_INITIALIZED);
  App_FeedWatchdog();
  /* USER CODE BEGIN 2 */
  /* Sleep manager owns the final RTC/LSE configuration after CubeMX init. */
  SleepManager_Init(&hrtc, &hiwdg, &hi2c1);
  AppSafety_SetBootStage(APP_BOOT_STAGE_SLEEP_MANAGER_INITIALIZED);
  SensorData_Init(&hi2c1);
  SensorData_SetDelayMsCallback(App_SensorDelayMs);
  RM126x_Init(&g_rm126x, &g_rm126x_config);
  (void)RM126x_HostUartResume(&g_rm126x);
  AppSafety_SetBootStage(APP_BOOT_STAGE_APPLICATION_READY);
  AppSafety_SampleStackWatermark();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    AppSafety_SampleStackWatermark();
    App_FeedWatchdog();
    HAL_GPIO_WritePin(WD_DONE_GPIO_Port, WD_DONE_Pin, GPIO_PIN_SET);
    (void)RM126x_HostUartResume(&g_rm126x);

    if (RM126x_IsConnected(&g_rm126x))
    {
      uint8_t tx_payload[APP_TX_PAYLOAD_MAX_BYTES] = {0};
      AppTxFrame tx_frame = {
          .fport = RM126X_FPORT_ERROR,
          .length = 0U,
          .is_sensor_family = false,
          .sensor_family = APP_SENSOR_FAMILY_SHT4X,
      };
      bool force_diagnostic = RM126x_ShouldSendDiagnosticPort9(&g_rm126x);
      bool send_diagnostic;
      bool pulse_uplink_pending = g_pulse_uplink_pending;
      uint32_t pulse_count_snapshot = g_pulse_counter;
      bool has_legacy_sensors;

      HAL_GPIO_WritePin(I2C_ENABLE_GPIO_Port, I2C_ENABLE_Pin, GPIO_PIN_SET);
      HAL_Delay(APP_SENSOR_POWER_UP_DELAY_MS);
      g_sensor_status = SensorData_ReadAll(&g_sensor_snapshot);
      has_legacy_sensors = (g_sensor_snapshot.present_count > 0U);
      send_diagnostic = force_diagnostic && has_legacy_sensors;

      if (send_diagnostic)
      {
        (void)App_SetTxFrame(&tx_frame, RM126X_FPORT_DIAGNOSTIC,
                             APP_PAYLOAD_LEN_DIAGNOSTIC, false,
                             APP_SENSOR_FAMILY_SHT4X);
        App_BuildDiagnosticPayload(&g_sensor_snapshot, g_reset_reason_code,
                                   tx_payload);
      }
      else if (!g_initial_send_pending && pulse_uplink_pending)
      {
        (void)App_SetTxFrame(&tx_frame, RM126X_FPORT_PULSE_COUNT,
                             APP_PAYLOAD_LEN_PULSE_COUNT, false,
                             APP_SENSOR_FAMILY_SHT4X);
        App_BuildPulseCountPayload(pulse_count_snapshot, tx_payload);
      }
      else if (!App_BuildRotatingSensorPayload(&g_sensor_snapshot, tx_payload,
                                               &tx_frame))
      {
        (void)App_SetTxFrame(&tx_frame, RM126X_FPORT_ERROR,
                             APP_PAYLOAD_LEN_ERROR, false,
                             APP_SENSOR_FAMILY_SHT4X);
        App_BuildErrorPayload(App_SelectErrorCode(&g_sensor_snapshot),
                              tx_payload);
      }

      HAL_GPIO_WritePin(I2C_ENABLE_GPIO_Port, I2C_ENABLE_Pin, GPIO_PIN_RESET);
      App_FeedWatchdog();

      if (RM126x_WakeAndPing(&g_rm126x) == RM126X_RESULT_OK)
      {
        if (g_rm126x.current_fport != tx_frame.fport)
        {
          (void)RM126x_SetFPort(&g_rm126x, tx_frame.fport);
        }

        if (g_rm126x.current_fport == tx_frame.fport)
        {
          RM126xResult send_result =
              RM126x_SendBytes(&g_rm126x, tx_payload, tx_frame.length);
          if (send_result == RM126X_RESULT_OK)
          {
            if (tx_frame.fport == RM126X_FPORT_PULSE_COUNT)
            {
              g_pulse_uplink_pending = false;
            }
            if (tx_frame.is_sensor_family)
            {
              App_AdvanceSensorFamilyRotation(tx_frame.sensor_family);
            }
            g_initial_send_pending = false;
          }
        }
      }
    }
    else
    {
      (void)RM126x_Join(&g_rm126x);
      HAL_GPIO_WritePin(I2C_ENABLE_GPIO_Port, I2C_ENABLE_Pin, GPIO_PIN_RESET);
    }

    App_FeedWatchdog();
    (void)RM126x_HostUartSuspend(&g_rm126x);
    HAL_GPIO_WritePin(WD_DONE_GPIO_Port, WD_DONE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(I2C_ENABLE_GPIO_Port, I2C_ENABLE_Pin, GPIO_PIN_RESET);

    if (g_initial_send_pending)
    {
      App_FeedWatchdog();
      HAL_Delay(APP_INITIAL_SEND_RETRY_DELAY_MS);
      App_FeedWatchdog();
      continue;
    }

    SleepManager_SleepUntilWake(APP_SLEEP_MINUTES_PER_SLOT);
    AppSafety_SampleStackWatermark();

    /* USER APP CODE BEGIN */
    /* Caller owns I2C_ENABLE. Power the sensors, wait
     * SENSOR_DATA_MIN_POWERUP_DELAY_MS, then call SensorData_Scan() or
     * SensorData_ReadAll() here.
     */
    /* Write the bulk of the application here. */
    /* USER APP CODE END */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection =
      RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  AppSafety_SetWatchdogActive(true);
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
   */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
   */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 15, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) !=
      HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DBG_LED_GPIO_Port, DBG_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WD_DONE_GPIO_Port, WD_DONE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(I2C_ENABLE_GPIO_Port, I2C_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PULSE_IN_Pin */
  GPIO_InitStruct.Pin = PULSE_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PULSE_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DBG_LED_Pin */
  GPIO_InitStruct.Pin = DBG_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DBG_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WD_DONE_Pin */
  GPIO_InitStruct.Pin = WD_DONE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(WD_DONE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2C_ENABLE_Pin */
  GPIO_InitStruct.Pin = I2C_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(I2C_ENABLE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* ES0292 2.1.11: PC13 transitions can disturb the LSE crystal on LQFP
   * packages. PC13 is unused on this board; park it in analog mode with no
   * pull so no noise edge can propagate to the LSE domain. */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  AppSafety_Fatal(APP_FATAL_REASON_ERROR_HANDLER,
                  (uint32_t)(uintptr_t)__builtin_return_address(0));
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
