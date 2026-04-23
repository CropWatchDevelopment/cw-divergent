#include "sleep_manager.h"

#include <stdbool.h>

#include "app_safety.h"

#define SLEEP_MANAGER_IWDG_RELOAD_MAX        0x0FFFU
#define SLEEP_MANAGER_RTC_ASYNCH_PREDIV      127U
#define SLEEP_MANAGER_RTC_LSE_SYNCH_PREDIV   255U
#define SLEEP_MANAGER_RTC_LSI_SYNCH_PREDIV   ((LSI_VALUE / 128U) - 1U)
#define SLEEP_MANAGER_WAKEUP_COUNTER         (SLEEP_MANAGER_WAKE_INTERVAL_S - 1U)
#define SLEEP_MANAGER_SECONDS_PER_MINUTE     60U
#define SLEEP_MANAGER_WAKE_CYCLES_PER_MINUTE \
  (SLEEP_MANAGER_SECONDS_PER_MINUTE / SLEEP_MANAGER_WAKE_INTERVAL_S)
#define SLEEP_MANAGER_GPIOA_KEEP_MASK        (DBG_LED_Pin | PULSE_IN_Pin | GPIO_PIN_13 | GPIO_PIN_14)
#define SLEEP_MANAGER_GPIOB_KEEP_MASK        I2C_ENABLE_Pin
#define SLEEP_MANAGER_GPIOC_KEEP_MASK        (WD_DONE_Pin | GPIO_PIN_14 | GPIO_PIN_15)

#if ((SLEEP_MANAGER_SECONDS_PER_MINUTE % SLEEP_MANAGER_WAKE_INTERVAL_S) != 0U)
#error "SLEEP_MANAGER_WAKE_INTERVAL_S must divide evenly into 60 seconds."
#endif

typedef enum
{
  SLEEP_MANAGER_RTC_SOURCE_LSE = 0U,
  SLEEP_MANAGER_RTC_SOURCE_LSI
} SleepManagerRtcSource;

typedef struct
{
  RTC_HandleTypeDef *rtc;
  IWDG_HandleTypeDef *iwdg;
  I2C_HandleTypeDef *i2c;
  volatile uint8_t wake_pending;
  volatile uint8_t external_wake_pending;
  volatile uint8_t lse_fault_pending;
  volatile uint8_t stop_active;
  uint8_t lsi_wakeup_cycles;
  uint8_t initialized;
  SleepManagerRtcSource active_rtc_source;
  uint32_t cadence_cycles_total;
  uint32_t cadence_cycles_remaining;
} SleepManagerState;

static SleepManagerState sleep_manager = {0};

static void sleep_manager_prepare_backup_domain(void);
static void sleep_manager_reset_backup_domain(void);
static void sleep_manager_configure_run_clocks(void);
static HAL_StatusTypeDef sleep_manager_set_lse_state(uint32_t lse_state);
static void sleep_manager_clear_lse_css_flags(void);
static void sleep_manager_enable_gpio_clocks(void);
static void sleep_manager_disable_gpio_clocks(void);
static void sleep_manager_configure_output(GPIO_TypeDef *port, uint16_t pin,
                                           GPIO_PinState state, uint32_t pull,
                                           uint32_t speed);
static void sleep_manager_configure_analog(GPIO_TypeDef *port, uint16_t pins);
static void sleep_manager_prepare_board_for_sleep(void);
static void sleep_manager_restore_board_after_sleep(void);
static HAL_StatusTypeDef sleep_manager_init_rtc(SleepManagerRtcSource source);
static HAL_StatusTypeDef sleep_manager_select_rtc_clock(SleepManagerRtcSource source);
static HAL_StatusTypeDef sleep_manager_switch_rtc_source(SleepManagerRtcSource source);
static HAL_StatusTypeDef sleep_manager_restore_i2c_after_sleep(void);
static void sleep_manager_recover_i2c_bus(void);
static void sleep_manager_handle_lse_fault(void);
static void sleep_manager_try_restore_lse(void);
static void sleep_manager_wait_for_next_wake(void);

static void sleep_manager_prepare_backup_domain(void)
{
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
}

static void sleep_manager_reset_backup_domain(void)
{
  sleep_manager_prepare_backup_domain();
  __HAL_RCC_BACKUPRESET_FORCE();
  __HAL_RCC_BACKUPRESET_RELEASE();
}

static void sleep_manager_configure_run_clocks(void)
{
  RCC_OscInitTypeDef osc_init = {0};
  RCC_ClkInitTypeDef clk_init = {0};
  RCC_PeriphCLKInitTypeDef periph_clk_init = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  osc_init.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_MSI |
                            RCC_OSCILLATORTYPE_HSI;
  osc_init.LSIState = RCC_LSI_ON;
  osc_init.MSIState = RCC_MSI_ON;
  osc_init.HSIState = RCC_HSI_ON;
  osc_init.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc_init.MSICalibrationValue = 0;
  osc_init.MSIClockRange = RCC_MSIRANGE_5;
  osc_init.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&osc_init) != HAL_OK)
  {
    Error_Handler();
  }

  clk_init.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk_init.APB1CLKDivider = RCC_HCLK_DIV1;
  clk_init.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  periph_clk_init.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  /* ES0292 2.12.3 workaround: keep I2C1 kernel clock on HSI16 (> 4 MHz). */
  periph_clk_init.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&periph_clk_init) != HAL_OK)
  {
    Error_Handler();
  }
}

static HAL_StatusTypeDef sleep_manager_set_lse_state(uint32_t lse_state)
{
  RCC_OscInitTypeDef osc_init = {0};

  sleep_manager_prepare_backup_domain();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  osc_init.OscillatorType = RCC_OSCILLATORTYPE_LSE;
  osc_init.LSEState = lse_state;
  osc_init.PLL.PLLState = RCC_PLL_NONE;

  return HAL_RCC_OscConfig(&osc_init);
}

static void sleep_manager_clear_lse_css_flags(void)
{
  __HAL_RCC_CLEAR_IT(RCC_IT_LSECSS);
  __HAL_RCC_LSECSS_EXTI_CLEAR_FLAG();
}

static void sleep_manager_enable_gpio_clocks(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
#ifdef GPIOH
  __HAL_RCC_GPIOH_CLK_ENABLE();
#endif
}

static void sleep_manager_disable_gpio_clocks(void)
{
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
#ifdef GPIOH
  __HAL_RCC_GPIOH_CLK_DISABLE();
#endif
}

static void sleep_manager_configure_output(GPIO_TypeDef *port, uint16_t pin,
                                           GPIO_PinState state, uint32_t pull,
                                           uint32_t speed)
{
  GPIO_InitTypeDef gpio_init = {0};

  HAL_GPIO_WritePin(port, pin, state);

  gpio_init.Pin = pin;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = pull;
  gpio_init.Speed = speed;
  HAL_GPIO_Init(port, &gpio_init);
}

static void sleep_manager_configure_analog(GPIO_TypeDef *port, uint16_t pins)
{
  GPIO_InitTypeDef gpio_init = {0};

  if (pins == 0U)
  {
    return;
  }

  gpio_init.Pin = pins;
  gpio_init.Mode = GPIO_MODE_ANALOG;
  gpio_init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(port, &gpio_init);
}

static void sleep_manager_prepare_board_for_sleep(void)
{
  sleep_manager_enable_gpio_clocks();

  sleep_manager_configure_output(DBG_LED_GPIO_Port, DBG_LED_Pin, GPIO_PIN_RESET,
                                 GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
  sleep_manager_configure_output(WD_DONE_GPIO_Port, WD_DONE_Pin, GPIO_PIN_RESET,
                                 GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
  sleep_manager_configure_output(I2C_ENABLE_GPIO_Port, I2C_ENABLE_Pin, GPIO_PIN_RESET,
                                 GPIO_PULLDOWN, GPIO_SPEED_FREQ_LOW);

  if ((sleep_manager.i2c != NULL) &&
      (sleep_manager.i2c->State != HAL_I2C_STATE_RESET))
  {
    if (HAL_I2C_DeInit(sleep_manager.i2c) != HAL_OK)
    {
      Error_Handler();
    }
  }

  /* Keep only the board pins that must retain a defined state during STOP. */
  sleep_manager_configure_analog(GPIOA,
                                 (uint16_t)(GPIO_PIN_All & ~SLEEP_MANAGER_GPIOA_KEEP_MASK));
  sleep_manager_configure_analog(GPIOB,
                                 (uint16_t)(GPIO_PIN_All & ~SLEEP_MANAGER_GPIOB_KEEP_MASK));
  sleep_manager_configure_analog(GPIOC,
                                 (uint16_t)(GPIO_PIN_All & ~SLEEP_MANAGER_GPIOC_KEEP_MASK));
#ifdef GPIOH
  sleep_manager_configure_analog(GPIOH, GPIO_PIN_All);
#endif

  sleep_manager_disable_gpio_clocks();
}

static void sleep_manager_restore_board_after_sleep(void)
{
  sleep_manager_enable_gpio_clocks();

  sleep_manager_configure_output(DBG_LED_GPIO_Port, DBG_LED_Pin, GPIO_PIN_RESET,
                                 GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
  sleep_manager_configure_output(WD_DONE_GPIO_Port, WD_DONE_Pin, GPIO_PIN_RESET,
                                 GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM);
  sleep_manager_configure_output(I2C_ENABLE_GPIO_Port, I2C_ENABLE_Pin, GPIO_PIN_RESET,
                                 GPIO_PULLDOWN, GPIO_SPEED_FREQ_HIGH);

  if (sleep_manager.i2c != NULL)
  {
    if (sleep_manager_restore_i2c_after_sleep() != HAL_OK)
    {
      AppSafety_Fatal(APP_FATAL_REASON_I2C_RECOVERY_FAILED, 0U);
    }
  }
}

static HAL_StatusTypeDef sleep_manager_init_rtc(SleepManagerRtcSource source)
{
  RTC_HandleTypeDef *rtc = sleep_manager.rtc;

  rtc->Instance = RTC;
  rtc->Init.HourFormat = RTC_HOURFORMAT_24;
  rtc->Init.AsynchPrediv = SLEEP_MANAGER_RTC_ASYNCH_PREDIV;
  rtc->Init.SynchPrediv = (source == SLEEP_MANAGER_RTC_SOURCE_LSE) ?
                          SLEEP_MANAGER_RTC_LSE_SYNCH_PREDIV :
                          SLEEP_MANAGER_RTC_LSI_SYNCH_PREDIV;
  rtc->Init.OutPut = RTC_OUTPUT_DISABLE;
  rtc->Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  rtc->Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  rtc->Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

  if (HAL_RTC_Init(rtc) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_RTCEx_SetWakeUpTimer_IT(rtc, SLEEP_MANAGER_WAKEUP_COUNTER,
                                  RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef sleep_manager_select_rtc_clock(SleepManagerRtcSource source)
{
  RCC_PeriphCLKInitTypeDef periph_clk_init = {0};

  sleep_manager_prepare_backup_domain();

  periph_clk_init.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  periph_clk_init.RTCClockSelection = (source == SLEEP_MANAGER_RTC_SOURCE_LSE) ?
                                      RCC_RTCCLKSOURCE_LSE :
                                      RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&periph_clk_init) != HAL_OK)
  {
    return HAL_ERROR;
  }

  __HAL_RCC_RTC_ENABLE();

  if (sleep_manager_init_rtc(source) != HAL_OK)
  {
    return HAL_ERROR;
  }

  sleep_manager_clear_lse_css_flags();
  if (source == SLEEP_MANAGER_RTC_SOURCE_LSE)
  {
    HAL_RCCEx_EnableLSECSS_IT();
  }

  sleep_manager.active_rtc_source = source;
  return HAL_OK;
}

static HAL_StatusTypeDef sleep_manager_switch_rtc_source(SleepManagerRtcSource source)
{
  if (source == SLEEP_MANAGER_RTC_SOURCE_LSE)
  {
    if (sleep_manager_set_lse_state(RCC_LSE_ON) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }
  }
  else
  {
    if (sleep_manager_set_lse_state(RCC_LSE_OFF) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return sleep_manager_select_rtc_clock(source);
}

static void sleep_manager_handle_lse_fault(void)
{
  sleep_manager.lse_fault_pending = 0U;
  sleep_manager.lsi_wakeup_cycles = 0U;

  HAL_RCCEx_DisableLSECSS();
  sleep_manager_clear_lse_css_flags();

  if (sleep_manager_switch_rtc_source(SLEEP_MANAGER_RTC_SOURCE_LSI) != HAL_OK)
  {
    Error_Handler();
  }
}

static void sleep_manager_try_restore_lse(void)
{
  HAL_StatusTypeDef status;

  status = sleep_manager_switch_rtc_source(SLEEP_MANAGER_RTC_SOURCE_LSE);
  if (status == HAL_OK)
  {
    sleep_manager.lsi_wakeup_cycles = 0U;
    return;
  }

  sleep_manager.lsi_wakeup_cycles = 0U;
  sleep_manager_clear_lse_css_flags();

  if (sleep_manager_set_lse_state(RCC_LSE_OFF) != HAL_OK)
  {
    Error_Handler();
  }
}

void SleepManager_Init(RTC_HandleTypeDef *rtc, IWDG_HandleTypeDef *iwdg,
                       I2C_HandleTypeDef *i2c)
{
  const bool had_prior_lse_css_fault = (__HAL_RCC_GET_FLAG(RCC_FLAG_LSECSS) != RESET);

  if ((rtc == NULL) || (iwdg == NULL))
  {
    Error_Handler();
  }

  sleep_manager.rtc = rtc;
  sleep_manager.iwdg = iwdg;
  sleep_manager.i2c = i2c;

  sleep_manager.wake_pending = 0U;
  sleep_manager.external_wake_pending = 0U;
  sleep_manager.lse_fault_pending = 0U;
  sleep_manager.stop_active = 0U;
  sleep_manager.lsi_wakeup_cycles = 0U;
  sleep_manager.cadence_cycles_total = 0U;
  sleep_manager.cadence_cycles_remaining = 0U;

  sleep_manager_configure_run_clocks();
  if (!AppSafety_IsWatchdogActive())
  {
    Error_Handler();
  }
  SleepManager_FeedWatchdog();

  sleep_manager_reset_backup_domain();

  if (had_prior_lse_css_fault)
  {
    if (sleep_manager_switch_rtc_source(SLEEP_MANAGER_RTC_SOURCE_LSI) != HAL_OK)
    {
      Error_Handler();
    }
  }
  else if (sleep_manager_switch_rtc_source(SLEEP_MANAGER_RTC_SOURCE_LSE) != HAL_OK)
  {
    if (sleep_manager_switch_rtc_source(SLEEP_MANAGER_RTC_SOURCE_LSI) != HAL_OK)
    {
      Error_Handler();
    }
  }

  sleep_manager.initialized = 1U;
}

static void sleep_manager_wait_for_next_wake(void)
{
  if (sleep_manager.initialized == 0U)
  {
    Error_Handler();
  }

  for (;;)
  {
    bool handled_lse_fault = false;
    bool woke_from_external = false;

    if (sleep_manager.lse_fault_pending != 0U)
    {
      SleepManager_FeedWatchdog();
      sleep_manager_handle_lse_fault();
    }

	    sleep_manager.wake_pending = 0U;
	    sleep_manager.external_wake_pending = 0U;
	    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

	    sleep_manager.stop_active = 1U;
	    AppSafety_SetBootStage(APP_BOOT_STAGE_STOP_MODE);
	    /* Refresh IWDG just before STOP so the full IWDG window covers
	     * (STOP duration + post-wake clock restore + next feed). */
	    SleepManager_FeedWatchdog();
	    HAL_SuspendTick();
	    HAL_PWREx_EnableUltraLowPower();
	    HAL_PWREx_EnableFastWakeUp();
	    HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
	    HAL_ResumeTick();

	    sleep_manager_configure_run_clocks();
	    sleep_manager.stop_active = 0U;
	    AppSafety_SetBootStage(APP_BOOT_STAGE_WAKE_RESUME);
	    SleepManager_FeedWatchdog();
	    AppSafety_SampleStackWatermark();

    if (sleep_manager.lse_fault_pending != 0U)
    {
      sleep_manager_handle_lse_fault();
      handled_lse_fault = true;
    }

    if ((sleep_manager.wake_pending == 0U) &&
        (sleep_manager.external_wake_pending == 0U))
    {
      continue;
    }

    woke_from_external = (sleep_manager.external_wake_pending != 0U);

    if ((woke_from_external == false) &&
        (sleep_manager.active_rtc_source == SLEEP_MANAGER_RTC_SOURCE_LSI) &&
        (handled_lse_fault == false))
    {
      sleep_manager.lsi_wakeup_cycles++;
      if (sleep_manager.lsi_wakeup_cycles >= SLEEP_MANAGER_LSE_RETRY_CYCLES)
      {
        sleep_manager_try_restore_lse();
      }
    }
    else if (sleep_manager.active_rtc_source == SLEEP_MANAGER_RTC_SOURCE_LSE)
    {
      sleep_manager.lsi_wakeup_cycles = 0U;
    }

    return;
  }
}

void SleepManager_SleepUntilWake(uint32_t sleep_minutes)
{
  uint32_t requested_cycles;

  if (sleep_minutes == 0U)
  {
    return;
  }

  if (sleep_minutes > SLEEP_MANAGER_MAX_SLEEP_MINUTES)
  {
    Error_Handler();
  }

  requested_cycles = sleep_minutes * SLEEP_MANAGER_WAKE_CYCLES_PER_MINUTE;
  if ((sleep_manager.cadence_cycles_total != requested_cycles) ||
      (sleep_manager.cadence_cycles_remaining == 0U))
  {
    sleep_manager.cadence_cycles_total = requested_cycles;
    sleep_manager.cadence_cycles_remaining = requested_cycles;
  }

  sleep_manager_prepare_board_for_sleep();
  while (sleep_manager.cadence_cycles_remaining > 0U)
  {
    sleep_manager_wait_for_next_wake();
    if (sleep_manager.external_wake_pending != 0U)
    {
      sleep_manager.external_wake_pending = 0U;
      break;
    }
    sleep_manager.cadence_cycles_remaining--;
  }
  sleep_manager_restore_board_after_sleep();
}

void SleepManager_FeedWatchdog(void)
{
  if (HAL_IWDG_Refresh(sleep_manager.iwdg) != HAL_OK)
  {
    Error_Handler();
  }
}

bool SleepManager_IsStopWakeInProgress(void)
{
  return (sleep_manager.stop_active != 0U);
}

void SleepManager_HandleRtcInterrupt(void)
{
  HAL_RCCEx_LSECSS_IRQHandler();
}

void SleepManager_HandleExternalWake(void)
{
  sleep_manager.external_wake_pending = 1U;
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  if (hrtc == sleep_manager.rtc)
  {
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    sleep_manager.wake_pending = 1U;
  }
}

void HAL_RCCEx_LSECSS_Callback(void)
{
  sleep_manager.lse_fault_pending = 1U;
}

static HAL_StatusTypeDef sleep_manager_restore_i2c_after_sleep(void)
{
  if (HAL_I2C_Init(sleep_manager.i2c) == HAL_OK)
  {
    if ((HAL_I2CEx_ConfigAnalogFilter(sleep_manager.i2c,
                                      I2C_ANALOGFILTER_ENABLE) == HAL_OK) &&
        (HAL_I2CEx_ConfigDigitalFilter(sleep_manager.i2c, 0U) == HAL_OK))
    {
      return HAL_OK;
    }
  }

  (void)HAL_I2C_DeInit(sleep_manager.i2c);
  sleep_manager_recover_i2c_bus();

  if (HAL_I2C_Init(sleep_manager.i2c) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_I2CEx_ConfigAnalogFilter(sleep_manager.i2c,
                                   I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_I2CEx_ConfigDigitalFilter(sleep_manager.i2c, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static void sleep_manager_recover_i2c_bus(void)
{
  GPIO_InitTypeDef gpio_init = {0};
  uint32_t pulse_index;

  sleep_manager_enable_gpio_clocks();

  gpio_init.Pin = I2C1_SCL_Pin | I2C1_SDA_Pin;
  gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I2C1_SCL_GPIO_Port, &gpio_init);

  HAL_GPIO_WritePin(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_SET);
  HAL_Delay(1U);

  for (pulse_index = 0U; pulse_index < 9U; ++pulse_index)
  {
    HAL_GPIO_WritePin(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_SET);
    HAL_Delay(1U);

    if (HAL_GPIO_ReadPin(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin) == GPIO_PIN_SET)
    {
      break;
    }
  }

  HAL_GPIO_WritePin(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_RESET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_SET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_SET);
  HAL_Delay(1U);
}
