#include "sleep_manager.h"

#include <stdbool.h>

#define SLEEP_MANAGER_IWDG_RELOAD_MAX        0x0FFFU
#define SLEEP_MANAGER_RTC_ASYNCH_PREDIV      127U
#define SLEEP_MANAGER_RTC_LSE_SYNCH_PREDIV   255U
#define SLEEP_MANAGER_RTC_LSI_SYNCH_PREDIV   ((LSI_VALUE / 128U) - 1U)
#define SLEEP_MANAGER_WAKEUP_COUNTER         (SLEEP_MANAGER_WAKE_INTERVAL_S - 1U)
#define SLEEP_MANAGER_SECONDS_PER_MINUTE     60U
#define SLEEP_MANAGER_WAKE_CYCLES_PER_MINUTE \
  (SLEEP_MANAGER_SECONDS_PER_MINUTE / SLEEP_MANAGER_WAKE_INTERVAL_S)

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
  volatile uint8_t wake_pending;
  volatile uint8_t lse_fault_pending;
  uint8_t lsi_wakeup_cycles;
  uint8_t initialized;
  SleepManagerRtcSource active_rtc_source;
} SleepManagerState;

static SleepManagerState sleep_manager = {0};

static void sleep_manager_prepare_backup_domain(void);
static void sleep_manager_reset_backup_domain(void);
static void sleep_manager_configure_run_clocks(void);
static HAL_StatusTypeDef sleep_manager_set_lse_state(uint32_t lse_state);
static void sleep_manager_clear_lse_css_flags(void);
static void sleep_manager_init_watchdog(void);
static HAL_StatusTypeDef sleep_manager_init_rtc(SleepManagerRtcSource source);
static HAL_StatusTypeDef sleep_manager_select_rtc_clock(SleepManagerRtcSource source);
static HAL_StatusTypeDef sleep_manager_switch_rtc_source(SleepManagerRtcSource source);
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

static void sleep_manager_init_watchdog(void)
{
  sleep_manager.iwdg->Instance = IWDG;
  sleep_manager.iwdg->Init.Prescaler = IWDG_PRESCALER_256;
  sleep_manager.iwdg->Init.Window = IWDG_WINDOW_DISABLE;
  sleep_manager.iwdg->Init.Reload = SLEEP_MANAGER_IWDG_RELOAD_MAX;

  if (HAL_IWDG_Init(sleep_manager.iwdg) != HAL_OK)
  {
    Error_Handler();
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

void SleepManager_Init(RTC_HandleTypeDef *rtc, IWDG_HandleTypeDef *iwdg)
{
  const bool had_prior_lse_css_fault = (__HAL_RCC_GET_FLAG(RCC_FLAG_LSECSS) != RESET);

  if ((rtc == NULL) || (iwdg == NULL))
  {
    Error_Handler();
  }

  sleep_manager.rtc = rtc;
  sleep_manager.iwdg = iwdg;

  sleep_manager.wake_pending = 0U;
  sleep_manager.lse_fault_pending = 0U;
  sleep_manager.lsi_wakeup_cycles = 0U;

  sleep_manager_configure_run_clocks();
  sleep_manager_init_watchdog();
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

    if (sleep_manager.lse_fault_pending != 0U)
    {
      SleepManager_FeedWatchdog();
      sleep_manager_handle_lse_fault();
    }

    sleep_manager.wake_pending = 0U;
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

    HAL_SuspendTick();
    HAL_PWREx_EnableUltraLowPower();
    HAL_PWREx_EnableFastWakeUp();
    HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
    HAL_ResumeTick();

    sleep_manager_configure_run_clocks();
    SleepManager_FeedWatchdog();

    if (sleep_manager.lse_fault_pending != 0U)
    {
      sleep_manager_handle_lse_fault();
      handled_lse_fault = true;
    }

    if (sleep_manager.wake_pending == 0U)
    {
      continue;
    }

    if ((sleep_manager.active_rtc_source == SLEEP_MANAGER_RTC_SOURCE_LSI) &&
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
  uint32_t remaining_cycles;

  if (sleep_minutes == 0U)
  {
    return;
  }

  if (sleep_minutes > SLEEP_MANAGER_MAX_SLEEP_MINUTES)
  {
    Error_Handler();
  }

  remaining_cycles = sleep_minutes * SLEEP_MANAGER_WAKE_CYCLES_PER_MINUTE;
  while (remaining_cycles > 0U)
  {
    sleep_manager_wait_for_next_wake();
    remaining_cycles--;
  }
}

void SleepManager_FeedWatchdog(void)
{
  if (HAL_IWDG_Refresh(sleep_manager.iwdg) != HAL_OK)
  {
    Error_Handler();
  }
}

void SleepManager_HandleRtcInterrupt(void)
{
  HAL_RCCEx_LSECSS_IRQHandler();
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
