#ifndef __SLEEP_MANAGER_H
#define __SLEEP_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "main.h"

#define SLEEP_MANAGER_WAKE_INTERVAL_S 10U
/* Initial cadence for retrying LSE after falling back to LSI. Counted in
 * wake cycles, so 30 * 10s = 5 min between the first few attempts. Repeated
 * failures double this up to SLEEP_MANAGER_LSE_RETRY_BACKOFF_MAX_CYCLES so a
 * dead crystal doesn't burn power retrying every wake. */
#define SLEEP_MANAGER_LSE_RETRY_CYCLES 30U
#define SLEEP_MANAGER_LSE_RETRY_BACKOFF_MAX_CYCLES 360U
#define SLEEP_MANAGER_MAX_SLEEP_MINUTES 60U

/* Call from `main()` inside `USER CODE BEGIN 2` after CubeMX peripheral init.
 * Pass the I2C handle used by external sensors so the bus can be parked in STOP.
 */
void SleepManager_Init(RTC_HandleTypeDef *rtc, IWDG_HandleTypeDef *iwdg,
                       I2C_HandleTypeDef *i2c);
void SleepManager_SleepUntilWake(uint32_t sleep_minutes);
void SleepManager_FeedWatchdog(void);
bool SleepManager_IsStopWakeInProgress(void);

/* Call from `RTC_IRQHandler()` inside `USER CODE BEGIN RTC_IRQn 0`. */
void SleepManager_HandleRtcInterrupt(void);

/* Call from a GPIO EXTI callback/IRQ that should wake the application from STOP. */
void SleepManager_HandleExternalWake(void);

#ifdef __cplusplus
}
#endif

#endif /* __SLEEP_MANAGER_H */
