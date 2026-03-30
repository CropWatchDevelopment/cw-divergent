#ifndef __SLEEP_MANAGER_H
#define __SLEEP_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define SLEEP_MANAGER_WAKE_INTERVAL_S 15U
#define SLEEP_MANAGER_LSE_RETRY_CYCLES 3U
#define SLEEP_MANAGER_MAX_SLEEP_MINUTES 60U

/* Call from `main()` inside `USER CODE BEGIN 2` after CubeMX peripheral init.
 * Pass the I2C handle used by external sensors so the bus can be parked in STOP.
 */
void SleepManager_Init(RTC_HandleTypeDef *rtc, IWDG_HandleTypeDef *iwdg,
                       I2C_HandleTypeDef *i2c);
void SleepManager_SleepUntilWake(uint32_t sleep_minutes);
void SleepManager_FeedWatchdog(void);

/* Call from `RTC_IRQHandler()` inside `USER CODE BEGIN RTC_IRQn 0`. */
void SleepManager_HandleRtcInterrupt(void);

#ifdef __cplusplus
}
#endif

#endif /* __SLEEP_MANAGER_H */
