#ifndef __SLEEP_MANAGER_H
#define __SLEEP_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define SLEEP_MANAGER_WAKE_INTERVAL_S 15U
#define SLEEP_MANAGER_LSE_RETRY_CYCLES 3U
#define SLEEP_MANAGER_MAX_SLEEP_MINUTES 60U

/* Call from `main()` inside `USER CODE BEGIN 2` after CubeMX peripheral init. */
void SleepManager_Init(RTC_HandleTypeDef *rtc, IWDG_HandleTypeDef *iwdg);
void SleepManager_SleepUntilWake(uint32_t sleep_minutes);
void SleepManager_FeedWatchdog(void);

/* Call from `RTC_IRQHandler()` inside `USER CODE BEGIN RTC_IRQn 0`. */
void SleepManager_HandleRtcInterrupt(void);

#ifdef __cplusplus
}
#endif

#endif /* __SLEEP_MANAGER_H */
