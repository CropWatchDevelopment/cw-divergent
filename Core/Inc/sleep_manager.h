#ifndef __SLEEP_MANAGER_H
#define __SLEEP_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define SLEEP_MANAGER_WAKE_INTERVAL_S 15U
#define SLEEP_MANAGER_LSE_RETRY_CYCLES 3U

void SleepManager_Init(void);
void SleepManager_SleepUntilWake(void);
void SleepManager_FeedWatchdog(void);
void SleepManager_RTC_IRQHandler(void);
void SleepManager_RTC_MspInit(RTC_HandleTypeDef *hrtc);
void SleepManager_RTC_MspDeInit(RTC_HandleTypeDef *hrtc);

#ifdef __cplusplus
}
#endif

#endif /* __SLEEP_MANAGER_H */
