#ifndef APP_SAFETY_H
#define APP_SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  APP_BOOT_STAGE_RESET = 0U,
  APP_BOOT_STAGE_HAL_INITIALIZED,
  APP_BOOT_STAGE_WATCHDOG_STARTED,
  APP_BOOT_STAGE_CLOCKS_CONFIGURED,
  APP_BOOT_STAGE_GPIO_INITIALIZED,
  APP_BOOT_STAGE_I2C_INITIALIZED,
  APP_BOOT_STAGE_UART_INITIALIZED,
  APP_BOOT_STAGE_SLEEP_MANAGER_INITIALIZED,
  APP_BOOT_STAGE_APPLICATION_READY,
  APP_BOOT_STAGE_STOP_MODE,
  APP_BOOT_STAGE_WAKE_RESUME
} AppBootStage;

typedef enum
{
  APP_FATAL_REASON_NONE = 0U,
  APP_FATAL_REASON_ERROR_HANDLER,
  APP_FATAL_REASON_NMI,
  APP_FATAL_REASON_HARDFAULT,
  APP_FATAL_REASON_EXIT,
  APP_FATAL_REASON_I2C_RECOVERY_FAILED
} AppFatalReason;

typedef struct
{
  uint32_t magic;
  uint32_t version;
  uint32_t boot_count;
  uint32_t reset_reason_code;
  uint32_t boot_stage;
  uint32_t fatal_reason;
  uint32_t fatal_sp;
  uint32_t fatal_lr;
  uint32_t fatal_pc;
  uint32_t fatal_xpsr;
  uint32_t stacked_r0;
  uint32_t stacked_r1;
  uint32_t stacked_r2;
  uint32_t stacked_r3;
  uint32_t stacked_r12;
  uint32_t min_stack_unused_bytes;
} AppFaultRecord;

void AppSafety_Init(void);
void AppSafety_SetBootStage(AppBootStage stage);
void AppSafety_SetResetReasonCode(uint8_t reset_reason_code);
void AppSafety_SetWatchdogActive(bool active);
bool AppSafety_IsWatchdogActive(void);

void AppSafety_PaintStack(void);
void AppSafety_SampleStackWatermark(void);
uint32_t AppSafety_GetMinStackUnusedBytes(void);
const AppFaultRecord* AppSafety_GetFaultRecord(void);

void AppSafety_Fatal(AppFatalReason reason, uint32_t return_address)
    __attribute__((noreturn));
void AppSafety_HandleNmi(uint32_t* stack_frame) __attribute__((noreturn));
void AppSafety_HandleHardFault(uint32_t* stack_frame) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

#endif /* APP_SAFETY_H */
