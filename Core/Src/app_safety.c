#include "app_safety.h"

#include <string.h>

#include "main.h"

#define APP_SAFETY_MAGIC          0x41504654UL
#define APP_SAFETY_VERSION        0x00000001UL
#define APP_STACK_PATTERN         0xA5U
#define APP_STACK_GUARD_BYTES     64U
#define APP_SRAM_START            0x20000000UL
#define APP_SRAM_END              0x20005000UL

extern uint8_t __StackLimit;
extern uint8_t _estack;

__attribute__((section(".noinit"), used))
static volatile AppFaultRecord g_app_fault_record;

static volatile bool g_watchdog_active = false;

static uint8_t* app_safety_stack_limit(void)
{
  return &__StackLimit;
}

static uint8_t* app_safety_stack_top(void)
{
  return &_estack;
}

static uint32_t app_safety_stack_reserved_bytes(void)
{
  return (uint32_t)(uintptr_t)(app_safety_stack_top() - app_safety_stack_limit());
}

static uint32_t app_safety_stack_scan_unused(void)
{
  uint8_t* cursor = app_safety_stack_limit();
  uint8_t* stack_top = app_safety_stack_top();

  while ((cursor < stack_top) && (*cursor == APP_STACK_PATTERN))
  {
    cursor++;
  }

  return (uint32_t)(uintptr_t)(cursor - app_safety_stack_limit());
}

static bool app_safety_stack_frame_is_valid(const uint32_t* stack_frame)
{
  uintptr_t frame_start = (uintptr_t)stack_frame;
  uintptr_t frame_end = frame_start + (8U * sizeof(uint32_t));

  if (stack_frame == NULL)
  {
    return false;
  }

  return (frame_start >= APP_SRAM_START) && (frame_end <= APP_SRAM_END) &&
         ((frame_start & (sizeof(uint32_t) - 1U)) == 0U);
}

static void app_safety_reset_record(void)
{
  AppFaultRecord reset_record = {0};

  reset_record.magic = APP_SAFETY_MAGIC;
  reset_record.version = APP_SAFETY_VERSION;
  reset_record.min_stack_unused_bytes = app_safety_stack_reserved_bytes();
  g_app_fault_record = reset_record;
}

static void app_safety_capture_fault(AppFatalReason reason, uint32_t sp,
                                     uint32_t lr, uint32_t pc, uint32_t xpsr,
                                     const uint32_t* stack_frame)
{
  g_app_fault_record.fatal_reason = (uint32_t)reason;
  g_app_fault_record.fatal_sp = sp;
  g_app_fault_record.fatal_lr = lr;
  g_app_fault_record.fatal_pc = pc;
  g_app_fault_record.fatal_xpsr = xpsr;

  if (stack_frame != NULL)
  {
    g_app_fault_record.stacked_r0 = stack_frame[0];
    g_app_fault_record.stacked_r1 = stack_frame[1];
    g_app_fault_record.stacked_r2 = stack_frame[2];
    g_app_fault_record.stacked_r3 = stack_frame[3];
    g_app_fault_record.stacked_r12 = stack_frame[4];
  }

  AppSafety_SampleStackWatermark();
}

static void app_safety_request_reset(void) __attribute__((noreturn));

static void app_safety_request_reset(void)
{
  __disable_irq();
  __DSB();
  NVIC_SystemReset();

  for (;;)
  {
    __NOP();
  }
}

void AppSafety_Init(void)
{
  if ((g_app_fault_record.magic != APP_SAFETY_MAGIC) ||
      (g_app_fault_record.version != APP_SAFETY_VERSION))
  {
    app_safety_reset_record();
  }

  g_app_fault_record.boot_count++;
  g_app_fault_record.boot_stage = (uint32_t)APP_BOOT_STAGE_RESET;
}

void AppSafety_SetBootStage(AppBootStage stage)
{
  g_app_fault_record.boot_stage = (uint32_t)stage;
}

void AppSafety_SetResetReasonCode(uint8_t reset_reason_code)
{
  g_app_fault_record.reset_reason_code = (uint32_t)reset_reason_code;
}

void AppSafety_SetWatchdogActive(bool active)
{
  g_watchdog_active = active;
}

bool AppSafety_IsWatchdogActive(void)
{
  return g_watchdog_active;
}

void AppSafety_PaintStack(void)
{
  uint8_t* limit = app_safety_stack_limit();
  uint8_t* current_sp = (uint8_t*)(uintptr_t)__get_MSP();

  if (current_sp > app_safety_stack_top())
  {
    current_sp = app_safety_stack_top();
  }

  if (current_sp < limit)
  {
    current_sp = limit;
  }

  if (((uintptr_t)current_sp - (uintptr_t)limit) > APP_STACK_GUARD_BYTES)
  {
    size_t fill_length =
        (size_t)(((uintptr_t)current_sp - (uintptr_t)limit) -
                 APP_STACK_GUARD_BYTES);
    (void)memset(limit, APP_STACK_PATTERN, fill_length);
  }

  AppSafety_SampleStackWatermark();
}

void AppSafety_SampleStackWatermark(void)
{
  uint32_t unused_bytes = app_safety_stack_scan_unused();

  if ((g_app_fault_record.min_stack_unused_bytes == 0U) ||
      (unused_bytes < g_app_fault_record.min_stack_unused_bytes))
  {
    g_app_fault_record.min_stack_unused_bytes = unused_bytes;
  }
}

uint32_t AppSafety_GetMinStackUnusedBytes(void)
{
  return g_app_fault_record.min_stack_unused_bytes;
}

const AppFaultRecord* AppSafety_GetFaultRecord(void)
{
  return (const AppFaultRecord*)&g_app_fault_record;
}

void AppSafety_Fatal(AppFatalReason reason, uint32_t return_address)
{
  uint32_t sp = __get_MSP();

  app_safety_capture_fault(reason, sp, 0U, return_address, 0U, NULL);
  app_safety_request_reset();
}

void AppSafety_FatalSpuriousIrq(void)
{
  AppSafety_Fatal(APP_FATAL_REASON_SPURIOUS_IRQ, 0U);
}

void AppSafety_HandleNmi(uint32_t* stack_frame)
{
  uint32_t sp = (uint32_t)(uintptr_t)stack_frame;
  const uint32_t* valid_stack_frame = NULL;
  uint32_t lr = 0U;
  uint32_t pc = 0U;
  uint32_t xpsr = 0U;

  if (app_safety_stack_frame_is_valid(stack_frame))
  {
    valid_stack_frame = stack_frame;
    lr = stack_frame[5];
    pc = stack_frame[6];
    xpsr = stack_frame[7];
  }

  app_safety_capture_fault(APP_FATAL_REASON_NMI, sp, lr, pc, xpsr,
                           valid_stack_frame);
  app_safety_request_reset();
}

void AppSafety_HandleHardFault(uint32_t* stack_frame)
{
  uint32_t sp = (uint32_t)(uintptr_t)stack_frame;
  const uint32_t* valid_stack_frame = NULL;
  uint32_t lr = 0U;
  uint32_t pc = 0U;
  uint32_t xpsr = 0U;

  if (app_safety_stack_frame_is_valid(stack_frame))
  {
    valid_stack_frame = stack_frame;
    lr = stack_frame[5];
    pc = stack_frame[6];
    xpsr = stack_frame[7];
  }

  app_safety_capture_fault(APP_FATAL_REASON_HARDFAULT, sp, lr, pc, xpsr,
                           valid_stack_frame);
  app_safety_request_reset();
}
