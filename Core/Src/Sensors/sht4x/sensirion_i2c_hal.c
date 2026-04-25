#include "sensirion_i2c_hal.h"

#include "sensirion_common.h"
#include "sensirion_i2c.h"

#define SENSIRION_I2C_HAL_TIMEOUT_MS 100U

static I2C_HandleTypeDef* sensirion_i2c_handle = NULL;

static int8_t sensirion_i2c_hal_translate_status(HAL_StatusTypeDef status) {
    uint32_t hal_error;

    if (status == HAL_OK) {
        return NO_ERROR;
    }

    if (sensirion_i2c_handle == NULL) {
        return I2C_BUS_ERROR;
    }

    hal_error = HAL_I2C_GetError(sensirion_i2c_handle);
    if ((hal_error & HAL_I2C_ERROR_AF) != 0U) {
        return I2C_NACK_ERROR;
    }

    return I2C_BUS_ERROR;
}

void sensirion_i2c_hal_set_handle(I2C_HandleTypeDef* hi2c) {
    sensirion_i2c_handle = hi2c;
}

int16_t sensirion_i2c_hal_select_bus(uint8_t bus_idx) {
    if (bus_idx == 0U) {
        return NO_ERROR;
    }

    return NOT_IMPLEMENTED_ERROR;
}

void sensirion_i2c_hal_init(void) {
}

void sensirion_i2c_hal_free(void) {
}

int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint8_t count) {
    HAL_StatusTypeDef status;

    if ((sensirion_i2c_handle == NULL) || (data == NULL)) {
        return I2C_BUS_ERROR;
    }

    status = HAL_I2C_Master_Receive(sensirion_i2c_handle, (uint16_t)(address << 1),
                                    data, count, SENSIRION_I2C_HAL_TIMEOUT_MS);
    return sensirion_i2c_hal_translate_status(status);
}

int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data,
                               uint8_t count) {
    HAL_StatusTypeDef status;

    if ((sensirion_i2c_handle == NULL) || (data == NULL)) {
        return I2C_BUS_ERROR;
    }

    status = HAL_I2C_Master_Transmit(sensirion_i2c_handle, (uint16_t)(address << 1),
                                     (uint8_t*)data, count,
                                     SENSIRION_I2C_HAL_TIMEOUT_MS);
    return sensirion_i2c_hal_translate_status(status);
}

/* Cycle-counted busy delay independent of SysTick.
 *
 * The previous implementation polled HAL_GetTick() with a "hard limit" guard
 * that itself was measured in HAL_GetTick() — so a stalled SysTick would
 * hang both the loop body and the escape clause. This version runs a fixed
 * inline-asm subs/bne loop whose iteration count is derived from
 * SystemCoreClock, so the call is naturally bounded even if the HAL tick
 * source is dead.
 *
 * Calibration: subs+bne is ~3 cycles/iteration on Cortex-M0+ in steady
 * state. We divide by 3000 (cycles_per_ms / cycles_per_iter) which rounds
 * the actual elapsed time slightly LONG. That direction is safe — the
 * SHT4x measurement spec is a minimum hold-off, not an exact deadline.
 */
static void sensirion_i2c_hal_busy_delay_ms(uint32_t milliseconds) {
    uint32_t iterations;

    if (milliseconds == 0U) {
        return;
    }

    iterations = (SystemCoreClock / 3000U) * milliseconds;
    if (iterations == 0U) {
        iterations = 1U;
    }

    /* "+l" constrains the loop counter to a low register (r0-r7); the
     * 3-operand subs form is the Thumb-1 encoding accepted by Cortex-M0+.
     * `.syntax unified` is required so GAS emits the 16-bit Thumb encoding
     * rather than rejecting the mnemonic as Thumb-2 only. */
    __asm__ volatile (
        ".syntax unified             \n"
        "1:                          \n"
        "    subs %[i], %[i], #1     \n"
        "    bne 1b                  \n"
        : [i] "+l" (iterations)
        :
        : "cc"
    );
}

void sensirion_i2c_hal_sleep_usec(uint32_t useconds) {
    uint32_t delay_ms;

    if (useconds == 0U) {
        return;
    }

    delay_ms = useconds / 1000U;
    if ((useconds % 1000U) != 0U) {
        delay_ms++;
    }

    sensirion_i2c_hal_busy_delay_ms(delay_ms);
}
