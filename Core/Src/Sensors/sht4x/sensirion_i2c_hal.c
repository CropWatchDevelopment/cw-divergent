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

void sensirion_i2c_hal_sleep_usec(uint32_t useconds) {
    uint32_t delay_ms;
    uint32_t start_tick;
    uint32_t hard_limit_ms;

    if (useconds == 0U) {
        return;
    }

    delay_ms = useconds / 1000U;
    if ((useconds % 1000U) != 0U) {
        delay_ms++;
    }

    /* Guard: if SysTick is stalled HAL_Delay never returns.
     * Poll HAL_GetTick and bail out after 2x the requested time. */
    hard_limit_ms = delay_ms * 2U;
    if (hard_limit_ms < 10U) {
        hard_limit_ms = 10U;
    }

    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < delay_ms) {
        if ((HAL_GetTick() - start_tick) >= hard_limit_ms) {
            break;  /* SysTick may be stalled — do not hang */
        }
    }
}
