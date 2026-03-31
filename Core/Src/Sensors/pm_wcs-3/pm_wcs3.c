#include "pm_wcs3.h"

#include <stddef.h>

/*
 * Integer-only STM32 port of the Tinovi PM-WCS-3 Arduino protocol.
 * Reference implementation and register map: https://github.com/tinovi/i2cArduino
 * Tinovi i2cArduino is distributed under the MIT License.
 */

#define PM_WCS3_I2C_TIMEOUT_MS     100U
#define PM_WCS3_CONVERSION_DELAY_MS 300U
#define PM_WCS3_REG_READ_START     0x01U
#define PM_WCS3_REG_GET_DATA       0x09U
#define PM_WCS3_RAW_DATA_SIZE      8U
#define PM_WCS3_READY_TRIALS       2U

typedef struct {
    I2C_HandleTypeDef* i2c;
    uint8_t address;
} PMWCS3Context;

static PMWCS3Context pm_wcs3_context = {0};

static uint16_t pm_wcs3_u16_from_le(const uint8_t* data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t pm_wcs3_i16_from_le(const uint8_t* data)
{
    return (int16_t)pm_wcs3_u16_from_le(data);
}

static int16_t pm_wcs3_write_register(uint8_t reg)
{
    HAL_StatusTypeDef status;

    if (pm_wcs3_context.i2c == NULL)
    {
        return PM_WCS3_STATUS_NOT_INITIALIZED;
    }

    status = HAL_I2C_Master_Transmit(pm_wcs3_context.i2c,
                                     (uint16_t)(pm_wcs3_context.address << 1),
                                     &reg, 1U, PM_WCS3_I2C_TIMEOUT_MS);
    return (status == HAL_OK) ? PM_WCS3_STATUS_OK : PM_WCS3_STATUS_WRITE_FAILED;
}

void PMWCS3_Init(I2C_HandleTypeDef* hi2c, uint8_t address)
{
    pm_wcs3_context.i2c = hi2c;
    pm_wcs3_context.address = address;
}

int16_t PMWCS3_Probe(void)
{
    HAL_StatusTypeDef status;

    if (pm_wcs3_context.i2c == NULL)
    {
        return PM_WCS3_STATUS_NOT_INITIALIZED;
    }

    status = HAL_I2C_IsDeviceReady(pm_wcs3_context.i2c,
                                   (uint16_t)(pm_wcs3_context.address << 1),
                                   PM_WCS3_READY_TRIALS,
                                   PM_WCS3_I2C_TIMEOUT_MS);
    return (status == HAL_OK) ? PM_WCS3_STATUS_OK : PM_WCS3_STATUS_NOT_READY;
}

int16_t PMWCS3_Read(PMWCS3Reading* reading)
{
    HAL_StatusTypeDef status;
    uint8_t raw_data[PM_WCS3_RAW_DATA_SIZE] = {0};

    if (reading == NULL)
    {
        return PM_WCS3_STATUS_INVALID_ARG;
    }

    if (pm_wcs3_context.i2c == NULL)
    {
        return PM_WCS3_STATUS_NOT_INITIALIZED;
    }

    if (pm_wcs3_write_register(PM_WCS3_REG_READ_START) != PM_WCS3_STATUS_OK)
    {
        return PM_WCS3_STATUS_WRITE_FAILED;
    }

    HAL_Delay(PM_WCS3_CONVERSION_DELAY_MS);

    if (pm_wcs3_write_register(PM_WCS3_REG_GET_DATA) != PM_WCS3_STATUS_OK)
    {
        return PM_WCS3_STATUS_WRITE_FAILED;
    }

    status = HAL_I2C_Master_Receive(pm_wcs3_context.i2c,
                                    (uint16_t)(pm_wcs3_context.address << 1),
                                    raw_data, PM_WCS3_RAW_DATA_SIZE,
                                    PM_WCS3_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return PM_WCS3_STATUS_READ_FAILED;
    }

    reading->e25_x100 = pm_wcs3_i16_from_le(&raw_data[0]);
    reading->ec_uS_cm = pm_wcs3_u16_from_le(&raw_data[2]);
    reading->temperature_c_x100 = pm_wcs3_i16_from_le(&raw_data[4]);
    reading->vwc_pct_x10 = pm_wcs3_u16_from_le(&raw_data[6]);

    return PM_WCS3_STATUS_OK;
}
