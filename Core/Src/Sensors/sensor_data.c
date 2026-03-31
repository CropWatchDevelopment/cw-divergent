/*
 * sensor_data.c
 *
 *  Created on: Mar 30, 2026
 *      Author: kevin
 */

#include "sensor_data.h"

#include <string.h>

#include "pm_wcs-3/pm_wcs3.h"
#include "sht4x/sensirion_common.h"
#include "sht4x/sensirion_i2c.h"
#include "sht4x/sensirion_i2c_hal.h"
#include "sht4x/sht4x_i2c.h"

#define SENSOR_DATA_SERIAL_REVALIDATE_INTERVAL 10U
#define SENSOR_DATA_MIN_HUMIDITY_MILLI_PCT_RH  0
#define SENSOR_DATA_MAX_HUMIDITY_MILLI_PCT_RH  100000
#define SENSOR_DATA_SOIL_LABEL                 "PM-WCS-3-I2C"

typedef struct {
    SensorDataSensorType type;
    uint8_t address;
    const char* label;
} SensorDataSlotConfig;

typedef struct {
    bool present;
    uint8_t successful_reads;
    uint32_t serial_number;
    int16_t last_error;
} SensorDataSlotState;

typedef struct {
    bool present;
    int16_t last_error;
} SensorDataSoilState;

typedef struct {
    I2C_HandleTypeDef* i2c;
    bool inventory_initialized;
    SensorDataSlotState slots[SENSOR_DATA_SENSOR_SLOT_COUNT];
    SensorDataSoilState soil;
} SensorDataContext;

static const SensorDataSlotConfig sensor_data_slot_configs[] = {
    {SENSOR_TYPE_SHT4X, SHT4X_I2C_ADDR_46, "SHT40-CD1B-R3"},
    {SENSOR_TYPE_SHT4X, SHT4X_I2C_ADDR_44, "SHT43-ADCB-R2"},
};

static SensorDataContext sensor_data = {0};

static int32_t sensor_data_crop_humidity(int32_t humidity_milli_pct_rh) {
    if (humidity_milli_pct_rh < SENSOR_DATA_MIN_HUMIDITY_MILLI_PCT_RH) {
        return SENSOR_DATA_MIN_HUMIDITY_MILLI_PCT_RH;
    }

    if (humidity_milli_pct_rh > SENSOR_DATA_MAX_HUMIDITY_MILLI_PCT_RH) {
        return SENSOR_DATA_MAX_HUMIDITY_MILLI_PCT_RH;
    }

    return humidity_milli_pct_rh;
}

static void sensor_data_prepare_snapshot(SensorDataSnapshot* snapshot) {
    uint8_t index;

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->sensor_count = SENSOR_DATA_SENSOR_SLOT_COUNT;
    snapshot->soil.address = PM_WCS3_DEFAULT_ADDRESS;
    snapshot->soil.label = SENSOR_DATA_SOIL_LABEL;
    snapshot->soil.present = sensor_data.soil.present;
    snapshot->soil.driver_error = sensor_data.soil.last_error;

    for (index = 0U; index < SENSOR_DATA_SENSOR_SLOT_COUNT; ++index) {
        snapshot->sensors[index].type = sensor_data_slot_configs[index].type;
        snapshot->sensors[index].address = sensor_data_slot_configs[index].address;
        snapshot->sensors[index].label = sensor_data_slot_configs[index].label;
        snapshot->sensors[index].present = sensor_data.slots[index].present;
        snapshot->sensors[index].serial_number = sensor_data.slots[index].serial_number;
        snapshot->sensors[index].driver_error = sensor_data.slots[index].last_error;

        if (sensor_data.slots[index].present) {
            snapshot->present_count++;
        }
    }
}

static int16_t sensor_data_read_serial(uint8_t address, uint32_t* serial_number) {
    sht4x_init(address);
    return sht4x_serial_number(serial_number);
}

static int16_t sensor_data_read_measurement(uint8_t address,
                                            int32_t* temperature_mdeg_c,
                                            int32_t* humidity_milli_pct_rh) {
    sht4x_init(address);
    return sht4x_measure_high_precision(temperature_mdeg_c, humidity_milli_pct_rh);
}

static void sensor_data_scan_soil_inventory(void) {
    int16_t error = PMWCS3_Probe();

    sensor_data.soil.last_error = error;
    sensor_data.soil.present = (error == PM_WCS3_STATUS_OK);
}

static void sensor_data_scan_inventory(void) {
    uint8_t index;

    for (index = 0U; index < SENSOR_DATA_SENSOR_SLOT_COUNT; ++index) {
        uint32_t serial_number = 0U;
        int16_t error =
            sensor_data_read_serial(sensor_data_slot_configs[index].address,
                                    &serial_number);

        sensor_data.slots[index].last_error = error;
        if (error == NO_ERROR) {
            sensor_data.slots[index].present = true;
            sensor_data.slots[index].serial_number = serial_number;
        } else {
            sensor_data.slots[index].present = false;
            sensor_data.slots[index].successful_reads = 0U;
            sensor_data.slots[index].serial_number = 0U;
        }
    }

    sensor_data_scan_soil_inventory();
    sensor_data.inventory_initialized = true;
}

static SensorDataStatus sensor_data_status_from_scan(
    const SensorDataSnapshot* snapshot) {
    if (snapshot->present_count == 0U) {
        return SENSOR_DATA_STATUS_NO_SENSORS;
    }

    if (snapshot->present_count < snapshot->sensor_count) {
        return SENSOR_DATA_STATUS_PARTIAL;
    }

    return SENSOR_DATA_STATUS_OK;
}

static SensorDataStatus sensor_data_status_from_read(
    const SensorDataSnapshot* snapshot, bool had_read_error) {
    if (snapshot->present_count == 0U) {
        return SENSOR_DATA_STATUS_NO_SENSORS;
    }

    if (had_read_error || (snapshot->present_count < snapshot->sensor_count)) {
        return SENSOR_DATA_STATUS_PARTIAL;
    }

    return SENSOR_DATA_STATUS_OK;
}

static void sensor_data_apply_user_math(SensorDataSnapshot* snapshot) {
    (void)snapshot;

    /* USER SENSOR MATH BEGIN */
    /* Apply any caller-specific math to the sensor payload here. */
    /* USER SENSOR MATH END */
}

void SensorData_Init(I2C_HandleTypeDef* hi2c) {
    memset(&sensor_data, 0, sizeof(sensor_data));
    sensor_data.i2c = hi2c;

    PMWCS3_Init(hi2c, PM_WCS3_DEFAULT_ADDRESS);
    sensirion_i2c_hal_set_handle(hi2c);
    sensirion_i2c_hal_init();
}

SensorDataStatus SensorData_Scan(SensorDataSnapshot* snapshot) {
    if ((snapshot == NULL) || (sensor_data.i2c == NULL)) {
        return SENSOR_DATA_STATUS_INVALID_ARG;
    }

    sensor_data_scan_inventory();
    sensor_data_prepare_snapshot(snapshot);

    return sensor_data_status_from_scan(snapshot);
}

SensorDataStatus SensorData_ReadAll(SensorDataSnapshot* snapshot) {
    bool had_read_error = false;
    uint8_t index;

    if ((snapshot == NULL) || (sensor_data.i2c == NULL)) {
        return SENSOR_DATA_STATUS_INVALID_ARG;
    }

    if (!sensor_data.inventory_initialized) {
        sensor_data_scan_inventory();
    }

    sensor_data_prepare_snapshot(snapshot);
    if (snapshot->present_count == 0U) {
        if (!snapshot->soil.present) {
            return SENSOR_DATA_STATUS_NO_SENSORS;
        }
    }

    for (index = 0U; index < SENSOR_DATA_SENSOR_SLOT_COUNT; ++index) {
        SensorDataSensor* sensor_slot = &snapshot->sensors[index];
        SensorDataSlotState* sensor_state = &sensor_data.slots[index];
        int32_t temperature_mdeg_c = 0;
        int32_t humidity_milli_pct_rh = 0;
        int16_t error;

        if (!sensor_state->present) {
            continue;
        }

        error = sensor_data_read_measurement(sensor_slot->address,
                                             &temperature_mdeg_c,
                                             &humidity_milli_pct_rh);
        sensor_state->last_error = error;
        sensor_slot->driver_error = error;
        if (error != NO_ERROR) {
            had_read_error = true;
            continue;
        }

        sensor_slot->temperature_mdeg_c = temperature_mdeg_c;
        sensor_slot->humidity_milli_pct_rh =
            sensor_data_crop_humidity(humidity_milli_pct_rh);
        sensor_slot->driver_error = NO_ERROR;

        sensor_state->successful_reads++;
        if (sensor_state->successful_reads >= SENSOR_DATA_SERIAL_REVALIDATE_INTERVAL) {
            uint32_t serial_number = 0U;

            sensor_state->successful_reads = 0U;
            error = sensor_data_read_serial(sensor_slot->address, &serial_number);
            sensor_state->last_error = error;
            sensor_slot->driver_error = error;
            if (error != NO_ERROR) {
                had_read_error = true;
                continue;
            }

            sensor_slot->serial_revalidated = true;
            snapshot->serials_revalidated_on_this_read = true;
            if (serial_number != sensor_state->serial_number) {
                sensor_slot->serial_changed = true;
                sensor_state->serial_number = serial_number;
            }
        }

        sensor_slot->serial_number = sensor_state->serial_number;
    }

    if (sensor_data.soil.present) {
        PMWCS3Reading soil_reading = {0};
        int16_t soil_error = PMWCS3_Read(&soil_reading);

        sensor_data.soil.last_error = soil_error;
        snapshot->soil.driver_error = soil_error;
        if (soil_error == PM_WCS3_STATUS_OK) {
            snapshot->soil.sample_valid = true;
            snapshot->soil.e25_x100 = soil_reading.e25_x100;
            snapshot->soil.ec_uS_cm = soil_reading.ec_uS_cm;
            snapshot->soil.temperature_c_x100 = soil_reading.temperature_c_x100;
            snapshot->soil.vwc_pct_x10 = soil_reading.vwc_pct_x10;
        }
    }

    sensor_data_apply_user_math(snapshot);

    return sensor_data_status_from_read(snapshot, had_read_error);
}
