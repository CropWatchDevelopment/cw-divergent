/*
 * sensor_data.c
 *
 *  Created on: Mar 30, 2026
 *      Author: kevin
 */

#include "sensor_data.h"

#include <string.h>

#include "pm_wcs-3/pm_wcs3.h"
#include "scd4x/scd4x_i2c.h"
#include "sht4x/sensirion_common.h"
#include "sht4x/sensirion_i2c.h"
#include "sht4x/sensirion_i2c_hal.h"
#include "sht4x/sht4x_i2c.h"

#define SENSOR_DATA_SERIAL_REVALIDATE_INTERVAL 10U
#define SENSOR_DATA_MIN_HUMIDITY_MILLI_PCT_RH  0
#define SENSOR_DATA_MAX_HUMIDITY_MILLI_PCT_RH  100000
#define SENSOR_DATA_SOIL_LABEL                 "PM-WCS-3-I2C"
#define SENSOR_DATA_SCD41_LABEL                "SCD41-CO2"

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
    bool present;
    uint16_t serial_words[SENSOR_DATA_SCD41_SERIAL_WORD_COUNT];
    int16_t last_error;
} SensorDataScd41State;

typedef struct {
    I2C_HandleTypeDef* i2c;
    bool inventory_initialized;
    SensorDataDelayMsCallback delay_ms_callback;
    SensorDataSlotState slots[SENSOR_DATA_SENSOR_SLOT_COUNT];
    SensorDataSoilState soil;
    SensorDataScd41State scd41;
} SensorDataContext;

static const SensorDataSlotConfig sensor_data_slot_configs[] = {
    {SENSOR_TYPE_SHT4X, SHT4X_I2C_ADDR_46, "SHT40-CD1B-R3"},
    {SENSOR_TYPE_SHT4X, SHT4X_I2C_ADDR_44, "SHT43-ADCB-R2"},
};

static SensorDataContext sensor_data = {0};

static bool sensor_data_scd41_serials_match(const uint16_t* left,
                                            const uint16_t* right) {
    uint8_t index;

    if ((left == NULL) || (right == NULL)) {
        return false;
    }

    for (index = 0U; index < SENSOR_DATA_SCD41_SERIAL_WORD_COUNT; ++index) {
        if (left[index] != right[index]) {
            return false;
        }
    }

    return true;
}

static void sensor_data_copy_scd41_serial(uint16_t* destination,
                                          const uint16_t* source) {
    if ((destination == NULL) || (source == NULL)) {
        return;
    }

    memcpy(destination, source,
           sizeof(uint16_t) * SENSOR_DATA_SCD41_SERIAL_WORD_COUNT);
}

static void sensor_data_delay_ms(uint32_t delay_ms) {
    if (delay_ms == 0U) {
        return;
    }

    if (sensor_data.delay_ms_callback != NULL) {
        sensor_data.delay_ms_callback(delay_ms);
    } else {
        HAL_Delay(delay_ms);
    }
}

static int32_t sensor_data_crop_humidity(int32_t humidity_milli_pct_rh) {
    if (humidity_milli_pct_rh < SENSOR_DATA_MIN_HUMIDITY_MILLI_PCT_RH) {
        return SENSOR_DATA_MIN_HUMIDITY_MILLI_PCT_RH;
    }

    if (humidity_milli_pct_rh > SENSOR_DATA_MAX_HUMIDITY_MILLI_PCT_RH) {
        return SENSOR_DATA_MAX_HUMIDITY_MILLI_PCT_RH;
    }

    return humidity_milli_pct_rh;
}

static uint8_t sensor_data_scd41_whole_humidity(uint32_t humidity_milli_pct_rh) {
    uint32_t rounded_pct = (humidity_milli_pct_rh + 500U) / 1000U;

    if (rounded_pct > 100U) {
        rounded_pct = 100U;
    }

    return (uint8_t)rounded_pct;
}

static void sensor_data_refresh_present_count(SensorDataSnapshot* snapshot) {
    uint8_t index;

    if (snapshot == NULL) {
        return;
    }

    snapshot->present_count = 0U;
    for (index = 0U; index < SENSOR_DATA_SENSOR_SLOT_COUNT; ++index) {
        if (snapshot->sensors[index].present) {
            snapshot->present_count++;
        }
    }
}

static bool sensor_data_snapshot_has_any_present(
    const SensorDataSnapshot* snapshot) {
    if (snapshot == NULL) {
        return false;
    }

    return (snapshot->present_count > 0U) || snapshot->soil.present ||
           snapshot->scd41.present;
}

static void sensor_data_prepare_snapshot(SensorDataSnapshot* snapshot) {
    uint8_t index;

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->sensor_count = SENSOR_DATA_SENSOR_SLOT_COUNT;
    snapshot->soil.address = PM_WCS3_DEFAULT_ADDRESS;
    snapshot->soil.label = SENSOR_DATA_SOIL_LABEL;
    snapshot->soil.present = sensor_data.soil.present;
    snapshot->soil.driver_error = sensor_data.soil.last_error;
    snapshot->scd41.address = SCD4X_I2C_ADDR_62;
    snapshot->scd41.label = SENSOR_DATA_SCD41_LABEL;
    snapshot->scd41.present = sensor_data.scd41.present;
    snapshot->scd41.driver_error = sensor_data.scd41.last_error;
    sensor_data_copy_scd41_serial(snapshot->scd41.serial_words,
                                  sensor_data.scd41.serial_words);

    for (index = 0U; index < SENSOR_DATA_SENSOR_SLOT_COUNT; ++index) {
        snapshot->sensors[index].type = sensor_data_slot_configs[index].type;
        snapshot->sensors[index].address = sensor_data_slot_configs[index].address;
        snapshot->sensors[index].label = sensor_data_slot_configs[index].label;
        snapshot->sensors[index].present = sensor_data.slots[index].present;
        snapshot->sensors[index].serial_number = sensor_data.slots[index].serial_number;
        snapshot->sensors[index].driver_error = sensor_data.slots[index].last_error;
    }

    sensor_data_refresh_present_count(snapshot);
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

static void sensor_data_mark_sht_absent(uint8_t index) {
    if (index >= SENSOR_DATA_SENSOR_SLOT_COUNT) {
        return;
    }

    sensor_data.slots[index].present = false;
    sensor_data.slots[index].successful_reads = 0U;
    sensor_data.slots[index].serial_number = 0U;
}

static void sensor_data_scan_sht_slot(uint8_t index) {
    uint32_t serial_number = 0U;
    int16_t error;

    if (index >= SENSOR_DATA_SENSOR_SLOT_COUNT) {
        return;
    }

    error = sensor_data_read_serial(sensor_data_slot_configs[index].address,
                                    &serial_number);
    sensor_data.slots[index].last_error = error;
    if (error == NO_ERROR) {
        sensor_data.slots[index].present = true;
        sensor_data.slots[index].serial_number = serial_number;
    } else {
        sensor_data_mark_sht_absent(index);
    }
}

static void sensor_data_mark_scd41_absent(int16_t error,
                                          SensorDataSnapshot* snapshot) {
    sensor_data.scd41.present = false;
    sensor_data.scd41.last_error = error;
    memset(sensor_data.scd41.serial_words, 0,
           sizeof(sensor_data.scd41.serial_words));

    if (snapshot != NULL) {
        snapshot->scd41.present = false;
        snapshot->scd41.sample_valid = false;
        snapshot->scd41.driver_error = error;
        memset(snapshot->scd41.serial_words, 0,
               sizeof(snapshot->scd41.serial_words));
    }
}

static int16_t sensor_data_probe_scd41(SensorDataSnapshot* snapshot) {
    uint16_t serial_words[SENSOR_DATA_SCD41_SERIAL_WORD_COUNT] = {0};
    bool was_present = sensor_data.scd41.present;
    int16_t error;

    scd4x_init(SCD4X_I2C_ADDR_62);
    error = scd4x_get_serial_number(serial_words);
    sensor_data.scd41.last_error = error;
    if (error != NO_ERROR) {
        sensor_data_mark_scd41_absent(error, snapshot);
        return error;
    }

    if (was_present &&
        !sensor_data_scd41_serials_match(sensor_data.scd41.serial_words,
                                         serial_words)) {
        if (snapshot != NULL) {
            snapshot->scd41.serial_changed = true;
        }
    }

    sensor_data.scd41.present = true;
    sensor_data_copy_scd41_serial(sensor_data.scd41.serial_words,
                                  serial_words);

    if (snapshot != NULL) {
        snapshot->scd41.present = true;
        snapshot->scd41.driver_error = NO_ERROR;
        snapshot->scd41.serial_revalidated = true;
        sensor_data_copy_scd41_serial(snapshot->scd41.serial_words,
                                      serial_words);
    }

    return NO_ERROR;
}

static void sensor_data_scan_scd41_inventory(void) {
    (void)sensor_data_probe_scd41(NULL);
}

static void sensor_data_scan_inventory(void) {
    uint8_t index;

    for (index = 0U; index < SENSOR_DATA_SENSOR_SLOT_COUNT; ++index) {
        sensor_data_scan_sht_slot(index);
    }

    sensor_data_scan_soil_inventory();
    sensor_data_scan_scd41_inventory();
    sensor_data.inventory_initialized = true;
}

static void sensor_data_scan_absent_inventory(void) {
    uint8_t index;

    for (index = 0U; index < SENSOR_DATA_SENSOR_SLOT_COUNT; ++index) {
        if (!sensor_data.slots[index].present) {
            sensor_data_scan_sht_slot(index);
        }
    }

    if (!sensor_data.soil.present) {
        sensor_data_scan_soil_inventory();
    }
}

static SensorDataStatus sensor_data_status_from_scan(
    const SensorDataSnapshot* snapshot) {
    if (!sensor_data_snapshot_has_any_present(snapshot)) {
        return SENSOR_DATA_STATUS_NO_SENSORS;
    }

    if ((snapshot->present_count > 0U) &&
        (snapshot->present_count < snapshot->sensor_count)) {
        return SENSOR_DATA_STATUS_PARTIAL;
    }

    return SENSOR_DATA_STATUS_OK;
}

static SensorDataStatus sensor_data_status_from_read(
    const SensorDataSnapshot* snapshot, bool had_read_error) {
    if (!sensor_data_snapshot_has_any_present(snapshot)) {
        return SENSOR_DATA_STATUS_NO_SENSORS;
    }

    if (had_read_error || ((snapshot->present_count > 0U) &&
                           (snapshot->present_count < snapshot->sensor_count))) {
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

static bool sensor_data_take_scd41_single_shot(SCD4xMeasurement* measurement) {
    int16_t error;

    error = scd4x_measure_single_shot();
    if (error != NO_ERROR) {
        sensor_data.scd41.last_error = error;
        return false;
    }

    sensor_data_delay_ms(SCD4X_SINGLE_SHOT_DELAY_MS);

    error = scd4x_read_measurement(measurement);
    sensor_data.scd41.last_error = error;
    return (error == NO_ERROR);
}

static bool sensor_data_read_scd41(SensorDataSnapshot* snapshot) {
    SCD4xMeasurement discarded_measurement = {0};
    SCD4xMeasurement measurement = {0};
    int16_t error;

    error = sensor_data_probe_scd41(snapshot);
    if (error != NO_ERROR) {
        return false;
    }

    /* SCD41 CO2 path: the sensor rail is switched, so discard the first
     * post-power-up single shot before exposing a sample. */
    if (!sensor_data_take_scd41_single_shot(&discarded_measurement)) {
        sensor_data_mark_scd41_absent(sensor_data.scd41.last_error, snapshot);
        return true;
    }

    if (!sensor_data_take_scd41_single_shot(&measurement)) {
        sensor_data_mark_scd41_absent(sensor_data.scd41.last_error, snapshot);
        return true;
    }

    snapshot->scd41.present = true;
    snapshot->scd41.sample_valid = true;
    snapshot->scd41.driver_error = NO_ERROR;
    snapshot->scd41.co2_ppm = measurement.co2_ppm;
    snapshot->scd41.temperature_mdeg_c = measurement.temperature_mdeg_c;
    snapshot->scd41.humidity_pct =
        sensor_data_scd41_whole_humidity(measurement.humidity_milli_pct_rh);

    return false;
}

void SensorData_Init(I2C_HandleTypeDef* hi2c) {
    memset(&sensor_data, 0, sizeof(sensor_data));
    sensor_data.i2c = hi2c;

    PMWCS3_Init(hi2c, PM_WCS3_DEFAULT_ADDRESS);
    scd4x_init(SCD4X_I2C_ADDR_62);
    sensirion_i2c_hal_set_handle(hi2c);
    sensirion_i2c_hal_init();
}

void SensorData_SetDelayMsCallback(SensorDataDelayMsCallback callback) {
    sensor_data.delay_ms_callback = callback;
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
    } else {
        sensor_data_scan_absent_inventory();
    }

    sensor_data_prepare_snapshot(snapshot);

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
            sensor_data_mark_sht_absent(index);
            sensor_slot->present = false;
            sensor_slot->serial_number = 0U;
            continue;
        }

        sensor_slot->present = true;
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
                sensor_data_mark_sht_absent(index);
                sensor_slot->present = false;
                sensor_slot->serial_number = 0U;
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

    sensor_data_refresh_present_count(snapshot);

    if (sensor_data.soil.present) {
        PMWCS3Reading soil_reading = {0};
        int16_t soil_error = PMWCS3_Read(&soil_reading);

        sensor_data.soil.last_error = soil_error;
        snapshot->soil.driver_error = soil_error;
        if (soil_error == PM_WCS3_STATUS_OK) {
            snapshot->soil.present = true;
            snapshot->soil.sample_valid = true;
            snapshot->soil.e25_x100 = soil_reading.e25_x100;
            snapshot->soil.ec_uS_cm = soil_reading.ec_uS_cm;
            snapshot->soil.temperature_c_x100 = soil_reading.temperature_c_x100;
            snapshot->soil.vwc_pct_x10 = soil_reading.vwc_pct_x10;
        } else {
            had_read_error = true;
            sensor_data.soil.present = false;
            snapshot->soil.present = false;
            snapshot->soil.sample_valid = false;
        }
    }

    if (sensor_data_read_scd41(snapshot)) {
        had_read_error = true;
    }

    sensor_data_apply_user_math(snapshot);

    return sensor_data_status_from_read(snapshot, had_read_error);
}
