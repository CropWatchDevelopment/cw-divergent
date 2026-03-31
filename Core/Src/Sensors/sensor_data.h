/*
 * sensor_data.h
 *
 *  Created on: Mar 30, 2026
 *      Author: kevin
 */

#ifndef SRC_SENSORS_SENSOR_DATA_H_
#define SRC_SENSORS_SENSOR_DATA_H_

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#define SENSOR_DATA_MIN_POWERUP_DELAY_MS 2U
#define SENSOR_DATA_SENSOR_SLOT_COUNT    2U

typedef enum {
    SENSOR_TYPE_NONE = 0,
    SENSOR_TYPE_SHT4X
} SensorDataSensorType;

typedef enum {
    SENSOR_DATA_STATUS_OK = 0,
    SENSOR_DATA_STATUS_PARTIAL,
    SENSOR_DATA_STATUS_NO_SENSORS,
    SENSOR_DATA_STATUS_INVALID_ARG
} SensorDataStatus;

typedef struct {
    SensorDataSensorType type;
    bool present;
    uint8_t address;
    const char* label;
    int32_t temperature_mdeg_c;
    int32_t humidity_milli_pct_rh;
    uint32_t serial_number;
    bool serial_revalidated;
    bool serial_changed;
    int16_t driver_error;
} SensorDataSensor;

typedef struct {
    bool present;
    bool sample_valid;
    uint8_t address;
    const char* label;
    int16_t e25_x100;
    uint16_t ec_uS_cm;
    int16_t temperature_c_x100;
    uint16_t vwc_pct_x10;
    int16_t driver_error;
} SensorDataSoil;

typedef struct {
    uint8_t present_count;
    uint8_t sensor_count;
    bool serials_revalidated_on_this_read;
    SensorDataSensor sensors[SENSOR_DATA_SENSOR_SLOT_COUNT];
    SensorDataSoil soil;
} SensorDataSnapshot;

void SensorData_Init(I2C_HandleTypeDef* hi2c);

/* Caller must power the sensor rail and wait at least
 * SENSOR_DATA_MIN_POWERUP_DELAY_MS before calling scan/read.
 */
SensorDataStatus SensorData_Scan(SensorDataSnapshot* snapshot);
SensorDataStatus SensorData_ReadAll(SensorDataSnapshot* snapshot);

#endif /* SRC_SENSORS_SENSOR_DATA_H_ */
