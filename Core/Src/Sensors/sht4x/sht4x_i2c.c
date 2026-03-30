#include "sht4x_i2c.h"

#include "sensirion_common.h"
#include "sensirion_i2c.h"
#include "sensirion_i2c_hal.h"

#define SHT4X_COMMUNICATION_BUFFER_SIZE 6U
#define SHT4X_HIGH_PRECISION_DELAY_US   (10U * 1000U)
#define SHT4X_MEDIUM_PRECISION_DELAY_US (5U * 1000U)
#define SHT4X_LOWEST_PRECISION_DELAY_US (2U * 1000U)
#define SHT4X_SERIAL_NUMBER_DELAY_US    (10U * 1000U)
#define SHT4X_SOFT_RESET_DELAY_US       (10U * 1000U)
#define SHT4X_HEATER_LONG_DELAY_US      (1100U * 1000U)
#define SHT4X_HEATER_SHORT_DELAY_US     (110U * 1000U)

static uint8_t communication_buffer[SHT4X_COMMUNICATION_BUFFER_SIZE] = {0};
static uint8_t sht4x_i2c_address = 0U;

static int32_t sht4x_convert_ticks_to_celsius(uint16_t ticks) {
    return ((21875 * (int32_t)ticks) >> 13) - 45000;
}

static int32_t sht4x_convert_ticks_to_percent_rh(uint16_t ticks) {
    return ((15625 * (int32_t)ticks) >> 13) - 6000;
}

static int16_t sht4x_read_measurement_ticks(uint8_t command, uint32_t delay_us,
                                            uint16_t* temperature_ticks,
                                            uint16_t* humidity_ticks) {
    int16_t error;
    uint16_t offset = 0U;

    if ((temperature_ticks == NULL) || (humidity_ticks == NULL)) {
        return BYTE_NUM_ERROR;
    }

    offset = sensirion_i2c_add_command8_to_buffer(communication_buffer, offset,
                                                  command);
    error = sensirion_i2c_write_data(sht4x_i2c_address, communication_buffer,
                                     offset);
    if (error != NO_ERROR) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(delay_us);

    error = sensirion_i2c_read_data_inplace(sht4x_i2c_address,
                                            communication_buffer, 4U);
    if (error != NO_ERROR) {
        return error;
    }

    *temperature_ticks =
        sensirion_common_bytes_to_uint16_t(&communication_buffer[0]);
    *humidity_ticks = sensirion_common_bytes_to_uint16_t(&communication_buffer[2]);

    return NO_ERROR;
}

static int16_t sht4x_read_measurement(uint8_t command, uint32_t delay_us,
                                      int32_t* temperature, int32_t* humidity) {
    int16_t error;
    uint16_t temperature_ticks;
    uint16_t humidity_ticks;

    if ((temperature == NULL) || (humidity == NULL)) {
        return BYTE_NUM_ERROR;
    }

    error = sht4x_read_measurement_ticks(command, delay_us, &temperature_ticks,
                                         &humidity_ticks);
    if (error != NO_ERROR) {
        return error;
    }

    *temperature = sht4x_convert_ticks_to_celsius(temperature_ticks);
    *humidity = sht4x_convert_ticks_to_percent_rh(humidity_ticks);

    return NO_ERROR;
}

void sht4x_init(uint8_t i2c_address) {
    sht4x_i2c_address = i2c_address;
}

int16_t sht4x_measure_high_precision(int32_t* temperature, int32_t* humidity) {
    return sht4x_read_measurement(
        SHT4X_MEASURE_HIGH_PRECISION_TICKS_CMD_ID, SHT4X_HIGH_PRECISION_DELAY_US,
        temperature, humidity);
}

int16_t sht4x_measure_medium_precision(int32_t* temperature,
                                       int32_t* humidity) {
    return sht4x_read_measurement(
        SHT4X_MEASURE_MEDIUM_PRECISION_TICKS_CMD_ID,
        SHT4X_MEDIUM_PRECISION_DELAY_US, temperature, humidity);
}

int16_t sht4x_measure_lowest_precision(int32_t* temperature,
                                       int32_t* humidity) {
    return sht4x_read_measurement(
        SHT4X_MEASURE_LOWEST_PRECISION_TICKS_CMD_ID,
        SHT4X_LOWEST_PRECISION_DELAY_US, temperature, humidity);
}

int16_t sht4x_activate_highest_heater_power_long(int32_t* temperature,
                                                 int32_t* humidity) {
    return sht4x_read_measurement(
        SHT4X_ACTIVATE_HIGHEST_HEATER_POWER_LONG_TICKS_CMD_ID,
        SHT4X_HEATER_LONG_DELAY_US, temperature, humidity);
}

int16_t sht4x_activate_highest_heater_power_short(int32_t* temperature,
                                                  int32_t* humidity) {
    return sht4x_read_measurement(
        SHT4X_ACTIVATE_HIGHEST_HEATER_POWER_SHORT_TICKS_CMD_ID,
        SHT4X_HEATER_SHORT_DELAY_US, temperature, humidity);
}

int16_t sht4x_activate_medium_heater_power_long(int32_t* temperature,
                                                int32_t* humidity) {
    return sht4x_read_measurement(
        SHT4X_ACTIVATE_MEDIUM_HEATER_POWER_LONG_TICKS_CMD_ID,
        SHT4X_HEATER_LONG_DELAY_US, temperature, humidity);
}

int16_t sht4x_activate_medium_heater_power_short(int32_t* temperature,
                                                 int32_t* humidity) {
    return sht4x_read_measurement(
        SHT4X_ACTIVATE_MEDIUM_HEATER_POWER_SHORT_TICKS_CMD_ID,
        SHT4X_HEATER_SHORT_DELAY_US, temperature, humidity);
}

int16_t sht4x_activate_lowest_heater_power_long(int32_t* temperature,
                                                int32_t* humidity) {
    return sht4x_read_measurement(
        SHT4X_ACTIVATE_LOWEST_HEATER_POWER_LONG_TICKS_CMD_ID,
        SHT4X_HEATER_LONG_DELAY_US, temperature, humidity);
}

int16_t sht4x_activate_lowest_heater_power_short(int32_t* temperature,
                                                 int32_t* humidity) {
    return sht4x_read_measurement(
        SHT4X_ACTIVATE_LOWEST_HEATER_POWER_SHORT_TICKS_CMD_ID,
        SHT4X_HEATER_SHORT_DELAY_US, temperature, humidity);
}

int16_t sht4x_measure_high_precision_ticks(uint16_t* temperature_ticks,
                                           uint16_t* humidity_ticks) {
    return sht4x_read_measurement_ticks(
        SHT4X_MEASURE_HIGH_PRECISION_TICKS_CMD_ID, SHT4X_HIGH_PRECISION_DELAY_US,
        temperature_ticks, humidity_ticks);
}

int16_t sht4x_measure_medium_precision_ticks(uint16_t* temperature_ticks,
                                             uint16_t* humidity_ticks) {
    return sht4x_read_measurement_ticks(
        SHT4X_MEASURE_MEDIUM_PRECISION_TICKS_CMD_ID,
        SHT4X_MEDIUM_PRECISION_DELAY_US, temperature_ticks, humidity_ticks);
}

int16_t sht4x_measure_lowest_precision_ticks(uint16_t* temperature_ticks,
                                             uint16_t* humidity_ticks) {
    return sht4x_read_measurement_ticks(
        SHT4X_MEASURE_LOWEST_PRECISION_TICKS_CMD_ID,
        SHT4X_LOWEST_PRECISION_DELAY_US, temperature_ticks, humidity_ticks);
}

int16_t
sht4x_activate_highest_heater_power_long_ticks(uint16_t* temperature_ticks,
                                               uint16_t* humidity_ticks) {
    return sht4x_read_measurement_ticks(
        SHT4X_ACTIVATE_HIGHEST_HEATER_POWER_LONG_TICKS_CMD_ID,
        SHT4X_HEATER_LONG_DELAY_US, temperature_ticks, humidity_ticks);
}

int16_t
sht4x_activate_highest_heater_power_short_ticks(uint16_t* temperature_ticks,
                                                uint16_t* humidity_ticks) {
    return sht4x_read_measurement_ticks(
        SHT4X_ACTIVATE_HIGHEST_HEATER_POWER_SHORT_TICKS_CMD_ID,
        SHT4X_HEATER_SHORT_DELAY_US, temperature_ticks, humidity_ticks);
}

int16_t
sht4x_activate_medium_heater_power_long_ticks(uint16_t* temperature_ticks,
                                              uint16_t* humidity_ticks) {
    return sht4x_read_measurement_ticks(
        SHT4X_ACTIVATE_MEDIUM_HEATER_POWER_LONG_TICKS_CMD_ID,
        SHT4X_HEATER_LONG_DELAY_US, temperature_ticks, humidity_ticks);
}

int16_t
sht4x_activate_medium_heater_power_short_ticks(uint16_t* temperature_ticks,
                                               uint16_t* humidity_ticks) {
    return sht4x_read_measurement_ticks(
        SHT4X_ACTIVATE_MEDIUM_HEATER_POWER_SHORT_TICKS_CMD_ID,
        SHT4X_HEATER_SHORT_DELAY_US, temperature_ticks, humidity_ticks);
}

int16_t
sht4x_activate_lowest_heater_power_long_ticks(uint16_t* temperature_ticks,
                                              uint16_t* humidity_ticks) {
    return sht4x_read_measurement_ticks(
        SHT4X_ACTIVATE_LOWEST_HEATER_POWER_LONG_TICKS_CMD_ID,
        SHT4X_HEATER_LONG_DELAY_US, temperature_ticks, humidity_ticks);
}

int16_t
sht4x_activate_lowest_heater_power_short_ticks(uint16_t* temperature_ticks,
                                               uint16_t* humidity_ticks) {
    return sht4x_read_measurement_ticks(
        SHT4X_ACTIVATE_LOWEST_HEATER_POWER_SHORT_TICKS_CMD_ID,
        SHT4X_HEATER_SHORT_DELAY_US, temperature_ticks, humidity_ticks);
}

int16_t sht4x_serial_number(uint32_t* serial_number) {
    int16_t error;
    uint16_t offset = 0U;

    if (serial_number == NULL) {
        return BYTE_NUM_ERROR;
    }

    offset = sensirion_i2c_add_command8_to_buffer(communication_buffer, offset,
                                                  SHT4X_SERIAL_NUMBER_CMD_ID);
    error = sensirion_i2c_write_data(sht4x_i2c_address, communication_buffer,
                                     offset);
    if (error != NO_ERROR) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(SHT4X_SERIAL_NUMBER_DELAY_US);

    error = sensirion_i2c_read_data_inplace(sht4x_i2c_address,
                                            communication_buffer, 4U);
    if (error != NO_ERROR) {
        return error;
    }

    *serial_number = sensirion_common_bytes_to_uint32_t(&communication_buffer[0]);

    return NO_ERROR;
}

int16_t sht4x_soft_reset(void) {
    int16_t error;
    uint16_t offset = 0U;

    offset = sensirion_i2c_add_command8_to_buffer(communication_buffer, offset,
                                                  SHT4X_SOFT_RESET_CMD_ID);
    error = sensirion_i2c_write_data(sht4x_i2c_address, communication_buffer,
                                     offset);
    if (error != NO_ERROR) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(SHT4X_SOFT_RESET_DELAY_US);
    return NO_ERROR;
}
