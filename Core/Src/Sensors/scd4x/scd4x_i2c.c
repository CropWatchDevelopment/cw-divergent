#include "scd4x_i2c.h"

#include "../sht4x/sensirion_common.h"
#include "../sht4x/sensirion_i2c.h"
#include "../sht4x/sensirion_i2c_hal.h"

#define SCD4X_COMMUNICATION_BUFFER_SIZE 9U
#define SCD4X_COMMAND_DELAY_US          1000U

typedef enum {
    SCD4X_GET_SERIAL_NUMBER_CMD_ID = 0x3682,
    SCD4X_MEASURE_SINGLE_SHOT_CMD_ID = 0x219D,
    SCD4X_READ_MEASUREMENT_CMD_ID = 0xEC05,
} SCD4X_CMD_ID;

static uint8_t scd4x_i2c_address = SCD4X_I2C_ADDR_62;

static int32_t scd4x_convert_ticks_to_celsius(uint16_t ticks) {
    return ((21875 * (int32_t)ticks) >> 13) - 45000;
}

static uint32_t scd4x_convert_ticks_to_percent_rh(uint16_t ticks) {
    return ((12500U * (uint32_t)ticks) >> 13);
}

static int16_t scd4x_write_command(uint16_t command) {
    uint8_t buffer[SENSIRION_COMMAND_SIZE] = {0};

    sensirion_common_uint16_t_to_bytes(command, buffer);
    return sensirion_i2c_write_data(scd4x_i2c_address, buffer, sizeof(buffer));
}

static int16_t scd4x_read_words(uint16_t command, uint8_t* buffer,
                                uint16_t buffer_size,
                                uint16_t expected_data_length) {
    int16_t error;

    if (buffer == NULL) {
        return BYTE_NUM_ERROR;
    }

    error = scd4x_write_command(command);
    if (error != NO_ERROR) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(SCD4X_COMMAND_DELAY_US);

    return sensirion_i2c_read_data_inplace(scd4x_i2c_address, buffer,
                                           buffer_size,
                                           expected_data_length);
}

void scd4x_init(uint8_t i2c_address) {
    scd4x_i2c_address = i2c_address;
}

int16_t scd4x_get_serial_number(uint16_t serial_words[SCD4X_SERIAL_WORD_COUNT]) {
    int16_t error;
    uint8_t buffer[SCD4X_COMMUNICATION_BUFFER_SIZE] = {0};

    if (serial_words == NULL) {
        return BYTE_NUM_ERROR;
    }

    error = scd4x_read_words(SCD4X_GET_SERIAL_NUMBER_CMD_ID, buffer,
                             sizeof(buffer), 6U);
    if (error != NO_ERROR) {
        return error;
    }

    serial_words[0] = sensirion_common_bytes_to_uint16_t(&buffer[0]);
    serial_words[1] = sensirion_common_bytes_to_uint16_t(&buffer[2]);
    serial_words[2] = sensirion_common_bytes_to_uint16_t(&buffer[4]);

    return NO_ERROR;
}

int16_t scd4x_measure_single_shot(void) {
    return scd4x_write_command(SCD4X_MEASURE_SINGLE_SHOT_CMD_ID);
}

int16_t scd4x_read_measurement(SCD4xMeasurement* measurement) {
    int16_t error;
    uint8_t buffer[SCD4X_COMMUNICATION_BUFFER_SIZE] = {0};
    uint16_t co2_ticks;
    uint16_t temperature_ticks;
    uint16_t humidity_ticks;

    if (measurement == NULL) {
        return BYTE_NUM_ERROR;
    }

    error = scd4x_read_words(SCD4X_READ_MEASUREMENT_CMD_ID, buffer,
                             sizeof(buffer), 6U);
    if (error != NO_ERROR) {
        return error;
    }

    co2_ticks = sensirion_common_bytes_to_uint16_t(&buffer[0]);
    temperature_ticks = sensirion_common_bytes_to_uint16_t(&buffer[2]);
    humidity_ticks = sensirion_common_bytes_to_uint16_t(&buffer[4]);

    measurement->co2_ppm = co2_ticks;
    measurement->temperature_mdeg_c =
        scd4x_convert_ticks_to_celsius(temperature_ticks);
    measurement->humidity_milli_pct_rh =
        scd4x_convert_ticks_to_percent_rh(humidity_ticks);

    return NO_ERROR;
}
