#ifndef SHT4X_I2C_H
#define SHT4X_I2C_H

#include "sensirion_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SHT4X_I2C_ADDR_44 0x44
#define SHT4X_I2C_ADDR_45 0x45
#define SHT4X_I2C_ADDR_46 0x46

typedef enum {
    SHT4X_MEASURE_HIGH_PRECISION_TICKS_CMD_ID = 0xFD,
    SHT4X_MEASURE_MEDIUM_PRECISION_TICKS_CMD_ID = 0xF6,
    SHT4X_MEASURE_LOWEST_PRECISION_TICKS_CMD_ID = 0xE0,
    SHT4X_ACTIVATE_HIGHEST_HEATER_POWER_LONG_TICKS_CMD_ID = 0x39,
    SHT4X_ACTIVATE_HIGHEST_HEATER_POWER_SHORT_TICKS_CMD_ID = 0x32,
    SHT4X_ACTIVATE_MEDIUM_HEATER_POWER_LONG_TICKS_CMD_ID = 0x2F,
    SHT4X_ACTIVATE_MEDIUM_HEATER_POWER_SHORT_TICKS_CMD_ID = 0x24,
    SHT4X_ACTIVATE_LOWEST_HEATER_POWER_LONG_TICKS_CMD_ID = 0x1E,
    SHT4X_ACTIVATE_LOWEST_HEATER_POWER_SHORT_TICKS_CMD_ID = 0x15,
    SHT4X_SERIAL_NUMBER_CMD_ID = 0x89,
    SHT4X_SOFT_RESET_CMD_ID = 0x94,
} SHT4X_CMD_ID;

void sht4x_init(uint8_t i2c_address);

int16_t sht4x_measure_high_precision(int32_t* temperature, int32_t* humidity);
int16_t sht4x_measure_medium_precision(int32_t* temperature,
                                       int32_t* humidity);
int16_t sht4x_measure_lowest_precision(int32_t* temperature,
                                       int32_t* humidity);

int16_t sht4x_activate_highest_heater_power_long(int32_t* temperature,
                                                 int32_t* humidity);
int16_t sht4x_activate_highest_heater_power_short(int32_t* temperature,
                                                  int32_t* humidity);
int16_t sht4x_activate_medium_heater_power_long(int32_t* temperature,
                                                int32_t* humidity);
int16_t sht4x_activate_medium_heater_power_short(int32_t* temperature,
                                                 int32_t* humidity);
int16_t sht4x_activate_lowest_heater_power_long(int32_t* temperature,
                                                int32_t* humidity);
int16_t sht4x_activate_lowest_heater_power_short(int32_t* temperature,
                                                 int32_t* humidity);

int16_t sht4x_measure_high_precision_ticks(uint16_t* temperature_ticks,
                                           uint16_t* humidity_ticks);
int16_t sht4x_measure_medium_precision_ticks(uint16_t* temperature_ticks,
                                             uint16_t* humidity_ticks);
int16_t sht4x_measure_lowest_precision_ticks(uint16_t* temperature_ticks,
                                             uint16_t* humidity_ticks);
int16_t
sht4x_activate_highest_heater_power_long_ticks(uint16_t* temperature_ticks,
                                               uint16_t* humidity_ticks);
int16_t
sht4x_activate_highest_heater_power_short_ticks(uint16_t* temperature_ticks,
                                                uint16_t* humidity_ticks);
int16_t
sht4x_activate_medium_heater_power_long_ticks(uint16_t* temperature_ticks,
                                              uint16_t* humidity_ticks);
int16_t
sht4x_activate_medium_heater_power_short_ticks(uint16_t* temperature_ticks,
                                               uint16_t* humidity_ticks);
int16_t
sht4x_activate_lowest_heater_power_long_ticks(uint16_t* temperature_ticks,
                                              uint16_t* humidity_ticks);
int16_t
sht4x_activate_lowest_heater_power_short_ticks(uint16_t* temperature_ticks,
                                               uint16_t* humidity_ticks);

int16_t sht4x_serial_number(uint32_t* serial_number);
int16_t sht4x_soft_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* SHT4X_I2C_H */
