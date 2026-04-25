#ifndef SCD4X_I2C_H
#define SCD4X_I2C_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SCD4X_I2C_ADDR_62             0x62
#define SCD4X_SERIAL_WORD_COUNT       3U
#define SCD4X_SINGLE_SHOT_DELAY_MS    5000U

typedef struct {
    uint16_t co2_ppm;
    int32_t temperature_mdeg_c;
    uint32_t humidity_milli_pct_rh;
} SCD4xMeasurement;

void scd4x_init(uint8_t i2c_address);
int16_t scd4x_get_serial_number(uint16_t serial_words[SCD4X_SERIAL_WORD_COUNT]);
int16_t scd4x_measure_single_shot(void);
int16_t scd4x_read_measurement(SCD4xMeasurement* measurement);

#ifdef __cplusplus
}
#endif

#endif /* SCD4X_I2C_H */
