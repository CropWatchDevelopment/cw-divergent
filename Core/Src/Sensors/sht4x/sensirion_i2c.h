#ifndef SENSIRION_I2C_H
#define SENSIRION_I2C_H

#include "sensirion_common.h"
#include "sensirion_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CRC_ERROR 1
#define I2C_BUS_ERROR 2
#define I2C_NACK_ERROR 3
#define BYTE_NUM_ERROR 4

#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xFF
#define CRC8_LEN 1

uint8_t sensirion_i2c_generate_crc(const uint8_t* data, uint16_t count);
int8_t sensirion_i2c_check_crc(const uint8_t* data, uint16_t count,
                               uint8_t checksum);
int16_t sensirion_i2c_add_command8_to_buffer(uint8_t* buffer,
                                             uint16_t buffer_size,
                                             uint16_t* offset,
                                             uint8_t command);
int16_t sensirion_i2c_write_data(uint8_t address, const uint8_t* data,
                                 uint16_t data_length);
int16_t sensirion_i2c_read_data_inplace(uint8_t address, uint8_t* buffer,
                                        uint16_t buffer_size,
                                        uint16_t expected_data_length);

#ifdef __cplusplus
}
#endif

#endif /* SENSIRION_I2C_H */
