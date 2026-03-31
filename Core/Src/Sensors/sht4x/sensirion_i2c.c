#include "sensirion_i2c.h"

#include "sensirion_i2c_hal.h"

uint8_t sensirion_i2c_generate_crc(const uint8_t* data, uint16_t count) {
    uint16_t current_byte;
    uint8_t crc = CRC8_INIT;
    uint8_t crc_bit;

    for (current_byte = 0; current_byte < count; ++current_byte) {
        crc ^= data[current_byte];
        for (crc_bit = 8; crc_bit > 0; --crc_bit) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((crc << 1) ^ CRC8_POLYNOMIAL);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

int8_t sensirion_i2c_check_crc(const uint8_t* data, uint16_t count,
                               uint8_t checksum) {
    if (sensirion_i2c_generate_crc(data, count) != checksum) {
        return CRC_ERROR;
    }

    return NO_ERROR;
}

int16_t sensirion_i2c_add_command8_to_buffer(uint8_t* buffer,
                                             uint16_t buffer_size,
                                             uint16_t* offset,
                                             uint8_t command) {
    if ((buffer == NULL) || (offset == NULL) || (*offset >= buffer_size)) {
        return BYTE_NUM_ERROR;
    }

    buffer[*offset] = command;
    (*offset)++;
    return NO_ERROR;
}

int16_t sensirion_i2c_write_data(uint8_t address, const uint8_t* data,
                                 uint16_t data_length) {
    if (data_length > UINT8_MAX) {
        return BYTE_NUM_ERROR;
    }

    return sensirion_i2c_hal_write(address, data, (uint8_t)data_length);
}

int16_t sensirion_i2c_read_data_inplace(uint8_t address, uint8_t* buffer,
                                        uint16_t buffer_size,
                                        uint16_t expected_data_length) {
    int16_t error;
    uint16_t i;
    uint16_t j;
    uint16_t read_length;

    if ((buffer == NULL) ||
        ((expected_data_length % SENSIRION_WORD_SIZE) != 0U)) {
        return BYTE_NUM_ERROR;
    }

    read_length = (expected_data_length / SENSIRION_WORD_SIZE) *
                  (SENSIRION_WORD_SIZE + CRC8_LEN);
    if ((read_length > UINT8_MAX) || (read_length > buffer_size)) {
        return BYTE_NUM_ERROR;
    }

    error = sensirion_i2c_hal_read(address, buffer, (uint8_t)read_length);
    if (error != NO_ERROR) {
        return error;
    }

    for (i = 0U, j = 0U; i < read_length;
         i += (SENSIRION_WORD_SIZE + CRC8_LEN)) {
        error = sensirion_i2c_check_crc(&buffer[i], SENSIRION_WORD_SIZE,
                                        buffer[i + SENSIRION_WORD_SIZE]);
        if (error != NO_ERROR) {
            return error;
        }

        buffer[j++] = buffer[i];
        buffer[j++] = buffer[i + 1U];
    }

    return NO_ERROR;
}
