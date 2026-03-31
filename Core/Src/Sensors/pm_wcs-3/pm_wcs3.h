#ifndef SRC_SENSORS_PM_WCS_3_PMWCS3_H_
#define SRC_SENSORS_PM_WCS_3_PMWCS3_H_

#include <stdint.h>

#include "main.h"

#define PM_WCS3_DEFAULT_ADDRESS 0x63U

typedef enum {
    PM_WCS3_STATUS_OK = 0,
    PM_WCS3_STATUS_INVALID_ARG = 1,
    PM_WCS3_STATUS_NOT_INITIALIZED = 2,
    PM_WCS3_STATUS_NOT_READY = 3,
    PM_WCS3_STATUS_WRITE_FAILED = 4,
    PM_WCS3_STATUS_READ_FAILED = 5
} PMWCS3Status;

typedef struct {
    int16_t e25_x100;
    uint16_t ec_uS_cm;
    int16_t temperature_c_x100;
    uint16_t vwc_pct_x10;
} PMWCS3Reading;

void PMWCS3_Init(I2C_HandleTypeDef* hi2c, uint8_t address);
int16_t PMWCS3_Probe(void);
int16_t PMWCS3_Read(PMWCS3Reading* reading);

#endif /* SRC_SENSORS_PM_WCS_3_PMWCS3_H_ */
