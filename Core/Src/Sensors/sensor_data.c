/*
 * sensor_data.c
 *
 *  Created on: Mar 30, 2026
 *      Author: kevin
 */
#include "sensor_data.h"
/**
  * @brief Returns true in the case that BOTH Sensirion SHT4x sensors are available.
  * @retval bool
  */
//bool hasBothSHT4xSensors(I2C_HandleTypeDef *hi2c)
//{
//	bool has_sensor_1 = false;
//	bool has_sensor_2 = false;
//
////	if (HAL_I2C_IsDeviceReady(&hi2c, 0x44 << 1, 1, 10) == HAL_OK) has_sensor_1 = true;
////	if (HAL_I2C_IsDeviceReady(&hi2c, 0x46 << 1, 1, 10) == HAL_OK) has_sensor_2 = true;
//
//	return has_sensor_1 && has_sensor_2;
//}
