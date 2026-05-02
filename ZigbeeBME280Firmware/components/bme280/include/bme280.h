#pragma once

#include "driver/i2c_types.h"
#include "esp_err.h"

#define BME280_I2C_ADDR 0x76
#define BME280_I2C_ADDR_ALT 0x77

#define BME280_REG_HUM_LSB 0xF2
#define BME280_REG_HUM_MSB 0xF3
#define BME280_REG_hUM_XLSB 0xF4

#define BME280_REG_TEMP_LSB 0xFA
#define BME280_REG_TEMP_MSB 0xFB
#define BME280_REG_TEMP_XLSB 0xFC

#define BME280_REG_PRESS_LSB 0xF7
#define BME280_REG_PRESS_MSB 0xF8
#define BME280_REG_PRESS_XLSB 0xF9

#define BME280_REG_CONFIG 0xF5
#define BME280_REG_CTRL_MEAS 0xF4

#define BME280_REG_STATUS 0xF3
#define BME280_STATUS_READY 0
#define BME280_STATUS_MEASURING (1 << 3)
#define BME280_STATUS_IM_UPDATE (1 << 0)

#define BME280_REG_CTRL_HUM 0xF2

#define BME280_REG_RESET 0xE0
#define BME280_SOFT_RESET 0xB6

#define BME280_REG_ID 0xD0
#define BME280_CHIP_ID 0x60

#define BME280_REG_COMP_1 0x88
#define BME280_REG_COMP_2 0xE1

#define BME280_CALIB_T1 0x88
#define BME280_CALIB_T2 0x8A
#define BME280_CALIB_T3 0x8C

#define BME280_CALIB_P1 0x8E
#define BME280_CALIB_P2 0x90
#define BME280_CALIB_P3 0x92
#define BME280_CALIB_P4 0x94
#define BME280_CALIB_P5 0x96
#define BME280_CALIB_P6 0x98
#define BME280_CALIB_P7 0x9A
#define BME280_CALIB_P8 0x9C
#define BME280_CALIB_P9 0x9E

#define BME280_CALIB_H1 0xA1
#define BME280_CALIB_H2 0xE1
#define BME280_CALIB_H3 0xE3
#define BME280_CALIB_H4 0xE4
#define BME280_CALIB_H5 0xE5
#define BME280_CALIB_H6 0xE7

/**
 * @brief Oversampling options for BME280
 */
typedef enum bme280_OverSampling {
  BME280_OVERSAMPLING_SKIP = 0b000,
  BME280_OVERSAMPLING_X1 = 0b001,
  BME280_OVERSAMPLING_X2 = 0b010,
  BME280_OVERSAMPLING_X4 = 0b011,
  BME280_OVERSAMPLING_X8 = 0b100,
  BME280_OVERSAMPLING_X16 = 0b101
} bme280_Oversampling_t;

/**
 * @brief Power modes for BME280
 */
typedef enum bme280_mode {
  BME280_MODE_SLEEP = 0b000,
  BME280_MODE_FORCED = 0b001,
  BME280_MODE_NORMAL = 0b011
} bme280_mode_t;

/**
 * @brief Standby modes for BME280
 */
typedef enum bme280_standby {
  BME280_STANDBY_0_5_MS = 0b000,
  BME280_STANDBY_62_5_MS = 0b001,
  BME280_STANDBY_125_MS = 0b010,
  BME280_STANDBY_250_MS = 0b011,
  BME280_STANDBY_500_MS = 0b100,
  BME280_STANDBY_1000_MS = 0b101,
  BME280_STANDBY_10_MS = 0b110,
  BME280_STANDBY_20_MS = 0b111
} bme280_standby_t;

/**
 * @brief Filter modes for BME280
 */
typedef enum bme280_filter {
  BME280_FILTER_OFF = 0b000,
  BME280_FILTER_X2 = 0b001,
  BME280_FILTER_X4 = 0b010,
  BME280_FILTER_X8 = 0b011,
  BME280_FILTER_X16 = 0b100
} bme280_filter_t;

/**
 * @brief Configuration struct for BME280
 */
typedef struct bme280_config {
  uint8_t i2c_addr;
  bme280_mode_t mode;
  bme280_standby_t standby_time;
  bme280_filter_t filter;
  bme280_Oversampling_t temp_oversampling;
  bme280_Oversampling_t press_oversampling;
  bme280_Oversampling_t hum_oversampling;
} bme280_config_t;

/**
 * @brief Context struct for BME280
 */
struct bme280_context;

/**
 * @brief Handle for BME280
 */
typedef struct bme280_context *bme280_handle_t;

/**
 * @brief Initialize the BME280 sensor
 * @param  bus: I2C bus handle
 * @param  *config: Pointer to the configuration structure
 * @param  *handle: Pointer to the handle for the BME280 sensor
 * @retval ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t bme280_init(i2c_master_bus_handle_t bus, bme280_config_t *config,
                      bme280_handle_t *handle);

/**
 * @brief Deinitialize the BME280 sensor
 * @param  handle: Pointer to the handle for the BME280 sensor
 * @retval ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t bme280_deinit(bme280_handle_t handle);