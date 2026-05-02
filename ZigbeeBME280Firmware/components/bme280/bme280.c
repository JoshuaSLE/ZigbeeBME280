#include "bme280.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

static const char *TAG = "BME280";

typedef struct bme280_compensation {
  uint16_t t1;
  int16_t t2;
  int16_t t3;

  uint16_t p1;
  int16_t p2;
  int16_t p3;
  int16_t p4;
  int16_t p5;
  int16_t p6;
  int16_t p7;
  int16_t p8;
  int16_t p9;

  uint8_t h1;
  int16_t h2;
  uint8_t h3;
  int16_t h4;
  int16_t h5;
  int8_t h6;
} bme280_compensation_t;

struct bme280_context {
  bme280_config_t config;
  i2c_master_dev_handle_t i2c_dev;
  bme280_compensation_t comp;
  int32_t t_fine;
};

static inline esp_err_t bme280_read_reg(bme280_handle_t handle, uint8_t reg,
                                        uint8_t *data, size_t len) {
  if (handle == NULL || data == NULL || len == 0)
    return ESP_ERR_INVALID_ARG;

  return i2c_master_transmit_receive(handle->i2c_dev, &reg, 1, data, len, -1);
}

static inline esp_err_t bme280_write_reg(bme280_handle_t handle, uint8_t reg,
                                         uint8_t *data, size_t len) {
  if (handle == NULL || data == NULL || len == 0)
    return ESP_ERR_INVALID_ARG;

  uint8_t write_buffer[len + 1];
  write_buffer[0] = reg;
  memcpy(&write_buffer[1], data, len);

  return i2c_master_transmit(handle->i2c_dev, write_buffer,
                             sizeof(write_buffer), -1);
}

static inline uint8_t bme280_status(bme280_handle_t handle) {
  uint8_t status;
  ESP_ERROR_CHECK(bme280_read_reg(handle, BME280_REG_STATUS, &status, 1));
  return status;
}

static esp_err_t bme280_reset(bme280_handle_t handle) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;

  uint8_t reset_cmd = BME280_SOFT_RESET;
  esp_err_t ret = bme280_write_reg(handle, BME280_REG_RESET, &reset_cmd, 1);
  if (ret != ESP_OK) {
    return ret;
  }

  int timeout = 10;
  while (timeout-- > 0) {
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t status = bme280_status(handle);

    if ((status & (BME280_STATUS_MEASURING | BME280_STATUS_IM_UPDATE)) ==
        BME280_STATUS_READY)
      return ESP_OK;
  }

  return ESP_ERR_TIMEOUT;
}

static esp_err_t bme280_get_compensation(bme280_handle_t handle) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;

  uint8_t data1[25];
  uint8_t data2[7];

  esp_err_t ret = bme280_read_reg(handle, BME280_REG_COMP_1, data1, 25);
  if (ret != ESP_OK)
    return ret;

  ret = bme280_read_reg(handle, BME280_REG_COMP_2, data2, 7);
  if (ret != ESP_OK)
    return ret;

  bme280_compensation_t *c = &handle->comp;

  // --- Temperature ---
  c->t1 = (uint16_t)((data1[1] << 8) | data1[0]);
  c->t2 = (int16_t)((data1[3] << 8) | data1[2]);
  c->t3 = (int16_t)((data1[5] << 8) | data1[4]);

  // --- Pressure ---
  c->p1 = (uint16_t)((data1[7] << 8) | data1[6]);
  c->p2 = (int16_t)((data1[9] << 8) | data1[8]);
  c->p3 = (int16_t)((data1[11] << 8) | data1[10]);
  c->p4 = (int16_t)((data1[13] << 8) | data1[12]);
  c->p5 = (int16_t)((data1[15] << 8) | data1[14]);
  c->p6 = (int16_t)((data1[17] << 8) | data1[16]);
  c->p7 = (int16_t)((data1[19] << 8) | data1[18]);
  c->p8 = (int16_t)((data1[21] << 8) | data1[20]);
  c->p9 = (int16_t)((data1[23] << 8) | data1[22]);

  // --- Humidity ---
  c->h1 = data1[24];
  c->h2 = (int16_t)((data2[1] << 8) | data2[0]);
  c->h3 = data2[2];
  c->h4 = (int16_t)((data2[3] << 4) | (data2[4] & 0x0F));
  c->h5 = (int16_t)((data2[5] << 4) | (data2[4] >> 4));
  c->h6 = (int8_t)data2[6];

  return ESP_OK;
}

esp_err_t bme280_init(i2c_master_bus_handle_t bus, bme280_config_t *config,
                      bme280_handle_t *handle) {
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Initializing BME280...");

  struct bme280_context *dev = calloc(1, sizeof(struct bme280_context));
  if (dev == NULL)
    return ESP_ERR_NO_MEM;

  const i2c_device_config_t i2c_dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = config->i2c_addr,
      .scl_speed_hz = 100000,
  };

  ESP_GOTO_ON_ERROR(
      i2c_master_bus_add_device(bus, &i2c_dev_config, &dev->i2c_dev),
      fail_alloc, TAG, "Failed to add I2C device to bus: %d", ret);

  ESP_GOTO_ON_ERROR(bme280_reset(dev), fail_device, TAG,
                    "Failed to reset BME280: %d");

  uint8_t id;
  ret = bme280_read_reg(dev, BME280_REG_ID, &id, 1);
  if (ret != ESP_OK || id != BME280_CHIP_ID) {
    ESP_LOGE(TAG, "Failed to read BME280 ID: %d", ret);
    goto fail_device;
  }

  ESP_GOTO_ON_ERROR(bme280_get_compensation(dev), fail_device, TAG,
                    "Failed to read compensation");

  uint8_t mode_val = BME280_MODE_SLEEP;
  ESP_GOTO_ON_ERROR(bme280_write_reg(dev, BME280_REG_CTRL_MEAS, &mode_val, 1),
                    fail_device, TAG, "Failed to set sleep mode");

  uint8_t ctrl_hum = dev->config.hum_oversampling & 0x07;
  ESP_GOTO_ON_ERROR(bme280_write_reg(dev, BME280_REG_CTRL_HUM, &ctrl_hum, 1),
                    fail_device, TAG, "Failed to set Humidity oversampling");

  uint8_t config_reg = ((dev->config.standby_time & 0x07) << 5) |
                       ((dev->config.filter & 0x07) << 2);
  ESP_GOTO_ON_ERROR(bme280_write_reg(dev, BME280_REG_CONFIG, &config_reg, 1),
                    fail_device, TAG, "Failed to set Config (Filter/Standby)");

  uint8_t ctrl_meas = ((dev->config.temp_oversampling & 0x07) << 5) |
                      ((dev->config.press_oversampling & 0x07) << 2) |
                      (dev->config.mode & 0x03);
  ESP_GOTO_ON_ERROR(bme280_write_reg(dev, BME280_REG_CTRL_MEAS, &ctrl_meas, 1),
                    fail_device, TAG, "Failed to set Meas/Mode");

  *handle = (bme280_handle_t)dev;
  ESP_LOGI(TAG, "BME280 initialized.");
  return ESP_OK;

fail_device:
  i2c_master_bus_rm_device(dev->i2c_dev);
fail_alloc:
  free(dev);
  return ret;
}

esp_err_t bme280_deinit(bme280_handle_t handle) {
  return i2c_master_bus_rm_device(handle->i2c_dev);
}