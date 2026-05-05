#include "vcnl4010.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include <stdint.h>
#include <stdlib.h>

static const char *TAG = "VCNL4010";

struct vcnl4010_context {
  vcnl4010_config_t config;
  i2c_master_dev_handle_t i2c_dev;
};

static inline esp_err_t vcnl4010_read_reg(vcnl4010_handle_t handle, uint8_t reg,
                                          uint8_t *data, size_t len) {
  if (handle == NULL || data == NULL || len == 0)
    return ESP_ERR_INVALID_ARG;

  return i2c_master_transmit_receive(handle->i2c_dev, &reg, 1, data, len, -1);
}

static inline esp_err_t vcnl4010_write_reg(vcnl4010_handle_t handle,
                                           uint8_t reg, uint8_t *data,
                                           size_t len) {
  if (handle == NULL || data == NULL || len == 0)
    return ESP_ERR_INVALID_ARG;

  uint8_t write_buffer[len + 1];
  write_buffer[0] = reg;
  memcpy(&write_buffer[1], data, len);

  return i2c_master_transmit(handle->i2c_dev, write_buffer,
                             sizeof(write_buffer), -1);
}

esp_err_t vcnl4010_init(i2c_master_bus_handle_t bus, vcnl4010_config_t *config,
                        vcnl4010_handle_t *handle) {
  esp_err_t ret = ESP_OK;

  struct vcnl4010_context *dev = calloc(1, sizeof(struct vcnl4010_context));
  if (dev == NULL)
    return ESP_ERR_NO_MEM;

  ESP_LOGI(TAG, "Initializing VCNL4010...");

  dev->config = *config;

  if (dev->config.ir_led_current > 20) {
    ESP_LOGE(TAG, "Invalid IR LED current: %d", dev->config.ir_led_current);
    free(dev);
    return ESP_ERR_INVALID_ARG;
  }

  const i2c_device_config_t i2c_dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = VCNL4010_I2C_ADDR,
      .scl_speed_hz = 100000,
  };
  ESP_GOTO_ON_ERROR(
      i2c_master_bus_add_device(bus, &i2c_dev_config, &dev->i2c_dev),
      fail_alloc, TAG, "Failed to add I2C device to bus");

  uint8_t data[2];
  ret = vcnl4010_read_reg(dev, VCNL4010_REG_PRODUCT_ID, data, 1);
  if (ret != ESP_OK || data[0] != VCNL4010_CHIP_ID) {
    ret = ESP_ERR_INVALID_RESPONSE;
    ESP_LOGE(TAG, "Failed to read chip ID: %d", ret);
    goto fail_device;
  }

  // --- Configure IR LED Current ---
  uint8_t led_current = dev->config.ir_led_current & 0x1F;
  ESP_GOTO_ON_ERROR(
      vcnl4010_write_reg(dev, VCNL4010_REG_IR_LED_CURRENT, &led_current, 1),
      fail_device, TAG, "Failed to set led current");

  // --- Configure Proximity Rate ---
  uint8_t prox_rate = dev->config.prox_rate & 0x7;
  ESP_GOTO_ON_ERROR(
      vcnl4010_write_reg(dev, VCNL4010_REG_PROX_RATE, &prox_rate, 1),
      fail_device, TAG, "Failed to set proximity rate");

  // --- Configure ALS Parameters ---
  uint8_t als_param = 0;
  als_param |= (dev->config.als.continuous_mode ? 1 : 0) << 7;
  als_param |= (dev->config.als.rate & 0x7) << 4;
  als_param |= (dev->config.als.offset_compensation ? 1 : 0) << 3;
  als_param |= (dev->config.als.averaging & 0x7);
  ESP_GOTO_ON_ERROR(
      vcnl4010_write_reg(dev, VCNL4010_REG_ALS_PARAM, &als_param, 1),
      fail_device, TAG, "Failed to set ambient config");

  // --- Configure Thresholds (only if enabled) ---

  if (dev->config.interrupts.enable) {
    uint8_t low_data[2];
    low_data[0] =
        (uint8_t)((dev->config.interrupts.low_threshold & 0xFF00) >> 8);
    low_data[1] = (uint8_t)(dev->config.interrupts.low_threshold & 0xFF);
    ESP_GOTO_ON_ERROR(
        vcnl4010_write_reg(dev, VCNL4010_REG_LOW_THRESH_H, low_data, 2),
        fail_device, TAG, "Failed to set low threshold");

    uint8_t high_data[2];
    high_data[0] =
        (uint8_t)((dev->config.interrupts.high_threshold & 0xFF00) >> 8);
    high_data[1] = (uint8_t)(dev->config.interrupts.high_threshold & 0xFF);
    ESP_GOTO_ON_ERROR(
        vcnl4010_write_reg(dev, VCNL4010_REG_HIGH_THRESH_H, high_data, 2),
        fail_device, TAG, "Failed to set high threshold");
  }

  // --- Configure Interrupt Control ---
  uint8_t int_ctrl = 0;
  int_ctrl |= (dev->config.interrupts.count & 0x03) << 4;
  int_ctrl |= (dev->config.interrupts.enable_prox_ready ? 1 : 0) << 3;
  int_ctrl |= (dev->config.interrupts.enable_als_ready ? 1 : 0) << 2;
  int_ctrl |= (dev->config.interrupts.enable ? 1 : 0) << 1;
  int_ctrl |= (dev->config.interrupts.use_als ? 1 : 0);
  ESP_GOTO_ON_ERROR(
      vcnl4010_write_reg(dev, VCNL4010_REG_INT_CTRL, &int_ctrl, 1), fail_device,
      TAG, "Failed to set interrupt config");

  ESP_GOTO_ON_ERROR(vcnl4010_reset_interrupt(dev), fail_device, TAG,
                    "Failed to clear interrupts");

  // --- Configure Command Register ---
  uint8_t command = 0;
  if (dev->config.enable_proximity) {
    command |= VCNL4010_CMD_PROX_EN;
  }
  if (dev->config.enable_als) {
    command |= VCNL4010_CMD_ALS_EN;
  }
  if (dev->config.enable_proximity || dev->config.als.continuous_mode) {
    command |= VCNL4010_CMD_SELFTIMED_EN;
  }

  ESP_GOTO_ON_ERROR(vcnl4010_write_reg(dev, VCNL4010_REG_COMMAND, &command, 1),
                    fail_device, TAG, "Failed to set command register");

  *handle = (vcnl4010_handle_t)dev;
  ESP_LOGI(TAG, "VCNL4010 initialized.");
  return ESP_OK;

fail_device:
  i2c_master_bus_rm_device(dev->i2c_dev);
fail_alloc:
  free(dev);
  return ret;
}

esp_err_t vcnl4010_deinit(vcnl4010_handle_t handle) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;
  ESP_LOGI(TAG, "VCNL4010 deinit...");
  esp_err_t ret = i2c_master_bus_rm_device(handle->i2c_dev);
  free(handle);
  ESP_LOGI(TAG, "VCNL4010 deinit.");
  return ret;
}

esp_err_t vcnl4010_get_ambient(vcnl4010_handle_t handle, uint16_t *value) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;
  esp_err_t ret;
  uint8_t data[2];
  ret = vcnl4010_read_reg(handle, VCNL4010_REG_ALS_RESULT_H, data, 2);
  *value = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
  return ret;
}

esp_err_t vcnl4010_get_proximity(vcnl4010_handle_t handle, uint16_t *value) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;
  esp_err_t ret;
  uint8_t data[2];
  ret = vcnl4010_read_reg(handle, VCNL4010_REG_PROX_RESULT_H, data, 2);
  *value = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
  return ret;
}

esp_err_t vcnl4010_reset_interrupt(vcnl4010_handle_t handle) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;
  uint8_t data = 0xFF;
  return vcnl4010_write_reg(handle, VCNL4010_REG_INT_STATUS, &data, 1);
}