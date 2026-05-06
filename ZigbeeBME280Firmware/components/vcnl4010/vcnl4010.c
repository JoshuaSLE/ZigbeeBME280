#include "vcnl4010.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "hal/gpio_types.h"
#include "portmacro.h"

#include <stdint.h>
#include <stdlib.h>

static const char *TAG = "VCNL4010";

ESP_EVENT_DEFINE_BASE(VCNL4010_EVENTS);

struct vcnl4010_context {
  vcnl4010_config_t config;
  i2c_master_dev_handle_t i2c_dev;
  SemaphoreHandle_t int_sem;
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

static void IRAM_ATTR vcnl4010_isr_handler(void *arg) {
  vcnl4010_handle_t handle = (vcnl4010_handle_t)arg;
  xSemaphoreGiveFromISR(handle->int_sem, NULL);
}

static void vcnl4010_interrupt_task(void *arg) {
  uint8_t status;
  vcnl4010_handle_t handle = (vcnl4010_handle_t)arg;

  while (1) {
    if (xSemaphoreTake(handle->int_sem, portMAX_DELAY)) {
      (void)vcnl4010_read_reg(handle, VCNL4010_REG_INT_STATUS, &status, 1);

      if (status & VCNL4010_EVENT_THR_HI) {
        (void)esp_event_post(VCNL4010_EVENTS, VCNL4010_EVENT_THR_HI, &handle,
                             sizeof(vcnl4010_handle_t), portMAX_DELAY);
      }
      if (status & VCNL4010_EVENT_THR_LO) {
        (void)esp_event_post(VCNL4010_EVENTS, VCNL4010_EVENT_THR_LO, &handle,
                             sizeof(vcnl4010_handle_t), portMAX_DELAY);
      }
      if (status & VCNL4010_EVENT_ALS) {
        (void)esp_event_post(VCNL4010_EVENTS, VCNL4010_EVENT_ALS, &handle,
                             sizeof(vcnl4010_handle_t), portMAX_DELAY);
      }
      if (status & VCNL4010_EVENT_PROX) {
        (void)esp_event_post(VCNL4010_EVENTS, VCNL4010_EVENT_PROX, &handle,
                             sizeof(vcnl4010_handle_t), portMAX_DELAY);
      }

      (void)vcnl4010_write_reg(handle, VCNL4010_REG_INT_STATUS, &status, 1);
    }
  }
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
  data[0] = dev->config.ir_led_current & 0x1F;
  ESP_GOTO_ON_ERROR(
      vcnl4010_write_reg(dev, VCNL4010_REG_IR_LED_CURRENT, data, 1),
      fail_device, TAG, "Failed to set led current");

  // --- Configure Proximity Rate ---
  data[0] = dev->config.prox_rate & 0x7;
  ESP_GOTO_ON_ERROR(vcnl4010_write_reg(dev, VCNL4010_REG_PROX_RATE, data, 1),
                    fail_device, TAG, "Failed to set proximity rate");

  // --- Configure ALS Parameters ---
  data[0] = ((dev->config.als.continuous_mode ? 1 : 0) << 7) |
            ((dev->config.als.rate & 0x7) << 4) |
            ((dev->config.als.offset_compensation ? 1 : 0) << 3) |
            (dev->config.als.averaging & 0x7);
  ESP_GOTO_ON_ERROR(vcnl4010_write_reg(dev, VCNL4010_REG_ALS_PARAM, data, 1),
                    fail_device, TAG, "Failed to set ambient config");

  // --- Configure Thresholds (only if enabled) ---
  if (dev->config.interrupts.enable) {
    data[0] = (uint8_t)((dev->config.interrupts.low_threshold & 0xFF00) >> 8);
    data[1] = (uint8_t)(dev->config.interrupts.low_threshold & 0xFF);
    ESP_GOTO_ON_ERROR(
        vcnl4010_write_reg(dev, VCNL4010_REG_LOW_THRESH_H, data, 2),
        fail_device, TAG, "Failed to set low threshold");

    data[0] = (uint8_t)((dev->config.interrupts.high_threshold & 0xFF00) >> 8);
    data[1] = (uint8_t)(dev->config.interrupts.high_threshold & 0xFF);
    ESP_GOTO_ON_ERROR(
        vcnl4010_write_reg(dev, VCNL4010_REG_HIGH_THRESH_H, data, 2),
        fail_device, TAG, "Failed to set high threshold");
  }

  // --- Configure Interrupt Control ---
  uint8_t int_ctrl = ((dev->config.interrupts.count & 0x03) << 4) |
                     ((dev->config.interrupts.enable_prox_ready ? 1 : 0) << 3) |
                     ((dev->config.interrupts.enable_als_ready ? 1 : 0) << 2) |
                     ((dev->config.interrupts.enable ? 1 : 0) << 1) |
                     (dev->config.interrupts.use_als ? 1 : 0);
  ESP_GOTO_ON_ERROR(
      vcnl4010_write_reg(dev, VCNL4010_REG_INT_CTRL, &int_ctrl, 1), fail_device,
      TAG, "Failed to set interrupt config");

  data[0] = 0xFF;
  ESP_GOTO_ON_ERROR(vcnl4010_write_reg(dev, VCNL4010_REG_INT_STATUS, data, 1),
                    fail_device, TAG, "Failed to clear interrupts");

  // --- Configure Command Register ---
  data[0] = 0;
  if (dev->config.enable_proximity) {
    data[0] |= VCNL4010_CMD_PROX_EN;
  }
  if (dev->config.enable_als) {
    data[0] |= VCNL4010_CMD_ALS_EN;
  }
  if (dev->config.enable_proximity || dev->config.als.continuous_mode) {
    data[0] |= VCNL4010_CMD_SELFTIMED_EN;
  }

  ESP_GOTO_ON_ERROR(vcnl4010_write_reg(dev, VCNL4010_REG_COMMAND, data, 1),
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

esp_err_t vcnl4010_read_int_status(vcnl4010_handle_t handle, uint8_t *status) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;
  return vcnl4010_read_reg(handle, VCNL4010_REG_INT_STATUS, status, 1);
}

esp_err_t vcnl4010_clear_int_status(vcnl4010_handle_t handle, uint8_t *status) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;
  return vcnl4010_write_reg(handle, VCNL4010_REG_INT_STATUS, status, 1);
}

esp_err_t vcnl4010_interrupt_init(vcnl4010_handle_t handle, uint8_t gpio_num,
                                  bool pullup_en) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Setting up isr handling...");

  handle->int_sem = xSemaphoreCreateBinary();
  if (handle->int_sem == NULL)
    return ESP_ERR_NO_MEM;

  const gpio_config_t io_conf = {
      .pin_bit_mask = BIT64(gpio_num),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = pullup_en,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_NEGEDGE,
  };
  ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG,
                      "Failed to configure gpio, error: %d", ret);

  ESP_RETURN_ON_ERROR(gpio_install_isr_service(0), TAG,
                      "Failed to create ISR, error: %d", ret);
  ESP_RETURN_ON_ERROR(
      gpio_isr_handler_add(gpio_num, vcnl4010_isr_handler, handle), TAG,
      "Failed to add ISR handler");

  if (xTaskCreate(vcnl4010_interrupt_task, "vcnl4010_interrupt", 2048, handle,
                  5, NULL) != pdPASS) {
    return ESP_ERR_NO_MEM;
  }

  ESP_LOGI(TAG, "Isr handling setup done.");
  return ESP_OK;
}