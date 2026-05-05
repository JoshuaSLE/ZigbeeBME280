#include "i2c_helper.h"
#include "bme280.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "sdkconfig.h"
#include "ssd1306.h"
#include "vcnl4010.h"

static const char *TAG = "I2C Helper";

i2c_master_bus_handle_t bus_hdl;

ssd1306_handle_t display_hdl;
bme280_handle_t bme280_hdl;
vcnl4010_handle_t vcnl4010_hdl;

void i2c_init() {
  ESP_LOGI(TAG, "I2C initialize starting...");

  const i2c_master_bus_config_t i2c_mst_config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = CONFIG_I2C_NUM,
      .scl_io_num = CONFIG_I2C_MASTER_SCL,
      .sda_io_num = CONFIG_I2C_MASTER_SDA,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = false,
  };
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_hdl));

  ssd1306_config_t display_cfg = I2C_SSD1306_128x64_CONFIG_DEFAULT;
  ESP_ERROR_CHECK(ssd1306_init(bus_hdl, &display_cfg, &display_hdl));

  ssd1306_clear_display(display_hdl, false);
  ssd1306_set_contrast(display_hdl, 0xff);
  ssd1306_display_text(display_hdl, 0, "Starting...     ", false);

  bme280_config_t bme280_cfg = {
      .i2c_addr = BME280_I2C_ADDR_ALT,
      .mode = BME280_MODE_FORCED,
      .press_oversampling = BME280_OVERSAMPLING_X1,
      .temp_oversampling = BME280_OVERSAMPLING_X1,
      .hum_oversampling = BME280_OVERSAMPLING_X1,
  };
  ESP_ERROR_CHECK(bme280_init(bus_hdl, &bme280_cfg, &bme280_hdl));

  vcnl4010_config_t vcnl4010_cfg = {
      .enable_proximity = true,
      .enable_als = false,
      .ir_led_current = VCNL4010_IR_LED_120MA,
      .prox_rate = VCNL4010_PROX_RATE_16_63,
      .als =
          {
              .continuous_mode = false,
              .offset_compensation = true,
          },
      .interrupts =
          {
              .enable = true,
              .low_threshold = 0,
              .high_threshold = 2350,
              .count = VCNL4010_INT_COUNT_2,
              .use_als = false,
              .enable_prox_ready = false,
              .enable_als_ready = false,
          },
  };
  ESP_ERROR_CHECK(vcnl4010_init(bus_hdl, &vcnl4010_cfg, &vcnl4010_hdl));

  ESP_LOGI(TAG, "I2C initialize done...");
}

void i2c_deinit() {
  ESP_LOGI(TAG, "I2C deinitialize starting...");

  if (display_hdl != NULL) {
    ssd1306_display_text(display_hdl, 0, "Ending...       ", false);
    vTaskDelay(pdMS_TO_TICKS(1000));

    ssd1306_clear_display(display_hdl, false);
    ssd1306_delete(display_hdl);
    display_hdl = NULL;
  }

  if (bme280_hdl != NULL) {
    bme280_deinit(bme280_hdl);
    bme280_hdl = NULL;
  }

  if (vcnl4010_hdl != NULL) {
    vcnl4010_deinit(vcnl4010_hdl);
    vcnl4010_hdl = NULL;
  }

  if (bus_hdl != NULL) {
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_hdl));
    bus_hdl = NULL;
  }

  gpio_reset_pin(CONFIG_I2C_MASTER_SDA);
  gpio_reset_pin(CONFIG_I2C_MASTER_SCL);

  ESP_LOGI(TAG, "I2C deinitialize done...");
}

void i2c_bus_scan() {
  ESP_LOGI(TAG, "Starting I2C scan...");

  for (int addr = 1; addr < 127; addr++) {
    if (i2c_master_probe(bus_hdl, addr, -1) == ESP_OK) {
      ESP_LOGI(TAG, "Found device at address: 0x%02x", addr);
    }
  }

  ESP_LOGI(TAG, "Scan complete.");
}