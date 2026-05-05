#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "gpio_helper.h"
#include "i2c_helper.h"
#include "vcnl4010.h"
#include <stdint.h>

static const char *TAG = "Zigbee BME280";

void app_main(void) {
  ESP_LOGI(TAG, "Staring...");

  gpio_init();

  i2c_init();

  uint16_t value;
  while (1) {
    ESP_ERROR_CHECK(vcnl4010_get_proximity(vcnl4010_hdl, &value));
    ESP_LOGI(TAG, "VCNL4010 proximity value: %d", value);
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  i2c_deinit();
  ESP_LOGI(TAG, "Ending...");
}
