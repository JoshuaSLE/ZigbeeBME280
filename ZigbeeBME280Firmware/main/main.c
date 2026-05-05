#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "gpio_helper.h"
#include "i2c_helper.h"

static const char *TAG = "Zigbee BME280";

void app_main(void) {
  ESP_LOGI(TAG, "Staring...");

  gpio_init();

  i2c_init();

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  i2c_deinit();
  ESP_LOGI(TAG, "Ending...");
}
