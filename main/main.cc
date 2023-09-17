#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "main_functions.h"

void tf_main(void) {
  setup();
  while (true) {
    loop();
  }
}

extern "C" void app_main() {
  xTaskCreatePinnedToCore((TaskFunction_t)&tf_main, "tensorflow", 40 * 1024, NULL, 2, NULL, CONFIG_MAIN_CORE_ID);
  vTaskDelete(NULL);
}
