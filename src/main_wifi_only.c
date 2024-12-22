#ifdef APP_MAIN_NIMBLE_ONLY

  #include <stdio.h>
  #include <math.h>
  #include <string.h>
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "esp_heap_caps.h"

  #include "esp_timer.h"
  #include "esp_log.h"
  #include "esp_err.h"
  #include "esp_system.h"
  #include <string.h>
  #include "ble_manager.h"

  void app_main() {
      ble_manager_t *ble_manager = NULL;
      ble_manager_init(&ble_manager);
      while (1) 
      {
        vTaskDelay(10);
      }
  }
#endif