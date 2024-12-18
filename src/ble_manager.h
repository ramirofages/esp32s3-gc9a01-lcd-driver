#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include "esp_err.h"

typedef struct {
  float timeSinceLastMessageReceived;
}  ble_manager_t;

esp_err_t ble_manager_init(ble_manager_t **ble_manager_addr);

#endif // BLE_MANAGER_H