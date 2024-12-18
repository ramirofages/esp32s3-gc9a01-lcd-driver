#include "ble_manager.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"

#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "nimble/ble.h"
#include "services/gap/ble_svc_gap.h"

static ble_manager_t ble_manager = {0.0f};

void ble_manager_start_advertising(void) {
    struct ble_gap_adv_params adv_params = {0};

    const char *device_name = "NimBLE TEST";

    ble_svc_gap_device_name_set(device_name);

    uint8_t adv_data[] = {
        0x02, 0x01, 0x06,                     // Flags
        0x03, 0x03, 0x1A, 0x18,
        0x0F, 0x09, 'E', 'S', 'P', '3', '2' // Device name
    };

    ble_gap_adv_set_data(adv_data, sizeof(adv_data));

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;  // Undirected connectable advertising
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // General discoverable mode
    adv_params.itvl_min = 160; // 100ms (160 * 0.625ms)
    adv_params.itvl_max = 160;

    ble_gap_adv_start(0, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL);
    printf("Advertising started.\n");
}

// Callback to process scanned advertisements
int ble_manager_scan_callback(struct ble_gap_event *event, void *arg) {
    if (event->type == BLE_GAP_EVENT_DISC) {
        struct ble_hs_adv_fields fields;
        int rc;
        rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
        if (rc != 0) {
            printf("Error parsing advertisement data.\n");
            return 0;
        }

        if (fields.uuids16 != NULL) {
            for (int i = 0; i < fields.num_uuids16; i++) {
                if (fields.uuids16[i].value == 0x181A) { // Match UUID
                    printf("ESP32 Advertiser Found!\n");
                    printf("Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
                           event->disc.addr.val[5], event->disc.addr.val[4],
                           event->disc.addr.val[3], event->disc.addr.val[2],
                           event->disc.addr.val[1], event->disc.addr.val[0]);
                    printf("RSSI: %d dBm\n", event->disc.rssi);
                    
                    ble_manager.timeSinceLastMessageReceived = esp_timer_get_time()/1000.0f/1000.0f;
                }
            }
        }
    }
    return 0;
}

// Function to start BLE scanning
void ble_manager_start_scanning(void) {
    struct ble_gap_disc_params scan_params = {0};

    scan_params.passive = 1;    // Passive scanning
    scan_params.itvl = 100*3;     // Scan interval (100 * 0.625ms = 62.5ms)
    scan_params.window = 50;    // Scan window (50 * 0.625ms = 31.25ms)
    scan_params.filter_policy = BLE_HCI_SCAN_FILT_NO_WL; // No filter
    scan_params.limited = 0;

    // Start scanning
    ble_gap_disc(0, BLE_HS_FOREVER, &scan_params, ble_manager_scan_callback, NULL);
    printf("Scanning started.\n");
}

void ble_manager_on_sync(void) {
    ble_manager_start_advertising(); // Start advertising
    ble_manager_start_scanning();    // Start scanning
}

void ble_manager_start_host_task(void *param)
{
    printf("BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_manager_init(ble_manager_t **ble_manager_addr)
{
  ble_manager.timeSinceLastMessageReceived = 5.0f;

  *ble_manager_addr = &ble_manager;


  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  printf("NVM FLASH INITIALIZED\n\n");

  ret = nimble_port_init();
  if (ret != ESP_OK) {
    printf("Failed to init nimble %d ", ret);
    return ret;
  }

  printf("NIMBLE PORT INITIALIZED\n\n");

  ble_hs_cfg.sync_cb = ble_manager_on_sync;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
  nimble_port_freertos_init(ble_manager_start_host_task);

  printf("NIMBLE PORT RUNNING\n\n");

  return ESP_OK;
}
