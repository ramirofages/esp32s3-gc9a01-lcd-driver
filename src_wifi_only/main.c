
#include <stdio.h>
#include <math.h>
#include <esp_mac.h>
#include "nvs_flash.h"
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

void esp_now_recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
    if (esp_now_info == NULL || data == NULL || data_len <= 0) {
        printf("Invalid data received\n");
        return;
    }
    int rssi = esp_now_info->rx_ctrl->rssi;

    printf("Message received width %d dBm\n", rssi);

    // Approximate distance based on RSSI
    int distance = pow(10, (0 - rssi) / (10 * 2));  // Free-space approximation (n = 2)
    printf("Estimated distance: ~%d meters\n", distance);
    printf("Message received Data: %s\n", (char *)data);
}

void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    printf("Message sent to " MACSTR ", status: %s\n", MAC2STR(mac_addr), status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void init_esp_now() {

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Wi-Fi in station mode
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(esp_now_send_cb));

    // Register the broadcast address as a peer
    esp_now_peer_info_t peer_info = {0};

    uint8_t broadcast_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    memcpy(peer_info.peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = 0;  // Default channel
    peer_info.encrypt = false;

    if (!esp_now_is_peer_exist(broadcast_mac)) {
        ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
    }
}

void broadcast_message() {
    uint8_t broadcast_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t data[] = "Hello nearby devices!";
    
    ESP_ERROR_CHECK(esp_now_send(broadcast_mac, data, sizeof(data)));
}

void app_main() {
    init_esp_now();
    while (true) 
    {
      broadcast_message();
      vTaskDelay(1000);
    }
}
