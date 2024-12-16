#include <math.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_io_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_heap_caps.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_lcd_gc9a01.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_spiffs.h"
#include "esp_system.h"

#include <esp_mac.h>
#include "nvs_flash.h"
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

#define LCD_HOST SPI2_HOST
#define PIN_NUM_DATA0          11  
#define PIN_NUM_PCLK           10
#define PIN_NUM_CS             9
#define PIN_NUM_DC             8
#define PIN_NUM_RST            14
#define PIN_NUM_BK_LIGHT       2

#define LCD_H_RES              240
#define LCD_V_RES              240
#define LCD_RES 240*240
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8


static const char *TAG = "lodepng_example";

static float timeSinceMessageReceived = 0.0f;

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
    timeSinceMessageReceived = esp_timer_get_time()/1000.0f/1000.0f;
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
    // peer_info.ifidx = ESP_IF_WIFI_STA; // Use station interface

    esp_wifi_set_channel(0, WIFI_SECOND_CHAN_NONE); 

    if (!esp_now_is_peer_exist(broadcast_mac)) {
        ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
    }
    esp_now_set_pmk(NULL);
}

void broadcast_message() {
    uint8_t broadcast_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t data[] = "Hello nearby devices!";
    
    ESP_ERROR_CHECK(esp_now_send(broadcast_mac, data, sizeof(data)));
}


typedef struct {
    float x;
    float y;
} vec2_t;


// Function to create a vec2_t
vec2_t create_vec2(float x, float y) {
    vec2_t v;
    v.x = x;
    v.y = y;
    return v;
}

uint16_t rgb888_to_rgb565(uint8_t red, uint8_t green, uint8_t blue) {
    uint16_t r = (red >> 3) & 0x1F;    // 5 bits for red
    uint16_t g = (green >> 2) & 0x3F;  // 6 bits for green
    uint16_t b = (blue >> 3) & 0x1F;   // 5 bits for blue

    return (r << 11) | (g << 5) | b;   // Combine into RGB565
}

// Function to load PNG from SPIFFS or SD card
esp_err_t load_image(const char *filename, uint8_t **image_data, uint16_t **color_table) {

    printf("LOAD IMAGE\n");

    // Open the file
    FILE *file = fopen(filename, "rb");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open file %s\n", filename);
        return ESP_FAIL;
    }

    printf("FILE OPENED\n");

    fseek(file, 0, SEEK_END);
    long size = ftell(file);
    fseek(file, 0, SEEK_SET);


    printf("SIZE CALCULATED %ld\n", size);

    // Allocate buffer for the file data
    long color_table_size = 16 * sizeof(uint16_t);
    *image_data = (uint8_t*)malloc(size-color_table_size);
    *color_table = (uint16_t*)malloc(color_table_size);


    printf("COLOR TABLE SIZE %ld\n", color_table_size);
    printf("IMAGE DATA SIZE %ld\n", size-color_table_size);

    if (*image_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for file data\n");
        fclose(file);
        return ESP_ERR_NO_MEM;
    }
    if (*color_table == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for file data\n");
        fclose(file);
        return ESP_ERR_NO_MEM;
    }

    printf("MEMORY ALOCATED \n");


    fread(*color_table, sizeof(uint16_t), 16, file);

    fseek(file, color_table_size, SEEK_SET);
    fread(*image_data, sizeof(uint8_t), (size-color_table_size), file);
    fclose(file);


    printf("FILE READ \n");

    return ESP_OK;
}

void app_main(void) 
{

    printf("START\n");
    printf("AVAILABLE MEMORY FOR DMA AT INIT: %d \n\n", heap_caps_get_free_size(MALLOC_CAP_DMA));


    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    printf("REGISTER SPIFFS\n");


    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            printf("Failed to mount or format filesystem\n");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            printf("Failed to find SPIFFS partition\n");
        } else {
            printf("Failed to initialize SPIFFS (%s)\n", esp_err_to_name(ret));
        }
        return;
    }


    uint16_t *full_screen_bitmap = (uint16_t *)heap_caps_malloc(sizeof(uint16_t) * LCD_RES, MALLOC_CAP_DMA);
    if (full_screen_bitmap == NULL) {
      ESP_LOGE("DMA_ALLOC", "Failed to allocate memory for the full screen bitmap, needed: %d, largest free block has %d", sizeof(uint16_t) * LCD_RES, heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
      return;
    }

    printf("AVAILABLE MEMORY FOR DMA AFTER FULL SCREEN FRAME BUFFER: %d \n\n", heap_caps_get_free_size(MALLOC_CAP_DMA));




    uint8_t *image_data = NULL;
    uint16_t *chicken_color_table = NULL;
    uint8_t *heart_image_data = NULL;
    uint16_t *heart_color_table = NULL;


    ESP_ERROR_CHECK(load_image("/spiffs/chicken", &image_data, &chicken_color_table));

    printf("AVAILABLE MEMORY FOR DMA AFTER CHICKEN DATA: %d \n\n", heap_caps_get_free_size(MALLOC_CAP_DMA));

    ESP_ERROR_CHECK(load_image("/spiffs/heart", &heart_image_data, &heart_color_table));
    printf("AVAILABLE MEMORY FOR DMA AFTER HEART DATA: %d \n\n", heap_caps_get_free_size(MALLOC_CAP_DMA));


    init_esp_now();



    spi_bus_config_t buscfg = {
      .mosi_io_num = PIN_NUM_DATA0,
      .sclk_io_num = PIN_NUM_PCLK,
      .miso_io_num = -1,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = LCD_H_RES * LCD_H_RES * sizeof(uint16_t),
    };

    // Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .pclk_hz = 80 * 1000 * 1000,
        .dc_gpio_num = PIN_NUM_DC,
        .cs_gpio_num = PIN_NUM_CS,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        
    };

    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));


    // Initialize the LCD configuration
    esp_lcd_panel_handle_t panel_handle;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_RST,
        .rgb_endian = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // Turn on backlight
    gpio_set_direction(PIN_NUM_BK_LIGHT, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_BK_LIGHT, 1);




    float previous_elapsed_time = esp_timer_get_time()/1000.0f/1000.0f;
    float accum_time = 0.0f;

    float animation_time = 0.0f;


    // for (int i = 0; i < 64*64; i++) {
    //   uint16_t color = image_data[i];
    //   image_data[i] = (color >> 8) | ((color & 0xFF) << 8);
    // }

    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, full_screen_bitmap));

    const float fixed_dt = 0.0166f;
    float dir = 1.0f;
    float speed = 20.0f;
    bool mirrored = false;

    vec2_t pos = {120.0f, 120.0f};
    vec2_t heart_pos = {120.0f, 200.0f};

    const int chicken_res = 64;
    const int chicken_half_res = 32;


    for(int i=0; i< LCD_RES; i++)
    {
      full_screen_bitmap[i] = 0b1111111111111111;
    }

    
    printf("AVAILABLE MEMORY FOR AFTER SETUP: %d \n\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));



    while (true) 
    {
      float elapsed_time = esp_timer_get_time()/1000.0f/1000.0f;
      float delta_time = elapsed_time - previous_elapsed_time;
      accum_time += delta_time;
      // printf("update: %f \n", delta_time);

      if(accum_time > fixed_dt)
      {
        accum_time = accum_time - fixed_dt;
        animation_time += fixed_dt;


        for(int i=0; i< LCD_RES; i++)
        {
          full_screen_bitmap[i] = 0b1111111111111111;
        }



  
        if(pos.x > 180.0f)
        {
          dir *= -1.0f;
          mirrored = true;
        }
        if(pos.x < 60.0f)
        {
          dir *= -1.0f;
          mirrored = false;
        }

        pos.x += fixed_dt * dir * speed;

        const int x = (int)pos.x - chicken_half_res;
        const int y = (int)pos.y - chicken_half_res;

        uint8_t index = 0;
        uint16_t col_0 = 0;
        uint16_t col_1 = 0;

        for(int i=0; i< chicken_res; i++)
        {
          for(int j=0; j<chicken_res/2; j++)
          {
            index = image_data[(i/2) * chicken_res + j];
            col_0 = chicken_color_table[(index >> 4)];
            col_1 = chicken_color_table[index & 0xF];

            if(mirrored)
            {
              full_screen_bitmap[((y+i)*240)+ x + chicken_res-(j*2+0)] = col_0;
              full_screen_bitmap[((y+i)*240)+ x + chicken_res-(j*2+1)] = col_1;
            }
            else
            {
              full_screen_bitmap[((y+i)*240)+ x + (j*2+0)] = col_0;
              full_screen_bitmap[((y+i)*240)+ x + (j*2+1)] = col_1;
            }
          }
        }

        const int heart_x = (int)heart_pos.x - chicken_half_res;
        const int heart_y = (int)heart_pos.y - chicken_half_res;

        if(elapsed_time - timeSinceMessageReceived < 1.0f)
        {

          for(int i=0; i< chicken_res; i++)
          {
            for(int j=0; j<chicken_res/2; j++)
            {
              index = heart_image_data[(i/2) * chicken_res + j];
              col_0 = heart_color_table[(index >> 4)];
              col_1 = heart_color_table[index & 0xF];

              if(mirrored)
              {
                full_screen_bitmap[((heart_y+i)*240)+ heart_x + chicken_res-(j*2+0)] = col_0;
                full_screen_bitmap[((heart_y+i)*240)+ heart_x + chicken_res-(j*2+1)] = col_1;
              }
              else
              {
                full_screen_bitmap[((heart_y+i)*240)+ heart_x + (j*2+0)] = col_0;
                full_screen_bitmap[((heart_y+i)*240)+ heart_x + (j*2+1)] = col_1;
              }
            }
          }
        }

        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, full_screen_bitmap));

      }
      previous_elapsed_time = elapsed_time;

      vTaskDelay(1);
    }
}