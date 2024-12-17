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
#include "nvs_flash.h"
// #include <esp_mac.h>
#include <string.h>

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "nimble/ble.h"
#include "services/gap/ble_svc_gap.h"

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


void start_advertising(void) {
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
static int scan_callback(struct ble_gap_event *event, void *arg) {
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
                    timeSinceMessageReceived = esp_timer_get_time()/1000.0f/1000.0f;
                }
            }
        }
        // printf("Advertisement received:\n");
        // printf("  Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
        //     event->disc.addr.val[5], event->disc.addr.val[4], event->disc.addr.val[3],
        //     event->disc.addr.val[2], event->disc.addr.val[1], event->disc.addr.val[0]);

        // printf("  RSSI: %d dBm\n", event->disc.rssi);
    }
    return 0;
}

// Function to start BLE scanning
void start_scanning(void) {
    struct ble_gap_disc_params scan_params = {0};

    scan_params.passive = 1;    // Passive scanning
    scan_params.itvl = 100*3;     // Scan interval (100 * 0.625ms = 62.5ms)
    scan_params.window = 50;    // Scan window (50 * 0.625ms = 31.25ms)
    scan_params.filter_policy = BLE_HCI_SCAN_FILT_NO_WL; // No filter
    scan_params.limited = 0;

    // Start scanning
    ble_gap_disc(0, BLE_HS_FOREVER, &scan_params, scan_callback, NULL);
    printf("Scanning started.\n");
}

void periodic_adv_host_task(void *param)
{
    printf("BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_app_on_sync(void) {
    start_advertising(); // Start advertising
    start_scanning();    // Start scanning
}
void init_nimble()
{
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
      return;
  }

  printf("NIMBLE PORT INITIALIZED\n\n");

  // ble_svc_gap_init();

  // printf("NIMBLE GAP INITIALIZED\n\n");

  ble_hs_cfg.sync_cb = ble_app_on_sync;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
  // nimble_port_run(); // This starts the NimBLE task.
  nimble_port_freertos_init(periodic_adv_host_task);
  printf("NIMBLE PORT RUNNING\n\n");

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

    
    printf("AVAILABLE MEMORY AFTER IMAGE SETUP: %d \n\n", heap_caps_get_free_size(MALLOC_CAP_DMA));
    // printf("AVAILABLE MEMORY FOR DMA AFTER FULL SCREEN FRAME BUFFER: %d \n\n", heap_caps_get_free_size(MALLOC_CAP_DMA));

    init_nimble();

    printf("AVAILABLE MEMORY AFTER NIMBLE SETUP: %d \n\n", heap_caps_get_free_size(MALLOC_CAP_DMA));

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