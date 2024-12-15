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
esp_err_t load_image(const char *filename, uint16_t **image_data) {

    printf("LOAD IMAGE\n");

    // Open the file
    FILE *file = fopen(filename, "rb");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open file %s\n", filename);
        return ESP_FAIL;
    }

    printf("FILE OPENED\n");

    // Get file size
    fseek(file, 0, SEEK_END);
    long size = ftell(file);
    fseek(file, 0, SEEK_SET);


    printf("SIZE CALCULATED %ld\n", size);

    // Allocate buffer for the file data
    // *image_data = (uint16_t*)malloc(size);
    *image_data = (uint16_t *)heap_caps_malloc(size, MALLOC_CAP_DMA);

    if (*image_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for file data\n");
        fclose(file);
        return ESP_ERR_NO_MEM;
    }

    printf("MEMORY ALOCATED \n");

    // Read the file into buffer
    fread(*image_data, sizeof(uint16_t), size/2, file);
    fclose(file);


    printf("FILE READ \n");

    return ESP_OK;
}

void app_main(void) 
{

    printf("START\n");

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





    uint16_t *image_data = NULL;


    // Load PNG image from SPIFFS (change path as needed)
    ESP_ERROR_CHECK(load_image("/spiffs/chicken565", &image_data));



    // free(image_data);










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
    // ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

    // Turn on backlight
    gpio_set_direction(PIN_NUM_BK_LIGHT, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_BK_LIGHT, 1);


    uint16_t *full_screen_bitmap = (uint16_t *)heap_caps_malloc(sizeof(uint16_t) * LCD_RES, MALLOC_CAP_DMA);
    if (full_screen_bitmap == NULL) {
      ESP_LOGE("DMA_ALLOC", "Failed to allocate memory for the bitmap");
      return;
    }


    for(int i=0; i< LCD_RES; i++)
    {
      full_screen_bitmap[i] = rgb888_to_rgb565(255,255,255);
      // float u = (i%LCD_H_RES)/(LCD_H_RES-1.0f);
      // float v = floor(i/LCD_H_RES)/(LCD_V_RES-1.0f);

      // uint16_t color = rgb888_to_rgb565((uint8_t)(u*255), (uint8_t)(v*255), 0);
      // full_screen_bitmap[i] = (color >> 8) | ((color & 0xFF) << 8);

    }
    // printf("AVAILABLE MEMORY %d \n\n", heap_caps_get_free_size(MALLOC_CAP_DMA));


    float previous_elapsed_time = esp_timer_get_time()/1000.0f/1000.0f;
    float accum_time = 0.0f;

    float animation_time = 0.0f;


    for (int i = 0; i < 64*64; i++) {
      
      uint16_t color = image_data[i];

      image_data[i] = (color >> 8) | ((color & 0xFF) << 8);
    }

    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, full_screen_bitmap));

    const float fixed_dt = 0.0166f;
    float dir = 1.0f;
    float speed = 20.0f;
    bool mirrored = false;

    vec2_t pos = {120.0f, 120.0f};
    const int chicken_res = 64;
    const int chicken_half_res = 32;


    for(int i=0; i< LCD_RES; i++)
    {
      full_screen_bitmap[i] = 0b1111111111111111;
    }


    // const int x = 120;
    // const int y = 120;

    // for(int i=0; i< 64; i++)
    // {
    //   for(int j=0; j<64; j++)
    //   {
    //     full_screen_bitmap[((y+i)*240)+ x + j] = image_data[i * 64 + j];
    //   }
    // }

    // ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, full_screen_bitmap));


    while (true) 
    {
      float elapsed_time = esp_timer_get_time()/1000.0f/1000.0f;
      float delta_time = elapsed_time - previous_elapsed_time;
      accum_time += delta_time;
      printf("update: %f \n", delta_time);

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
          // ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
          mirrored = true;
        }
        if(pos.x < 60.0f)
        {
          dir *= -1.0f;
          // ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));
          mirrored = false;
        }

        pos.x += fixed_dt * dir * speed;

        const int x = (int)pos.x - chicken_half_res;
        const int y = (int)pos.y - chicken_half_res;

        for(int i=0; i< chicken_res; i++)
        {
          for(int j=0; j<chicken_res; j++)
          {
            if(mirrored)
            {
              full_screen_bitmap[((y+i)*240)+ x + chicken_res-j] = image_data[i * chicken_res + j];
            }
            else
            {
              full_screen_bitmap[((y+i)*240)+ x + j] = image_data[i * chicken_res + j];
            }
          }
        }

        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, full_screen_bitmap));

      }
      previous_elapsed_time = elapsed_time;

      vTaskDelay(1);
    }
}