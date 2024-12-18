#include <math.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "esp_lcd_panel_io_interface.h"
// #include "esp_lcd_panel_io.h"
// #include "esp_lcd_panel_ops.h"
// #include "esp_lcd_panel_vendor.h"
#include "esp_heap_caps.h"
// #include "driver/spi_master.h"
// #include "driver/gpio.h"
// #include "esp_lcd_gc9a01.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include <string.h>
#include "ble_manager.h"
#include "image_loader.h"
#include "screen_manager.h"

// #define LCD_HOST SPI2_HOST
// #define PIN_NUM_DATA0          11  
// #define PIN_NUM_PCLK           10
// #define PIN_NUM_CS             9
// #define PIN_NUM_DC             8
// #define PIN_NUM_RST            14
// #define PIN_NUM_BK_LIGHT       2

// #define LCD_H_RES              240
// #define LCD_V_RES              240
#define LCD_RES 240*240
// #define LCD_CMD_BITS           8
// #define LCD_PARAM_BITS         8



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



void app_main(void) 
{

    printf("START\n");
    printf("AVAILABLE MEMORY FOR DMA AT INIT: %d \n\n", heap_caps_get_free_size(MALLOC_CAP_DMA));

    image_loader_init();

    screen_manager_t screen_manager;
    screen_manager_init(&screen_manager);


    uint8_t *image_data = NULL;
    uint16_t *chicken_color_table = NULL;
    uint8_t *heart_image_data = NULL;
    uint16_t *heart_color_table = NULL;


    ESP_ERROR_CHECK(image_loader_load("/spiffs/chicken", &image_data, &chicken_color_table));
    ESP_ERROR_CHECK(image_loader_load("/spiffs/heart", &heart_image_data, &heart_color_table));


    float previous_elapsed_time = esp_timer_get_time()/1000.0f/1000.0f;
    float accum_time = 0.0f;

    float animation_time = 0.0f;



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
      screen_manager.full_screen_bitmap[i] = 0b1111111111111111;
    }
    screen_manager_draw(&screen_manager);

    
    printf("AVAILABLE MEMORY AFTER IMAGE SETUP: %d \n\n", heap_caps_get_free_size(MALLOC_CAP_DMA));

    ble_manager_t *ble_manager = NULL;
    ble_manager_init(&ble_manager);
    

    printf("TIME SINCE LAST RECEIVED %f\n", ble_manager->timeSinceLastMessageReceived);
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
          screen_manager.full_screen_bitmap[i] = 0b1111111111111111;
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
              screen_manager.full_screen_bitmap[((y+i)*240)+ x + chicken_res-(j*2+0)] = col_0;
              screen_manager.full_screen_bitmap[((y+i)*240)+ x + chicken_res-(j*2+1)] = col_1;
            }
            else
            {
              screen_manager.full_screen_bitmap[((y+i)*240)+ x + (j*2+0)] = col_0;
              screen_manager.full_screen_bitmap[((y+i)*240)+ x + (j*2+1)] = col_1;
            }
          }
        }

        const int heart_x = (int)heart_pos.x - chicken_half_res;
        const int heart_y = (int)heart_pos.y - chicken_half_res;
        
        if(elapsed_time - ble_manager->timeSinceLastMessageReceived < 1.0f)
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
                screen_manager.full_screen_bitmap[((heart_y+i)*240)+ heart_x + chicken_res-(j*2+0)] = col_0;
                screen_manager.full_screen_bitmap[((heart_y+i)*240)+ heart_x + chicken_res-(j*2+1)] = col_1;
              }
              else
              {
                screen_manager.full_screen_bitmap[((heart_y+i)*240)+ heart_x + (j*2+0)] = col_0;
                screen_manager.full_screen_bitmap[((heart_y+i)*240)+ heart_x + (j*2+1)] = col_1;
              }
            }
          }
        }

        // ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, full_screen_bitmap));
        screen_manager_draw(&screen_manager);

      }
      previous_elapsed_time = elapsed_time;

      vTaskDelay(1);
    }
}