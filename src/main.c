#include <math.h>
#include <stdio.h>
#include "sdkconfig.h"
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

#define LCD_HOST SPI2_HOST
#define PIN_NUM_DATA0          11  
#define PIN_NUM_PCLK           10
#define PIN_NUM_CS             9
#define PIN_NUM_DC             8
#define PIN_NUM_RST            14
#define PIN_NUM_BK_LIGHT       2

#define LCD_H_RES              240
#define LCD_V_RES              240

#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8

uint16_t rgb888_to_rgb565(uint8_t red, uint8_t green, uint8_t blue) {
    uint16_t r = (red >> 3) & 0x1F;    // 5 bits for red
    uint16_t g = (green >> 2) & 0x3F;  // 6 bits for green
    uint16_t b = (blue >> 3) & 0x1F;   // 5 bits for blue

    return (r << 11) | (g << 5) | b;   // Combine into RGB565
}

uint16_t get_pixel(float u, float v, float col)
{
  float length = sqrt(pow(u*2.0-1.0, 2.0)+pow(v*2.0-1.0, 2.0));
  length -= 1.0;
  if(length < 0.01)
  {
    return rgb888_to_rgb565(col*255, col*255, col*255);
  }
  else
  {
    return rgb888_to_rgb565(0, 0, 0);
  }
}

void app_main(void) 
{


    printf("START");

    spi_bus_config_t buscfg = {
      .mosi_io_num = PIN_NUM_DATA0,
      .sclk_io_num = PIN_NUM_PCLK,
      .miso_io_num = -1,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
    };

    // Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .pclk_hz = 40 * 1000 * 1000,
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


    uint16_t black  = 0b0000000000000000;
    uint16_t red    = 0b1111100000000000;
    uint16_t green  = 0b0000011111100000;
    uint16_t blue   = 0b0000000000011111;




    uint16_t *full_screen_bitmap = (uint16_t *)heap_caps_malloc(sizeof(uint16_t) * LCD_H_RES * LCD_V_RES, MALLOC_CAP_DMA);
    uint16_t *bitmap_dma = (uint16_t *)heap_caps_malloc(sizeof(uint16_t) * 10 * 10, MALLOC_CAP_DMA);
    if (bitmap_dma == NULL) {
      ESP_LOGE("DMA_ALLOC", "Failed to allocate memory for the bitmap");
      return;
    }

    for (int i = 0; i < LCD_H_RES*LCD_V_RES; i++) {
      full_screen_bitmap[i] = black;

      if(i%LCD_H_RES <80)
      {
        full_screen_bitmap[i] = (red >> 8) | ((red & 0xFF) << 8);
      }
      else
      {
        if(i%LCD_H_RES <160)
        {
          full_screen_bitmap[i] = (green >> 8) | ((green & 0xFF) << 8);
        }
        else
        {
          full_screen_bitmap[i] = (blue >> 8) | ((blue & 0xFF) << 8);
        }
      }
    }

    float previous_elapsed_time = esp_timer_get_time()/1000.0/1000.0;
    float accum_time = 0.0;

    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, full_screen_bitmap));
    while (true) 
    {
      float elapsed_time = esp_timer_get_time()/1000.0/1000.0;
      float delta_time = elapsed_time - previous_elapsed_time;
      accum_time += delta_time;

      if(accum_time > 0.0166)
      {
        accum_time = accum_time - 0.0166;

        float col = sin(elapsed_time*2)*0.5+0.5;

        for (int i = 0; i < 10*10; i++) {
          float x = i%10;
          float y = floor(i/10.0);

          uint16_t color = get_pixel(x/9.0,y/9.0, col);
          bitmap_dma[i] = (color >> 8) | ((color & 0xFF) << 8);
        }

        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 115, 115, 125, 125, bitmap_dma));
      }
      previous_elapsed_time = elapsed_time;

      vTaskDelay(2);
    }
}