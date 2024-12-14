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
#define LCD_RES 240*240
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8

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


#define SINE_TABLE_SIZE 360
static float sine_table[SINE_TABLE_SIZE];

void initialize_sine_table() {
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
        sine_table[i] = sinf((i/359.0f)* 6.283164f);
    }
}

float fast_sine_01(float t)
{
  return sine_table[(int)(t*359)];
}
// float fast_sine(float angle) {
//     int index = ((int)angle) % SINE_TABLE_SIZE;
//     if (index < 0) index += SINE_TABLE_SIZE; // Handle negative angles
//     return sine_table[index];
// }

uint16_t get_pixel(float u, float v, float time)
{
  // float wave = 0.1;
  
  // float wave = sinf(u * 6.283164f + time)*0.2f;
  float val = u  + time;
  val = val - (int)val;
  float wave = fast_sine_01(val)*0.2f;

  float y = v*2.0f-1.0f;

  uint8_t color = fabsf(wave-y) < 0.05f? 255 : 0;
  return rgb888_to_rgb565(color, color, color);
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


    uint16_t *full_screen_bitmap = (uint16_t *)heap_caps_malloc(sizeof(uint16_t) * LCD_RES, MALLOC_CAP_DMA);
    if (full_screen_bitmap == NULL) {
      ESP_LOGE("DMA_ALLOC", "Failed to allocate memory for the bitmap");
      return;
    }
    // printf("AVAILABLE MEMORY %d \n\n", heap_caps_get_free_size(MALLOC_CAP_DMA));
    vec2_t *uvs = (vec2_t *)malloc(sizeof(vec2_t) * LCD_RES);
    if (uvs == NULL) {
      ESP_LOGE("DMA_ALLOC", "Failed to allocate memory for the uvs");
      return;
    }
    for (int i = 0; i < LCD_RES; i++) 
    {
      float u = (i%LCD_H_RES)/(LCD_H_RES-1.0f);
      float v = floor(i/LCD_H_RES)/(LCD_V_RES-1.0f);
      uvs[i] = create_vec2(u, v);
    }

    initialize_sine_table();


    float previous_elapsed_time = esp_timer_get_time()/1000.0f/1000.0f;
    float accum_time = 0.0f;

    float animation_time = 0.0f;

    // ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, full_screen_bitmap));
    while (true) 
    {
      float elapsed_time = esp_timer_get_time()/1000.0f/1000.0f;
      float delta_time = elapsed_time - previous_elapsed_time;
      accum_time += delta_time;
      printf("update: %f \n", delta_time);

      if(accum_time > 0.0166f)
      {
        accum_time = accum_time - 0.0166f;
        animation_time += 0.0166f;

        float t = animation_time*0.5f;

        for (int i = 0; i < LCD_RES; i++) {
          
          uint16_t color = get_pixel(uvs[i].x, uvs[i].y, t);
          full_screen_bitmap[i] = (color >> 8) | ((color & 0xFF) << 8);
        }

        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, full_screen_bitmap));
      }
      previous_elapsed_time = elapsed_time;

      vTaskDelay(1);
    }
}