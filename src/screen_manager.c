#include "screen_manager.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "esp_lcd_gc9a01.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"

#define LCD_HOST SPI2_HOST
#define PIN_NUM_DATA0          11  
#define PIN_NUM_PCLK           10
#define PIN_NUM_CS             9
#define PIN_NUM_DC             8
#define PIN_NUM_RST            14
#define PIN_NUM_BK_LIGHT       2
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240
esp_err_t screen_manager_init(screen_manager_t *screen_manager)
{
  spi_bus_config_t buscfg = {
      .mosi_io_num = PIN_NUM_DATA0,
      .sclk_io_num = PIN_NUM_PCLK,
      .miso_io_num = -1,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(uint16_t),
    };

    // Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .pclk_hz = 80 * 1000 * 1000,
        .dc_gpio_num = PIN_NUM_DC,
        .cs_gpio_num = PIN_NUM_CS,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
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

    screen_manager->io_handle = io_handle;
    screen_manager->panel_handle = panel_handle;
    
    size_t full_screen_size = sizeof(uint16_t) * SCREEN_WIDTH * SCREEN_HEIGHT;
    screen_manager->full_screen_bitmap = (uint16_t *)heap_caps_malloc(full_screen_size, MALLOC_CAP_DMA);
    if (screen_manager->full_screen_bitmap == NULL) {
      ESP_LOGE("DMA_ALLOC", "Failed to allocate memory for the full screen bitmap, needed: %d, largest free block has %d", full_screen_size, heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
      return ESP_ERR_NO_MEM;
    }

    screen_manager->width = SCREEN_WIDTH;
    screen_manager->height = SCREEN_HEIGHT;


    return ESP_OK;
}

esp_err_t screen_manager_draw(screen_manager_t *screen_manager)
{
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(screen_manager->panel_handle, 0, 0, screen_manager->width, screen_manager->height, screen_manager->full_screen_bitmap));
    return ESP_OK;
}
