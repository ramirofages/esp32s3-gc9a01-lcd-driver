#ifndef SCREEN_MANAGER_H
#define SCREEN_MANAGER_H

#include "esp_err.h"
#include "esp_lcd_panel_io_interface.h"

typedef struct {
  esp_lcd_panel_io_handle_t io_handle;
  esp_lcd_panel_handle_t panel_handle;
  uint16_t *full_screen_bitmap;
  int width;
  int height;
}  screen_manager_t;

esp_err_t screen_manager_init(screen_manager_t *screen_manager);
esp_err_t screen_manager_draw(screen_manager_t *screen_manager);
esp_err_t screen_manager_draw_bitmap_with_color_table(screen_manager_t *screen_manager, uint8_t *bitmap, uint16_t *color_table, int pos_x, int pos_y, int bitmap_width, bool mirrored);


#endif 