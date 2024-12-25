#ifndef IMAGE_LOADER_H
#define IMAGE_LOADER_H
#include "esp_err.h"
#include "color_table/color_table.h"

esp_err_t image_loader_load_image(const char *filename, uint8_t **image_data);
esp_err_t image_loader_load_color_table(const char *filename, color_table_t *color_table);
esp_err_t image_loader_init();

#endif // IMAGE_LOADER_H
