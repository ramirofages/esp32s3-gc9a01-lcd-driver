#ifndef IMAGE_LOADER_H
#define IMAGE_LOADER_H
#include "esp_err.h"


esp_err_t image_loader_load_image(const char *filename, uint8_t **image_data);
esp_err_t image_loader_load_color_table(const char *filename, uint16_t **color_table, uint8_t **alpha_table);
esp_err_t image_loader_init();

#endif // IMAGE_LOADER_H
