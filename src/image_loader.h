#ifndef IMAGE_LOADER_H
#define IMAGE_LOADER_H
#include "esp_err.h"


esp_err_t image_loader_load(const char *filename, uint8_t **image_data, uint16_t **color_table);
esp_err_t image_loader_init();

#endif // IMAGE_LOADER_H
