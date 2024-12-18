#include "esp_err.h"
#include "esp_spiffs.h"

esp_err_t image_loader_load(const char *filename, uint8_t **image_data, uint16_t **color_table) {

    printf("LOAD IMAGE\n");

    // Open the file
    FILE *file = fopen(filename, "rb");
    if (!file) {
        printf("Failed to open file %s\n", filename);
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
        printf("Failed to allocate memory for file data\n");
        fclose(file);
        return ESP_ERR_NO_MEM;
    }
    if (*color_table == NULL) {
        printf("Failed to allocate memory for file data\n");
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


esp_err_t image_loader_init()
{
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


    if (ret != ESP_OK) 
    {
      if (ret == ESP_FAIL) {
        printf("Failed to mount or format filesystem\n");
      } else if (ret == ESP_ERR_NOT_FOUND) {
        printf("Failed to find SPIFFS partition\n");
      } else {
        printf("Failed to initialize SPIFFS (%s)\n", esp_err_to_name(ret));
      }
      return ret;
    }

    return ESP_OK;
}
