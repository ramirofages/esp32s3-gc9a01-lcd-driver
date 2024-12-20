#include "esp_err.h"
#include "esp_spiffs.h"

esp_err_t image_loader_load(const char *filename, uint8_t **image_data, uint16_t **color_table, uint8_t **alpha_table) {

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

    long color_table_size = 16 * sizeof(uint16_t);
    long alpha_table_size = 16 * sizeof(uint8_t);
    long header_size = color_table_size + alpha_table_size;

    *image_data = (uint8_t*)malloc(size-header_size);
    *color_table = (uint16_t*)malloc(color_table_size);
    *alpha_table = (uint8_t*) malloc(alpha_table_size);


    printf("COLOR TABLE SIZE %ld\n", color_table_size);
    printf("ALPHA TABLE SIZE %ld\n", alpha_table_size);
    printf("HEADER DATA SIZE %ld\n", header_size);
    printf("IMAGE DATA SIZE %ld\n", size-header_size);

    if (*image_data == NULL) {
        printf("Failed to allocate memory for file image data\n");
        fclose(file);
        return ESP_ERR_NO_MEM;
    }
    if (*color_table == NULL) {
        printf("Failed to allocate memory for file color table data\n");
        fclose(file);
        return ESP_ERR_NO_MEM;
    }
    if (*alpha_table == NULL) {
        printf("Failed to allocate memory for file alpha table data\n");
        fclose(file);
        return ESP_ERR_NO_MEM;
    }

    printf("MEMORY ALOCATED \n");


    for(int i=0; i< 16; i++)
    {
      fseek(file, sizeof(uint8_t) * 3 * i, SEEK_SET);
      fread(&((*color_table)[i]), sizeof(uint16_t), 1, file);


      fseek(file, sizeof(uint8_t) * 3 * i + sizeof(uint16_t), SEEK_SET);
      fread(&((*alpha_table)[i]), sizeof(uint8_t), 1, file);

      // (*color_table)[i] = (((*table)[i*3+0])) | ((*table)[i*3+1] << 8);
    }
    printf("TABLE READ \n");

    fseek(file, header_size, SEEK_SET);
    fread(*image_data, sizeof(uint8_t), (size-header_size), file);
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
