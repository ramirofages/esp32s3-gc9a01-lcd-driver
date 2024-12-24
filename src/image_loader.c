#include "esp_err.h"
#include "esp_spiffs.h"
#include "esp_err.h"
#include "sprite/sprite.h"
#include "color_table/color_table.h"

esp_err_t image_loader_load_image(const char *filename, uint8_t **image_data) {

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


    *image_data = (uint8_t*)malloc(size);

    printf("IMAGE DATA SIZE %ld\n", size);

    if (*image_data == NULL) {
        printf("Failed to allocate memory for file image data\n");
        fclose(file);
        return ESP_ERR_NO_MEM;
    }

    printf("MEMORY ALOCATED \n");


    fread(*image_data, sizeof(uint8_t), size, file);
    fclose(file);


    printf("FILE READ \n");

    return ESP_OK;
}

esp_err_t image_loader_load_color_table(const char *filename, color_table_t *color_table)
{
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
    
    
    // *color_table = (uint16_t*)malloc(color_table_size);
    // *alpha_table = (uint8_t*) malloc(alpha_table_size);


    printf("COLOR TABLE SIZE %ld\n", color_table_size);
    printf("ALPHA TABLE SIZE %ld\n", alpha_table_size);
    printf("HEADER DATA SIZE %ld\n", header_size);
    printf("IMAGE DATA SIZE %ld\n", size-header_size);




    color_table->color_array = (uint16_t*)malloc(color_table_size);

    if (color_table->color_array == NULL) {
        printf("Failed to allocate memory for file color table data\n");
        fclose(file);
        return ESP_ERR_NO_MEM;
    }

    color_table->alpha_array = (uint8_t*) malloc(alpha_table_size);

    if (color_table->alpha_array == NULL) {
        printf("Failed to allocate memory for file alpha table data\n");
        fclose(file);
        return ESP_ERR_NO_MEM;
    }

    printf("MEMORY ALOCATED \n");


    for(int i=0; i< 16; i++)
    {
      fseek(file, sizeof(uint8_t) * 3 * i, SEEK_SET);
      fread(&((color_table->color_array)[i]), sizeof(uint16_t), 1, file);


      fseek(file, sizeof(uint8_t) * 3 * i + sizeof(uint16_t), SEEK_SET);
      fread(&((color_table->alpha_array)[i]), sizeof(uint8_t), 1, file);
    }
    printf("TABLE READ \n");
    fclose(file);


    printf("FILE READ \n");

    return ESP_OK;
}

sprite_t image_loader_load_sprite(const char *bitmap_filename, const char *color_table_filename, int width, int height)
{
  uint8_t *bitmap = NULL;
  uint16_t *color_table = NULL;
  uint8_t *alpha_table = NULL;

  ESP_ERROR_CHECK(image_loader_load_image(*bitmap_filename, &bitmap));
  ESP_ERROR_CHECK(image_loader_load_color_table(*color_table_filename, &color_table, &alpha_table));

  return sprite_new(bitmap, color_table, alpha_table, width, height);
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
