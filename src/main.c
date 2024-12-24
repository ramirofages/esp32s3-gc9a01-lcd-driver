#ifdef APP_MAIN_LCD

  #include <math.h>
  #include <stdio.h>
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "esp_heap_caps.h"

  #include "esp_timer.h"
  #include "esp_log.h"
  #include "esp_err.h"
  #include "esp_system.h"
  #include <string.h>
  #include "ble_manager.h"
  #include "image_loader.h"
  #include "screen_manager.h"
  #include "sprite/sprite.h"
  #include "color_table/color_table.h"

  #define LCD_RES 240*240

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



  void app_main(void) 
  {

      printf("START\n");
      printf("AVAILABLE MEMORY FOR DMA AT INIT: %d \n\n", heap_caps_get_free_size(MALLOC_CAP_DMA));

      image_loader_init();

      screen_manager_t screen_manager;
      screen_manager_init(&screen_manager);


      uint8_t *chicken_image_data = NULL;
      uint8_t *heart_image_data = NULL;
      // uint16_t *heart_color_table = NULL;
      // uint8_t *heart_alpha_table = NULL;

      // uint16_t *chicken_color_table_0 = NULL;
      // uint8_t *chicken_alpha_table_0 = NULL;

      // uint16_t *chicken_color_table_1 = NULL;
      // uint8_t *chicken_alpha_table_1 = NULL;

      color_table_t chicken_color_table_0;
      color_table_t chicken_color_table_1;
      color_table_t heart_color_table;

      ESP_ERROR_CHECK(image_loader_load_image("/spiffs/chicken", &chicken_image_data));
      ESP_ERROR_CHECK(image_loader_load_image("/spiffs/heart", &heart_image_data));

      ESP_ERROR_CHECK(image_loader_load_color_table("/spiffs/chicken_color_table_0", &chicken_color_table_0));
      ESP_ERROR_CHECK(image_loader_load_color_table("/spiffs/chicken_color_table_1", &chicken_color_table_1));
      ESP_ERROR_CHECK(image_loader_load_color_table("/spiffs/heart_color_table",     &heart_color_table));


      sprite_t chicken_sprite_0 = sprite_new(chicken_image_data, &chicken_color_table_0, 64, 64);
      sprite_t chicken_sprite_1 = sprite_new(chicken_image_data, &chicken_color_table_1, 64, 64);
      sprite_t heart_sprite = sprite_new(heart_image_data, &heart_color_table, 64, 64);

      float previous_elapsed_time = esp_timer_get_time()/1000.0f/1000.0f;
      float accum_time = 0.0f;

      float animation_time = 0.0f;



      const float fixed_dt = 0.0166f;
      float speed = 20.0f;

      vec2_t chicken_pos_0 = {120.0f, 120.0f};
      vec2_t chicken_dir_0 = {1.0f, 1.0f};
      
      vec2_t chicken_pos_1 = {45.0f, 70.0f};
      vec2_t chicken_dir_1 = {1.0f, 1.0f};
      
      vec2_t heart_pos = {120.0f, 200.0f};


      const int chicken_res = 64;

      screen_manager_draw(&screen_manager);

      
      printf("AVAILABLE MEMORY AFTER IMAGE SETUP: %d \n\n", heap_caps_get_free_size(MALLOC_CAP_DMA));

      ble_manager_t *ble_manager = NULL;
      ble_manager_init(&ble_manager);
      

      printf("TIME SINCE LAST RECEIVED %f\n", ble_manager->timeSinceLastMessageReceived);
      printf("AVAILABLE MEMORY AFTER NIMBLE SETUP: %d \n\n", heap_caps_get_free_size(MALLOC_CAP_DMA));
      uint16_t background_color = rgb888_to_rgb565(powf(14.0f/255.0f, 0.4545f)*255,powf(63.0f/255.0f, 0.4545f)*255,powf(27.0f/255.0f, 0.4545f)*255);
      background_color = (background_color >> 8) | ((background_color & 0xFF) << 8);
      while (true) 
      {
        float elapsed_time = esp_timer_get_time()/1000.0f/1000.0f;
        float delta_time = elapsed_time - previous_elapsed_time;
        accum_time += delta_time;
        // printf("update: %f \n", delta_time);

        if(accum_time > fixed_dt)
        {

          // float start_measure = esp_timer_get_time()/1000.0f/1000.0f;
          accum_time = accum_time - fixed_dt;
          animation_time += fixed_dt;


          for(int i=0; i< LCD_RES; i++)
          {
            screen_manager.full_screen_bitmap[i] = background_color;
          }
    
          if(chicken_pos_0.x > 180.0f)
          {
            chicken_dir_0.x = -1.0f;
          }
          if(chicken_pos_0.x < 60.0f)
          {
            chicken_dir_0.x = 1.0f;
          }

          if(chicken_pos_0.y > 200.0f)
          {
            chicken_dir_0.y = -1.0f;
          }
          if(chicken_pos_0.y < 40.0f)
          {
            chicken_dir_0.y = 1.0f;
          }
          chicken_pos_0.x += fixed_dt * chicken_dir_0.x * speed;
          chicken_pos_0.y += fixed_dt * chicken_dir_0.y * speed;



          if(chicken_pos_1.x > 180.0f)
          {
            chicken_dir_1.x = -1.0f;
          }
          if(chicken_pos_1.x < 60.0f)
          {
            chicken_dir_1.x = 1.0f;
          }

          if(chicken_pos_1.y > 200.0f)
          {
            chicken_dir_1.y = -1.0f;
          }
          if(chicken_pos_1.y < 40.0f)
          {
            chicken_dir_1.y = 1.0f;
          }
          chicken_pos_1.x += fixed_dt * chicken_dir_1.x * speed;
          chicken_pos_1.y += fixed_dt * chicken_dir_1.y * speed;
          // printf("time difference %f \n",elapsed_time - ble_manager->timeSinceLastMessageReceived );
          if(elapsed_time - ble_manager->timeSinceLastMessageReceived > 10.0f)
          {
            screen_manager_draw_sprite(&screen_manager, &chicken_sprite_0, (int)chicken_pos_0.x, (int)chicken_pos_0.y, chicken_res, chicken_dir_0.x < 0.0f);
          }
          else
          {
            if(chicken_pos_0.y > chicken_pos_1.y)
            {
              screen_manager_draw_sprite(&screen_manager, &chicken_sprite_0, (int)chicken_pos_0.x, (int)chicken_pos_0.y, chicken_res, chicken_dir_0.x < 0.0f);
              screen_manager_draw_sprite(&screen_manager, &chicken_sprite_1, (int)chicken_pos_1.x, (int)chicken_pos_1.y, chicken_res, chicken_dir_1.x < 0.0f);
            }
            else
            {
              screen_manager_draw_sprite(&screen_manager, &chicken_sprite_1, (int)chicken_pos_1.x, (int)chicken_pos_1.y, chicken_res, chicken_dir_1.x < 0.0f);
              screen_manager_draw_sprite(&screen_manager, &chicken_sprite_0, (int)chicken_pos_0.x, (int)chicken_pos_0.y, chicken_res, chicken_dir_0.x < 0.0f);
            }
          }
          
          const int heart_x = (int)heart_pos.x;
          const int heart_y = (int)heart_pos.y;
          
          if(elapsed_time - ble_manager->timeSinceLastMessageReceived < 1.0f)
          {
            screen_manager_draw_sprite(&screen_manager, &heart_sprite, heart_x, heart_y, chicken_res, false);
          }

          screen_manager_draw(&screen_manager);

          // float delta_measure = (esp_timer_get_time()/1000.0f/1000.0f)-start_measure;
          // printf("update time %f\n", delta_measure);
        }
        previous_elapsed_time = elapsed_time;

        vTaskDelay(1);
      }
  }

#endif
