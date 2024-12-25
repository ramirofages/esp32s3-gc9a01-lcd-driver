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
      uint8_t *egg_0_image_data = NULL;
      uint8_t *egg_1_image_data = NULL;
      uint8_t *egg_2_image_data = NULL;
      uint8_t *heart_image_data = NULL;

      color_table_t chicken_color_table_0;
      color_table_t chicken_color_table_1;
      color_table_t heart_color_table;
      color_table_t egg_0_color_table;
      color_table_t egg_1_color_table;
      color_table_t egg_2_color_table;

      ESP_ERROR_CHECK(image_loader_load_image("/spiffs/chicken", &chicken_image_data));
      ESP_ERROR_CHECK(image_loader_load_image("/spiffs/egg_0", &egg_0_image_data));
      ESP_ERROR_CHECK(image_loader_load_image("/spiffs/egg_1", &egg_1_image_data));
      ESP_ERROR_CHECK(image_loader_load_image("/spiffs/egg_2", &egg_2_image_data));
      ESP_ERROR_CHECK(image_loader_load_image("/spiffs/heart", &heart_image_data));

      ESP_ERROR_CHECK(image_loader_load_color_table("/spiffs/chicken_color_table_0", &chicken_color_table_0));
      ESP_ERROR_CHECK(image_loader_load_color_table("/spiffs/chicken_color_table_1", &chicken_color_table_1));
      ESP_ERROR_CHECK(image_loader_load_color_table("/spiffs/heart_color_table",     &heart_color_table));
      ESP_ERROR_CHECK(image_loader_load_color_table("/spiffs/egg_0_color_table",     &egg_0_color_table));
      ESP_ERROR_CHECK(image_loader_load_color_table("/spiffs/egg_1_color_table",     &egg_1_color_table));
      ESP_ERROR_CHECK(image_loader_load_color_table("/spiffs/egg_2_color_table",     &egg_2_color_table));


      sprite_t egg_0_sprite     = sprite_new(egg_0_image_data, &egg_0_color_table, 64, 64);
      sprite_t heart_sprite     = sprite_new(heart_image_data, &heart_color_table, 64, 64);
      sprite_t chicken_sprite_0 = sprite_new(chicken_image_data, &chicken_color_table_0, 64, 64);
      sprite_t chicken_sprite_1 = sprite_new(chicken_image_data, &chicken_color_table_1, 64, 64);

      sprite_t egg_1_sprite = sprite_new(egg_1_image_data, &egg_1_color_table, 64, 64);
      sprite_t egg_2_sprite = sprite_new(egg_2_image_data, &egg_2_color_table, 64, 64);

      float previous_elapsed_time = esp_timer_get_time()/1000.0f/1000.0f;
      float accum_time = 0.0f;

      float animation_time = 0.0f;



      const float fixed_dt = 0.0166f;
      float speed = 20.0f;

      sprite_set_position(&chicken_sprite_0, 120.0f, 120.0f);
      sprite_set_position(&egg_0_sprite, 100.0f, 120.0f);
      sprite_set_position(&egg_1_sprite, 140.0f, 120.0f);
      sprite_set_position(&egg_2_sprite, 180.0f, 120.0f);

      chicken_sprite_0.dir_x = 1.0f;
      chicken_sprite_0.dir_y = 1.0f;

      chicken_sprite_1.pos_x = 45.0f;
      chicken_sprite_1.pos_y = 70.0f;
      chicken_sprite_1.dir_x = 1.0f;
      chicken_sprite_1.dir_y = 1.0f;
      
      heart_sprite.pos_x = 120.0f;
      heart_sprite.pos_y = 120.0f;

      printf("AVAILABLE MEMORY AFTER IMAGE SETUP: %d \n\n", heap_caps_get_free_size(MALLOC_CAP_DMA));

      ble_manager_t *ble_manager = NULL;
      ble_manager_init(&ble_manager);
      

      printf("TIME SINCE LAST RECEIVED %f\n", ble_manager->timeSinceLastMessageReceived);
      printf("AVAILABLE MEMORY AFTER NIMBLE SETUP: %d \n\n", heap_caps_get_free_size(MALLOC_CAP_DMA));
      uint16_t background_color = rgb888_to_rgb565(powf(14.0f/255.0f, 0.4545f)*255,powf(63.0f/255.0f, 0.4545f)*255,powf(27.0f/255.0f, 0.4545f)*255);
      background_color = (background_color >> 8) | ((background_color & 0xFF) << 8);


      for(int i=0; i< LCD_RES; i++)
      {
        screen_manager.full_screen_bitmap[i] = background_color;
      }

      screen_manager_draw(&screen_manager);

      bool screen_needs_update = false;
      
      while (true) 
      {
        float elapsed_time = esp_timer_get_time()/1000.0f/1000.0f;
        float delta_time = elapsed_time - previous_elapsed_time;
        accum_time += delta_time;
        // printf("update: %f \n", delta_time);
        // printf("Is transfering data: %s\n", screen_manager.busy_transfering ? "true" : "false");
        // printf("Is transfering data: %s\n", screen_manager.busy_transfering? "true":"false");

        if(accum_time > fixed_dt)
        {
          // float start_measure = esp_timer_get_time()/1000.0f/1000.0f;
          accum_time = accum_time - fixed_dt;
          animation_time += fixed_dt;



    
          if(chicken_sprite_0.pos_x > 180.0f)
          {
            chicken_sprite_0.dir_x = -1.0f;
          }
          if(chicken_sprite_0.pos_x < 60.0f)
          {
            chicken_sprite_0.dir_x = 1.0f;
          }

          if(chicken_sprite_0.pos_y > 200.0f)
          {
            chicken_sprite_0.dir_y = -1.0f;
          }
          if(chicken_sprite_0.pos_y < 40.0f)
          {
            chicken_sprite_0.dir_y = 1.0f;
          }
          chicken_sprite_0.pos_x += fixed_dt * chicken_sprite_0.dir_x * speed;
          chicken_sprite_0.pos_y += fixed_dt * chicken_sprite_0.dir_y * speed;



    
          if(chicken_sprite_1.pos_x > 180.0f)
          {
            chicken_sprite_1.dir_x = -1.0f;
          }
          if(chicken_sprite_1.pos_x < 60.0f)
          {
            chicken_sprite_1.dir_x = 1.0f;
          }

          if(chicken_sprite_1.pos_y > 200.0f)
          {
            chicken_sprite_1.dir_y = -1.0f;
          }
          if(chicken_sprite_1.pos_y < 40.0f)
          {
            chicken_sprite_1.dir_y = 1.0f;
          }
          chicken_sprite_1.pos_x += fixed_dt * chicken_sprite_1.dir_x * speed;
          chicken_sprite_1.pos_y += fixed_dt * chicken_sprite_1.dir_y * speed;

          // printf("time difference %f \n",elapsed_time - ble_manager->timeSinceLastMessageReceived );
          
          screen_needs_update = true;  
          

          // float delta_measure = (esp_timer_get_time()/1000.0f/1000.0f)-start_measure;
          // printf("update time %f\n", delta_measure);
        }

        if(screen_needs_update == true && screen_manager.busy_transfering == false)
        {
          for(int i=0; i< LCD_RES; i++)
          {
            screen_manager.full_screen_bitmap[i] = background_color;
          }

          if(elapsed_time - ble_manager->timeSinceLastMessageReceived > 10.0f)
          {
            screen_manager_draw_sprite(&screen_manager, &chicken_sprite_0);
            screen_manager_draw_sprite(&screen_manager, &egg_0_sprite);
            screen_manager_draw_sprite(&screen_manager, &egg_1_sprite);
            screen_manager_draw_sprite(&screen_manager, &egg_2_sprite);
          }
          else
          {
            if(chicken_sprite_0.pos_y > chicken_sprite_1.pos_y)
            {
              screen_manager_draw_sprite(&screen_manager, &chicken_sprite_0);
              screen_manager_draw_sprite(&screen_manager, &chicken_sprite_1);
            }
            else
            {
              screen_manager_draw_sprite(&screen_manager, &chicken_sprite_1);
              screen_manager_draw_sprite(&screen_manager, &chicken_sprite_0);
            }
          }
          
          if(elapsed_time - ble_manager->timeSinceLastMessageReceived < 1.0f)
          {
            screen_manager_draw_sprite(&screen_manager, &heart_sprite);
          }
          screen_manager_draw(&screen_manager);
          
        }
        previous_elapsed_time = elapsed_time;

        vTaskDelay(1);
      }
  }

#endif
