#ifndef SPRITE_H
#define SPRITE_H

#include <stdio.h>
#include "color_table/color_table.h"
typedef struct {
  int width;
  int height;
  uint8_t *bitmap;
  color_table_t *color_table;
  float pos_x;
  float pos_y;
  float dir_x;
  float dir_y;
}  sprite_t;


sprite_t sprite_new(uint8_t *bitmap, color_table_t *color_table, int width, int height);
void sprite_set_position(sprite_t *sprite, float x, float y);
#endif // SPRITE_H