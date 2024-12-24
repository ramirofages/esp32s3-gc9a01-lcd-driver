#ifndef SPRITE_H
  #define SPRITE_H

#include <stdio.h>
#include "color_table/color_table.h"
typedef struct {
  int width;
  int height;
  uint8_t *bitmap;
  color_table_t *color_table;
}  sprite_t;


sprite_t sprite_new(uint8_t *bitmap, color_table_t *color_table, int width, int height);
#endif // SPRITE_H