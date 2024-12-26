#ifndef SPRITE_H
#define SPRITE_H

#include <stdio.h>
#include <stdbool.h>
#include "color_table/color_table.h"
#include "core/vector2.h"

typedef struct {
  int width;
  int height;
  uint8_t *bitmap;
  color_table_t *color_table;
  vec2_t position;
  bool mirrored;
}  sprite_t;


sprite_t sprite_new(uint8_t *bitmap, color_table_t *color_table, int width, int height);
void sprite_set_position(float x, float y, sprite_t *sprite);
#endif // SPRITE_H