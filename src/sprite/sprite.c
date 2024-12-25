#include "sprite.h"
#include "color_table/color_table.h"

sprite_t sprite_new(uint8_t *bitmap, color_table_t *color_table, int width, int height)
{
  sprite_t sprite;
  sprite.bitmap = bitmap;
  sprite.color_table = color_table;
  sprite.width = width;
  sprite.height = height;
  sprite.pos_x = 0.0f;
  sprite.pos_y = 0.0f;
  sprite.dir_x = 0.0f;
  sprite.dir_x = 0.0f;
  return sprite;
}

void sprite_set_position(sprite_t *sprite, float x, float y)
{
  sprite->pos_x = x;
  sprite->pos_y = y;
}