#include "sprite.h"
#include "color_table/color_table.h"

sprite_t sprite_new(uint8_t *bitmap, color_table_t *color_table, int width, int height)
{
  sprite_t sprite;
  sprite.bitmap = bitmap;
  sprite.color_table = color_table;
  sprite.width = width;
  sprite.height = height;
  sprite.position = vec2_new(0.0f, 0.0f);
  sprite.mirrored = false;
  
  return sprite;
}

void sprite_set_position(float x, float y, sprite_t *sprite)
{
  sprite->position.x = x;
  sprite->position.y = y;
}