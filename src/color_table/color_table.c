#include "color_table.h"

color_table_t color_table_new(uint16_t* color_array, uint8_t* alpha_array)
{
  color_table_t color_table;
  color_table.color_array = color_array;
  color_table.alpha_array = alpha_array;
  return color_table;
}
