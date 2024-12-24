#ifndef COLOR_TABLE_H
  #define COLOR_TABLE_H

#include <stdio.h>

typedef struct {
  uint16_t *color_array;
  uint8_t *alpha_array;
}  color_table_t;


color_table_t color_table_new(uint16_t* color_table, uint8_t* alpha_table);
#endif // COLOR_TABLE_H