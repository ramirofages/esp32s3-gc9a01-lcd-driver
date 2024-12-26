#ifndef BIRD_H
#define BIRD_H

#include <stdint.h>
#include "core/vector2.h"

typedef struct bird_t bird_t;

struct bird_t
{
  uint32_t elapsed_seconds;
  float elapsed_seconds_accumulator;
  uint8_t growth_state;
  uint8_t max_growth_states;

  vec2_t position;
  vec2_t direction;
  float speed;

  void (*update)       ( float delta_time, bird_t *self);
  void (*set_position) ( float x, float y, bird_t *self);
  void (*set_direction)( float x, float y, bird_t *self);
};


bird_t bird_new();
#endif