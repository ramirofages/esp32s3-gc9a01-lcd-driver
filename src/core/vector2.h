#ifndef VECTOR2_H
#define VECTOR2_H

#include <stdint.h>


typedef struct vec2_t vec2_t;

struct vec2_t
{
  float x;
  float y;
  void (*multiply_scalar) (float scalar,      vec2_t *self);
  void (*set)             (float x, float y,  vec2_t *self);
  void (*copy)            (vec2_t *vec2,      vec2_t *self);
  void (*add)             (vec2_t *vec2,      vec2_t *self);
};


vec2_t vec2_new(float x, float y);

#endif