#include "vector2.h"


static void copy(vec2_t *vec2, vec2_t *self)
{
  self->x = vec2->x;
  self->y = vec2->y;
}
static void add(vec2_t *vec2, vec2_t *self)
{
  self->x += vec2->x;
  self->y += vec2->y;
}

static void multiply_scalar(float scalar, vec2_t *self)
{
  self->x *= scalar;
  self->y *= scalar;
}

static void set(float x, float y, vec2_t *self)
{
  self->x = x;
  self->y = y;
}

vec2_t vec2_new(float x, float y)
{
  vec2_t vec = {
    .x = x,
    .y = y,
    .copy = copy,
    .set = set,
    .multiply_scalar = multiply_scalar,
    .add = add
  };

  return vec;
}
