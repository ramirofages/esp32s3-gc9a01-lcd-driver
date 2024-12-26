#include "bird.h"


static void update(float delta_time, bird_t *self)
{
  self->elapsed_seconds_accumulator += delta_time;

  
  if(self->elapsed_seconds_accumulator-0.00001f > 1.0f)
  {
    self->elapsed_seconds++;
    if(self->elapsed_seconds > 4 && self->growth_state < self->max_growth_states)
    {
      self->growth_state++;
    }
  }

  if(self->position.x > 180.0f)
  {
    self->direction.x = -1.0f;
  }
  if(self->position.x < 60.0f)
  {
    self->direction.x = 1.0f;
  }

  if(self->position.y > 200.0f)
  {
    self->direction.y = -1.0f;
  }
  if(self->position.y < 40.0f)
  {
    self->direction.y = 1.0f;
  }

  if(self->growth_state == 3)
  {
    self->position.x += delta_time * self->direction.x * self->speed;
    self->position.y += delta_time * self->direction.y * self->speed;
  }
}

static void set_position(float x, float y, bird_t *self)
{
  self->position.set(x,y, &self->position);
}
static void set_direction(float x, float y, bird_t *self)
{
  self->direction.set(x, y, &self->direction);
}


bird_t bird_new()
{
  bird_t bird = {
    .elapsed_seconds = 0,
    .elapsed_seconds_accumulator = 0.0f,
    .update = update,
    .set_position = set_position,
    .set_direction = set_direction,
    .growth_state = 0,
    .max_growth_states = 3,
    .position = vec2_new(0.0f, 0.0f),
    .direction = vec2_new(0.0f, 0.0f),
    .speed = 0.0f
  };
  return bird;
}