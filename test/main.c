#include <stdio.h>
#include "../src/app/bird/bird.h"
#include "../src/app/bird/bird.c"

int main (int argc, char *argv[]) 
{  
  bird_t bird = bird_new();

  bird.update(&bird, 1.0f);

  printf("ELAPSED TIME %f\n", bird.elapsed_seconds_accumulator);
  return 0;
}