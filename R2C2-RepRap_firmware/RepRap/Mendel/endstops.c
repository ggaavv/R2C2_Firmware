#include <stdbool.h>

#include "config.h"
#include "pinout.h"

bool hit_home_stop_x (unsigned dir)
{
  if (config.home_direction_x < 0)
  {
    return !x_min();
  }
  else
  {
    return !x_max();
  }
}


bool hit_home_stop_y (unsigned dir)
{
  if (config.home_direction_y < 0)
  {
    return !y_min();
  }
  else
  {
    return !y_max();
  }
}

bool hit_home_stop_z (unsigned dir)
{
  if (config.home_direction_z < 0)
  {
    return !z_min();
  }
  else
  {
    return !z_max();
  }
}



