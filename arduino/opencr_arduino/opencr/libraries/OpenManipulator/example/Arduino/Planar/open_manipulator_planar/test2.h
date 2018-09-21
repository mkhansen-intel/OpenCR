#ifndef TEST2_H_
#define TEST2_H_

#include "Planar.h"

#define MAX_MOTION_NUM 5

uint8_t motion_erase = 1;
uint8_t motion_page = 20;
uint8_t motion_repeat = 0;

float start_angular_position = 0.0f;
const float move_time = 30.0f;
float init_arg[2] = {move_time, ACTUATOR_CONTROL_TIME};
void *p_init_arg = init_arg;
float radius = 0.020f;

bool motion[1] = {false};


void test2()
{
  if (planar.moving() || planar.drawing())
  {
    return;
  }
  else
  {
    if (motion_page == SPIRAL)
    {
      radius = 0.015f;
      planar.drawInit(SPIRAL, move_time, p_init_arg);
      planar.setRadiusForDrawing(SPIRAL, radius);
      planar.setStartPositionForDrawing(SPIRAL, planar.getComponentPositionToWorld(TOOL));
      planar.setStartAngularPositionForDrawing(SPIRAL, start_angular_position);
      planar.draw(SPIRAL);

      motion_page++;
    }
    else
    {
      radius = 0.015f;
      planar.drawInit(SPIRAL2, move_time, p_init_arg);
      planar.setRadiusForDrawing(SPIRAL2, radius);
      planar.setStartPositionForDrawing(SPIRAL2, planar.getComponentPositionToWorld(TOOL));
      planar.setStartAngularPositionForDrawing(SPIRAL2, start_angular_position);
      planar.draw(SPIRAL2);
      motion_page=20;
    }
  }
}
#endif // TEST2_H_
