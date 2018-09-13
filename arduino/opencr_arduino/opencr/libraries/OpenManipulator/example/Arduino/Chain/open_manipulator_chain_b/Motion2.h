#define CIRCLE 11
#define RHOMBUS 13
#define HEART 15
#define BOTTLESHAKE 17

//------------------------------//

OPEN_MANIPULATOR::Draw *circle = new OM_PATH::Circle();
OPEN_MANIPULATOR::Draw *rhombus = new OM_PATH::Rhombus();
OPEN_MANIPULATOR::Draw *heart = new OM_PATH::Heart();
OPEN_MANIPULATOR::Draw *bottleshake = new OM_PATH::BottleShake();

//------------------------------//



//-------------Draw Objects-------------//
void drawObj(OpenManipulator* manipulator, float move_time, float object, float radius, float start_angular_position)
{
  float init_arg[2] = {move_time, ACTUATOR_CONTROL_TIME};
  void *p_init_arg = init_arg;
  manipulator->drawInit(object, move_time, p_init_arg);
  manipulator->setRadiusForDrawing(object, radius);  
  manipulator->setStartPositionForDrawing(object, manipulator->getComponentPositionToWorld(TOOL));
  manipulator->setStartAngularPositionForDrawing(object, start_angular_position);
  manipulator->draw(object);
}

//------------------------------//

bool shakeMotion4(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator, 0, -1.05, 0.35,  0.7, 2.0);     break;
  case 1: drawObj(manipulator, 2.0, BOTTLESHAKE, 0.030, 0.0); break;
  case 2: moveJS(manipulator, 1.0, -1.05, 0.35,  0.7, 2.0);   break;
  case 3: drawObj(manipulator, 2.0, BOTTLESHAKE, 0.030, 0.0); break;
  case 4: moveJS(manipulator, -1.0, -1.05, 0.35,  0.7, 2.0);  break;
  case 5: drawObj(manipulator, 2.0, BOTTLESHAKE, 0.030, 0.0); 
          return 1; break;
  }
  return 0;
}

//------------------------------//
    case 1:
      if(shakeMotion4(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
//------------------------------//
