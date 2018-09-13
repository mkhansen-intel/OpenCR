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







class BottleShake3 : public OPEN_MANIPULATOR::Draw
{
private:
  MinimumJerk path_generator_;
  MatrixXf coefficient_;

  uint8_t joint_num_;

  Vector3f start_position_;
  float radius_;
  float start_angular_position_;

  float *get_arg_;

public:
  BottleShake3();
  virtual ~BottleShake3();

  void init(float move_time, float control_time);
  Pose bottleshake3(float time_var);

  MatrixXf getCoefficient();

  virtual void initDraw(const void *arg);
  virtual void setRadius(float radius);  
  virtual void setStartPosition(Vector3f start_position);
  virtual void setAngularStartPosition(float start_angular_position);
  virtual Pose getPose(float tick);
};





//-------------------- BottleShake3 --------------------//

BottleShake3::BottleShake3() {}
BottleShake3::~BottleShake3() {}

void BottleShake3::init(float move_time, float control_time)
{
  Trajectory start;
  Trajectory goal;

  start.position = 0.0;
  start.velocity = 0.0;
  start.acceleration = 0.0;

  goal.position = 6 * M_PI;
  goal.velocity = 0.0;
  goal.acceleration = 0.0;

  path_generator_.calcCoefficient(start,
                                  goal,
                                  move_time,
                                  control_time);

  coefficient_ = path_generator_.getCoefficient();
}

void BottleShake3::setStartPosition(Vector3f start_position)
{
  start_position_ = start_position;
}

void BottleShake3::setAngularStartPosition(float start_angular_position)
{
  start_angular_position_ = start_angular_position;
}

void BottleShake3::setRadius(float radius)
{
  radius_ = radius;
}

Pose BottleShake3::bottleshake3(float time_var)
{
  Pose pose;
  double obj_pose[3];
  double diff_pose[3];

  obj_pose[0] = (cos(time_var)-1)*cos(start_angular_position_) - sin(time_var)*sin(start_angular_position_);
  obj_pose[1] = (cos(time_var)-1)*sin(start_angular_position_) + sin(time_var)*cos(start_angular_position_);
  obj_pose[2] = 0;

  diff_pose[0] = -sin(PI/4)*obj_pose[0] + sin(PI/4)*obj_pose[2];
  diff_pose[1] = obj_pose[1];
  diff_pose[2] = cos(PI/4)*obj_pose[0] + cos(PI/4)*obj_pose[2];

  pose.position(0) = start_position_(0) + radius_ * diff_pose[0];
  pose.position(1) = start_position_(1) + radius_ * diff_pose[1];
  pose.position(2) = start_position_(2) + radius_ * diff_pose[2];

  return pose;
}

Pose BottleShake3::getPose(float tick)
{
  float get_time_var = 0.0;

  get_time_var = coefficient_(0) +
                 coefficient_(1) * pow(tick, 1) +
                 coefficient_(2) * pow(tick, 2) +
                 coefficient_(3) * pow(tick, 3) +
                 coefficient_(4) * pow(tick, 4) +
                 coefficient_(5) * pow(tick, 5);

  return bottleshake3(get_time_var);
}

void BottleShake3::initDraw(const void *arg)
{
  get_arg_ = (float *)arg;

  init(get_arg_[0], get_arg_[1]);
}