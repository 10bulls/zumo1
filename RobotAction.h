#ifndef RobotAction_h_defined
#define RobotAction_h_defined

class RobotAction
{
public:
  RobotAction()
  {
    duration = 0;
  }
  
  virtual void start()
  {
    tstart = millis();
  }

  virtual void end()
  {
  }
  
  unsigned long tstart;
  unsigned long duration;
};

class SpinAction : RobotAction
{
public:
  SpinAction(boolean dir, float degree, int pwm )
  {
    direction = dir;
    degree_target = degree;
    speed_pwm = pwm;
  }

  virtual void start()
  {
    RobotAction::start();
  }

  virtual void end()
  {
    RobotAction::end();
  }

  
  boolean direction;
  int speed_pwm;
  float degree_target;
  float heading_target;
  float heading_start;
  
  
};

#endif
