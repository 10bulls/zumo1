#ifndef RobotAction_h_defined
#define RobotAction_h_defined

class RobotAction
{
public:
  RobotAction(unsigned long d=0)
  {
    duration = d;
    pnext_action = 0;
  }
  
  virtual void start()
  {
    tstart = millis();
  }

  virtual boolean loop()
  {
    if (duration != 0)
    {
      if (elapsed() >= duration) return false;
    }
    return true;
  }
  
  unsigned long elapsed()
  {
    return millis()-tstart;
  }

  virtual void end()
  {
  }
  
  virtual void dump()
  {
    // robot.sout->println("?");
    robot.sout->println("?");
  }
  
  unsigned long tstart;
  unsigned long duration;
  RobotAction * pnext_action;
};

class ActionRest : public RobotAction
{
public:
  ActionRest(unsigned long d=0) : RobotAction(d)
  {
  }

  virtual void start()
  {
    RobotAction::start();
    robot.stop();
  }

  virtual void dump()
  {
    robot.sout->println("rest");
  }
};

class ActionSpin : public RobotAction
{
public:
  ActionSpin(boolean dir=CW, float degree=0, int pwm=SPEED_SLOW, unsigned long d=0 ) : RobotAction(d)
  {
    direction = dir;
    degree_target = degree;
    speed_pwm = pwm;
  }

  virtual void start()
  {
    RobotAction::start();
    if (degree_target > 0)
    {
      StartIMU();
    }
    robot.spin_pwm(direction,speed_pwm);
  }

  virtual boolean loop()
  {
    if (!RobotAction::loop()) return false;

    if (degree_target > 0)
    {
      // Read compass
      if (!imu_timer_running)
      {
        compass.read();
      }
      
      float diff = 0;
    
      if (direction==CW)
      {
        diff = degree_target - imu.degrees;
      }
      else
      {
        diff = degree_target + imu.degrees;
      }
      
      //robot.sout->print("d=");      
      //robot.sout->println(diff);      
      
      if (diff < 0)
      {
        return false;
      }
    }
    return true;
  }

  virtual void end()
  {
    RobotAction::end();
    robot.stop();
    if (degree_target > 0)
    {
      StopIMU();
    }
    robot.sout->print("gyro change=");      
    robot.sout->println(imu.degrees);      
  }

  virtual void dump()
  {
    robot.sout->print("spin(");
    robot.sout->print(direction ? "CCW," : "CW,");
    robot.sout->print(degree_target);
    robot.sout->println(" deg)");  
  }
  
  boolean direction;
  int speed_pwm;
  float degree_target;
  float heading_target;
  float heading_start;
  
  
};

#endif
