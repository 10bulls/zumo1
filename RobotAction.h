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
    // rely on RestAction to stop, to prevent jerky continuous movement
    // robot.stop();
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

class ActionMove : public RobotAction
{
public:
  ActionMove(boolean dir=FORWARD, int pwm=SPEED_SLOW, unsigned long d=0 ) : RobotAction(d)
  {
    direction = dir;
    speed_pwm = pwm;
  }

  virtual void start()
  {
    RobotAction::start();
    robot.move_pwm(direction,speed_pwm);
  }

  virtual boolean loop()
  {
    if (!RobotAction::loop()) return false;
    return true;
  }

  virtual void end()
  {
    RobotAction::end();
    // rely on RestAction to stop, to prevent jerky continuous movement
    // robot.stop();
  }

  virtual void dump()
  {
    robot.sout->print("move(");
    robot.sout->print(direction ? "REVERSE" : "FORWARD");
    robot.sout->print("pwm=");
    robot.sout->print(speed_pwm);
    robot.sout->println(")");  
  }
  
  boolean direction;
  int speed_pwm;
};

class ActionRepel : public RobotAction
{
public:
  ActionRepel(float dtarget, unsigned long d=0 ) : RobotAction(d)
  {
    distance_target = dtarget;
  }

  virtual void start()
  {
    RobotAction::start();
  }

  virtual boolean loop()
  {
    if (!RobotAction::loop()) return false;

    // robot.sout->println(robot.proximity->distance);
  
    if (robot.proximity->distance > 60 || robot.proximity->distance < 8 )
    {
      // error or really close to target
      robot.stop();
    }
    else if (abs(robot.proximity->distance - distance_target) <= 1.0)
    {
      // where we want to be
      robot.stop();
    }
    else
    {
      boolean dir = FORWARD;
      float diff = robot.proximity->distance - distance_target;
      if (diff < 0) 
      {
        dir = REVERSE;
        diff = -diff;
      }
      if (diff > 10.0) diff = 10.0;
      int speed = SPEED_MED * diff / 10.0;
      
      robot.sout->print("v=");
      robot.sout->println(speed);
      
      robot.move_pwm(dir,speed);
    }

    return true;
  }

  virtual void dump()
  {
    robot.sout->println("repel");
  }

  float distance_target;  
};



#endif
