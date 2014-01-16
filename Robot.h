#ifndef Robot_h_defined
#define Robot_h_defined

class Robot
{
public:
  Robot()
  {
    sout = &Serial3;
  }
  
  virtual void setup()
  {
  }
  
  virtual void stop()
  {
  }
  
  virtual void move_pwm( int dir, int pwm )
  {
  }
  
  virtual void forward_pwm( int pwm )
  {
    move_pwm(FORWARD,pwm);
  }

  virtual void reverse_pwm( int pwm )
  {
    move_pwm(REVERSE,pwm);
  }

  virtual void spin_pwm( boolean dir, int pwm )
  {
  }
  
  virtual void spin_dir( boolean dir )
  {
  }
  
  virtual void dump()
  {
  }
  

  Stream *sout;
};

class RobotTank : public Robot
{
public:
  RobotTank(RobotMotor * lm, RobotMotor * rm )
  {
    motor_L = lm;
    motor_R = rm;
  }

  virtual void setup()
  {
    motor_L->setup();
    motor_R->setup();
  }
  
  void stop()
  {
    motor_L->stop();  
    motor_R->stop();
  }

  void move_pwm( int dir, int pwm )
  {
    motor_L->setDir(dir );
    motor_R->setDir(dir );
    motor_L->setPWM(pwm);
    motor_R->setPWM(pwm);
  }

  void spin_pwm( boolean dir, int pwm )
  {
    motor_L->setDir(dir );
    motor_R->setDir(!dir );
    motor_L->setPWM(pwm);
    motor_R->setPWM(pwm);
  }

  void spin_dir( boolean dir )
  {
    motor_L->setDir(dir );
    motor_R->setDir(!dir );
  }

  void dump()
  {
    sout->print("PWM(L,R)=");
    sout->print(motor_L->getPWM());
    sout->print(", ");
    sout->println(motor_R->getPWM());
  }


  RobotMotor * motor_L;
  RobotMotor * motor_R;
};


#endif
