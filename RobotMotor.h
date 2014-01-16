#ifndef RobotEncoder_h_defined
#define RobotEncoder_h_defined

#include <Arduino.h>

#define FORWARD  0
#define REVERSE  1

class RobotMotor
{
 public:
   RobotMotor( int PWM, int DIR )
   {
     _pwm_pin = PWM;
     _dir_pin = DIR;
//    pwmFast = 255;
//    pwmMed = 128;
//    pwmSlow = 64;
    _dir = FORWARD;
    _pwm = 0;
    invertDir = false;
   }
   
//  int pwmFast;
//  int pwmMed;
//  int pwmSlow;

  void setup()
  {
    pinMode( _dir_pin, OUTPUT );
    pinMode( _pwm_pin, OUTPUT );
  }  

  void setPWM( int pwm )
  {
    _pwm = pwm;
    analogWrite( _pwm_pin, pwm );
  }
  int getPWM()
  {
    return _pwm;
  }
  
  void setDir( boolean dir )
  {
    _dir = dir;
    if (invertDir)
      digitalWrite(_dir_pin,!dir);
    else
      digitalWrite(_dir_pin,dir);
  }
  int getDir()
  {
    return _dir;
  }

  void stop( void )
  {
    setPWM(0);
  }
  
  void forward( int pwm )
  {
    setDir(FORWARD);
    setPWM(pwm);
  }

  void reverse( int pwm )
  {
    setDir(REVERSE);
    setPWM(pwm);
  }

  boolean invertDir;
  
  
protected:
  int _pwm_pin;
  int _dir_pin;
  int _pwm;
  boolean _dir;
};


#endif

