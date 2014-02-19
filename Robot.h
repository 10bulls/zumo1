#ifndef Robot_h_defined
#define Robot_h_defined

class RobotAction;

#include <QTRSensors.h>
#include "RobotMotor.h"
#include "RobotIMU.h"
#include "RobotProximity.h"

#define CW      0
#define CCW     1

//int SPEED_FAST = 128;  // 0x1e
//int SPEED_MED  = 64;
//int SPEED_SLOW = 32;  // 0x5a
// int SPEED_FAST = 128;  // 0x1e
#define SPEED_FAST 0xff
#define SPEED_MED  128
#define SPEED_SLOW 64

#define NUM_SENSORS 6

class Robot
{
public:
	Robot(RobotIMU * pimu = 0, RobotProximity * pproxL = 0, RobotProximity * pproxR = 0, QTRSensorsRC * preflectance=0 )
	{
		// Use bluetooth for output
		// sout = &Serial3;
		// Use USB for output
		sout = &Serial;
		imu = pimu;
		proximityL = pproxL;
		proximityR = pproxR;
		preflect = preflectance;
		// Set a global robot pointer (should only be one)
		BOT = this;
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
  
	virtual void left_pwm( int dir, int pwm )
	{
	}
  
	virtual void right_pwm( int dir, int pwm )
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

	virtual void setAction( RobotAction * action );
	/*
	virtual void setAction( RobotAction * action )
	{
	if (paction) paction->end();
	paction = action;
	if (paction) paction->start();
	}
	*/

	virtual void dump()
	{
		if (imu)
		{
			imu->read();
			sout->print("heading=");
			sout->println(imu->getHeading());
		}
		if (proximityL && proximityR)
		{
			sout->print("proximity(L,R)=");
			sout->print(proximityL->read());
			sout->print(", ");
			sout->println(proximityR->read());
		}
	}

	virtual void MotorTest( unsigned long duration, unsigned long pause );

	Stream *sout;
	RobotIMU * imu;
	RobotProximity * proximityL;
	RobotProximity * proximityR;
	QTRSensorsRC * preflect;
	// hard code NUM_SENSORS=6 for now...
	// int num_reflect_sensors=0;
  
	RobotAction *paction;

	static Robot * BOT;

	
	unsigned int sensorValues[NUM_SENSORS];

};

class RobotTank : public Robot
{
public:
	RobotTank(RobotMotor * lm, RobotMotor * rm, RobotIMU * pimu=0, RobotProximity * pproxL=0, RobotProximity * pproxR=0, QTRSensorsRC * preflectance=0  ) : Robot(pimu, pproxL, pproxR, preflectance)
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

	virtual void left_pwm( int dir, int pwm )
	{
		motor_L->setDir(dir );
		motor_L->setPWM(pwm);
	}
  
	virtual void right_pwm( int dir, int pwm )
	{
		motor_R->setDir(dir );
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
		Robot::dump();
		sout->print("PWM(L,R)=");
		sout->print(motor_L->getPWM());
		sout->print(", ");
		sout->println(motor_R->getPWM());
	}

	virtual void MotorTest( unsigned long duration, unsigned long pause );

	RobotMotor * motor_L;
	RobotMotor * motor_R;
};


#endif
