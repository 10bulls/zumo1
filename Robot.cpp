#include "Robot.h"

Robot * Robot::BOT = 0;


void RobotTank::MotorTest( unsigned long duration, unsigned long pause )
{
	static byte speeds[] = { 64, 128, 192, 255 };

	int i;

	for (int j=0;j<4;j++)
	{
		motor_R->setDir(j&1);
		motor_L->setDir(j>>1);

		// Right motor only test
		motor_L->setPWM(0);
		motor_R->setPWM(0);
		for(i=0; i < 4; i++)
		{
			motor_R->setPWM(speeds[i]);
			delay(duration);
		}
		motor_R->setPWM(0);
		
		delay(pause);
		
		// Left motor only test
		for(i=0; i < 4; i++)
		{
			motor_L->setPWM(speeds[i]);
			delay(duration);
		}
		motor_L->setPWM(0);
    
		delay(pause);
    
		// Both motors
		for(i=0; i < 4; i++)
		{
			motor_L->setPWM(speeds[i]);
			motor_R->setPWM(speeds[i]);
			delay(duration);
		}
		motor_L->setPWM(0);
		motor_R->setPWM(0);
    
		delay(pause);
	}

}