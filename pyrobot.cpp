#include "pysystem.h"

#include "Robot.h"
#include "RobotAction.h"


extern "C" {

void robot_move_pwm( int dir, int pwm )
{
	Robot::BOT->move_pwm(dir,pwm);
}

void robot_spin_pwm( int dir, int pwm )
{
	Robot::BOT->move_pwm(dir,pwm);
}

void robot_stop()
{
	Robot::BOT->stop();
}

int robot_proximity()
{
	// TODO: test and get float working
	return (int)Robot::BOT->proximity->distance;
}

// Python Action
PythonAction pybot_python_action;
void robot_set_python_action(mp_obj_t a)
{
	pybot_python_action.SetActionObject(a);

	Robot::BOT->setAction(&pybot_python_action);
}

// Move
ActionMove pybot_move_action;
void robot_move( int direction, int pwm, float distance_targe, unsigned long d )
{
	pybot_move_action.direction = (boolean)direction;
	pybot_move_action.speed_pwm = pwm;
	pybot_move_action.distance_target = distance_targe;
	pybot_move_action.duration = d;

	Robot::BOT->setAction(&pybot_move_action);
}

// Repel
ActionRepel pybot_repel_action(10);
void robot_repel( int distance, unsigned long d )
{
	pybot_repel_action.distance_target = distance;
	pybot_repel_action.duration = d;

	Robot::BOT->setAction(&pybot_repel_action);
}

// Rest
ActionRest pybot_rest_action;
void robot_rest( unsigned long d )
{
	pybot_rest_action.duration = d;

	Robot::BOT->setAction(&pybot_rest_action);
}

// Spin
ActionSpin pybot_spin_action;
void robot_spin(int dir, float degree, int pwm, unsigned long d )
{
	pybot_spin_action.direction = dir;
	pybot_spin_action.degree_target = degree;
	pybot_spin_action.speed_pwm = pwm;
	pybot_spin_action.duration = d;

	Robot::BOT->setAction(&pybot_spin_action);
}

// Scan
ActionScan pybot_scan_action;
void robot_scan(boolean dir, float degree, int pwm, unsigned long d )
{
	pybot_scan_action.direction = dir;
	pybot_scan_action.degree_target = degree;
	pybot_scan_action.speed_pwm = pwm;
	pybot_scan_action.duration = d;

	Robot::BOT->setAction(&pybot_scan_action);
}

// Scan2
ActionScan2 pybot_scan2_action;
void robot_scan2(boolean dir, float degree, int pwm, float distance, unsigned long d )
{
	pybot_scan2_action.direction = dir;
	pybot_scan2_action.degree_target = degree;
	pybot_scan2_action.speed_pwm = pwm;
	pybot_scan2_action.distance_target = distance;
	pybot_scan2_action.duration = d;

	Robot::BOT->setAction(&pybot_scan2_action);
}

// Heading
ActionHeading pybot_heading_action;
void robot_set_heading(float heading, unsigned long d )
{
	pybot_heading_action.heading_target = heading;
	pybot_heading_action.duration = d;

	Robot::BOT->setAction(&pybot_heading_action);
}

// Line Follow
ActionLineDetect pybot_linefollow_action;
void robot_line_follow(unsigned long d )
{
	pybot_linefollow_action.duration = d;

	Robot::BOT->setAction(&pybot_linefollow_action);
}

// Stay in Boundary
ActionStayInBoundary pybot_boundary_action;
void robot_boundary(unsigned long d )
{
	pybot_boundary_action.duration = d;

	Robot::BOT->setAction(&pybot_boundary_action);
}

extern ActionBalance action_balance;

void balance_pid( float Kp, float Ki, float Kd, float target )
{
	if (target==0)
		target = action_balance.pitch_target;

	Robot::BOT->sout->print("Kp=");
	Robot::BOT->sout->print(Kp);
	Robot::BOT->sout->print(", Ki=");
	Robot::BOT->sout->print(Ki);
	Robot::BOT->sout->print(", Kd=");
	Robot::BOT->sout->print(Kd);
	Robot::BOT->sout->print(", pitch=");
	Robot::BOT->sout->println(target);


	action_balance.Kp = Kp;
	action_balance.Ki = Ki;
	action_balance.Kd = Kd;
	action_balance.pitch_target = target;
}



}