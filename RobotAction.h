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
	degrees=0;
  }

  virtual void start()
  {
    RobotAction::start();
    if (degree_target > 0)
    {
	  imu.start();
      // StartIMU();
    }
	degrees=0;
	gyro.read();
    robot.spin_pwm(direction,speed_pwm);
	time_us = micros();
  }

  virtual boolean loop()
  {
    if (!RobotAction::loop()) return false;

    if (degree_target > 0)
    {
		/*
      // Read compass
      if (!imu_timer_running)
      {
        compass.read();
      }
	  */

	  	imu.loop();
//!		 gyro.read();

		unsigned long us = micros();

		float dt = us - time_us;

		time_us = us;

		degrees += (gyro.g.z - imu.gzero.z) * 0.00875 * (float)dt / 1.0e6;
      
      float diff = 0;
    
      if (direction==CW)
      {
        // diff = degree_target - imu.degrees;
		  diff = degree_target - degrees;
      }
      else
      {
        // diff = degree_target + imu.degrees;
		  diff = degree_target + degrees;
      }
      
      //robot.sout->print("d=");      
      //robot.sout->println(diff);      
      
	  // NOTE: allow 1.2 degrees 'stopping' distance
	  // TODO: should instead PID adjust pwm depending on error
      if (diff - 1.2 < 0)
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
		// StopIMU();
		robot.stop();
		for(int i=0;i<50;i++)
		{
			imu.loop();
			// gyro.read();
			unsigned long us = micros();
			float dt = us - time_us;
			time_us = us;
			degrees += (gyro.g.z - imu.gzero.z) * 0.00875 * (float)dt / 1.0e6;
		}
		imu.stop();
		//imu.dump_buffer(robot.sout);
    }
    robot.sout->print("gyro change=");      
    // robot.sout->println(imu.degrees);      
	robot.sout->println(degrees);      
  }

  virtual void dump()
  {
    robot.sout->print("spin(");
    robot.sout->print(direction ? "CCW," : "CW,");
    robot.sout->print(degree_target);
    robot.sout->println(" deg)");  
  }
  
	unsigned long time_us;
	float degrees;
	boolean direction;
	int speed_pwm;
	float degree_target;
	float heading_target;
	float heading_start;
  
  
};

#define NUM_SAMPLES 10

class ActionHeading : public RobotAction
{
public:
	ActionHeading(float heading=0, unsigned long d=0 ) : RobotAction(d)
	{
		direction = CW;
		heading_target = heading;
	}

	virtual void start()
	{
		RobotAction::start();
		compass.read();
		heading = imu.getHeading();
	
		speed_pwm = 0;
		error = 0;
		total_error = 0;
	
		isample = 0;
		samplesum = heading * NUM_SAMPLES;
		for (int i=0;i<NUM_SAMPLES;i++) samples[i] = heading;

		robot.sout->print("heading");
		robot.sout->print("\t");
		robot.sout->print("error");
		robot.sout->print("\t");
		robot.sout->print("total_error");
		robot.sout->print("\t");
		robot.sout->print("predicted");
		robot.sout->print("\t");
		robot.sout->print("pid");
		robot.sout->print("\t");
		robot.sout->println("speed_pwm");

		time_us = micros();

	
	}
	
	float normalise_angle_diff( float d )
	{
		if (d > 360)
			d -= 360;

		// 0 : 180
		if (d >= 0 && d <= 180) 
			return d;

		// 180 : 360
		if (d > 180)
			return d - 360;

		// 0 : - 180
		if (d < 0 && d >= -180)
			return d;
		
		// -180 : -360
		return 360 + d;
	}


	virtual boolean loop()
	{
		if (!RobotAction::loop()) return false;

		compass.read();
		gyro.read();

		unsigned long us = micros();

		float dt = us - time_us;

		time_us = us;

		// heading = imu.getHeading();
		// running average...
		samplesum -= samples[isample];
		samples[isample] = imu.getHeading();
		samplesum += samples[isample];
		isample++;
		if (isample == NUM_SAMPLES) isample=0;
		heading = samplesum / (float)NUM_SAMPLES;

		error = normalise_angle_diff(heading_target - heading);

		// total_error += error;

//		if (abs(error) < 1.0)
//		{
//			// close enough
//			speed_pwm = 0;
//			robot.stop();
//		}
//		else
//		{

			// use gyro to calculate predicted degree change
			float ddegree = (gyro.g.z - imu.gzero.z) * 0.00875 * (float)dt / 1.0e6;

			float pid = Kp*error + Ki*total_error + Kd*(-ddegree);
			// float pid = Kp*error + Ki*total_error + Kd*(error-last_error);

			speed_pwm = constrain( speed_pwm + pid, -200, 200);

			robot.sout->print(heading);
			robot.sout->print("\t");
			robot.sout->print(error);
			robot.sout->print("\t");
			robot.sout->print(total_error);
			robot.sout->print("\t");
			robot.sout->print(error-ddegree);
			robot.sout->print("\t");
			robot.sout->print(pid);
			robot.sout->print("\t");
			robot.sout->println(speed_pwm);

			if (speed_pwm >= 0)
				direction = CW;
			else
				direction = CCW;

			// speed_pwm = SPEED_SLOW + (SPEED_FAST-SPEED_SLOW) * diff / 180.0;
			if (abs(speed_pwm) < 20)
				robot.stop();
			else
				robot.spin_pwm(direction,(int)abs(speed_pwm));
//		}

		last_error = error;

		total_error += error;

		return true;
	}

	virtual void end()
	{
		RobotAction::end();
		robot.sout->print("heading=");      
		// robot.sout->println(imu.degrees);      
		robot.sout->println(imu.getHeading());      
	}

	virtual void dump()
	{
		robot.sout->print("heading(");
		robot.sout->print(heading_target);
		robot.sout->println(" deg)");  
	}
  

	unsigned long time_us;
	boolean direction;
	float speed_pwm;
	float heading_target;
	float heading;
	float samples[NUM_SAMPLES];
	float samplesum;
	int isample;
  
	// PID
	float Kp = 0.020;
	//float Ki = 0.0000002;
	float Ki = 0.0000000;
	// float Kd = 1.5;
	float Kd = 3.5;

	float error;
	float total_error;
	float last_error;
};


class ActionScan : public ActionSpin
{
public:
  ActionScan(boolean dir=CW, float degree=0, int pwm=SPEED_SLOW, unsigned long d=0 ) : ActionSpin(dir,degree,pwm,d)
  {
    start_direction = dir;
    start_degree_target = degree;
  }

  virtual void start()
  {
    degree_target = start_degree_target;
    ActionSpin::start();
    stage = 1;  // 1 = spin in default direction
                // 2 = reverse spin
                // 3 = original spin, stop at best proximity
                
    distance_max = robot.proximity->distance;                
  }

  virtual boolean loop()
  {
    if (!ActionSpin::loop()) 
    {
      robot.imu->degrees = 0;
      
      stage++;
      if (stage > 3) return false;

      direction = !direction;
      
      if (stage == 2)
      {
        degree_target *= 2;
        duration *= 2;
      }
      
      robot.spin_dir(direction);

    }
    
    if (robot.proximity->distance >= distance_max)
    {
      if (stage == 3)
      {
        // stop at this heading
        return false;
      }
      else
      {
        distance_max = robot.proximity->distance;
      }
    }

    return true;
  }

  virtual void end()
  {
    ActionSpin::end();
  }

  virtual void dump()
  {
    robot.sout->print("scan(");
    robot.sout->print(direction ? "CCW," : "CW,");
    robot.sout->print(degree_target);
    robot.sout->println(" deg)");  
  }

  boolean start_direction;  
  float start_degree_target;
  int stage = 1;
  float distance_max;
};

class ActionScan2 : public ActionSpin
{
public:
  ActionScan2(boolean dir=CW, float degree=180, int pwm=SPEED_SLOW, float distance=30, unsigned long d=0 ) : ActionSpin(dir,degree,pwm,d)
  {
    distance_target = distance;
  }

  virtual void start()
  {
    ActionSpin::start();
    degree_ok = 0;
    distance_ok = false;
  }

  virtual boolean loop()
  {
    if (!ActionSpin::loop()) return false; 
    
    if (robot.proximity->distance >= distance_target)
    {
      if (distance_ok)
      {
        // if been OK for 5? degrees, end action
        if (abs(robot.imu->degrees - degree_ok) >= 15)
        {
          return false;
        }
      }
      else
      {
        distance_ok = true;
        degree_ok = robot.imu->degrees;
      }
    }
    else
    {
      distance_ok = false;
    }
 
    return true;
  }

  virtual void end()
  {
    ActionSpin::end();
  }

  virtual void dump()
  {
    robot.sout->print("scan2(");
    robot.sout->print(direction ? "CCW," : "CW,");
    robot.sout->print(degree_target);
    robot.sout->println(" deg)");  
  }

  boolean distance_ok;
  float degree_ok;
  float distance_target;
};

class ActionMove : public RobotAction
{
public:
  ActionMove(boolean dir=FORWARD, int pwm=SPEED_SLOW, float dtarget=10.0, unsigned long d=0 ) : RobotAction(d)
  {
    direction = dir;
    speed_pwm = pwm;
    distance_target = dtarget;
  }

  virtual void start()
  {
    RobotAction::start();
    robot.move_pwm(direction,speed_pwm);
  }

  virtual boolean loop()
  {
    if (!RobotAction::loop()) return false;
    
    if (robot.proximity->distance < distance_target) return false;
    
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
  float distance_target;
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
  
    if (robot.proximity->distance > 40 || robot.proximity->distance < 5 )
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
        diff = -2.0 * diff;	// reverse faster than forwards
      }
      if (diff > alpha) diff = alpha;
      // int speed = SPEED_MED * diff / alpha;
	  int speed = SPEED_FAST * diff / alpha;
      
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
  float alpha = 10.0;
  // over this distance will be 'full' speed
  // float alpha = 5.0;
};

#define MAX_SPEED  SPEED_MED

class ActionLineDetect : public RobotAction
{
public:
  ActionLineDetect(unsigned long d=0) : RobotAction(d)
  {
  }

  virtual void start()
  {
    RobotAction::start();
  }

  virtual boolean loop()
  {
	if (!RobotAction::loop()) return false;

    boolean debugmsg = false;
/*    
    int maxval = 2000;
    
//    pinMode(A7,OUTPUT);    
    digitalWrite(A7,HIGH);
    pinMode(A7,OUTPUT);
    
    delayMicroseconds(10);
//    delayMicroseconds(100);
    
    pinMode(A7,INPUT);
    digitalWrite(A7,LOW);
    
    unsigned long pval = maxval;
    
    unsigned long startTime = micros();	
    // while (micros() - startTime < maxval)
    do
    {
      unsigned int time = micros() - startTime;
      if (digitalRead(A7) == LOW)
      {
        pval = time;
        break;
      }
    }
    while (micros() - startTime < maxval);
    robot.sout->println(pval);
*/    
    // read calibrated sensor values and obtain a measure of the line position.
    // Note: the values returned will be incorrect if the sensors have not been properly
    // calibrated during the calibration phase.

/*  
    // To get raw sensor values instead, call:  
    reflectanceSensors.read(sensorValues);
  
    for (byte i = 0; i < NUM_SENSORS; i++)
    {
      robot.sout->print(sensorValues[i]);
      robot.sout->print(' ');
    }
    
    robot.sout->print(" --> ");
*/  
    unsigned int position = reflectanceSensors.readLine(sensorValues);
    // reflectanceSensors.readCalibrated(sensorValues);

if (debugmsg)
{
    for (byte i = 0; i < NUM_SENSORS; i++)
    {
      robot.sout->print(sensorValues[i]);
      robot.sout->print(' ');
    }

    
    robot.sout->print("    ");
    robot.sout->println(position);
//    robot.sout->println();
//    delay(250);
}   
    
    // Our "error" is how far we are away from the center of the line, which
  // corresponds to position 2500.
  int error = position - 2500;

  // Get motor speed difference using proportional and derivative PID terms
  // (the integral term is generally not very useful for line following).
  // Here we are using a proportional constant of 1/4 and a derivative
  // constant of 6, which should work decently for many Zumo motor choices.
  // You probably want to use trial and error to tune these constants for
  // your particular Zumo and line course.
  // int speedDifference = error / 4 + 6 * (error - lastError);
  int speedDifference = error / 2 + 1 * (error - lastError);
   speedDifference /= 4;

  lastError = error;

  // Get individual motor speeds.  The sign of speedDifference
  // determines if the robot turns left or right.
  int m1Speed = MAX_SPEED + speedDifference;
  int m2Speed = MAX_SPEED - speedDifference;

  // Here we constrain our motor speeds to be between 0 and MAX_SPEED.
  // Generally speaking, one motor will always be turning at MAX_SPEED
  // and the other will be at MAX_SPEED-|speedDifference| if that is positive,
  // else it will be stationary.  For some applications, you might want to
  // allow the motor speed to go negative so that it can spin in reverse.
  if (m1Speed < 0)
    m1Speed = 0;
  if (m2Speed < 0)
    m2Speed = 0;
  if (m1Speed > MAX_SPEED)
    m1Speed = MAX_SPEED;
  if (m2Speed > MAX_SPEED)
    m2Speed = MAX_SPEED;
    
    robot.left_pwm(FORWARD,m1Speed);
    robot.right_pwm(FORWARD,m2Speed);
    
    return true;
  }

  virtual void dump()
  {
    robot.sout->println("line detect");
  }
  
  int lastError = 0;
};

class ActionStayInBoundary : public RobotAction
{
public:
  ActionStayInBoundary(unsigned long d=0) : RobotAction(d)
  {
    /*
    duration_reverse = 300;
    duration_turn = 250;
    */
    duration_reverse = 150;
    duration_turn = 150;
  }

  virtual void start()
  {
    RobotAction::start();
    stage = 0;
    // robot.forward_pwm(SPEED_SLOW);
    robot.forward_pwm(SPEED_FAST);
  }


  virtual boolean loop()
  {
	if (!RobotAction::loop()) return false;

    unsigned long tnow = millis();
    if (stage == 1)
    {
      // reversing
      if (tnow - tstart >= duration_reverse + random(100))
      {
        // turn
        // robot.spin_pwm(turn_dir,SPEED_SLOW);
        robot.spin_pwm(turn_dir,SPEED_FAST);
        stage = 2;
        tstart = tnow;
      }
      return true;
    }
    if (stage == 2)
    {
      // turning
      if (tnow - tstart >= duration_reverse + random(100))
      {
        // forward
        // robot.forward_pwm(SPEED_SLOW);
        robot.forward_pwm(SPEED_FAST);
        stage = 0;
        tstart = tnow;
      }
    }
    
    // read raw sensor values
    reflectanceSensors.read(sensorValues);
    detect = 0;
    for (byte i = 0; i < NUM_SENSORS; i++)
    {
      if (sensorValues[i] > 600)
      {
        if (i < NUM_SENSORS/2)
          detect = -1;
        else
          detect = 1;
        break;
      }
      // robot.sout->print(sensorValues[i]);
      // robot.sout->print(' ');
    }
    
    if (detect)
    {
      tstart = millis();
      
      // reverse
      if (detect < 0)
        // to right
        turn_dir = CW;
      else
        // to left
        turn_dir = CCW;
        
      if (turn_dir != last_turn_dir && tstart-last_turn_time < 2 * (duration_reverse+duration_turn))
      {
        // if turning CW / CCW in short duration probably stuck in a corner
        turn_dir = !turn_dir;
      }
      if (random(100) < 20)
      {
        // randomly change direction
         turn_dir = !turn_dir;
      }

      last_turn_time = tstart;
      last_turn_dir = turn_dir;
      
      stage = 1;

      // robot.reverse_pwm(SPEED_SLOW);      
      robot.reverse_pwm(SPEED_FAST);      
    }
      
    
    // delay(250);
    
    return true;
  }

  virtual void dump()
  {
    robot.sout->println("stay in boundary");
  }
  
  int stage = 0;  // 0 = forward
                  // 1 = reversing
                  // 2 - turning
                  
  int detect = 0;  // 0 = not detected
                   // -1 = detected to left
                   // 1 = detected to right
                   // 2 = detected straight
                   
  unsigned long tstart;
  
  int duration_reverse;
  int duration_turn;
  
  boolean turn_dir;
  boolean last_turn_dir;
  unsigned long last_turn_time;
};


class ActionAccelerometerTest : public RobotAction
{
public:
  ActionAccelerometerTest(unsigned long d=0) : RobotAction(d)
  {
  }

  virtual void start()
  {
	RobotAction::start();

	imu.log_max = true;
	// StartIMU();
	imu.start();
	robot.forward_pwm(SPEED_MED);
	time_us = micros();
	acceleration = 0;
	velocity = 0;
	distance = 0;
  }

  virtual void end()
  {
	imu.stop();
	// StopIMU();
	robot.stop();
	// delay(200);
	// StopIMU();

	robot.sout->println("MIN.X\tMIN.Y\tMIN.Z\tMAX.X\tMAX.Y\tMAX.Z");

    robot.sout->print(imu.amin.x >> 4);
    robot.sout->print("\t");
    robot.sout->print(imu.amin.y >> 4);
    robot.sout->print("\t");
    robot.sout->print(imu.amin.z >> 4);
    robot.sout->print("\t");
    robot.sout->print(imu.amax.x >> 4);
    robot.sout->print("\t");
    robot.sout->print(imu.amax.y >> 4);
    robot.sout->print("\t");
    robot.sout->println(imu.amax.z >>4 );

	robot.sout->print("distance=");
	robot.sout->println(distance);

//	imu.dump_buffer(robot.sout);

  }


  virtual boolean loop()
  {
	if (!RobotAction::loop()) return false;

	imu.loop();

	unsigned long us = micros();

	float dt = us - time_us;

	time_us = us;

	float acc = (compass.a.x >> 4)*9.81/1000.0;

	velocity += (acc + acceleration)/2.0 * (float)dt / 1.0e6;

	distance += velocity * (float)dt / 1.0e6;

	acceleration = acc;
	
	if ((imu.amin.x >> 4) < -1000) 
	{
		robot.sout->println("OUCH!");
		return false;
	}


	
	/*
//    unsigned long tnow = millis();
//    dt = (float)(tnow - t) / 1000.0;
    
//    t = tnow;
    
    // convert 16bit (left justified) to 12bit
    // scale by 1/000 as reading is 1mg per LSB
    float ax = (compass.a.x >> 4);
    float ay = (compass.a.y >> 4);
    float az = (compass.a.z >> 4);
    
//    a += 10;
    
    //a_avg += a;
//    v += a * dt * 9.8 / 2000.0;  // ms-1
//    d += v * dt / 2.0;
//    samples++;

	*/
    return true;
  }

  virtual void dump()
  {
    robot.sout->println("accelerometer");
  }

  unsigned long time_us;

  float velocity;
  float distance;
  float acceleration;
};

#endif
