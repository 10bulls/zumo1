#include <mpython.h>

#include "pysystem.h"

#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>

#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>

// #include <EEPROM.h>
#include <avr/EEPROM.h>

#include <XModem.h>
#include "zmodem.h"
extern Stream * _xmodem_s;

#include <SPI.h>
#include <SD.h>
#define SD_CS	10

class RobotAction;
class Robot;

#include "RobotMotor.h"
#include "RobotIMU.h"
#include "RobotProximity.h"
#include "Robot.h"

#define LED_PIN     13
#define BUZZER_PIN  20
// #define BUTTON_PIN  12
#define BUTTON_PIN  33

// If old arduino
//#define MOTOR_R_DIR  7
//#define MOTOR_L_DIR  8
// use 3 and 4 for Teensy (7&8 are Serial3)

// #define MOTOR_R_DIR  3
// #define MOTOR_L_DIR  4
#define MOTOR_R_DIR  1
#define MOTOR_L_DIR  2

#define MOTOR_R_PWM  9
// #define MOTOR_L_PWM  10
#define MOTOR_L_PWM  4

#define IR_FRONT   A9

//int distance_target_l = 0;
//int distance_target_r = 0;

// define a circular buffer
char serial_buffer[30];
int iserial_buffer;
int serial_buffer_start;
int serial_buffer_len;

// Motor output controllers 
// MotorPID motor_left(MC_B1, MC_B2, ENCODER_L);
// MotorPID motor_right(MC_A1, MC_A2, ENCODER_R);

///////////////////////////////////

#include <IRremote.h>
#include "irstuff.h"

// const int RECV_PIN = 11;
const int RECV_PIN = 0;

IRrecv irrecv(RECV_PIN);
// IRsend irsend;

decode_results results;

////////////////////////////////////

// unsigned char sensorPins[] = { A7, A3, A8, A0, A2, 6 };
unsigned char sensorPins[] = { A7, A3, A8, A0, A2, 24 };

ZumoReflectanceSensorArray reflectanceSensors;
// ZumoReflectanceSensorArray reflectanceSensors(sensorPins, sizeof(sensorPins), 2000, QTR_NO_EMITTER_PIN );

// Define an array for holding sensor values.
#define NUM_SENSORS 6
unsigned int sensorValues[NUM_SENSORS];

#define EEPROM_REFLECT      0
#define EEPROM_REFLECT_LEN  (2*NUM_SENSORS*sizeof(unsigned int))
#define EEPROM_GYRO         (EEPROM_REFLECT+EEPROM_REFLECT_LEN)
#define EEPROM_GYRO_LEN     (sizeof(imu.gzero))
#define EEPROM_COMPASS      (EEPROM_GYRO+EEPROM_GYRO_LEN)
#define EEPROM_COMPASS_LEN	(2*sizeof(LSM303::vector<int16_t>))

////////////////////////////////////

L3G gyro;
LSM303 compass;
SharpIR sharpIR(IR_FRONT);
RobotIMU imu(&compass, &gyro);

boolean imu_timer_running = false;
IntervalTimer imu_timer;
boolean dump_imu = false;

// Robot robot;

RobotMotor motor_left( MOTOR_L_PWM, MOTOR_L_DIR );
RobotMotor motor_right( MOTOR_R_PWM, MOTOR_R_DIR );

RobotTank robot(&motor_left, &motor_right, &imu, &sharpIR);

#include "RobotAction.h"

ActionRest action_rest;

#define DEFAULT_DURATION 150 // NOTE, take just over 100ms to process IR remote
                             // so 150ms allows continuous, non jerky movements

ActionMove action_forward_slow(FORWARD,SPEED_SLOW,10,DEFAULT_DURATION);
ActionMove action_forward_med(FORWARD,SPEED_MED,10,DEFAULT_DURATION);
ActionMove action_forward_fast(FORWARD,SPEED_FAST,10,DEFAULT_DURATION);

ActionMove action_reverse_slow(REVERSE,SPEED_SLOW,0,DEFAULT_DURATION);
ActionMove action_reverse_med(REVERSE,SPEED_MED,0,DEFAULT_DURATION);
ActionMove action_reverse_fast(REVERSE,SPEED_FAST,0,DEFAULT_DURATION);

ActionSpin action_spin;
ActionSpin action_spin_100ms(CW,0,SPEED_SLOW,DEFAULT_DURATION);
ActionSpin action_spin_45deg(CW,45,SPEED_SLOW,10000);
ActionSpin action_fast_spin(CW,0,SPEED_FAST,4000);

ActionRepel action_repel(10);

ActionScan action_scan(CW,34,SPEED_SLOW,3000);

ActionScan action_scan_rove(CW,34,SPEED_SLOW,3000);
ActionMove action_forward_rove(FORWARD,SPEED_MED,15, 5000);

ActionScan2 action_scan2(CW,180,SPEED_MED,30,3000);
ActionScan2 action_scan2_rove(CW,180,SPEED_MED,30,3000);
ActionMove action_forward_rove2(FORWARD,SPEED_MED,15, 5000);

ActionScan2 action_scan2_ccw(CCW,180,SPEED_MED,30,3000);
ActionScan2 action_scan2_rove_ccw(CCW,180,SPEED_MED,30,3000);
ActionMove action_forward_rove2_ccw(FORWARD,SPEED_MED,15, 5000);

ActionLineDetect action_line_detect;
ActionStayInBoundary action_in_boundary;

ActionHeading action_heading(90.0,0);

ActionAccelerometerTest accelerometer_test(500);

//ActionPlaySong action_song;

RobotAction * QuickActions []
{
  &action_rest,        // 0
  &action_scan_rove,   // 1
  &action_repel,       // 2
  &action_scan,        // 3
  &action_fast_spin,   // 4
//  &action_scan2,       // 5
  &action_heading,		 // 5
  &action_scan2_rove,    // 6
//  &action_scan2_ccw,       // 7
  &accelerometer_test,      // 7
//  &action_song,      // 7
//  &action_scan2_rove_ccw,    // 8
  &action_line_detect,    // 8
  &action_in_boundary      // 9
    
};

extern "C"
{
void * set_stdout_callback(void (*fn)(void *, const char *, unsigned int ));

void stdout_print_strn_robot(void *data, const char *str, unsigned int len) 
{
	robot.sout->write(str,len);
}
}

#define NUM_QUICK_ACTIONS  sizeof(QuickActions)/sizeof(QuickActions[0])
int quick_action = 0;

//unsigned long tlast = millis();

// prototypes from compass.cpp
void dump_compass_config(LSM303 * compass, Stream * sout);
void compass_calibrate(Robot * bot, LSM303 * compass, RobotIMU * imu, boolean auto_rotate);
void read_compass_config_from_eeprom(LSM303 * compass, int a);
void write_compass_config_to_eeprom(LSM303 * compass, int a);
// prototypes from reflectance.cpp
void dump_reflectance_calibration(ZumoReflectanceSensorArray * sensor, int num_sensors, Stream * sout);
void calibrate_reflectance_array(ZumoReflectanceSensorArray * sensor, int num_sensors, Stream * sout);
void save_reflectance_to_eeprom(ZumoReflectanceSensorArray * sensor, int num_sensors, int address);
void read_reflectance_from_eeprom(ZumoReflectanceSensorArray * sensor, int num_sensors, int address);
// 
void dump_gzero(RobotIMU * imu, Stream * sout);
void calibrate_gyro(RobotIMU * imu, Stream * sout);
void write_gyro_zero_to_eeprom(RobotIMU * imu, int a);
void read_gyro_zero_from_eeprom(RobotIMU * imu, int a);

void Robot::setAction( RobotAction * action )
{
	// if current action already set ignore (likely due to switch bounce or multiple IR)
	if (paction == action) 
	{
		// reset start time?
		paction-> tstart = millis();
		return;
	}

	if (paction) paction->end();
	
	if (action)
		paction = action;
	else
		paction = &action_rest;
	if (paction) 
	{
		//    unsigned long tnow = millis();
		//    sout->println(tnow-tlast);
		//    tlast = tnow;
		paction->dump();
		paction->start();
	}
}

void setup()
{
	delay(500);
	//  Serial.begin(9600);
	Serial.begin(57600);
	//  Serial3.begin(9600);
	Serial3.begin(115200);

	delay(500);

	Serial3.println("Here goes...");

	python_setup();

  /*
  delay(1000);
  change the bluetooth baud rate to 115200 bps
  Serial.print("sending AT+BAUD8...");
  Serial3.print("AT+BAUD8");
	for(;;)
	{
		if (Serial3.available() > 0)
		{
			int ch = Serial3.read();
			// echo character to Serial
			Serial.write(ch);
		}
	}
*/

	Wire.begin();
  
	pinMode( LED_PIN, OUTPUT );

	int sdcard_ok = true;
	Serial.print("Initializing SD card...");
	if (!SD.begin(SD_CS)) 
	{
		Serial.println("failed!");
		sdcard_ok = false;
		// return;
	}
	else
		Serial.println("OK");

  
	robot.setup();
  
	//  analogReference(EXTERNAL);
	pinMode( IR_FRONT, INPUT );
	pinMode( A1, INPUT );
	pinMode( A3, INPUT );
  
	pinMode( BUTTON_PIN, INPUT_PULLUP );
  
	irrecv.enableIRIn(); // Start the receiver
  
	compass.init();
	compass.enableDefault();

	// 0x47 = 0b01000111
	// ODR = 0100 (50 Hz ODR); LPen = 0 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)
	// 0x77 = 0b01110111
	// ODR = 0111 (400 Hz ODR); LPen = 0 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)

	compass.writeAccReg(LSM303::CTRL_REG1_A, 0x77);

	/*
	compass.m_min = (LSM303::vector<int16_t>){-295, -57, -501};
	compass.m_max = (LSM303::vector<int16_t>){+589, +1014, +385};
	*/
	read_compass_config_from_eeprom(&compass,EEPROM_COMPASS);

	if (!gyro.init())
	{
		robot.sout->println("Failed to autodetect gyro type!");
	}
	else
	{
		gyro.enableDefault();
	}

	// 0x0F = 0b00001111
	// Normal power mode, all axes enabled, 90Hz
	// 0x8F = 0b10001111
	// Normal power mode, all axes enabled, 380Hz
	gyro.writeReg(L3G_CTRL_REG1, 0x8F);

	// reflectanceSensors.init();
	reflectanceSensors.init(sensorPins, sizeof(sensorPins), 2000, QTR_NO_EMITTER_PIN );
	
	// Read calibration and config settings from EEPROM
	read_reflectance_from_eeprom(&reflectanceSensors,NUM_SENSORS,EEPROM_REFLECT);
	read_gyro_zero_from_eeprom(&imu,EEPROM_GYRO);

	action_scan_rove.pnext_action = &action_forward_rove;
	action_forward_rove.pnext_action = &action_scan_rove;

	action_scan2_rove.pnext_action = &action_forward_rove2;
	action_forward_rove2.pnext_action = &action_scan2_rove;

	action_scan2_rove_ccw.pnext_action = &action_forward_rove2_ccw;
	action_forward_rove2_ccw.pnext_action = &action_scan2_rove_ccw;

	robot.setAction(&action_rest);
	
	// Serial.println("Hello minion!");
/*
	tone(BUZZER_PIN,1000);
	delay(500);
	noTone(BUZZER_PIN);
	pinMode(BUZZER_PIN,INPUT);
*/
	// MotorTest( 1000, 500 );
  
	// imu.log_max = true;

//	run_python_cmd_str("print(\"python says hi!\")");

//	do_file("boot.py");

	// OK, now let robot decide where stdout goes
	set_stdout_callback(stdout_print_strn_robot);


}

#if 0
void loop()
{

  /* add main program code here */
	//for(int i=0;i<10;i++)
	{
		digitalWrite(LED_BUILTIN,1);
		delay(1000);
		digitalWrite(LED_BUILTIN,0);
		delay(500);
	}

}
#endif

void loop()
{
  static boolean button_state=1;
  static boolean state = 1;
  static unsigned long tprev = millis();
  int d = 0;

  IRMenu();
  if (CheckSerial( &Serial )) return;
  if (CheckSerial( &Serial3 )) return;

  if (dump_imu)
  {
    if (imu.log_max)
    {
      imu.dump_mmax(robot.sout);
    }
    else
    {
      imu.dump_current(robot.sout);
    }
    // imu.dump_buffer(robot.sout);
    dump_imu = false;
  }

  boolean butt = digitalRead(BUTTON_PIN);
  if (butt != button_state)
  {
    if (butt)
    {
      // TODO: cycle through action mode list (0-9)
      quick_action++;
      if (quick_action >= NUM_QUICK_ACTIONS) quick_action=0;
      robot.setAction(QuickActions[quick_action]);
    
    }
    button_state = butt;
    delay(20);
  }
 
/*
  if (distance_target_l > 0 || distance_target_r > 0)
  {
    if (distance_target_l > 0 && motor_left.Encoder.Oddometry >= distance_target_l)
    {
      motor_left.Stop();
      distance_target_l = 0;
    }
    if (distance_target_r > 0 && motor_right.Encoder.Oddometry >= distance_target_r)
    {
      motor_right.Stop();
      distance_target_r = 0;
    }
    if (distance_target_l==0 && distance_target_r==0)
    {
      action = ACTION_REST;
    }
  }
*/


  // Read IR sensor
  robot.proximity->read();

  if (robot.paction)
  {
    if (!robot.paction->loop())
    {
      robot.setAction(robot.paction->pnext_action);
    }
  }

}


boolean CheckSerial( Stream * s )
{
  bool handled = false;
  
  if (s->available() > 0)
  {
    // use this stream for output
    robot.sout = s;
	_xmodem_s = s;
    
    int ch = s->read();

	// s->println((int)ch);

    // echo character
    if (ch != 13 && ch != 10)
    {
      s->write(ch);
    }
	if (ch == 13)
    {
      s->print("\r\n");
      parse_serial_buffer();
    }
	else if (ch != 10)
    {
      if (iserial_buffer > sizeof(serial_buffer)-1)
      {
        iserial_buffer = 0;
      }
      serial_buffer[iserial_buffer] = ch;
      iserial_buffer++;
    
      if (serial_buffer_len > sizeof(serial_buffer))
      {
        // serial buffer overload!!!
        s->print("!");
        if (serial_buffer_start > sizeof(serial_buffer)-1)
        {
          serial_buffer_start = 0;
        }
        else
        {
          serial_buffer_start++;
        }
      }
      else
      {
        serial_buffer_len++;
      }
    }
  }
}

void StartIMU()
{
  if (imu_timer_running) return;
  imu_timer_running = true;
  
  /*
  if (imu.log_max)
  {
    imu.dump_mmax_header(robot.sout);
  }
  else
  {
    imu.dump_header(robot.sout);
  }
  */
  
  imu.start();
  imu_timer.begin(UpdateIMU,10000);
}

void StopIMU()
{
  if (!imu_timer_running) return;
	
  imu_timer.end();
  imu.stop();
  
//   dump_imu = true;
/*  
  Serial.print("d=");
  Serial.println(imu.d);

  Serial.print("v=");
  Serial.println(imu.v);
  imu_timer_running = false;

  Serial.print("a(avg)=");
  Serial.println(imu.a_avg/imu.samples);
*/
  imu_timer_running = false;
}

void UpdateIMU()
{
  // if (imu.samples > 1000)
  /*
  if (imu.buffer_overflow)
  {
    StopIMU();
    return;
  }
  */
  
  // compass.read();
  imu.loop();

  // update display every 10 reads
//  dump_imu = ((imu.buff_index % 10) == 0);

}


void SetQuickAction(int i)
{
  if (i < NUM_QUICK_ACTIONS)
  {
    quick_action = i;
    robot.setAction(QuickActions[i]);
  }
}

void IRMenu()
{
  boolean handled = false;
  
  //if (!gmodes[Application::gmode]->allowIRMenu()) return;
  // if (Application::gmode == MODE_IR) return;
  
  if (irrecv.decode(&results)) 
  {
    unsigned int button = IRButtonMap(results.value & 0x7ff);

//    robot.sout->println(button);
 
    switch(button)
    {
      case BUTTON_NUM_0:
      case BUTTON_NUM_1:
      case BUTTON_NUM_2:
      case BUTTON_NUM_3:
      case BUTTON_NUM_4:
      case BUTTON_NUM_5:
      case BUTTON_NUM_6:
      case BUTTON_NUM_7:
      case BUTTON_NUM_8:
      case BUTTON_NUM_9:
        SetQuickAction(button-BUTTON_NUM_0);
        break;
      case BUTTON_RIGHT:
        action_spin_100ms.direction = CW;
        robot.setAction( &action_spin_100ms );
        break;
      case BUTTON_LEFT:
        action_spin_100ms.direction = CCW;
        robot.setAction( &action_spin_100ms );
        break;
      case BUTTON_UP:
        robot.setAction( &action_forward_med );
        break;
      case BUTTON_DOWN:
        robot.setAction( &action_reverse_med );
        break;
      case BUTTON_NEXT:
        action_spin_45deg.direction = CW;
        robot.setAction( &action_spin_45deg );
        break;
      case BUTTON_PREV:
        action_spin_45deg.direction = CCW;
        robot.setAction( &action_spin_45deg );
        break;
      case BUTTON_PLAY:
        imu.log_max = false;
        robot.sout->println("IMU");      
        delay(100);
        StartIMU();
        break;
      case BUTTON_REC:
        imu.log_max = true;
        robot.sout->println("IMU-MAX");      
        delay(100);
        StartIMU();
        break;
      case BUTTON_STOP:
        robot.sout->println("IMU");      
        StopIMU();
        delay(100);
        break;
      case BUTTON_OK:
        //imu.test_buffer_write(222);
        //imu.dump_buffer(&Serial);
        //delay(100);
        break;
    }
    
    irrecv.resume(); // Receive the next value

  }
}

void MotorTest( unsigned long duration, unsigned long pause )
{
  static byte speeds[] = { 64, 128, 192, 255 };

  int i;

  for (int j=0;j<4;j++)
  {
    digitalWrite( MOTOR_R_DIR, j&1 ); 
    digitalWrite( MOTOR_L_DIR, j>>1 ); 

    // Right motor only test
    analogWrite( MOTOR_R_PWM, 0 );  
    analogWrite( MOTOR_L_PWM, 0 );  
    
    for(i=0; i < 4; i++)
    {
      analogWrite( MOTOR_R_PWM, speeds[i] );
      //analogWrite( MOTOR_L_PWM, speeds[i] );
      delay(duration);
    }
    analogWrite( MOTOR_R_PWM, 0 );  
    delay(pause);
  
    // Left motor only test
    for(i=0; i < 4; i++)
    {
      //analogWrite( MOTOR_R_PWM, speeds[i] );
      analogWrite( MOTOR_L_PWM, speeds[i] );
      delay(duration);
    }
    analogWrite( MOTOR_L_PWM, 0 );
    
    delay(pause);
    
    // Both motors
    for(i=0; i < 4; i++)
    {
      analogWrite( MOTOR_L_PWM, speeds[i] );
      analogWrite( MOTOR_R_PWM, speeds[i] );
      delay(duration);
    }
    analogWrite( MOTOR_L_PWM, 0 );
    analogWrite( MOTOR_R_PWM, 0 );
    
    delay(pause);
  }
}

int ChHexToInt( int ch )
{
  if (ch >= 'a' && ch <= 'f')
    return ch - 'a' + 10;

  if (ch >= '0' && ch <= '9')
    return ch - '0';

  if (ch >= 'A' && ch <= 'F')
    return ch - 'A' + 10;
  
  return -1;  
}

char pycmdbuff[3];

void parse_serial_buffer()
{
    int value_length=0;
    int value;
    int val_neg = 0;
    int command=0;
    int ch;
    int nibble;
    
	if (serial_buffer_len > 0 && serial_buffer[serial_buffer_start]=='>')
	{
//		int ipy=0;

		serial_buffer[serial_buffer_len] = 0;
		run_python_cmd_str(&serial_buffer[serial_buffer_start+1]);	// ignore the 1st character

		// Fixes gpio(13,) SCK killing SD
		SPI.begin();
		//
		robot.sout->flush();

		iserial_buffer = 0;
		serial_buffer_start = 0;
		serial_buffer_len = 0;
		return;

		// pycmdbuff[0] = 0;

// NOT USING THIS.....
#if 0

		while(serial_buffer_len > 0 && ipy < (sizeof(pycmdbuff)-1))
		{
			ch = serial_buffer[serial_buffer_start];
			pycmdbuff[ipy++] = ch;
    
			serial_buffer_len--;
		}

			if (serial_buffer_start > sizeof(serial_buffer)-1)
			{
				serial_buffer_start = 0;
			}
			else
			{
				serial_buffer_start++;
			}
		}
		pycmdbuff[ipy++] = 0;
		run_python_cmd_str(&pycmdbuff[1]);	// ignore the 1st character
		return;
#endif
}


  while(serial_buffer_len > 0)
  {
    ch = serial_buffer[serial_buffer_start];
    
    serial_buffer_len--;
    if (serial_buffer_start > sizeof(serial_buffer)-1)
    {
      serial_buffer_start = 0;
    }
    else
    {
      serial_buffer_start++;
    }
    
    if (value_length > 0)
    {
      if (ch == '-')
      {
        val_neg=1;
        continue;
      }
      nibble = ChHexToInt(ch);
      if (nibble < 0)
      {
        robot.sout->println("ERROR!");
        break;
      }
      value = (value << 4) | nibble;
      value_length--;
      if (value_length == 0)
      {
        if (val_neg)
        {
          value = -value;
        }
        switch(command)
        {
/*          
          case 'L':
              motor_left.SetTarget(value);
              // motor_l_target = value;
            break;
            
          case 'P':
            motor_left.Kp = value;
            motor_right.Kp = value;
            break;
          case 'I':
            motor_left.Ki = value;
            motor_right.Ki = value;
            break;
          case 'D':
            motor_left.Kd = value;
            motor_right.Kd = value;
            break;
*/
          case 'l':
            if (value <0)
            {
              robot.motor_L->setDir(REVERSE);
              value = -value;
            }
            else
            {
              robot.motor_L->setDir(FORWARD);
            }
            robot.motor_L->setPWM(value);
            break;
/*            
          case 'R':
              motor_right.SetTarget(value);
            break;
*/            
          case 'r':
            if (value <0)
            {
              robot.motor_R->setDir(REVERSE);
              value = -value;
            }
            else
            {
              robot.motor_R->setDir(FORWARD);
            }
            robot.motor_R->setPWM(value);
            break;
/*            
          case 't':
              // distance_target = value;
              action_duration = value;
*/              break;
/*              
          case 'u': // left distance target
            distance_target_l = abs(value);
            motor_left.Encoder.Oddometry = 0;
            break;
          case 'v': // right distance target
            distance_target_r = abs(value);
            motor_right.Encoder.Oddometry = 0;
            break;
*/
        }
      }
      continue;
    }


    switch(ch)
    {
      case 'a': // autonomous
        robot.setAction( &action_scan_rove );
        break;
        
      case 'b': // backwards
        robot.setAction( &action_reverse_med );
        break;
       
      case 'c':  // compass callibrate
        StopIMU();
        //compass.m_min = imu.mmin;
        //compass.m_max = imu.mmax;
		/*
        robot.sout->println("CALIBRATE");
        calibrate_gyro();
        delay(100);
        calibrate_reflectance_array();
		*/
		compass_calibrate(&robot,&compass,&imu,true);
        break;

      case 'P':
      case 'I':
      case 'D':
        //Serial.print("distance=");
        //Serial.println(analogRead(1));
        command = ch;
        value_length=2;
        value=0;
        val_neg=0;        
        break;
        
      //case 'g': // go
      //  scan(5000);
      //  break;
        
      case 'f': // forward
        robot.setAction( &action_forward_med );
        break;
        
      case 'l': // left
        command = ch;
        value_length=2;
        value=0;
        val_neg=0;        
        break;


      case '>':
        action_spin_45deg.direction = CW;
        robot.setAction( &action_spin_45deg );
        break;
      case '<':
        action_spin_45deg.direction = CW;
        robot.setAction( &action_spin_45deg );
        break;

        
/*        
      case 'L':  // set the left target
        auto_mode = MANUAL;
        command = ch;
        value_length=2;
        value=0;
        val_neg=0;        
        if (motor_left.Direction==STOP) motor_left.Forward();
        break;
*/        
      case 'm': // manual
        robot.setAction( &action_rest );
        break;
/*        
      case 'R':  // set the left target
        auto_mode = MANUAL;
        command = ch;
        value_length=2;
        value=0;
        val_neg=0;        
        if (motor_right.Direction==STOP) motor_right.Forward();
        break;
*/        
      case 'r': // right
        command = ch;
        value_length=2;
        value=0;
        val_neg=0;
        break;
        
      case 's': // stop
        StopIMU();
        robot.setAction( &action_rest );
        break;
/*        
      case 't': // time/duration
        auto_mode = MANUAL;
        command = ch;
        value_length=4;
        value=0;
        val_neg=0;
        break;
*/        
      case 'u': // left distance target
      case 'v': // right distance target
        command = ch;
        value_length=2;
        value=0;
        val_neg=0;
        break;

      case 'w': // write calibration date to eeprom;
        robot.sout->println("Saving reflectance data...");
        save_reflectance_to_eeprom(&reflectanceSensors,NUM_SENSORS,EEPROM_REFLECT);
        robot.sout->println("Saving gyro data...");
        write_gyro_zero_to_eeprom(&imu,EEPROM_GYRO);
		robot.sout->println("Saving compass data...");
		write_compass_config_to_eeprom(&compass,EEPROM_COMPASS);
        robot.sout->println("done.");
        break;

/*        
     case 'z':
       motor_left.Encoder.Oddometry = 0;
       motor_right.Encoder.Oddometry = 0;
       break;
*/        
      case '?':
        robot.dump();
        robot.sout->print("action=");
        if (robot.paction)
        {
          robot.paction->dump();
        }
		
		compass.read();
		robot.sout->print("pitch=");
		robot.sout->println(imu.pitch());
		robot.sout->print("roll=");
		robot.sout->println(imu.roll());
        
        robot.sout->print("batt=");
        robot.sout->print((float)analogRead(A1) * 3.3 * 1.5 / 1023.0);
        robot.sout->println("V");
        robot.sout->print("A9=");
        robot.sout->println(analogRead(A9));
        robot.sout->print("A3=");
        robot.sout->println(analogRead(A3));

        dump_reflectance_calibration(&reflectanceSensors,NUM_SENSORS,robot.sout);
        dump_gzero(&imu,robot.sout);
		dump_compass_config(&compass,robot.sout);

/*        
        Serial.print("duration=");
        Serial.println(action_duration);
        Serial.print("distance=");
        Serial.println(analogRead(1));
*/
/*
        Serial.print("fast(L,R)=");
        Serial.print(SPEED_FAST_L);
        Serial.print(", ");
        Serial.println(SPEED_FAST_R);

        Serial.print("med(L,R)=");
        Serial.print(SPEED_MED_L);
        Serial.print(", ");
        Serial.println(SPEED_MED_R);
        
        Serial.print("slow(L,R)=");
        Serial.print(SPEED_SLOW_L);
        Serial.print(", ");
        Serial.println(SPEED_SLOW_R);
*/      
/*        
        Serial.print("ENC.ms(L,R)=");
//        Serial.print(GetLTime());
        Serial.print(motor_left.Encoder.GetTime());
        Serial.print(", ");
        Serial.println(motor_right.Encoder.GetTime());
        Serial.print("R:P,D=");
        Serial.print(motor_right.Kp);
        Serial.print(", ");
//        Serial.print(motor_right.Ki);
//        Serial.print(", ");
        Serial.println(motor_right.Kd);
        
        Serial.print("Odd (L,R)");
        Serial.print(motor_left.Encoder.Oddometry);
        Serial.print(", ");
        Serial.println(motor_right.Encoder.Oddometry);

//        Serial.print("R:Err=");
//        Serial.println(motor_right._pidTotalError);

        
        //Serial.print("enc(L)=");
        // Serial.println(encoder_count_l);
        //Serial.println(encoder_count_total_l);
        //Serial.print("enc(R)=");
        // Serial.println(encoder_count_r);
        //Serial.println(encoder_count_total_l);
*/

        break;    
    }          
  }
  
  serial_buffer_start = 0;
  iserial_buffer = 0;
}

