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

#include "CmdLine.h"

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


const char os_cmd_prompt[] = ">";
const char python_cmd_prompt[] = "PY>";
CmdLine cmdline(&Serial,os_cmd_prompt);



int cmd_mode = 0;
#define CMD_MODE_OS 0
#define CMD_MODE_PYTHON 1
#define CMD_MODE_DRIVE 2

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
//#define NUM_SENSORS 6
//unsigned int sensorValues[NUM_SENSORS];

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

boolean dump_imu = false;

// Robot robot;

RobotMotor motor_left( MOTOR_L_PWM, MOTOR_L_DIR );
RobotMotor motor_right( MOTOR_R_PWM, MOTOR_R_DIR );

RobotTank robot(&motor_left, &motor_right, &imu, &sharpIR, &reflectanceSensors );

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

ActionBalance action_balance;

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
//  &accelerometer_test,      // 7
  &action_balance,      // 7
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
uint8_t quick_action = 0;

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
		paction->bot = &robot;
		//    unsigned long tnow = millis();
		//    sout->println(tnow-tlast);
		//    tlast = tnow;
		paction->dump();
		paction->start();
	}
}

void SetQuickAction(uint8_t i)
{
	if (i < NUM_QUICK_ACTIONS)
	{
		quick_action = i;
		robot.setAction(QuickActions[i]);
	}
}

// delare here so we have access to robot and robot actions
class CmdLineDrive : public CmdLine
{
public:
	virtual void CursorUp()
	{
		robot.setAction( &action_forward_med );
	}

	virtual void CursorDown()
	{
		robot.setAction( &action_reverse_med );
	}
	
	virtual void CursorLeft()
	{
		action_spin_100ms.direction = CCW;
		robot.setAction( &action_spin_100ms );
	}

	virtual void CursorRight()
	{
		action_spin_100ms.direction = CW;
		robot.setAction( &action_spin_100ms );
	}
};

CmdLineDrive cmdDrive;

void setup()
{
	Serial.begin(115200);
	Serial3.begin(115200);

	delay(200);

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

	Serial.print("Initializing SD card...");
	if (!SD.begin(SD_CS)) 
	{
		Serial.println("failed!");
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
	
/*
	tone(BUZZER_PIN,1000);
	delay(500);
	noTone(BUZZER_PIN);
	pinMode(BUZZER_PIN,INPUT);
*/

	// OK, now let robot decide where stdout goes
	set_stdout_callback(stdout_print_strn_robot);

	do_file("boot.py");

	cmdline.Prompt();
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

	IRMenu();

	if (CheckSerial()) return;

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
			// cycle through action mode list (0-9)
			//quick_action++;
			//if (quick_action >= NUM_QUICK_ACTIONS) quick_action=0;
			//robot.setAction(QuickActions[quick_action]);
			
			robot.setAction(&action_rest);
		}
		button_state = butt;
		delay(20);
	}
 
	// Read IR sensor
	robot.proximity->read();

	if (robot.paction)
	{
		if (!robot.paction->loop())
		{
			if (!python_robot_event("onEnd"))
			{
				robot.setAction(robot.paction->pnext_action);
			}
		}
	}
}

boolean CheckSerial()
{
	if (cmd_mode == CMD_MODE_DRIVE)
	{
		if (cmdDrive.CheckStream(&Serial) || cmdDrive.CheckStream(&Serial3))
		{
			robot.sout = cmdline.output;
			_xmodem_s = robot.sout;
			if (cmdDrive.CommandReady) 
			{
				cmdline._prompt = os_cmd_prompt;
				cmd_mode=CMD_MODE_OS;
				cmdline.Prompt();
				return true;
			}
		}
	}
	else
	{
		if (cmdline.CheckStream(&Serial) || cmdline.CheckStream(&Serial3))
		{
			robot.sout = cmdline.output;
			_xmodem_s = robot.sout;
			if (cmdline.CommandReady) 
			{
				if (cmd_mode==CMD_MODE_OS)
				{
					parse_serial_buffer(cmdline.Command());
				}
				else if (cmd_mode==CMD_MODE_PYTHON)
				{
					if (strcmp(cmdline.Command(),"<<")==0)
					{
						cmdline._prompt = os_cmd_prompt;
						cmd_mode=CMD_MODE_OS;
					}
					else
					{
						run_python_cmd_str(cmdline.Command());	
						// Fixes gpio(13,) SCK killing SD
						SPI.begin();
					}
				}
				cmdline.Prompt();
				return true;
			}
		}
	}
	return false;
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

void parse_serial_buffer(char * sbuff)
{
    int value_length=0;
    int value;
    int val_neg = 0;
    int command=0;
    int ch;
    int nibble;

	if (*sbuff=='/')
	{
		// enter python command line mode...
		cmd_mode=CMD_MODE_DRIVE;
		return;
	}

	if (*sbuff=='>')
	{
		sbuff++;

		if (*sbuff=='>')
		{
			// enter python command line mode...
			cmdline._prompt = python_cmd_prompt;
			cmd_mode=CMD_MODE_PYTHON;
			return;
		}

		run_python_cmd_str(sbuff);	// ignore the 1st character

		// Fixes gpio(13,) SCK killing SD
		SPI.begin();

		return;
	}

	for(;*sbuff;sbuff++)
	{
		ch = (int)*sbuff;
		if (value_length > 0)
		{
			if (*sbuff == '-')
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
			imu.StopTimer();
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
			imu.StopTimer();
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

			case 'x':
				// python_test_call();
				// python_robot_event("doTest");
				robot.setAction( &action_balance );
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
}

void IRMenu()
{
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
				if (robot.paction == &action_balance)
				{
					action_balance.turn(CW,20);
				}
				else
				{
					action_spin_100ms.direction = CW;
					robot.setAction( &action_spin_100ms );
				}
				break;
			case BUTTON_LEFT:
				if (robot.paction == &action_balance)
				{
					action_balance.turn(CCW,20);
				}
				else
				{
					action_spin_100ms.direction = CCW;
					robot.setAction( &action_spin_100ms );
				}
				break;
			case BUTTON_UP:
				if (robot.paction == &action_balance)
				{
					action_balance.move(-1.5);
				}
				else
				{
					robot.setAction( &action_forward_med );
				}
				break;
			case BUTTON_DOWN:
				if (robot.paction == &action_balance)
				{
					action_balance.move(1.5);
				}
				else
				{
					robot.setAction( &action_reverse_med );
				}
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
				imu.StartTimer(10);
				break;
			case BUTTON_REC:
				imu.log_max = true;
				robot.sout->println("IMU-MAX");      
				delay(100);
				imu.StartTimer(10);
				break;
			case BUTTON_STOP:
				robot.sout->println("IMU");      
				imu.StopTimer();
				delay(100);
				break;
			case BUTTON_OK:
				//imu.test_buffer_write(222);
				//imu.dump_buffer(&Serial);
				//delay(100);
				break;
			case BUTTON_TEXT:
				robot.MotorTest( 1000, 500 );
				break;
		}
    
		irrecv.resume(); // Receive the next value
	}
}
