
#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>

#include "RobotMotor.h"
#include "RobotIMU.h"
#include "RobotProximity.h"
#include "Robot.h"
#include "RobotAction.h"

#define SCAN_PWM  0x50
#define SCAN_DURATION  100

#define LED_PIN     13
#define BUZZER_PIN  20
#define BUTTON_PIN  12

// If old arduino
//#define MOTOR_R_DIR  7
//#define MOTOR_L_DIR  8
// use 3 and for for Teensy (7&8 are Serial3)
#define MOTOR_R_DIR  3
#define MOTOR_L_DIR  4

#define MOTOR_R_PWM  9
#define MOTOR_L_PWM  10

#define CW      0
#define CCW     1

// #define IR_FRONT   4
#define IR_FRONT   A2

#define ACTION_REST             0
#define ACTION_FORWARD_FAST     1
#define ACTION_FORWARD_MED      2
#define ACTION_FORWARD_SLOW     3
#define ACTION_REVERSE          4
#define ACTION_SCAN             5
#define ACTION_CUSTOM           6

#define ACTION_SCAN1            7
#define ACTION_SCAN2            8
#define ACTION_SCAN3            9
#define ACTION_ROVE             10

#define ACTION_FOLLOW          11

#define ACTION_SPIN            12
#define ACTION_MOVE            13

#define   AUTO                  1
#define   MANUAL                0

// #define FOLLOW_TARGET 800
#define FOLLOW_TARGET       12   // cm note < 8 cm not accurate

//int SPEED_FAST = 128;  // 0x1e
//int SPEED_MED  = 64;
//int SPEED_SLOW = 32;  // 0x5a
// int SPEED_FAST = 128;  // 0x1e
int SPEED_FAST = 0xff;  // 0x1e
int SPEED_MED  = 128;
int SPEED_SLOW = 64;  // 0x5a


// int auto_mode = AUTO;
int auto_mode = MANUAL;
int action = ACTION_REST;

//int distance_target_l = 0;
//int distance_target_r = 0;

// int action_duration = 0;
int action_duration = -1;
// define a circular buffer
char serial_buffer[30];
int iserial_buffer;
int serial_buffer_start;
int serial_buffer_len;

unsigned long time_last;
int time_step = 100;
// int time_step = 250;

// Motor output controllers 
// MotorPID motor_left(MC_B1, MC_B2, ENCODER_L);
// MotorPID motor_right(MC_A1, MC_A2, ENCODER_R);

//int proximity_max = 0;
//int proximity = 0;
float distance_max = 0;

float heading = -1;
float heading_target = -1;
float spin_target = 0;
int spin_pwm = 0;
int spin_direction = 0;

#define COMPASS_ERROR 1


///////////////////////////////////

#include <IRremote.h>
#include "irstuff.h"

const int RECV_PIN = 11;

IRrecv irrecv(RECV_PIN);
// IRsend irsend;

decode_results results;

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

void setup()
{
  delay(500);
//  Serial.begin(9600);
  Serial.begin(57600);
  Serial3.begin(9600);
  Wire.begin();
  
  pinMode( LED_PIN, OUTPUT );
  
  robot.setup();
  
//  analogReference(EXTERNAL);
  pinMode( IR_FRONT, INPUT );
  pinMode( A3, INPUT );
  
  pinMode( BUTTON_PIN, INPUT_PULLUP );
  
  time_last = millis();
  
  irrecv.enableIRIn(); // Start the receiver
  
  compass.init();
  compass.enableDefault();
  // TODO: run an automated calibration
  /*
  compass.m_min = (LSM303::vector<int16_t>){-261, -29, -443};
  compass.m_max = (LSM303::vector<int16_t>){+666, +897, +453};
  */
  /*
  compass.m_min = (LSM303::vector<int16_t>){-467, -178, -588};
  compass.m_max = (LSM303::vector<int16_t>){+581, +884, +410};
*/
  compass.m_min = (LSM303::vector<int16_t>){-295, -57, -501};
  compass.m_max = (LSM303::vector<int16_t>){+589, +1014, +385};


  if (!gyro.init())
  {
    robot.sout->println("Failed to autodetect gyro type!");
  }
  else
  {
    gyro.enableDefault();
  }

  /*
  tone(BUZZER_PIN,1000);
  delay(500);
  noTone(BUZZER_PIN);
  */
  // MotorTest( 1000, 500 );
  
  // imu.log_max = true;
}


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
//      Serial.println("SCAN");
//      scan(0);
      if (action == ACTION_REST)
      {
        // action = ACTION_FOLLOW;
        robot.sout->println("SCAN");
        scan(0);
      }
      else if (action == ACTION_FOLLOW)
      {
        rest(0);
        // action = ACTION_REST;
        robot.sout->println("REST");
      }
      else
      {
        action = ACTION_FOLLOW;
        robot.sout->println("FOLLOW");
      }
      
        
    }
    button_state = butt;
    delay(20);
  }
 
/*    
  unsigned long tnow = millis();
  
  if (tnow - tprev >= 500)
  {
    tprev = tnow;
    state = !state;
    digitalWrite(LED_PIN,state);
    
    d = analogRead( IR_FRONT );
    Serial.println(d);
    
  }
*/
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
//  proximity = analogRead( IR_FRONT );

  robot.proximity->read();
    
  if (action == ACTION_SCAN3)
  {
    // if (proximity < proximity_max)
    if (robot.proximity->distance > distance_max)
    {
      // rest(0);
      rove();
    }
  }
  else if (action == ACTION_SCAN1 || action == ACTION_SCAN2)
  { 
    // if (proximity < proximity_max)
    if (robot.proximity->distance > distance_max)
    {
      // proximity_max = proximity;
      distance_max = robot.proximity->distance;
    }
  }
  else if (action == ACTION_ROVE)
  {
    // if (proximity > 300)
    if (robot.proximity->distance < 20)
    {
      scan(0);
    }
  }

    if (heading_target != -1)
    {
      // Read compass
      if (!imu_timer_running)
      {
        compass.read();
      }
      heading = compass.heading();
      robot.sout->print("h=");      
      robot.sout->print(heading);   
      robot.sout->print("    ");         
      
      float diff = 0;
  
      if (spin_direction==CW)
      {
        diff = spin_target - imu.degrees;
      }
      else
      {
        diff = spin_target + imu.degrees;
      }
      
/*     
      
      
      if (spin_direction==CW)
      {
        if (heading > heading_target)
          diff = 360 + heading - heading_target;
        else 
          diff = heading_target - heading;
      }
      else if (spin_direction==CCW)
      {
        if (heading < heading_target)
          diff = 360 - heading_target + heading;
        else 
          diff = heading - heading_target;
      }
      if (diff > spin_target + 90) diff = 360 - diff;
*/      
      robot.sout->print("d=");      
      robot.sout->println(diff);      

      // if (diff < COMPASS_ERROR)
      if (diff < 0)
      {
        heading_target = -1;
        rest(0);
        delay(100);
        StopIMU();
        // imu.dump_buffer(robot.sout);
        //if (imu.buffer_overflow)
        //  robot.sout->println("OVERFLOW!");
        //float degdiff = imu.get_degree_change();
        robot.sout->print("gyro change=");      
        robot.sout->println(imu.degrees);      
        return;
      }
    }


  unsigned long time_now = millis();
  
  if ((time_now > time_last && (time_now-time_last) >= time_step) ||
      (time_now < time_last && (time_last-time_now) >= time_step))
  {
    time_last = time_now;
    
   if (action == ACTION_FOLLOW)
    {    
      action_follow();
      return;
    }
    
    if (auto_mode == AUTO)
    {
      // rover(); 
      scan(0);
    }
    else
    {
      if (action_duration == 0)
      {
        if (action == ACTION_SCAN1)
        {
          scan2();
        }
        else if (action == ACTION_SCAN2)
        {
          scan3();
        }
        else if (action == ACTION_SCAN3)
        {
          rove();
        }
        else if (action == ACTION_ROVE)
        {
          // rest(0);
          scan(0);
        }
        else
        {
          rest(0);
        }
      }
 

//    motor_left.UpdateMotor();
//    motor_right.UpdateMotor();
    }  

    if (action_duration > 0)
    {
      if (action_duration < time_step)
        action_duration = 0;
       else
        action_duration -= time_step;
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
    
    int ch = s->read();
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
    else
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
  
  if (imu.log_max)
  {
    imu.dump_mmax_header(robot.sout);
  }
  else
  {
    imu.dump_header(robot.sout);
  }
  
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
  dump_imu = ((imu.buff_index % 10) == 0);

}

void calibrate_gyro( void )
{
  robot.sout->println("Starting gyro calibrate...");

  StartIMU();
  
  while( !imu.buffer_overflow )
  {
    delay(100);    
  }
  
  StopIMU();
  
  imu.gzero = imu.get_g_avg();
  robot.sout->print("Gyro zero(x,y,z) = ");
  robot.sout->print(imu.gzero.x);
  robot.sout->print(",");
  robot.sout->print(imu.gzero.y);
  robot.sout->print(",");
  robot.sout->println(imu.gzero.z);
}

void IRMenu()
{
  boolean handled = false;
  
  //if (!gmodes[Application::gmode]->allowIRMenu()) return;
  // if (Application::gmode == MODE_IR) return;
  
  if (irrecv.decode(&results)) 
  {
    unsigned int button = IRButtonMap(results.value & 0x7ff);

    robot.sout->println(button);
    
    switch(button)
    {
      case BUTTON_NUM_0:
        robot.sout->println("REST");
        rest(0);
//        action = ACTION_REST;
        break;
      case BUTTON_NUM_1:
        robot.sout->println("SCAN");      
        scan(0);
        break;
      case BUTTON_NUM_2:
        robot.sout->println("FOLLOW");      
        action = ACTION_FOLLOW;
        break;      
      case BUTTON_RIGHT:
        robot.sout->println("SPIN(CW)");      
        spin(CW,SPEED_SLOW,100);
        break;
      case BUTTON_LEFT:
        robot.sout->println("SPIN(CCW)");      
        spin(CCW,SPEED_SLOW,100);
        break;
      case BUTTON_UP:
        robot.sout->println("FORWARD");      
//        move(FORWARD,SPEED_SLOW,100);
        move(FORWARD,SPEED_MED,100);
  
        break;
      case BUTTON_DOWN:
        robot.sout->println("SPIN(CCW)");      
//        move(REVERSE,SPEED_SLOW,100);
        move(REVERSE,SPEED_MED,100);
          break;
      case BUTTON_NEXT:
        robot.sout->println("SPIN(CW)");      
        spin_degree(CW,SPEED_SLOW,90);
        break;
      case BUTTON_PREV:
        robot.sout->println("SPIN(CCW)");      
        spin_degree(CCW,SPEED_SLOW,90);
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


void action_follow()
{
//  robot.sout->println(proximity);
  robot.sout->println(robot.proximity->distance);
  
  // proximity 50 (far)
  // if (proximity < 50)
  if (robot.proximity->distance > 60 || robot.proximity->distance < 8 )
  {
    // error or really close to target
    robot.stop();
  }
  // else if (proximity >= FOLLOW_TARGET -10 && proximity <= FOLLOW_TARGET +10)
  else if (abs(robot.proximity->distance - FOLLOW_TARGET) <= 1.0)
  {
    // where we want to be
    robot.stop();
  }
  else
  {
    boolean dir = FORWARD;
    // int diff = FOLLOW_TARGET - proximity;
    float diff = robot.proximity->distance - FOLLOW_TARGET;
    if (diff < 0) 
    {
      dir = REVERSE;
      diff = -diff;
    }
    if (diff > 10.0) diff = 10.0;
//    int speed = SPEED_FAST * diff / 255;
      int speed = SPEED_MED * diff / 10.0;
    
    robot.sout->print("v=");
    robot.sout->println(speed);
    
    robot.move_pwm(dir,speed);
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

void parse_serial_buffer()
{
    int value_length=0;
    int value;
    int val_neg = 0;
    int command=0;
    int ch;
    int nibble;
    
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
            
          case 't':
              // distance_target = value;
              action_duration = value;
              break;
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
        auto_mode = AUTO;
        scan(5000);
        break;
        
      case 'b': // backwards
        auto_mode = MANUAL;
        robot.reverse_pwm(SPEED_MED);
        break;
        
      case 'c':  // compass callibrate
        StopIMU();
        //compass.m_min = imu.mmin;
        //compass.m_max = imu.mmax;
        robot.sout->println("CALIBRATE");
        calibrate_gyro();
        delay(100);
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
        
      case 'g': // go
        scan(5000);
        break;
        
      case 'f': // forward
        auto_mode = MANUAL;
        robot.forward_pwm(SPEED_MED);
        break;
        
      case 'l': // left
        auto_mode = MANUAL;
        command = ch;
        value_length=2;
        value=0;
        val_neg=0;        
        break;


      case '>':
        robot.sout->println("SPIN(CW)");      
        spin_degree(CW,SPEED_SLOW,90);
        break;
      case '<':
        robot.sout->println("SPIN(CCW)");      
        spin_degree(CCW,SPEED_SLOW,90);
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
        auto_mode = MANUAL;
        rest(-1);
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
        auto_mode = MANUAL;
        command = ch;
        value_length=2;
        value=0;
        val_neg=0;
        break;
        
      case 's': // stop
        StopIMU();
        auto_mode = MANUAL;
        rest(-1);
        break;
        
      case 't': // time/duration
        auto_mode = MANUAL;
        command = ch;
        value_length=4;
        value=0;
        val_neg=0;
        break;
        
      case 'u': // left distance target
      case 'v': // right distance target
        command = ch;
        value_length=2;
        value=0;
        val_neg=0;
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
        switch(action)
        {
            case ACTION_REST:
                robot.sout->println("REST");
                break;
            case ACTION_FORWARD_FAST:
                robot.sout->println("FORWARD_FAST");
                break;
            case ACTION_FORWARD_MED:
                robot.sout->println("FORWARD_MED");
                break;
            case ACTION_FORWARD_SLOW:
                robot.sout->println("FORWARD_SLOW");
                break;
            case ACTION_REVERSE:
                robot.sout->println("REVERSE");
                break;
            case ACTION_SCAN:
                robot.sout->println("SCAN");
                break;
            case ACTION_CUSTOM:
                robot.sout->println("CUSTOM");
                break;
            default:
                robot.sout->println("UNKNOWN");
                break;
        }
        
        robot.sout->print("A2=");
        robot.sout->println(analogRead(A2));
        robot.sout->print("A3=");
        robot.sout->println(analogRead(A3));


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

void reverse(int duration)
{
  action = ACTION_REVERSE;
  action_duration = duration;
  robot.reverse_pwm(SPEED_MED);
}

void forward_fast(int duration)
{
  action = ACTION_FORWARD_FAST;
  action_duration = duration;
  robot.forward_pwm(SPEED_FAST);
}

void forward_med(int duration)
{
  action = ACTION_FORWARD_MED;
  action_duration = duration;
  robot.forward_pwm(SPEED_MED);
}

void forward_slow(int duration)
{
  action = ACTION_FORWARD_SLOW;
  action_duration = duration;
  robot.forward_pwm(SPEED_SLOW);
}

boolean scan_direction = 0;

int scan_duration = SCAN_DURATION;

void scan(int duration)
{
  static int scan_count = 0;
  
  scan_count++;
  
  // start turning left
  action = ACTION_SCAN1;
  
  if (scan_count > 100)
  {
    scan_count = 0;
    scan_direction = !scan_direction;  
  }

  int speed = SPEED_SLOW;
  // if (proximity < 100)
  if (robot.proximity->distance > 60)
  {
    // coast is clear
    speed = SPEED_FAST;  
//    motor_left.setPWM(SPEED_FAST);
//    motor_right.setPWM(SPEED_FAST);
    action_duration = SCAN_DURATION;
  }
  // else if (proximity < 200)
  if (robot.proximity->distance > 30)
  {
    // coast is clear
    speed = SPEED_MED;    
//    motor_left.setPWM(SPEED_MED);
//    motor_right.setPWM(SPEED_MED);
    action_duration = 2 * SCAN_DURATION;
  }
  else
  {
    // coast is clear
//    motor_left.setPWM(SPEED_SLOW);
//    motor_right.setPWM(SPEED_SLOW);
    action_duration = 2 * SCAN_DURATION;
  }
  robot.spin_pwm(scan_direction,speed);
//  motor_left.setDir(scan_direction);
//  motor_right.setDir(!scan_direction);
  
//  proximity_max = proximity;
  distance_max = robot.proximity->distance;
/*  
  action = ACTION_SCAN;
  action_duration = duration;
  motor_left.reverse(SPEED_MED);
  motor_right.forward(SPEED_MED);
*/
}

void scan2()
{
  // start turning left
  action = ACTION_SCAN2;
//  motor_left.setPWM(SCAN_PWM);
//  motor_right.setPWM(SCAN_PWM);
//  motor_left.setDir(!scan_direction);
//  motor_right.setDir(scan_direction);
  robot.spin_dir(!scan_direction);
  action_duration = 2*scan_duration;
}

void scan3()
{
  // start turning left
  action = ACTION_SCAN3;
//  motor_left.setDir(scan_direction);
//  motor_right.setDir(!scan_direction);
//  action_duration = 0x200;
  robot.spin_dir(scan_direction);
  action_duration = 2*scan_duration;
}

void rove()
{
  action = ACTION_ROVE;
  // if (proximity < 100)
  if (robot.proximity->distance > 60)
  {
    action_duration = 2000;
    robot.forward_pwm(SPEED_FAST);
  }
  else
  {
    action_duration = 1000;
    robot.forward_pwm(SPEED_MED);
  }
}


void rest(int duration)
{
  action = ACTION_REST;
  action_duration = duration;
  robot.stop();
  heading_target = -1;
}

/*
void rover()
{
  if (action_duration == 0)
  {
    if (auto_mode == AUTO)
    {
      // finished current action
      if (action == ACTION_REVERSE)
      {
        scan(10000);
      }
      else if (action == ACTION_SCAN)
      {
        rest(10000);
      }
      else if (action == ACTION_REST)
      {
        scan(5000);
      }
      else
      {
        
      }
    }
    else
    {
      rest(-1);
    }
  }

//  int distance = analogRead(IR_FRONT);
  int distance = proximity;

  if (distance > 500)
  {
      if (action == ACTION_FORWARD_FAST || 
          action ==  ACTION_FORWARD_MED  || 
          action ==  ACTION_FORWARD_SLOW  )
      {
        reverse(1000);
      }
  }
  else if (distance > 400)
  {
      if (action == ACTION_FORWARD_FAST || 
          action ==  ACTION_FORWARD_MED  || 
          action ==  ACTION_FORWARD_SLOW  )
      {
        scan(5000);
      }
      else if (action == ACTION_SCAN && action_duration <= 1000)
      {
        forward_slow(2000);
      }
  }
  else if (distance > 300)
  {
    if (action == ACTION_FORWARD_FAST)
    {
      // slow down
      forward_med(-1);
    }
    else if (action == ACTION_FORWARD_MED)
    {
      if (distance > 350)
            forward_slow(-1);
      // turn left a little
      if (motor_left.getPWM() - 5 < SPEED_SLOW)
        scan(5000);
      else
        motor_left.setPWM(motor_left.getPWM() - 5);
    }
    else if (action == ACTION_FORWARD_SLOW)
    {
      if (motor_left.getPWM() - 5 < SPEED_SLOW)
        scan(5000);
      else
        motor_left.setPWM(motor_left.getPWM() - 5);
    }
    else if (action == ACTION_SCAN && action_duration <= 2000)
    {
      forward_slow(2000);
    }
  }
  else
  {
    // clear ahead
    if (action == ACTION_SCAN)
    {
      forward_slow(2000);
    }
    else if (action_duration == 0)
    {
      if (action == ACTION_FORWARD_SLOW)
      {
        forward_med(2000);
      }
      else if (action == ACTION_FORWARD_MED)
      {
        forward_fast(2000);
      }
    }
  }
}
*/

void spin(int dir, int pwm, int duration)
{
  // start turning left
  action = ACTION_SPIN;
  spin_pwm = pwm;
  spin_direction = dir;
  robot.spin_pwm(dir,pwm);
//  motor_left.setDir(dir);
//  motor_right.setDir(!dir);
//  motor_left.setPWM(pwm);
//  motor_right.setPWM(pwm);
  action_duration = duration;
}

void spin_degree(int dir, int pwm, float target)
{
  // Read compass
  compass.read();
  heading = compass.heading();

  spin_target = target;

  if (dir == CW)
    heading_target = heading + target;
  else
    heading_target = heading - target;
    
  if (heading_target <= 0) 
    heading_target += 360;
  else
    if (heading_target > 360.0) heading_target -= 360;
 
  robot.sout->print("heading_target=");
  robot.sout->println(heading_target);
  
  StartIMU();
  
  // start turning left
  action = ACTION_SPIN;
  spin_pwm = pwm;
  spin_direction = dir;
  robot.spin_pwm(dir,pwm);
  action_duration = 10000;
}


int move_pwm = 0;
int move_direction = 0;

void move(int dir, int pwm, int duration)
{
  // start turning left
  action = ACTION_MOVE;
  move_pwm = pwm;
  move_direction = dir;
  robot.move_pwm(dir,pwm);
  action_duration = duration;
}
