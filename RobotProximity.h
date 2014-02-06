#ifndef RobotProximity_h_defined
#define RobotProximity_h_defined

class RobotProximity
{
public:
  RobotProximity()
  {
    distance = 0;
  }
  
  virtual void setup()
  {
  }
  
  virtual float read()
  {
    return distance;
  }
  
  float distance;    // cm
};

/*
prog_uchar transferFunctionLUT3V[] PROGMEM =
	{
		255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
		255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 79,
		76, 73, 71, 69, 67, 65, 63, 62, 60, 58, 57, 52, 50, 49, 48, 47,
		49, 48, 47, 46, 45, 44, 43, 43, 42, 41, 40, 40, 39, 38, 37, 37,
		36, 36, 35, 35, 34, 33, 33, 32, 32, 31, 31, 31, 30, 30, 29, 29,
		29, 28, 28, 27, 27, 27, 26, 26, 26, 25, 25, 25, 25, 24, 24, 24,
		23, 23, 23, 23, 22, 22, 22, 22, 22, 21, 21, 21, 21, 20, 20, 20,
		20, 20, 20, 19, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 18, 17,
		17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
		15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14,
		13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12,
		12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11,
		11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10,
		10, 10, 10, 10, 10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	};
*/

class SharpIR : public RobotProximity
{
public:
  SharpIR(int pin)
  {
    _ir_pin = pin;
  }

  virtual void setup()
  {
    pinMode( _ir_pin, INPUT );
  }
  
  virtual float read()
  {
    raw = analogRead( _ir_pin );
    
//    distance = pgm_read_byte_near (transferFunctionLUT3V + (raw/4));

  float volts = raw*3.3/1023.0;   // value from sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
//  distance = 65*pow(volts, -1.10);          // worked out from graph 65 = theretical distance / (1/Volts)S - luckylarry.co.uk
//  distance = 27*pow(volts, -1.10);          // worked out from graph 65 = theretical distance / (1/Volts)S - luckylarry.co.uk
//   distance = 27*pow(volts, -1.20);          // worked out from graph 65 = theretical distance / (1/Volts)S - luckylarry.co.uk
   distance = 26.0*pow(volts, -1.10);          // worked out from graph 65 = theretical distance / (1/Volts)S - luckylarry.co.uk

  return distance;
  }
  
  int raw;

private:
  int _ir_pin;
};


#endif
