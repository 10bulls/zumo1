#ifndef RobotIMU_h_defined
#define RobotIMU_h_defined

#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>

#define IMU_BUFF_SIZE  100

class RobotIMU
{
public:

  RobotIMU(LSM303 * c, L3G * g)
  {
    pcompass = c;
    pgyro = g;
    t=0;
    v=0;
    d=0;
    log_max = false;
  }
  
  void start();
  
  void stop()
  {
  }

  void read()
  {
    pcompass->read();
    pgyro->read();
  }

  float getHeading()
  {
    return heading(pcompass->m, pcompass->a, (LSM303::vector<int>){1, 0, 0} );
  }

  void loop();
  
  L3G::vector get_g_avg();
  
  float get_degree_change();
  
  float pitch();

  float roll();

  void dump_buffer( Stream * s );

  void dump_header( Stream * s );

  void dump_mmax_header( Stream * s );

  void dump_mmax(Stream * s);
  
  void dump_max(Stream * s);

  void dump_min(Stream * s);

  void dump_buffer_item( Stream * s, int i );
  
  void dump_current( Stream * s);
  
  template <typename Ta, typename Tb, typename To> void vector_cross(const LSM303::vector<Ta> *a,const LSM303::vector<Tb> *b, LSM303::vector<To> *out)
  {
    out->x = (a->y * b->z) - (a->z * b->y);
    out->y = (a->z * b->x) - (a->x * b->z);
    out->z = (a->x * b->y) - (a->y * b->x);
  }
  
  template <typename Ta, typename Tb> float vector_dot(const LSM303::vector<Ta> *a, const LSM303::vector<Tb> *b)
  {
    return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
  }
  
  void vector_normalize(LSM303::vector<float> *a)
  {
    float mag = sqrt(vector_dot(a, a));
    a->x /= mag;
    a->y /= mag;
    a->z /= mag;
  }

    
	/*
	Returns the angular difference in the horizontal plane between the
	"from" vector and north, in degrees.
	*/
	template <typename T> float heading(LSM303::vector<int16_t> m, LSM303::vector<int16_t> a, LSM303::vector<T> from);

	void StartTimer(unsigned int ms)
	{
		if (_timer_running) return;
		_timer_running = true;

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
		
		start();
		_timer_imu = this;
		imu_timer.begin(UpdateIMU,ms*1000);
	}

	void StopTimer()
	{
		if (!_timer_running) return;
	
		imu_timer.end();
		stop();
  
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
		_timer_running = false;
	}

	static void UpdateIMU()
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
		_timer_imu->loop();

		// update display every 10 reads
		//  dump_imu = ((imu.buff_index % 10) == 0);
	}



	unsigned long t;
	float dt;
  
	float v;  // velocity
	float d;  // distance
  
	LSM303 * pcompass;
	L3G * pgyro;
  
	int a_avg;
	int samples;
  
	// accelerometer buffer
	unsigned long tbuff[IMU_BUFF_SIZE];
	LSM303::vector<int16_t> abuff[IMU_BUFF_SIZE];
	LSM303::vector<int16_t> mbuff[IMU_BUFF_SIZE];
	L3G::vector gbuff[IMU_BUFF_SIZE];
	int16_t buff_start;
	int16_t buff_len;
	int16_t buff_index;
	boolean buffer_overflow;
  
	boolean log_max;
	LSM303::vector<int16_t> amax;
	LSM303::vector<int16_t> amin;
	LSM303::vector<int16_t> mmax;
	LSM303::vector<int16_t> mmin;
	L3G::vector gmax; 
	L3G::vector gmin; 
	L3G::vector gzero; 
  
	float degrees;

private:
	boolean _timer_running = false;
	IntervalTimer imu_timer;
	static RobotIMU * _timer_imu;
};


#endif
