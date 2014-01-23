#ifndef RobotIMU_h_defined
#define RobotIMU_h_defined

#define IMU_BUFF_SIZE  400

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
  
  void start()
  {
    degrees = 0;
    t = millis();
    v = 0;
    d = 0;
    a_avg = 0;
    samples = 0;    
    buff_start = 0;
    buff_index = 0;
    buff_len = 0;
    buffer_overflow = false;
    
    pcompass->read();
    
    if (log_max)
    {
      amax = pcompass->a;
      amin = pcompass->a;
      mmax = pcompass->m;
      mmin = pcompass->m;
      gmax = pgyro->g;
      gmin = pgyro->g;
    }
  }
  
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

  void loop()
  {
    pcompass->read();
    pgyro->read();
	tbuff[buff_index] = millis();
    abuff[buff_index] = pcompass->a;
    mbuff[buff_index] = pcompass->m;
    gbuff[buff_index] = pgyro->g;
    
	// 10 / 1000 for 10ms sample rate
	// 
    degrees += (gbuff[buff_index].z - gzero.z) * 0.00875 * 10.0 / 1000.0;
    
    if (log_max)
    {
      amax.x = max(amax.x, pcompass->a.x);
      amax.y = max(amax.y, pcompass->a.y);
      amax.z = max(amax.z, pcompass->a.z);      
      amin.x = min(amin.x, pcompass->a.x);
      amin.y = min(amin.y, pcompass->a.y);
      amin.z = min(amin.z, pcompass->a.z);      
      mmax.x = max(mmax.x, pcompass->m.x);
      mmax.y = max(mmax.y, pcompass->m.y);
      mmax.z = max(mmax.z, pcompass->m.z);      
      mmin.x = min(mmin.x, pcompass->m.x);
      mmin.y = min(mmin.y, pcompass->m.y);
      mmin.z = min(mmin.z, pcompass->m.z);      
      gmax.x = max(gmax.x, pgyro->g.x);
      gmax.y = max(gmax.y, pgyro->g.y);
      gmax.z = max(gmax.z, pgyro->g.z);      
      gmin.x = min(gmin.x, pgyro->g.x);
      gmin.y = min(gmin.y, pgyro->g.y);
      gmin.z = min(gmin.z, pgyro->g.z);      
    }
    
    
    buff_index++;
    if (buff_index >= IMU_BUFF_SIZE)
    {
      buff_index = 0;
      buffer_overflow = true;
    }
    if (buff_len == IMU_BUFF_SIZE)
    {
      buff_start = buff_index;
    }
    else
    {
      buff_len++;
    }
  }
  
  /*
  void test_buffer_write(int len)
  {
    start();
    for(int i=0;i<len;i++)
    {
      abuff[buff_index].x = i+1;
      buff_index++;
      if (buff_index >= IMU_BUFF_SIZE)
      {
        buff_index = 0;
      }
      if (buff_len == IMU_BUFF_SIZE)
      {
        buff_start = buff_index;
      }
      else
      {
        buff_len++;
      }
    }
  }
  */

  L3G::vector get_g_avg()
  {
    L3G::vector avg;
    avg.x = 0;
    avg.y = 0;
    avg.z = 0;
    
    if (!buff_len) return avg;
    
    int n = 0;
    for (int i=buff_start; n < buff_len; i++,n++)
    {
      if (i >= IMU_BUFF_SIZE) i=0;
      avg.x += gbuff[i].x;
      avg.y += gbuff[i].y;
      avg.z += gbuff[i].z;
    }
    avg.x /= (float)buff_len;
    avg.y /= (float)buff_len;
    avg.z /= (float)buff_len;
    
    return avg;
  }
  
  float get_degree_change()
  {
    float d = 0;

    int n = 0;
    for (int i=buff_start; n < buff_len; i++,n++)
    {
      if (i >= IMU_BUFF_SIZE) i=0;
      
      // assume 2000 dps sensitivity and 10ms samples
      // d += (gbuff[i].z - gzero.z) * .07 * 10.0 / 1000.0;
      // assume 250 dps sensitivity and 10ms samples
      d += (gbuff[i].z - gzero.z) * 0.00875 * 10.0 / 1000.0;
      
    }
    return d;
  }
  
  float pitch()
  {
    // scale by 1/000 as reading is 1mg per LSB
    float ax = (pcompass->a.x >> 4);
    float ay = (pcompass->a.y >> 4);
    float az = (pcompass->a.z >> 4);

	return (atan2(ax,sqrt(ay*ay+az*az)) * 180.0)/M_PI;
  }

  float roll()
  {
    // scale by 1/000 as reading is 1mg per LSB
    float ax = (pcompass->a.x >> 4);
    float ay = (pcompass->a.y >> 4);
    float az = (pcompass->a.z >> 4);

	return (atan2(ay,-az) * 180.0)/M_PI;
  }


  void dump_buffer( Stream * s )
  {
    int n = 0;
    s->println("ms\tG.X\tG.Y\tG.Z\tA.X\tA.Y\tA.Z\tM.X\tM.Y\tM.Z\tHEADING");
    for (int i=buff_start; n < buff_len; i++,n++)
    {
      if (i >= IMU_BUFF_SIZE) i=0;
      dump_buffer_item(s,i);
    }
    
    if (log_max)
    {
      s->println("MIN");
      s->println("G.X\tG.Y\tG.Z\tA.X\tA.Y\tA.Z\tM.X\tM.Y\tM.Z");
      dump_min(s);
      s->println("MAX");
      s->println("G.X\tG.Y\tG.Z\tA.X\tA.Y\tA.Z\tM.X\tM.Y\tM.Z");
      dump_max(s);
    }
  }

  void dump_header( Stream * s )
  {
    s->println("G.X\tG.Y\tG.Z\tA.X\tA.Y\tA.Z\tM.X\tM.Y\tM.Z\tHEADING");
  }

  void dump_mmax_header( Stream * s )
  {
    s->println("MIN.X\tMIN.Y\tMIN.Z\tMAX.X\tMAX.Y\tMAX.Z");
  }

  void dump_mmax(Stream * s)
  {
    s->print(mmin.x);
    s->print("\t");
    s->print(mmin.y);
    s->print("\t");
    s->print(mmin.z);
    s->print("\t");
    s->print(mmax.x);
    s->print("\t");
    s->print(mmax.y);
    s->print("\t");
    s->println(mmax.z);
  }
  
  void dump_max(Stream * s)
  {
    s->print(gmax.x);
    s->print("\t");
    s->print(gmax.y);
    s->print("\t");
    s->print(gmax.z);
    s->print("\t");
    s->print(amax.x);
    s->print("\t");
    s->print(amax.y);
    s->print("\t");
    s->print(amax.z);
    s->print("\t");
    s->print(mmax.x);
    s->print("\t");
    s->print(mmax.y);
    s->print("\t");
    s->println(mmax.z);
  }

  void dump_min(Stream * s)
  {
    s->print(gmin.x);
    s->print("\t");
    s->print(gmin.y);
    s->print("\t");
    s->print(gmin.z);
    s->print("\t");
    s->print(amin.x);
    s->print("\t");
    s->print(amin.y);
    s->print("\t");
    s->print(amin.z);
    s->print("\t");
    s->print(mmin.x);
    s->print("\t");
    s->print(mmin.y);
    s->print("\t");
    s->println(mmin.z);
  }


  void dump_buffer_item( Stream * s, int i )
  {
	s->print(tbuff[i] - tbuff[buff_start]);
	s->print("\t");
    s->print(gbuff[i].x);
    s->print("\t");
    s->print(gbuff[i].y);
    s->print("\t");
    s->print(gbuff[i].z);
    s->print("\t");
    s->print(abuff[i].x >> 4);
    s->print("\t");
    s->print(abuff[i].y >> 4);
    s->print("\t");
    s->print(abuff[i].z >> 4);
    s->print("\t");
    s->print(mbuff[i].x);
    s->print("\t");
    s->print(mbuff[i].y);
    s->print("\t");
    s->print(mbuff[i].z);
    s->print("\t");
    s->println(heading(mbuff[i], abuff[i], (LSM303::vector<int>){1, 0, 0}));
  }

  void dump_current( Stream * s)
  {
    if (buff_len==0) return;
    int i = buff_index -1;
    if (i < 0) 
    {
      i = IMU_BUFF_SIZE-1;
    }
    dump_buffer_item(s,i);
  }

  
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
  
  Description of heading algorithm:
  Shift and scale the magnetic reading based on calibration data to find
  the North vector. Use the acceleration readings to determine the Up
  vector (gravity is measured as an upward acceleration). The cross
  product of North and Up vectors is East. The vectors East and North
  form a basis for the horizontal plane. The From vector is projected
  into the horizontal plane and the angle between the projected vector
  and horizontal north is returned.
  */
  template <typename T> float heading(LSM303::vector<int16_t> m, LSM303::vector<int16_t> a, LSM303::vector<T> from)
  {
      LSM303::vector<int32_t> temp_m = {m.x, m.y, m.z};
  
      // subtract offset (average of min and max) from magnetometer readings
      temp_m.x -= ((int32_t)pcompass->m_min.x + pcompass->m_max.x) / 2;
      temp_m.y -= ((int32_t)pcompass->m_min.y + pcompass->m_max.y) / 2;
      temp_m.z -= ((int32_t)pcompass->m_min.z + pcompass->m_max.z) / 2;
  
      // compute E and N
      LSM303::vector<float> E;
      LSM303::vector<float> N;
      vector_cross(&temp_m, &a, &E);
      vector_normalize(&E);
      vector_cross(&a, &E, &N);
      vector_normalize(&N);
  
      // compute heading
      float heading = atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / M_PI;
      if (heading < 0) heading += 360;
      return heading;
  }
  
  /*
  void loop()
  {
    pcompass->read();
    
    unsigned long tnow = millis();
    
    dt = (float)(tnow - t) / 1000.0;
    
    t = tnow;
    
    float a = (pcompass->a.x >> 4);
    a += 10;
    
    //a_avg += a;
    
    // Serial.print(pcompass->a.x >> 4);
    // Serial.println(d);
    
    v += a * dt * 9.8 / 2000.0;  // ms-1
    
    d += v * dt / 2.0;
    
    samples++;

  }
*/

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
};


#endif
