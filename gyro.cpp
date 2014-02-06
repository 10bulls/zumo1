#include <Arduino.h>
#include <avr/EEPROM.h>

#include "Robot.h"

void dump_gzero(RobotIMU * imu, Stream * sout)
{
  sout->print("Gyro zero(x,y,z) = ");
  sout->print(imu->gzero.x);
  sout->print(",");
  sout->print(imu->gzero.y);
  sout->print(",");
  sout->println(imu->gzero.z);
}

void calibrate_gyro(RobotIMU * imu, Stream * sout)
{
  sout->println("Starting gyro calibrate...");

  /*
  StartIMU();
  
  while( !imu.buffer_overflow )
  {
    delay(100);    
  }
  
  StopIMU();
  */

	imu->start();
	for(unsigned int t=millis(); !imu->buffer_overflow && millis()-t < 10000; )
	{
		imu->loop();
	}

	imu->stop();
  
  imu->gzero = imu->get_g_avg();
  
  dump_gzero(imu,sout);
  
  // write_gyro_zero_to_eeprom();
}


void write_gyro_zero_to_eeprom(RobotIMU * imu, int a)
{
  // int a = EEPROM_GYRO;
  eeprom_write_block((const void*)&imu->gzero, (void*)a, sizeof(imu->gzero));
}

void read_gyro_zero_from_eeprom(RobotIMU * imu, int a)
{
  // int a = EEPROM_GYRO;
  eeprom_read_block((void*)&imu->gzero, (const void*)a, sizeof(imu->gzero));
}
