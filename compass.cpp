#include <Arduino.h>
#include <avr/EEPROM.h>

#include "Robot.h"

void dump_compass_config(LSM303 * compass, Stream * sout)
{
  sout->print("compass.min(x,y,z) = ");
  sout->print(compass->m_min.x);
  sout->print(",");
  sout->print(compass->m_min.y);
  sout->print(",");
  sout->println(compass->m_min.z);
  sout->print("compass.max(x,y,z) = ");
  sout->print(compass->m_max.x);
  sout->print(",");
  sout->print(compass->m_max.y);
  sout->print(",");
  sout->println(compass->m_max.z);
}

void compass_calibrate(Robot * bot, LSM303 * compass, RobotIMU * imu, boolean auto_rotate)
{
	bot->sout->println("Rotate compass about all axis");

	// save current min/max Z
	int minz = compass->m_min.z;
	int maxz = compass->m_max.z;

	if (auto_rotate)
	{
		bot->spin_pwm(CW,SPEED_SLOW);
	}

	imu->start();

	imu->log_max = true;

	for(unsigned int t=millis(); millis()-t < 10000; )
	{
		imu->loop();
	}

	if (auto_rotate)
	{
		bot->stop();
		// if auto rotate, restore current min/max Z as we are just spinning about Z axis...
		imu->mmin.z = minz;
		imu->mmax.z = maxz;
	}

	imu->stop();

	compass->m_min = imu->mmin;
	compass->m_max = imu->mmax;

	dump_compass_config(compass, bot->sout);
}

void read_compass_config_from_eeprom(LSM303 * compass, int a)
{
	// int a = EEPROM_COMPASS;
	eeprom_read_block((void*)&compass->m_min, (const void*)a, sizeof(LSM303::vector<int16_t>));
	a += sizeof(LSM303::vector<int16_t>);
	eeprom_read_block((void*)&compass->m_max, (const void*)a, sizeof(LSM303::vector<int16_t>));
}

void write_compass_config_to_eeprom(LSM303 * compass, int a)
{
	// int a = EEPROM_COMPASS;
	eeprom_write_block((const void*)&compass->m_min, (void*)a, sizeof(LSM303::vector<int16_t>));
	a += sizeof(LSM303::vector<int16_t>);
	eeprom_write_block((const void*)&compass->m_max, (void*)a, sizeof(LSM303::vector<int16_t>));
}


