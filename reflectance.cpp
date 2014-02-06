#include <Arduino.h>
#include <avr/EEPROM.h>

#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include "Robot.h"

void dump_reflectance_calibration(ZumoReflectanceSensorArray * sensor, int num_sensors, Stream * sout)
{
  // print the calibration minimum values measured when emitters were on
  sout->print("min: ");
  for (byte i = 0; i < num_sensors; i++)
  {
    sout->print(sensor->calibratedMinimumOn[i]);
    sout->print(' ');
  }
  sout->println();
  
  // print the calibration maximum values measured when emitters were on
  sout->print("max: ");
  for (byte i = 0; i < num_sensors; i++)
  {
    sout->print(sensor->calibratedMaximumOn[i]);
    sout->print(' ');
  }
  sout->println();
}

void calibrate_reflectance_array(ZumoReflectanceSensorArray * sensor, int num_sensors, Stream * sout)
{
  sout->println("Calibrating array...");
  
  sensor->resetCalibration();
  
  unsigned long startTime = millis();
  while(millis() - startTime < 10000)   // make the calibration take 10 seconds
  {
    sensor->calibrate();
  }
  sout->println("done");
  
  dump_reflectance_calibration(sensor,num_sensors,sout);
  
//  save_reflectance_to_eeprom(0);
}

void save_reflectance_to_eeprom(ZumoReflectanceSensorArray * sensor, int num_sensors, int address)
{
//  int address = EEPROM_REFLECT;
  int n = num_sensors * sizeof(unsigned int);
  
  eeprom_write_block((const void*)sensor->calibratedMinimumOn, (void*)address, n);
  eeprom_write_block((const void*)sensor->calibratedMaximumOn, (void*)(address+n), n);
}

void read_reflectance_from_eeprom(ZumoReflectanceSensorArray * sensor, int num_sensors, int address)
{
//  int address = EEPROM_REFLECT;
	int n = num_sensors * sizeof(unsigned int);

	if (!sensor->calibratedMinimumOn)
		sensor->calibratedMinimumOn = (unsigned int*)malloc(n);

	if (!sensor->calibratedMaximumOn)
		sensor->calibratedMaximumOn = (unsigned int*)malloc(n);
  
	eeprom_read_block((void*)sensor->calibratedMinimumOn, (const void*)address, n);
	eeprom_read_block((void*)sensor->calibratedMaximumOn, (const void*)(address+n), n);
}

