/*
 Copyright (C) 2012 Sebastien Jean <baz dot jean at gmail dot com>

 This program is free software: you can redistribute it and/or modify
 it under the terms of the version 3 GNU General Public License as
 published by the Free Software Foundation.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <Arduino.h>
#include <pins.h>
#include <defs.h>
#include <sensors_module.h>


// absolute pressure sensor value
int absolutePressureSensorValue;

// differential pressure sensor value
int differentialPressureSensorValue;

// internal temperature sensor value
int internalTemperatureSensorValue;

// external temperature sensor value
int externalTemperatureSensorValue;

// battery voltage sensor value
int batteryVoltageSensorValue;

/**
 * Inits sensors.
 */
void initSensors(void)
{
  absolutePressureSensorValue = 0;
  differentialPressureSensorValue = 0;
  internalTemperatureSensorValue = 0;
  externalTemperatureSensorValue = 0;
  batteryVoltageSensorValue = 0;
}

/**
 * Reads all sensors values and stores them in appropriate variables.
 *
 */
void readSensors()
{
  // absolute pressure processing
  absolutePressureSensorValue = analogRead(ABSOLUTE_PRESSURE_ANALOG_SENSOR);

  // differential pressure processing
  differentialPressureSensorValue = analogRead(DIFFERENTIAL_PRESSURE_ANALOG_SENSOR);

  // internal temperature processing
  internalTemperatureSensorValue = analogRead(INTERNAL_TEMPERATURE_ANALOG_SENSOR);

  // external temperature processing
  externalTemperatureSensorValue = analogRead(EXTERNAL_TEMPERATURE_ANALOG_SENSOR);

  // battery voltage processing
  batteryVoltageSensorValue = analogRead(BATTERY_VOLTAGE_ANALOG_SENSOR);
}
