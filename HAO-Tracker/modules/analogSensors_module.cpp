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
#include <analogSensors_module.h>

/**
 * Inits sensors.
 */
void initSensors(void)
{
}

/**
 * Reads a given sensor value.
 *
 * @param sensorNumber the number of the sensor to read
 * @return value of sensor <tt>sensorNumber</tt> if it exists, -1 else
 *
 */
int readSensor(int sensorNumber)
{
  switch (sensorNumber)
  {
    case 1: return analogRead(ABSOLUTE_PRESSURE_ANALOG_SENSOR);

    case 2: return analogRead(DIFFERENTIAL_PRESSURE_ANALOG_SENSOR);

    case 3: return analogRead(INTERNAL_TEMPERATURE_ANALOG_SENSOR);

    case 4: return analogRead(EXTERNAL_TEMPERATURE_ANALOG_SENSOR);

    case 5: return analogRead(BATTERY_VOLTAGE_ANALOG_SENSOR);

    default: return -1;
  }
}
