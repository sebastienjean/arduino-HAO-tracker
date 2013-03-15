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

#ifndef ANALOG_SENSORS_h
#define ANALOG_SENSORS_h

#include <pins_arduino.h>
#include <AnalogSensor.h>

class AnalogSensors
{
private:

  AnalogSensor* analogSensors[NUM_ANALOG_INPUTS];
  int analogSensorsAmount;

public:

  AnalogSensors(AnalogSensor* analogSensors[], int analogSensorsAmount);

  /**
   * Reads a given sensor value.
   *
   * @param sensorNumber the number of the sensor to read
   * @return value of sensor <tt>sensorNumber</tt> if it exists, -1 else
   *
   */
  int read(int sensorNumber);

  /**
   * Returns the amount of analog sensors
   *
   * @return the amount of analog sensors
   *
   */
  int getAmount(void);
};

#endif


