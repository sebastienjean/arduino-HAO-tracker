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

#include "AnalogSensor.h"

/**
 * This class wraps a set of analog sensors. The number of sensors that can be grouped in an instance of this
 * class depends on the number of analog channels available on the platform
 * (see <tt>NUM_ANALOG_INPUT</tt> in <tt>pins_arduino.h</tt>).
 */
class AnalogSensors
{
private:

  /**
   * Sensors (pointers) contained in this set
   */
  AnalogSensor* analogSensors[NUM_ANALOG_INPUTS];

  /**
   * Number of sensors in this set
   */
  int analogSensorsAmount;

public:

  /**
   * Creates a new analog sensors set containing existing analog sensors
   *
   * @param analogSensors an array containing sensors to add to this set
   * @param analogSensorsAmount the number of sensors to add from <tt>analogSensors</tt>
   */
  AnalogSensors(AnalogSensor* analogSensors[], int analogSensorsAmount);

  /**
   * Reads a given sensor value.
   *
   * @param sensorNumber the number of the sensor to read
   * @return value of sensor <tt>sensorNumber</tt> if it exists, -1 else
   */
  int
  read(int sensorNumber);

  /**
   * Returns the amount of analog sensors in this set
   *
   * @return the amount of analog sensors in this set
   */
  int
  getAmount(void);
};

#endif

