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

#ifndef AD7995_ANALOG_SENSOR_h
#define AD7995_ANALOG_SENSOR_h

#include "AnalogSensor.h"
#include "AD7995.h"
/**
 * This class allows to handle an analog sensor, attached to a channel of an AD7995 I2C ADC
 */
class AD7995AnalogSensor : public AnalogSensor
{
private:

  /**
   * AD7995 to which the sensor is attached.
   */
  AD7995 *ad7995;

  /**
   * Analog channel to which the sensor is attached.
   */
  int channel;

public:

  /**
   * Creates a sensor attached to a given channel.
   *
   * @param add7995 AD7995 to which the sensor is attached
   * @param channel analog channel to which the sensor is attached
   */
  AD7995AnalogSensor(AD7995 *ad7995, int channel);

  /**
   * Reads analog sensor value.
   *
   * @return analog sensor value
   */
  int
  read(void);

};

#endif

