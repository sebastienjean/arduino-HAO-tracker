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
#include <pins_arduino.h>
#include <pins.h>
#include <AnalogSensors.h>

  AnalogSensors::AnalogSensors(AnalogSensor* analogSensors[], int analogSensorsAmount)
  {
    for (int i=0;(i<analogSensorsAmount)&&(i<NUM_ANALOG_INPUTS);i++)
    {
        this->analogSensors[i] = analogSensors[i];
    }
    if (analogSensorsAmount > NUM_ANALOG_INPUTS)
    {
        analogSensorsAmount = NUM_ANALOG_INPUTS;
    }
    this->analogSensorsAmount = analogSensorsAmount;
  }

  /**
   * Reads a given sensor value.
   *
   * @param sensorNumber the number of the sensor to read
   * @return value of sensor <tt>sensorNumber</tt> if it exists, -1 else
   *
   */
  int AnalogSensors::read(int sensorNumber)
  {
    switch (sensorNumber)
      {
        case 1: return analogRead(ABSOLUTE_PRESSURE_ANALOG_SENSOR_CHANNEL);

        case 2: return analogRead(DIFFERENTIAL_PRESSURE_ANALOG_SENSOR_CHANNEL);

        case 3: return analogRead(INTERNAL_TEMPERATURE_ANALOG_SENSOR_CHANNEL);

        case 4: return analogRead(EXTERNAL_TEMPERATURE_ANALOG_SENSOR_CHANNEL);

        case 5: return analogRead(BATTERY_VOLTAGE_ANALOG_SENSOR_CHANNEL);

        default: return -1;
      }
  }

  /**
   * Returns the amount of analog sensors
   *
   * @return the amount of analog sensors
   *
   */
  int AnalogSensors::getAmount()
  {
    return this->analogSensorsAmount;
  }
