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

#include "AnalogSensors.h"
#include "AnalogSensor.h"

AnalogSensors::AnalogSensors(AnalogChannelAnalogSensor* analogSensors[],
    int analogSensorsAmount)
{
  for (int i = 0; (i < analogSensorsAmount) && (i < NUM_ANALOG_INPUTS); i++)
    {
      this->analogSensors[i] = analogSensors[i];
    }
  if (analogSensorsAmount > NUM_ANALOG_INPUTS)
    {
      analogSensorsAmount = NUM_ANALOG_INPUTS;
    }
  this->analogSensorsAmount = analogSensorsAmount;
}

int
AnalogSensors::read(int sensorNumber)
{
  if ((sensorNumber < 0) || (sensorNumber > getAmount()))
    {
      return -1;
    }
  return analogSensors[sensorNumber - 1]->read();
}

int
AnalogSensors::getAmount()
{
  return this->analogSensorsAmount;
}
