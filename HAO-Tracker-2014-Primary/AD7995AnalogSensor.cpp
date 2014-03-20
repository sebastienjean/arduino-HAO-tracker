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

#include "AD7995.h"
#include "AD7995AnalogSensor.h"

AD7995AnalogSensor::AD7995AnalogSensor(AD7995 *ad7995, int channel)
{
  this->ad7995 = ad7995;
  this-> channel = channel;
}

int
AD7995AnalogSensor::read(void)
{
  return this->ad7995->read(this->channel);
}
