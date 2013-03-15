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
#include <Counter.h>


Counter::Counter(int baseAddress)
{
  this->baseAddress = baseAddress;
}

/**
 * Reads counter value.
 *
 * @return counter value
 *
 */
int Counter::read(void)
{
  // TODO implement this
  // see http://arduino.cc/en/Reference/WordCast
  return 0;
}

/**
 * Sets counter value.
 *
 * @param value counter value
 *
 */
void Counter::set(int value)
{
  // TODO implement this
  // see http://arduino.cc/en/Reference/LowByte and http://arduino.cc/en/Reference/HighByte
}

/**
 * Increments counter value.
 *
 * @param value the increment value
 *
 */
void Counter::increment(int value)
{
  // TODO implement this
}
