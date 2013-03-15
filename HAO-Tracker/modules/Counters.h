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

#ifndef COUNTERS_h
#define COUNTERS_h

#include <Arduino.h>
#include <Counter.h>

#define MAX_COUNTERS (E2END/2) +1
class Counters
{
private:

  Counter* counters[MAX_COUNTERS];
  int countersAmount;

public:

  Counters(Counter* counters[], int countersAmount);

  /**
   * Reads a given counter value.
   *
   * @param counterNumber the number of the counter to read
   * @return value of counter <tt>CounterNumber</tt> if it exists, -1 else
   *
   */
  int read(int counterNumber);

  /**
   * Returns the amount of counters
   *
   * @return the amount of counters
   *
   */
  int getAmount(void);
};

#endif


