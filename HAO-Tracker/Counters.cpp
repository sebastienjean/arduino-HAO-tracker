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

#include "Counters.h"
#include "Counter.h"

Counters::Counters(Counter* counters[], int countersAmount)
{
  for (int i = 0; (i < countersAmount) && (i < MAX_COUNTERS); i++)
    {
      this->counters[i] = counters[i];
    }
  if (countersAmount > MAX_COUNTERS)
    {
      countersAmount = MAX_COUNTERS;
    }
  this->countersAmount = countersAmount;
}

int
Counters::read(int counterNumber)
{
  if ((counterNumber < 0) || (counterNumber > getAmount()))
    {
      return -1;
    }
  return counters[counterNumber - 1]->read();
}

int
Counters::getAmount()
{
  return this->countersAmount;
}

void
Counters::reset()
{
  for (int i = 0; (i < countersAmount) && (i < MAX_COUNTERS); i++)
      {
        this->counters[i]->set(0);
      }

}

Counter *
Counters::get(int counterNumber)
{
  if ((counterNumber < 1) || (counterNumber > countersAmount))
    {
      return NULL;
    }
  return this->counters[counterNumber-1];
}
