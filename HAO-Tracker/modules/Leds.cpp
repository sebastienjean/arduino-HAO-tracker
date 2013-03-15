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

#include <Leds.h>

Leds::Leds(Led* leds[], int ledsAmount)
{
  for (int i = 0; (i < ledsAmount) && (i < MAX_LEDS); i++)
    {
      this->leds[i] = leds[i];
    }
  if (ledsAmount > MAX_LEDS)
    {
      ledsAmount = MAX_LEDS;
    }
  this->ledsAmount = ledsAmount;
}

void
Leds::on()
{
  for (int i = 0; i < this->ledsAmount; i++)
    {
      (this->leds[i])->on();
    }
}

void
Leds::off()
{
  for (int i = 0; i < this->ledsAmount; i++)
    {
      (this->leds[i])->off();
    }
}

void
Leds::quicklyMakeBlinkSeveralTimes(int times)
{
  for (int i = 0; i < times; i++)
    {
      this->on();
      delay(100);
      this->off();
      delay(100);
    }
}

