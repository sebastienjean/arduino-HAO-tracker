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

#include <Led.h>

Led::Led(int pin)
{
  this->pin = pin;
  pinMode(this->pin, OUTPUT);
}

void
Led::on()
{
  digitalWrite(this->pin, HIGH);
}

void
Led::off()
{
  digitalWrite(this->pin, LOW);
}

void
Led::showStatus(boolean status)
{
  digitalWrite(this->pin, status);
}

void
Led::quicklyMakeBlinkSeveralTimes(int times)
{
  for (int i = 0; i < times; i++)
    {
      this->on();
      delay(100);
      this->off();
      delay(100);
    }
}
