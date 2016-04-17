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
#include <rotor.h>

Rotor::Rotor(uint8_t pwmPin)
{
  this->pwmPin = pwmPin;
  pinMode(this->pwmPin, OUTPUT);
}


void
Rotor::signal(uint16_t pulseWidthMicros)
{
  for (int i = 0; i < PWM_PERIODS; i++)
    {
      digitalWrite(this->pwmPin, HIGH);
      delayMicroseconds(pulseWidthMicros);
      digitalWrite(this->pwmPin, LOW);
      delay(PERIOD_MILLIS);
    }
}

void
Rotor::goMiddle()
{
  signal(MIDDLE_HIGH_MICROS);
}

void
Rotor::goTop()
{
  signal(TOP_HIGH_MICROS);
}

void
Rotor::goBottom()
{
  signal(BOTTOM_HIGH_MICROS);
}


