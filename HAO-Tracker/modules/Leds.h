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

#ifndef LEDS_h
#define LEDS_h

#include <Led.h>
#define MAX_LEDS 10

class Leds
{
private:

  Led* leds[MAX_LEDS];
  int ledsAmount;

public:

  Leds(Led* leds[], int ledsAmount);

  /**
   * Turns alls LEDs on.
   *
   */
  void on(void);

  /**
   * Turns alls LEDs off.
   *
   */
  void off(void);

  /**
   * Blinks all LEDs at 5Hz a given number of times.
   * @param times the number of times LEDs should blink
   */
  void quicklyMakeBlinkSeveralTimes(int times);

};

#endif
