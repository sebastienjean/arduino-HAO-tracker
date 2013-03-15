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

#ifndef LED_h
#define LED_h

/**
 * This class allows to handle a LED, attached to a given IO pin.
 */
class Led
{
private:

  /**
   * IO pin to which the LED is attached.
   */
  int pin;

public:

  /**
   * Creates a LED attached to a given IO pin.
   *
   * @param pin IO pin to which the LED is attached
   */
  Led(int pin);

  /**
   * Turns the LED on.
   */
  void
  on(void);

  /**
   * Turns the LED off.
   */
  void
  off(void);

  /**
   * Displays status (OK/KO = on/off) using the LED
   *
   * @param status the status to display (OK/KO)
   */
  void
  showStatus(boolean status);

  /**
   * Blinks the LED at 5Hz a given number of times.
   *
   * @param times the number of times the LED should blink
   */
  void
  quicklyMakeBlinkSeveralTimes(int times);

};

#endif
