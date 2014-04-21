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

#ifndef ROTOR_h
#define ROTOR_h

#include <inttypes.h>

/**
 * Number of periods (20ms) of PWM signal.
 */
#define PWM_PERIODS 25

/**
 * Width, in microseconds, of MIDDLE pulse.
 */
#define MIDDLE_HIGH_MICROS 1500

/**
 * Width, in microseconds, of TOP pulse.
 */
#define TOP_HIGH_MICROS 600

/**
 * Width, in microseconds, of BOTTOM pulse.
 */
#define BOTTOM_HIGH_MICROS 2600

/**
 * PWM period, in milliseconds.
 */
#define PERIOD_MILLIS 20

/**
 * A Rotor, attached to a given digital output.
 */
class Rotor
{
private:
  // per object data

  /**
   * Pin used as output for PWM signal
   */
  uint8_t pwmPin;

private:
  // private methods

  /**
   * Sending a custom signal, corresponding to a given pulse width repeated a given time.
   * @param pulseWidthMicros the pulse width in microseconds
   */
  void
  signal(uint16_t pulseWidthMicros);

public:
  // public methods
  /**
   * Creates a new rotor instance, using digital IO <tt>pwmPin</tt> for PWM output.
   * @param pwmPin digital IO where rotor is attached
   */
  Rotor(uint8_t pwmPin);

  /**
   *
   */
  void
  goMiddle();

  /**
   *
   */
  void
  goTop();

  /**
   *
   */
  void
  goBottom();
};
#endif
