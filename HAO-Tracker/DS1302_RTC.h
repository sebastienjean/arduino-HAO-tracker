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

#ifndef DS1302_RTC_h
#define DS1302_RTC_h

#include <DS1302.h>

/**
 * This class is a subclass of DS1302 providing a simplified interface for pre-formatted (hhmmss) time retrieval.
 */
class DS1302_RTC : public DS1302
{
public:

  /**
   * Creates a new DS1302 driver, bound to given I/O pins
   * @param ce_pin Chip Enabled Pin
   * @param io_pin IO (data) Pin
   * @param sclk_pin Clock Pin
   */
  DS1302_RTC(int ce_pin, int io_pin, int sclk_pin);

  /**
   * Retrieve RTC time, as a pre-formatted (hhmmss) string .
   *
   * @param timeString an external buffer of at least 7 bytes used to get time as "hhmmss"
   */
  void
  getRtcTimeString(char *timeString);
};

#endif

