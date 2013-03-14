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

class DS1302_RTC : public DS1302
{
  public:

  DS1302_RTC(int ce_pin, int io_pin, int sclk_pin);

  /**
   * Retrieve RTC time, as a string.
   *
   * @param timeString an external buffer of at least 7 bytes used to get time as "hhmmss"
   */
  void getRtcTimeString(char *timeString);
};

#endif



