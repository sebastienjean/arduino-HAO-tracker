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
#include <pins.h>
#include <defs.h>

#include <rtc_module.h>
#include <DS1302.h>
#include <stdio.h>
#include <string.h>

DS1302 rtc(RTC_CE, RTC_IO, RTC_SCLK);

char timeString[7];
/**
 * Initializes RTC.
 */
void initRTC()
{
  rtc.write_protect(true);
  rtc.halt(false);
  memset(timeString, 0, sizeof(timeString));
}

/**
 * Returns local time.
 *
 * @return local time as hhmmss
 */
void getRtcTime(char * timeString)
{
   strcpy(timeString,"000000");
   Time time = rtc.time();

   snprintf(timeString, 7, "%02d%02d%02d", time.hr, time.min, time.sec);
}


