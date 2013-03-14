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

#include <DS1302_RTC.h>
#include <DS1302.h>
#include <stdio.h>
#include <string.h>

DS1302_RTC::DS1302_RTC(int ce_pin, int io_pin, int sclk_pin) : DS1302::DS1302(ce_pin, io_pin, sclk_pin)
{
}

/**
 * Returns local time.
 *
 * @return local time as hhmmss
 */
void DS1302_RTC::getRtcTimeString(char * timeString)
{
   strcpy(timeString,"000000");
   Time time = DS1302::time();

   snprintf(timeString, 7, "%02d%02d%02d", time.hr, time.min, time.sec);
}


