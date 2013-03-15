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
#ifndef DEFS_h
#define DEF_h

#include <GPS3D.h>
#include <DS1302_RTC.h>
#include <AnalogSensors.h>
#include <Counters.h>

#define SERIAL_DEBUG Serial

#define SERIAL_NMEA_GPS_BAUDRATE 4800

#define SERIAL_NMEA_GPS_READING_MILLIS_TIMEOUT 2000

#define SERIAL_NMEA_GPS_READING_CHARS_TIMEOUT 2000

#define LOG_FILE_PATH "data.txt"

#define HAO_NAME "POKEBALL"

#define FRAME_COUNTER_BASE_ADDRESS 0x0000
#define RESET_COUNTER_BASE_ADDRESS 0x0002

#endif


