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
#define DEFS_h

/**
 * Serial used for debug
 */
#define SERIAL_DEBUG Serial

/**
 * GPS serial baudrate
 */
#define SERIAL_DEBUG_BAUDRATE 9600

/**
 * GPS serial baudrate
 */
#define SERIAL_NMEA_GPS_BAUDRATE 4800

/**
 * GPS reading timeout, in milliseconds
 */
#define SERIAL_NMEA_GPS_READING_MILLIS_TIMEOUT 2000

/**
 * GPS reading timeout, in characters
 */
#define SERIAL_NMEA_GPS_READING_CHARS_TIMEOUT 2000

/**
 * Log file path
 */
#define LOG_FILE_PATH "data.txt"

/**
 * HAO name
 */
#define HAO_NAME "POKEBALL"

/**
 * Frame counter EEPROM base address
 */
#define FRAME_COUNTER_BASE_ADDRESS 0x0000

/**
 * Reset counter EEPROM base address
 */
#define RESET_COUNTER_BASE_ADDRESS 0x0002

/**
 * Phase counter EEPROM base address
 */
#define PHASE_COUNTER_BASE_ADDRESS 0x0004

/**
 * Time phase 1 counter base address
 */
#define TIME_PHASE_1_COUNTER_BASE_ADDRESS 0x0006

/**
 * Time phase 2 counter base address
 */
#define TIME_PHASE_2_COUNTER_BASE_ADDRESS 0x0008

/**
 * Time phase 3 counter base address
 */
#define TIME_PHASE_3_COUNTER_BASE_ADDRESS 0x0010

/**
 * Time phase 4 counter base address
 */
#define TIME_PHASE_4_COUNTER_BASE_ADDRESS 0x0012

/**
 * Time phase 5 counter base address
 */
#define TIME_PHASE_5_COUNTER_BASE_ADDRESS 0x0014

/**
 * Time of loop 1 pause
 */
#define TIME_PAUSE_1 500

/**
 * Time of loop 2 pause
 */
#define TIME_PAUSE_2 500

/**
 * Time of loop 3 pause
 */
#define TIME_PAUSE_3 500

/**
 * Time of loop 4 pause
 */
#define TIME_PAUSE_4 500

/**
 * Time of loop 5 pause
 */
#define TIME_PAUSE_5 500

/**
 * Number of phase value
 */
#define PHASE_1 0
#define PHASE_2 1
#define PHASE_3 2
#define PHASE_4 3
#define PHASE_5 4

/**
 * Limit of each phase in meters
 */
#define LIMIT_1 5000
#define LIMIT_2 25000


#endif

