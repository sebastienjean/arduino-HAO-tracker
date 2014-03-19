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

//---------------------------
// Serial related definitions
//---------------------------

/**
 * Serial used for debug
 */
#define SERIAL_DEBUG Serial

/**
 * Debug serial baudrate
 */
#define SERIAL_DEBUG_BAUDRATE 600

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

//----------------------------
// Logging related definitions
//----------------------------

/**
 * Log file path
 */
#define LOG_FILE_PATH "2014data.txt"

//---------------------------
// Naming related definitions
//---------------------------

/**
 * HAO name
 */
#define HAO_NAME "STRATERRESTRE"

//-----------------------------
// Counters related definitions
//-----------------------------

/**
 * Frame counter EEPROM base address
 */
#define FRAME_COUNTER_BASE_ADDRESS 0x0000

/**
 * Reset counter EEPROM base address
 */
#define RESET_COUNTER_BASE_ADDRESS 0x0002

/**
 * Current flight phase counter EEPROM base address
 */
#define CURRENT_FLIGHT_PHASE_COUNTER_BASE_ADDRESS 0x0004

/**
 * Flight current phase duration counter base address
 */
#define FLIGHT_PHASE_DURATION_COUNTER_BASE_ADDRESS 0x0006

/**
 * HAO stillness duration (in seconds) counter base address
 */
#define STILLNESS_DURATION_IN_SECONDS_COUNTER_BASE_ADDRESS 0x0008


//------------------------------------------
// Flight phase duration related definitions
//------------------------------------------

/**
 * Flight phase number 0 pause duration (in milliseconds)
 */
#define FLIGHT_PHASE_0_PAUSE_MILLIS 15000

/**
 * Flight phase number 1 pause duration (in milliseconds)
 */
#define FLIGHT_PHASE_1_PAUSE_MILLIS 1

/**
 * Flight phase number 2 pause duration (in milliseconds)
 */
#define FLIGHT_PHASE_2_PAUSE_MILLIS 1

/**
 * Flight phase number 3 pause duration (in milliseconds)
 */
#define FLIGHT_PHASE_3_PAUSE_MILLIS 1

/**
 * Flight phase number 4 pause duration (in milliseconds)
 */
#define FLIGHT_PHASE_4_PAUSE_MILLIS 1

/**
 * Flight phase number 5 pause duration (in milliseconds)
 */
#define FLIGHT_PHASE_5_PAUSE_MILLIS 15000

//-------------------------------------------
// Flight phase numbering related definitions
//-------------------------------------------

/**
 * First flight phase, before taking off
 */
#define BEFORE_TAKING_OFF_FLIGHT_PHASE 0

/**
 * Second flight phase, ascending, below Lower altitude limit in meters
 */
#define ASCENDING_BELOW_LOWER_LIMIT_FLIGHT_PHASE 1

/**
 * Third flight phase, ascending, above lower and below upper altitude limits in meters
 */
#define ASCENDING_BETWEEN_LOWER_AND_UPPER_LIMIT_FLIGHT_PHASE 2

/**
 * Fourth flight phase, ascending, above 20000 meters and before burst
 */
#define BEFORE_BURST_FLIGHT_PHASE 3

/**
 * Fifth flight phase, descending, below lower altitude limit in meters
 */
#define DESCENDING_BELOW_LOWER_LIMIT_FLIGHT_PHASE 4

/**
 * Sixth flight phase,  after landing
 */
#define AFTER_LANDING_FLIGHT_PHASE 5

//-----------------------------------------------------
// Flight phase altitude triggering related definitions
//-----------------------------------------------------
#define FLIGHT_PHASE_1_TO_2_ALTITUDE_TRIGGER 3000
#define FLIGHT_PHASE_2_TO_3_ALTITUDE_TRIGGER 20000
#define FLIGHT_PHASE_3_TO_4_ALTITUDE_TRIGGER 5000
#define DELTA_ALTITUDE_IN_METERS_CONSIDERED_AS_STILLNESS 150
#define STILLNESS_DURATION_IN_LOOPS_LIMIT 10


//-----------------------------------------------------
// Flight phase duration triggering related definitions
//-----------------------------------------------------
#define FLIGHT_PHASE_1_MAX_SECONDS_DURATION 1200 // 20 minutes to takeoff and reach 5km
#define FLIGHT_PHASE_2_MAX_SECONDS_DURATION 3600 // 60 minutes to fly from 5k to 20k
// #define FLIGHT_PHASE_3_MAX_SECONDS_DURATION 1200 // 4000 //3600
// #define FLIGHT_PHASE_4_MAX_SECONDS_DURATION 1200 // 3600 //4000

#endif

