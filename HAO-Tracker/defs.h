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

//----------------------------
// Logging related definitions
//----------------------------

/**
 * Log file path
 */
#define LOG_FILE_PATH "data.txt"

//---------------------------
// Naming related definitions
//---------------------------

/**
 * HAO name
 */
#define HAO_NAME "POKEBALL"

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
 * Flight phase number 0 duration counter base address
 */
#define FLIGHT_PHASE_0_DURATION_COUNTER_BASE_ADDRESS 0x0006

/**
 * Flight phase number 1 duration counter base address
 */
#define FLIGHT_PHASE_1_DURATION_COUNTER_BASE_ADDRESS 0x0008

/**
 * Flight phase number 2 duration counter base address
 */
#define FLIGHT_PHASE_2_DURATION_COUNTER_BASE_ADDRESS 0x000A

/**
 * Flight phase number 3 duration counter base address
 */
#define FLIGHT_PHASE_3_DURATION_COUNTER_BASE_ADDRESS 0x000C

/**
 * Flight phase number 4 duration counter base address
 */
#define FLIGHT_PHASE_4_DURATION_COUNTER_BASE_ADDRESS 0x000E

/**
 * Flight phase number 5 duration counter base address
 */
#define FLIGHT_PHASE_5_DURATION_COUNTER_BASE_ADDRESS 0x0010

/**
 * Flight phase number 6 duration counter base address
 */
#define FLIGHT_PHASE_6_DURATION_COUNTER_BASE_ADDRESS 0x0012

//------------------------------------------
// Flight phase duration related definitions
//------------------------------------------

/**
 * Flight phase number 0 pause duration (in ms)
 */
#define FLIGHT_PHASE_0_PAUSE_DURATION 500

/**
 * Flight phase number 1 pause duration (in ms)
 */
#define FLIGHT_PHASE_1_PAUSE_DURATION 500

/**
 * Flight phase number 2 pause duration (in ms)
 */
#define FLIGHT_PHASE_2_PAUSE_DURATION 500

/**
 * Flight phase number 3 pause duration (in ms)
 */
#define FLIGHT_PHASE_3_PAUSE_DURATION 500

/**
 * Flight phase number 4 pause duration (in ms)
 */
#define FLIGHT_PHASE_4_PAUSE_DURATION 500

/**
 * Flight phase number 5 pause duration (in ms)
 */
#define FLIGHT_PHASE_5_PAUSE_DURATION 500

/**
 * Flight phase number 6 pause duration (in ms)
 */
#define FLIGHT_PHASE_6_PAUSE_DURATION 500

//-------------------------------------------
// Flight phase numbering related definitions
//-------------------------------------------

#define BEFORE_TAKING_OFF_FLIGHT_PHASE 0
#define ASCENDING_BELOW_5000M_FLIGHT_PHASE 1
#define ASCENDING_BETWEEN_5000M_AND_20000M_FLIGHT_PHASE 2
#define BEFORE_BURST_FLIGHT_PHASE 3
#define DESCENDING_ABOVE_5000M_FLIGHT_PHASE 4
#define BEFORE_LANDING_FLIGHT_PHASE 5
#define AFTER_LANDING_FLIGHT_PHASE 6

//-----------------------------------------------------
// Flight phase altitude triggering related definitions
//-----------------------------------------------------

#define HAO_FALLING_TRIGGER 3000
#define FLIGHT_PHASE_1_TO_2_ALTITUDE_TRIGGER 5000
#define FLIGHT_PHASE_2_TO_3_ALTITUDE_TRIGGER 20000
#define FLIGHT_PHASE_4_TO_5_ALTITUDE_TRIGGER 5000

//-----------------------------------------------------
// Flight phase duration triggering related definitions
//-----------------------------------------------------

#endif

