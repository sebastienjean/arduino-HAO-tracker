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

/**
 * Motorized camera running status counter base address
 */
#define MOTORIZED_CAMERA_RUNNING_STATUS_COUNTER_BASE_ADDRESS 0x000A

/**
 * Ground camera running status counter base address
 */
#define GROUND_CAMERA_RUNNING_STATUS_COUNTER_BASE_ADDRESS 0x000C

/**
 * Sky camera running status counter base address
 */
#define SKY_CAMERA_RUNNING_STATUS_COUNTER_BASE_ADDRESS 0x000E

/**
 * Motorized camera mode counter base address
 */
#define MOTORIZED_CAMERA_MODE_COUNTER_BASE_ADDRESS 0x0010

/**
 * Ground camera mode counter base address
 */
#define GROUND_CAMERA_MODE_COUNTER_BASE_ADDRESS 0x0012

/**
 * Sky camera mode counter base address
 */
#define SKY_CAMERA_MODE_COUNTER_BASE_ADDRESS 0x0014

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

/**
 * First flight phase, before taking off
 */
#define BEFORE_TAKING_OFF_FLIGHT_PHASE 0

/**
 * Second flight phase, ascending, below 5000 meters
 */
#define ASCENDING_BELOW_5000M_FLIGHT_PHASE 1

/**
 * Third flight phase, ascending, above 5000 meters and below 20000 meters
 */
#define ASCENDING_BETWEEN_5000M_AND_20000M_FLIGHT_PHASE 2

/**
 * Fourth flight phase, ascending, above 20000 meters and before burst
 */
#define BEFORE_BURST_FLIGHT_PHASE 3

/**
 * Fifth flight phase, descending, above 5000 meters
 */
#define DESCENDING_ABOVE_5000M_FLIGHT_PHASE 4

/**
 * Sixth flight phase, before landing, below 5000 meters
 */
#define BEFORE_LANDING_FLIGHT_PHASE 5

/**
 * Seventh flight phase, after landing
 */
#define AFTER_LANDING_FLIGHT_PHASE 6

//-----------------------------------------------------
// Flight phase altitude triggering related definitions
//-----------------------------------------------------
/**
 * Fifth flight phase, descending, above 5000 meters
 */
#define HAO_FALLING_TRIGGER 3000
#define FLIGHT_PHASE_1_TO_2_ALTITUDE_TRIGGER 5000
#define FLIGHT_PHASE_2_TO_3_ALTITUDE_TRIGGER 20000
#define FLIGHT_PHASE_4_TO_5_ALTITUDE_TRIGGER 5000
#define STILLNESS_DURATION_IN_SECONDS_LIMIT 100

//-----------------------------------------------------
// Flight phase duration triggering related definitions
//-----------------------------------------------------

#define FLIGHT_PHASE_1_MAX_DURATION 40
#define FLIGHT_PHASE_2_MAX_DURATION 30
#define FLIGHT_PHASE_3_MAX_DURATION 30
#define FLIGHT_PHASE_4_MAX_DURATION 30
#define FLIGHT_PHASE_5_MAX_DURATION 30

// ---------------
// Cameras' output
// ---------------
#define MOTORIZED_CAMERA_PWM_PIN 22
#define GROUND_CAMERA_PWM_PIN 3
#define SKY_CAMERA_PWM_PIN 26

// ----------------------------------------------------
// Delay of the number of frame to reset camera's video
// ----------------------------------------------------
#define DELAY_FRAGMENTATION 10

#endif

