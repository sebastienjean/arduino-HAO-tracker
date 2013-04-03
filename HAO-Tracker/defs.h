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
 * Flight current phase duration counter base address
 */
#define FLIGHT_PHASE_DURATION_COUNTER_BASE_ADDRESS 0x0006

/**
 * Time of unmoving ball counter base address
 */
#define TIME_OF_UNMOVING_BALL_COUNTER_BASE_ADDRESS 0x0008

/**
 * State of the mobile video camera counter base address
 */
#define STATE_OF_MOBILE_VIDEO_CAMERA_COUNTER_BASE_ADDRESS 0x000A

/**
 * State of the ground video camera counter base address
 */
#define STATE_OF_GROUND_VIDEO_CAMERA_COUNTER_BASE_ADDRESS 0x000C

/**
 * State of the sky video camera counter base address
 */
#define STATE_OF_SKY_VIDEO_CAMERA_COUNTER_BASE_ADDRESS 0x000E

/**
 * Mode of mobile camera counter base address
 */
#define MODE_OF_MOBILE_CAMERA_COUNTER_BASE_ADDRESS 0x0010

/**
 * Mode of ground camera counter base address
 */
#define MODE_OF_GROUND_CAMERA_COUNTER_BASE_ADDRESS 0x0010

/**
 * Mode of sky camera counter base address
 */
#define MODE_OF_SKY_CAMERA_COUNTER_BASE_ADDRESS 0x0010

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
#define TIME_LIMIT_OF_UNMOVING_BALL 100

//-----------------------------------------------------
// Flight phase duration triggering related definitions
//-----------------------------------------------------

#define FLIGHT_PHASE_1_MAX_DURATION 65
#define FLIGHT_PHASE_2_MAX_DURATION 75
#define FLIGHT_PHASE_3_MAX_DURATION 30
#define FLIGHT_PHASE_4_MAX_DURATION 35
#define FLIGHT_PHASE_5_MAX_DURATION 45

// ---------------
// Cameras' output
// ---------------
#define CAMERA_MOBILE_PWM 22
#define CAMERA_GROUND_PWM 3
#define CAMERA_SKY_PWM 26

// ----------------------------------------------------
// Delay of the number of frame to reset camera's video
// ----------------------------------------------------
#define DELAY_FRAGMENTATION 10

// -------------------
// State of the camera
// -------------------

#define VIDEO_CAMERA_ON 0
#define VIDEO_CAMERA_OFF 1

#define CAMERA_MODE_VIDEO 0
#define CAMERA_MODE_PHOTO_SINGLE 1
#define CAMERA_MODE_PHOTO_SERIAL 2

#endif

