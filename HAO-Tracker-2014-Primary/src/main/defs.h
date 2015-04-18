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
// Frames related definitions
//---------------------------

#define NUMBER_OF_ANALOG_SENSORS_IN_CUSTOM_FRAME 14

#define NUMBER_OF_ANALOG_SENSORS_IN_KIWI_FRAME 8

#define NUMBER_OF_COUNTERS_IN_CUSTOM_FRAME 2

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
#define HAO_NAME "FIREFLY26"

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
 * State of configurable address bit 0 of onboard first MCP3428
 */
#define MCP3428_0_ADDRESS_BIT0 LOW

/**
 * State of configurable address bit 1 of onboard first MCP3428
 */
#define MCP3428_0_ADDRESS_BIT1 LOW

/**
 * State of configurable address bit 0 of onboard second MCP3428
 */
#define MCP3428_1_ADDRESS_BIT0 HIGH

/**
 * State of configurable address bit 1 of onboard second MCP3428
 */
#define MCP3428_1_ADDRESS_BIT1 HIGH

#endif

