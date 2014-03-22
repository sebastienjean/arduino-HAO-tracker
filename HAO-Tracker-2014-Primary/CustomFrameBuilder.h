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

#ifndef CUSTOM_FRAME_BUILDER_h
#define CUSTOM_FRAME_BUILDER_h

#include <GPS3D.h>
#include <DS1302.h>
#include <AnalogSensor.h>
#include <AnalogSensors.h>

#include "Counters.h"



/**
 * Maximum length of custom frame
 */
#define CUSTOM_FRAME_MAX_LENGTH 200

/**
 * Custom frame field separator character
 */
#define CUSTOM_FRAME_FIELD_SEPARATOR_CHAR ','

/**
 * Custom frame start-of-frame character
 */

#define CUSTOM_FRAME_START_OF_FRAME_CHAR '#'

/**
 * Custom frame end-of-frame string
 */

#define CUSTOM_FRAME_END_OF_FRAME_STRING "\r\n"

/**
 * This class allows to build custom frame to be sent by the HAO, retrieving data from sensors, RTC, GPS, ...
 */
class CustomFrameBuilder
{
private:

  /**
   * Internal pointer used to maintain insertion point of next character in string when building it
   */
  char *whereToAppend;

  /**
   * Pointer to counters
   */
  Counters *counters;

  /**
   * Pointer to Real Time Clock
   */
  DS1302 *rtc;

  /**
   * Pointer to sensors
   */
  AnalogSensors *sensors;

  /**
   * Pointer to voltage sensor
   */
  AnalogSensor *voltage;

  /**
   * Pointer to GPS
   */
  GPS3D *gps;

  /**
   * Internal function used to append start-of-frame char to custom frame
   *
   */
  void
  appendStartOfFrameChar(void);

  /**
   * Internal function used to append field separator char to custom frame
   *
   */
  void
  appendFieldSeparatorChar(void);

  /**
   * Internal function used to append end-of-frame string to custom frame
   *
   */
  void
  appendEndOfFrameString(void);

  /**
   * Internal function used to append HAO name to custom frame
   *
   */
  void
  appendHaoName(void);

  /**
   * Internal function used to append counters to custom frame
   *
   */
  void
  appendCounters(void);

  /**
   * Internal function used to append system time (seconds since last reset) to custom frame
   *
   */
  void
  appendSystemTime(void);

  /**
   * Internal function used to append RTC time to custom frame
   *
   */
  void
  appendRtcTime(void);

  /**
   * Internal function used to append analog sensor values to custom frame
   *
   */
  void
  appendAnalogSensorValues(void);

  /**
   * Internal function used to append voltage value to custom frame
   *
   */
  void
  appendVoltage(void);

  /**
   * Internal function used to append positioning data to custom frame
   *
   */
  void
  appendPositioningData(void);

public:

  /**
   * Creates a new frame builder retrieving data from given providers
   *
   * @param counters counter data provider
   * @param sensors analog sensor data provider
   * @param voltage voltage data provider
   * @param rtc timing data provider
   * @param gps positioning data provider
   */
  CustomFrameBuilder(Counters *counters, AnalogSensors *sensors,
     AnalogSensor *voltage, DS1302 *rtc, GPS3D *gps);

  /**
   * Builds custom frame (time, location, sensor data, ...)
   *
   * @param customFrame an external buffer of at least <tt>CUSTOM_FRAME_MAX_LENGTH</tt> bytes,
   * to store custom frame bytes
   *
   */
  void
  buildCustomFrame(char *customFrame);
};

#endif

