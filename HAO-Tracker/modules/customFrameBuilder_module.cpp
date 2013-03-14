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
#include <stdlib.h>
#include <defs.h>
#include <GPS2D.h>
#include <GPS3D.h>

#include <customFrameBuilder_module.h>
#include <analogSensors_module.h>
#include <rtc_module.h>

/**
 * Internal function used to append start-of-frame char to custom frame
 *
 * @param customFrame a pointer on an external buffer where to append the start-of-frame character
 * @return a pointer to the external buffer where to add the next custom frame character
 */
char *appendStartOfFrameChar(char* customFrame)
{
  customFrame[0] = CUSTOM_FRAME_START_OF_FRAME_CHAR;
  return customFrame+1;
}

/**
 * Internal function used to append the name of the ball to custom frame
 *
 * @param customFrame a pointer on an external buffer where to append the name of the ball
 * @return a pointer to the external buffer where to add the next custom frame character
 */
char * appendNameOfBall(char *customFrame)
{
  strcpy(customFrame, HAO_NAME);
  return customFrame + strlen(customFrame);
}

/**
 * Internal function used to append field separator char to custom frame
 *
 * @param customFrame a pointer on an external buffer where to append the separator
 * @return a pointer to the external buffer where to add the next custom frame character
 */
char *appendFieldSeparatorChar(char *customFrame)
{
  customFrame[0] = CUSTOM_FRAME_FIELD_SEPARATOR_CHAR;
  return customFrame+1;
}

/**
 * Internal function used to append end-of-frame string to custom frame
 *
 * @param customFrame a pointer on an external buffer where to append end-of-frame
 * @return a pointer to the external buffer where to add the next custom frame character
 */
char * appendEndOfFrameString(char *customFrame)
{
  strcpy(customFrame, CUSTOM_FRAME_END_OF_FRAME_STRING);
  return customFrame + strlen(customFrame);
}

/**
 * Internal function used to append system time (seconds since last reset) to custom frame
 *
 * @param customFrame a pointer on an external buffer where to append system time
 * @return a pointer to the external buffer where to add the next custom frame character
 */
char *appendSystemTime(char* customFrame)
{
  // seconds elapsed since last reset, as a decimal coded ASCII string
  itoa(millis() / 1000, customFrame, 10);
  return customFrame+strlen(customFrame);
}

/**
 * Internal function used to append RTC time to custom frame
 *
 * @param customFrame a pointer on an external buffer where to append RTC time
 * @return a pointer to the external buffer where to add the next custom frame character
 */
char *appendRtcTime(char* customFrame)
{
  getRtcTime(customFrame);
  return customFrame+strlen(customFrame);
}

/**
 * Internal function used to append analog sensor values to custom frame
 *
 * @param customFrame a pointer on an external buffer where to append analog sensor values
 * @return a pointer to the external buffer where to add the next custom frame character
 */
char *appendAnalogSensorValues(char* customFrame)
{
  for (int i=1;i<=ANALOG_SENSORS_COUNT;i++)
  {
      // append next sensor value
      itoa(readSensor(i), customFrame, 10);
      customFrame += strlen(customFrame);

      // separator (if not last)
      if (i < ANALOG_SENSORS_COUNT)
      {
        customFrame = appendFieldSeparatorChar(customFrame);
      }
  }
  return customFrame;
}

/**
 * Internal function used to append positioning data to custom frame
 *
 * @param customFrame a pointer on an external buffer where to append positioning data
 * @return a pointer to the external buffer where to add the next custom frame character
 */
char *appendPositioningData(char *customFrame)
{
  // time of fix
  strncpy(customFrame, nmeaGPS.getTimeOfFix(),6);
  customFrame += 6;

  // separator
  customFrame = appendFieldSeparatorChar(customFrame);

  // fix
  if (nmeaGPS.getFix())
  {
      customFrame++[0]='A';
  }
  else
  {
      customFrame++[0]='V';
  }

  // separator
  customFrame = appendFieldSeparatorChar(customFrame);

  // longitude
  dtostrf(nmeaGPS.getLongitude(),2,3,customFrame);
  customFrame += strlen(customFrame);

  // separator
  customFrame = appendFieldSeparatorChar(customFrame);

  // latitude
  dtostrf(nmeaGPS.getLatitude(),2,3,customFrame);
  customFrame += strlen(customFrame);

  // separator
  customFrame = appendFieldSeparatorChar(customFrame);

  // altitude
  dtostrf(nmeaGPS.getAltitude(),2,1,customFrame);
  customFrame += strlen(customFrame);

  // separator
  customFrame = appendFieldSeparatorChar(customFrame);

  // speed
  dtostrf(nmeaGPS.getSpeedOverGround(),2,1,customFrame);
  customFrame += strlen(customFrame);

  // separator
  customFrame = appendFieldSeparatorChar(customFrame);

  // course
  dtostrf(nmeaGPS.getCourseOverGround(),2,1,customFrame);
  customFrame += strlen(customFrame);

  // separator
  customFrame = appendFieldSeparatorChar(customFrame);

  // satellites in use
  itoa(nmeaGPS.getSatellitesInUse(),customFrame,10);
  customFrame += strlen(customFrame);

  // separator
  customFrame = appendFieldSeparatorChar(customFrame);

  // HDOP
  dtostrf(nmeaGPS.getHDOP(),2,1,customFrame);
  return  customFrame +strlen(customFrame);
}

/**
 * Builds custom frame (time, location, sensor data, ...)
 */
void buildCustomFrame(char *customFrame)
{
  customFrame = appendStartOfFrameChar(customFrame);

  // name of the ball
  customFrame = appendNameOfBall(customFrame);

  // separator
  customFrame = appendFieldSeparatorChar(customFrame);

  // system time
  customFrame = appendSystemTime(customFrame);

  // separator
  customFrame = appendFieldSeparatorChar(customFrame);

  // RTC time
  customFrame = appendRtcTime(customFrame);

  // separator
  customFrame = appendFieldSeparatorChar(customFrame);

  // positioning data
  customFrame = appendPositioningData(customFrame);

  // separator
  customFrame = appendFieldSeparatorChar(customFrame);

  // analog sensors
  customFrame = appendAnalogSensorValues(customFrame);

  // end of frame
  customFrame = appendEndOfFrameString(customFrame);
}
