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
#include <sensors_module.h>
#include <rtc_module.h>


// custom frame, as an ASCII string
char customFrame[CUSTOM_FRAME_MAX_LENGTH];

// custom frame length
int customFrameLength;

void appendSystemTime()
{
  // seconds elapsed since last reset, as a decimal coded ASCII string
  itoa(millis() / 1000, customFrame+customFrameLength, 10);
  customFrameLength = strlen(customFrame);
}

void appendRtcTime()
{
  getRtcTime(customFrame+customFrameLength);
  customFrameLength = strlen(customFrame);
}

void appendFieldSeparator()
{
  customFrame[customFrameLength++] = CUSTOM_FRAME_FIELD_SEPARATOR;
}

void appendAnalogValue(int value)
{
  itoa(value, customFrame + customFrameLength, 10);
  customFrameLength = strlen(customFrame);
}

void appendPositioningData()
{
  // time of fix
  strncpy(customFrame+customFrameLength, nmeaGPS.getTimeOfFix(),6);
  customFrameLength +=6;

  // separator
  appendFieldSeparator();

  // fix
  if (nmeaGPS.getFix())
  {
    customFrame[customFrameLength++]='A';
  }
  else
  {
      customFrame[customFrameLength++]='V';
  }

  // separator
  appendFieldSeparator();

  // longitude
  dtostrf(nmeaGPS.getLongitude(),2,3,customFrame+customFrameLength);
  customFrameLength = strlen(customFrame);

  // separator
  appendFieldSeparator();

  // latitude
  dtostrf(nmeaGPS.getLatitude(),2,3,customFrame+customFrameLength);
  customFrameLength = strlen(customFrame);

  // separator
  appendFieldSeparator();

  // altitude
  dtostrf(nmeaGPS.getAltitude(),2,1,customFrame+customFrameLength);
  customFrameLength = strlen(customFrame);

  // separator
  appendFieldSeparator();

  // speed
  dtostrf(nmeaGPS.getSpeedOverGround(),2,1,customFrame+customFrameLength);
  customFrameLength = strlen(customFrame);

  // separator
  appendFieldSeparator();

  // course
  dtostrf(nmeaGPS.getCourseOverGround(),2,1,customFrame+customFrameLength);
  customFrameLength = strlen(customFrame);

  // separator
  appendFieldSeparator();

  // satellites in use
  itoa(nmeaGPS.getSatellitesInUse(),customFrame+customFrameLength,10);
  customFrameLength = strlen(customFrame);

  // separator
  appendFieldSeparator();

  // HDOP
  dtostrf(nmeaGPS.getHDOP(),2,1,customFrame+customFrameLength);
  customFrameLength = strlen(customFrame);
}

void appendEndOfFrame()
{
  customFrame[customFrameLength++]='\r';
  customFrame[customFrameLength++]='\n';
  customFrame[customFrameLength] = '\0';
}

/**
 * Builds custom frame (time, sensors data, ...) from values retrieved from global variables.
 */
void buildCustomFrame()
{
  customFrameLength = 0;

  // system time
  appendSystemTime();

  // separator
  appendFieldSeparator();

  // RTC time
  appendRtcTime();

  // separator
  appendFieldSeparator();

  // positioning data
  appendPositioningData();

  // separator
  appendFieldSeparator();

  // absolute pressure
  appendAnalogValue(absolutePressureSensorValue);

  // separator
  appendFieldSeparator();

  // differential pressure
  appendAnalogValue(differentialPressureSensorValue);

  // separator
  appendFieldSeparator();

  // internal temperature
  appendAnalogValue(internalTemperatureSensorValue);

  // separator
  appendFieldSeparator();

  // external temperature
  appendAnalogValue(externalTemperatureSensorValue);

  //separator
  appendFieldSeparator();

  // battery voltage
  appendAnalogValue(batteryVoltageSensorValue);

  // end of frame
  appendEndOfFrame();
}
