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
#include <customFrameBuilder_module.h>
#include <sensors_module.h>
#include <gps_module.h>

// custom frame, as an ASCII string
char customFrame[CUSTOM_FRAME_MAX_LENGTH];

// custom frame length
int customFrameLength;

// TODO move time management to a dedicated module
void appendTime()
{
  // seconds elapsed since last reset, as a decimal coded ASCII string
  itoa(millis() / 1000, customFrame+customFrameLength, 10);
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

void appendEndOfFrame()
{
  customFrame[customFrameLength++]='\r';
  customFrame[customFrameLength++]='\n';
  customFrame[customFrameLength] = '\0';
}

void addCharGpsValue(char *value)
{
  strcat(customFrame,value);
  customFrameLength = strlen(customFrame);
}

/**
 * Builds custom frame (time, sensors data, ...) from values retrieved from global variables.
 */
void buildCustomFrame()
{
  customFrameLength = 0;

  // time
  appendTime();

  // separator
  appendFieldSeparator();

  // GPS clock
  addCharGpsValue(gpsTime);

  // separator
    appendFieldSeparator();

  // GPS latitude
  addCharGpsValue(gpsLatitude);

  // separator
  appendFieldSeparator();

  // GPS longitude
  addCharGpsValue(gpsLongitude);

  // separator
  appendFieldSeparator();

  // GPS altitude
  addCharGpsValue(gpsAltitude);

  // separator
  appendFieldSeparator();

  // GPS Nb Sat
  addCharGpsValue(gpsNbSat);

  // separator
  appendFieldSeparator();

  // GPS Data status
  addCharGpsValue(gpsDataStatus);

  // separator
  appendFieldSeparator();

  // GPS Speed Over Ground
  addCharGpsValue(gpsSpeedOverGround);

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
