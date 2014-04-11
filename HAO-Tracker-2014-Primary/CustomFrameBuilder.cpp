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

#include <GPS2D.h>
#include <GPS3D.h>
#include <AnalogSensor.h>
#include <AnalogSensors.h>
#include <DS1302.h>

#include "defs.h"
#include "CustomFrameBuilder.h"
#include "Counters.h"

void
CustomFrameBuilder::appendStartOfFrameChar()
{
  this->whereToAppend++[0] = CUSTOM_FRAME_START_OF_FRAME_CHAR;
}

void
CustomFrameBuilder::appendFieldSeparatorChar()
{
  this->whereToAppend++[0] = CUSTOM_FRAME_FIELD_SEPARATOR_CHAR;
}

void
CustomFrameBuilder::appendEndOfFrameString()
{
  strcpy(this->whereToAppend, CUSTOM_FRAME_END_OF_FRAME_STRING);
  this->whereToAppend += strlen(this->whereToAppend);
}

void
CustomFrameBuilder::appendHaoName()
{
  strcpy(this->whereToAppend, HAO_NAME);
  this->whereToAppend += strlen(this->whereToAppend);
}

void
CustomFrameBuilder::appendCounters()
{
  for (int i = 1; i <= this->counters->getAmount(); i++)
  {
    // append next counter value
    itoa(this->counters->read(i), this->whereToAppend, 10);
    this->whereToAppend += strlen(this->whereToAppend);

    // separator (if not last)
    if (i < this->counters->getAmount())
    {
      this->appendFieldSeparatorChar();
    }
  }
}

void
CustomFrameBuilder::appendSystemTime()
{
  // seconds elapsed since last reset, as a decimal coded ASCII string
  itoa((millis() * 5) / 2500, this->whereToAppend, 10);
  this->whereToAppend += strlen(this->whereToAppend);
}

void
CustomFrameBuilder::appendRtcTime()
{
  this->rtc->getTimeBrief(this->whereToAppend);
  this->whereToAppend += strlen(this->whereToAppend);
}

void
CustomFrameBuilder::appendAnalogSensorValues()
{
  for (int analogSensorNumber = 1; analogSensorNumber <= this->sensors->getAmount(); analogSensorNumber++)
  {
    // append next sensor value
    itoa(this->sensors->read(analogSensorNumber), this->whereToAppend, 10);
    this->whereToAppend += strlen(this->whereToAppend);

    // separator (if not last)
    if (analogSensorNumber < this->sensors->getAmount())
    {
      this->appendFieldSeparatorChar();
    }
  }
}

void
CustomFrameBuilder::appendVoltage()
{
  itoa(this->voltage->read(), this->whereToAppend, 10);
  this->whereToAppend += strlen(this->whereToAppend);
}

void
CustomFrameBuilder::appendPositioningData()
{
  // time of fix
  strncpy(this->whereToAppend, this->gps->getTimeOfFix(), 6);
  this->whereToAppend += 6;

  // separator
  this->appendFieldSeparatorChar();

  // fix
  if (this->gps->getFix())
  {
    this->whereToAppend++[0] = 'A';
  }
  else
  {
    this->whereToAppend++[0] = 'V';
  }

  // separator
  this->appendFieldSeparatorChar();

  // longitude
  dtostrf(this->gps->getLongitude(), 2, 3, this->whereToAppend);
  this->whereToAppend += strlen(this->whereToAppend);

  // separator
  this->appendFieldSeparatorChar();

  // latitude
  dtostrf(this->gps->getLatitude(), 2, 3, this->whereToAppend);
  this->whereToAppend += strlen(this->whereToAppend);

  // separator
  this->appendFieldSeparatorChar();

  // altitude
  dtostrf(this->gps->getAltitude(), 2, 1, this->whereToAppend);
  this->whereToAppend += strlen(this->whereToAppend);

  // separator
  this->appendFieldSeparatorChar();

  // speed
  dtostrf(this->gps->getSpeedOverGround(), 2, 1, this->whereToAppend);
  this->whereToAppend += strlen(this->whereToAppend);

  // separator
  this->appendFieldSeparatorChar();

  // course
  dtostrf(this->gps->getCourseOverGround(), 2, 1, this->whereToAppend);
  this->whereToAppend += strlen(this->whereToAppend);

  // separator
  this->appendFieldSeparatorChar();

  // satellites in use
  itoa(this->gps->getSatellitesInUse(), this->whereToAppend, 10);
  this->whereToAppend += strlen(this->whereToAppend);

  // separator
  this->appendFieldSeparatorChar();

  // HDOP
  dtostrf(this->gps->getHDOP(), 2, 1, this->whereToAppend);
  this->whereToAppend += strlen(this->whereToAppend);
}

CustomFrameBuilder::CustomFrameBuilder(Counters *counters, AnalogSensors *sensors, AnalogSensor *voltage, DS1302 *rtc, GPS3D *gps)
{
  this->counters = counters;
  this->rtc = rtc;
  this->sensors = sensors;
  this->gps = gps;
  this->voltage = voltage;
}

void
CustomFrameBuilder::buildCustomFrame(char *customFrame)
{
  this->whereToAppend = customFrame;

  this->appendStartOfFrameChar();

  // HAO name
  this->appendHaoName();

  // separator
  this->appendFieldSeparatorChar();

  // counters
  this->appendCounters();

  // separator
  this->appendFieldSeparatorChar();

  // system time
  this->appendSystemTime();

  // separator
  this->appendFieldSeparatorChar();

  // RTC time
  this->appendRtcTime();

  // separator
  this->appendFieldSeparatorChar();

  // positioning data
  this->appendPositioningData();

  // separator
  this->appendFieldSeparatorChar();

  // analog sensors
  this->appendAnalogSensorValues();

  // separator
  this->appendFieldSeparatorChar();

  // voltage
  this->appendVoltage();

  // end of frame
  this->appendEndOfFrameString();
}
