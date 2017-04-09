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

#include <core/AnalogSensors.h>
#include <Counters.h>
#include <core/GPS3D.h>
#include <DS1302.h>

#include <main/defs.h>
#include <modules/framebuilder/CustomFrameBuilder.h>

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

  // Format the time and date and insert into the temporary buffer

 sprintf(this->whereToAppend, rtc->getTimeStr());
 this->whereToAppend += strlen(this->whereToAppend);
}

void
CustomFrameBuilder::appendAnalogSensorValues()
{
  for (int analogSensorNumber = 1; analogSensorNumber <= this->analogSensors->getAmount(); analogSensorNumber++)
  {
    // append next sensor value
    itoa(this->analogSensors->read(analogSensorNumber), this->whereToAppend, 10);
    this->whereToAppend += strlen(this->whereToAppend);

    // separator (if not last)
    if (analogSensorNumber < this->analogSensors->getAmount())
    {
      this->appendFieldSeparatorChar();
    }
  }
}

void
CustomFrameBuilder::appendPositioningData()
{
  int mostReliableGpsNumber = 0;
  int numberOfSats = 0;

  for (int index=0; index< SERIAL_NMEA_GPS_AMOUNT; index++)
  {
    if (this->gpsArray[index]->getFix())
      if (this->gpsArray[index]->getSatellitesInUse() > numberOfSats)
        mostReliableGpsNumber = index;
  }

  // GPS used
  itoa(mostReliableGpsNumber, this->whereToAppend, 10);
  this->whereToAppend += strlen(this->whereToAppend);

  // separator
  this->appendFieldSeparatorChar();

  // time of fix
  strncpy(this->whereToAppend, this->gpsArray[mostReliableGpsNumber]->getTimeOfFix(), 6);
  this->whereToAppend += 6;

  // separator
  this->appendFieldSeparatorChar();

  // fix
  if (this->gpsArray[mostReliableGpsNumber]->getFix())
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
  dtostrf(this->gpsArray[mostReliableGpsNumber]->getLongitude(), 2, 3, this->whereToAppend);
  this->whereToAppend += strlen(this->whereToAppend);

  // separator
  this->appendFieldSeparatorChar();

  // latitude
  dtostrf(this->gpsArray[mostReliableGpsNumber]->getLatitude(), 2, 3, this->whereToAppend);
  this->whereToAppend += strlen(this->whereToAppend);

  // separator
  this->appendFieldSeparatorChar();

  // altitude
  dtostrf(this->gpsArray[mostReliableGpsNumber]->getAltitude(), 2, 1, this->whereToAppend);
  this->whereToAppend += strlen(this->whereToAppend);

  // separator
  this->appendFieldSeparatorChar();

  // speed
  dtostrf(this->gpsArray[mostReliableGpsNumber]->getSpeedOverGround(), 2, 1, this->whereToAppend);
  this->whereToAppend += strlen(this->whereToAppend);

  // separator
  this->appendFieldSeparatorChar();

  // course
  dtostrf(this->gpsArray[mostReliableGpsNumber]->getCourseOverGround(), 2, 1, this->whereToAppend);
  this->whereToAppend += strlen(this->whereToAppend);

  // separator
  this->appendFieldSeparatorChar();

  // satellites in use
  itoa(this->gpsArray[mostReliableGpsNumber]->getSatellitesInUse(), this->whereToAppend, 10);
  this->whereToAppend += strlen(this->whereToAppend);

  // separator
  this->appendFieldSeparatorChar();

  // HDOP
  dtostrf(this->gpsArray[mostReliableGpsNumber]->getHDOP(), 2, 1, this->whereToAppend);
  this->whereToAppend += strlen(this->whereToAppend);
}

CustomFrameBuilder::CustomFrameBuilder(Counters *counters, AnalogSensors *analogSensors, DS1302 *rtc, GPS3D **gpsArray)
{
  this->counters = counters;
  this->rtc = rtc;
  this->analogSensors = analogSensors;
  this->gpsArray = gpsArray;
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

  // end of frame
  this->appendEndOfFrameString();
}
